// No hardware backend: exercise the real C++ ControllerInterface against MuJoCo.
// ROS plans and observations run in a separate executor; no logging in update.
#include "mech_controllers/ref01_impedance_controller.hpp"
#include "ref01_model.hpp"
#include "mech_exo_state/msg/robot_state.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

using Clock = std::chrono::steady_clock;
using Controller = mech::mech_controllers::Ref01ImpedanceController;
using namespace std::chrono_literals;
static std::int64_t ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
struct Row {
  double force[2]{};
  double sim_time{}, q[2]{}, dq[2]{}, estimated[2]{}, velocity[2]{}, applied[2]{}, scale{}, closure{}, compute{};
  std::int64_t cycle{}, feedback{}, application{}, finish{};
  double generation{};
  int mode{}, reason{};
};
struct Snapshot { double residual{}, sample_sequence{}; Row row{}; double position[2]{}, speed[2]{}; bool valid{}; };

struct Spinner {
  rclcpp::executors::SingleThreadedExecutor& executor;
  std::thread thread;
  explicit Spinner(rclcpp::executors::SingleThreadedExecutor& e)
      : executor(e), thread([&e]() { e.spin(); }) {}
  void stop() { executor.cancel(); if (thread.joinable()) thread.join(); }
  ~Spinner() { stop(); }
};

int main(int argc, char** argv) {
  if (argc != 6 || (std::string(argv[5]) != "--software-only" &&
                    std::string(argv[5]) != "--software-bench-gains")) {
    std::cerr << "Usage: ref01_impedance_software_loop MODEL.xml OUTPUT.csv psi|beta|loss|duplicate SECONDS --software-only\n";
    return 2;
  }
  // Both modes use only in-memory loaned interfaces and a MuJoCo plant; no hardware plugin.
  const bool bench_gains = std::string(argv[5]) == "--software-bench-gains";
  const std::string scenario = argv[3];
  const double duration = std::stod(argv[4]);
  if ((scenario != "psi" && scenario != "beta" && scenario != "loss" && scenario != "duplicate") ||
      !std::isfinite(duration) || duration < 2.5 || duration > 120.) return 2;
  // Fail before starting if output is unavailable/existing.
  if (std::ifstream(argv[2]).good()) return 2;
  std::ofstream output(argv[2]);
  if (!output) return 2;
  rclcpp::init(0, nullptr);
  int result = 0;
  try {
    ph_exo::Ref01Model plant_model(argv[1]);  // separate model/data from controller estimator
    auto* model = plant_model.model();
    auto* plant = plant_model.data();
    const int a[2]{mj_name2id(model, mjOBJ_JOINT, "alpha1"), mj_name2id(model, mjOBJ_JOINT, "alpha2")};
    const int q[2]{mj_name2id(model, mjOBJ_JOINT, "psi"), mj_name2id(model, mjOBJ_JOINT, "beta")};
    const int actuator[2]{mj_name2id(model, mjOBJ_ACTUATOR, "alpha1_torque"),
                          mj_name2id(model, mjOBJ_ACTUATOR, "alpha2_torque")};
    const int rod[2]{mj_name2id(model, mjOBJ_SITE, "rod_end1"), mj_name2id(model, mjOBJ_SITE, "rod_end2")};
    const int target[2]{mj_name2id(model, mjOBJ_SITE, "Q1"), mj_name2id(model, mjOBJ_SITE, "Q2")};
    mj_forward(model, plant);
    if (std::abs(model->opt.timestep - .0005) > 1e-12 ||
        std::abs(plant->qpos[q[0]]) > 1e-8 ||
        std::abs(plant->qpos[q[1]] - 3.14159265358979323846/9.) > 1e-8)
      throw std::runtime_error("Unexpected model home or timestep");
    Controller controller;
    const std::string controller_name = "ph_exo_native_" + scenario;
    if (controller.init(controller_name) != controller_interface::return_type::OK)
      throw std::runtime_error("Controller init failed");
    controller.get_node()->set_parameters({{"simulation_only", !bench_gains},
      {"bench_profile", std::string(bench_gains?"ref01_supported_v1":"")}, {"model_path", std::string(argv[1])}});
    rclcpp_lifecycle::State lifecycle;
    if (controller.on_configure(lifecycle) != controller_interface::CallbackReturn::SUCCESS)
      throw std::runtime_error("Controller configure failed");
    double states[8]{}, commands[4]{};
    const std::array<std::string, 2> joints{"motor104_joint", "motor105_joint"};
    const std::array<std::string, 4> fields{"position", "velocity", "sample_time_s", "sequence"};
    std::vector<hardware_interface::StateInterface> state_handles;
    std::vector<hardware_interface::CommandInterface> command_handles;
    state_handles.reserve(8); command_handles.reserve(4);
    for (int i = 0; i < 2; ++i) {
      for (int k = 0; k < 4; ++k)
        state_handles.emplace_back(joints[i], fields[k], &states[i*4+k]);
      command_handles.emplace_back(joints[i], "effort", &commands[i*2]);
      command_handles.emplace_back(joints[i], "command_generation", &commands[i*2+1]);
    }
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    for (auto& s : state_handles) loaned_states.emplace_back(s);
    for (auto& c : command_handles) loaned_commands.emplace_back(c);
    controller.assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
    std::int64_t feedback_ns = 0;
    double sequence = 0.;
    auto feedback = [&]() {
      feedback_ns = ns();
      for (int i = 0; i < 2; ++i) {
        states[i*4] = plant->qpos[a[1-i]] + 3.14159265358979323846/4.;
        states[i*4+1] = plant->qvel[a[1-i]];
        states[i*4+2] = static_cast<double>(feedback_ns)*1e-9;
        states[i*4+3] = sequence + 1.;
      }
      sequence += 1.;
    };
    feedback();
    if (controller.on_activate(lifecycle) != controller_interface::CallbackReturn::SUCCESS)
      throw std::runtime_error("Controller activate failed");
    auto io = std::make_shared<rclcpp::Node>(controller_name + "_supervisor");
    auto plans = io->create_publisher<std_msgs::msg::Float64MultiArray>("/"+controller_name+"/plan", 1);
    auto observations = io->create_publisher<mech_exo_state::msg::RobotState>(
        "/"+controller_name+"/state", rclcpp::SensorDataQoS());
    std::mutex snapshot_mutex;
    Snapshot latest;
    bool available = false;
    std::uint64_t plan_seq = 0;
    std::atomic<unsigned> published{0}, seen{0};
    auto observer_sub = io->create_subscription<mech_exo_state::msg::RobotState>(
        "/"+controller_name+"/state", rclcpp::SensorDataQoS(),
        [&](const mech_exo_state::msg::RobotState::SharedPtr m) {
          if (m->q_valid && m->q_velocity_valid && !m->hardware_freshness_verified) ++seen;
        });
    auto plan_timer = io->create_wall_timer(20ms, [&]() {
      std_msgs::msg::Float64MultiArray plan;
      plan.data = {1., static_cast<double>(++plan_seq), static_cast<double>(ns())*1e-9, 1.,
                   0., 3.14159265358979323846/9., bench_gains?10.:2.4, bench_gains?10.:2.,
                   bench_gains?1.:.18, bench_gains?1.:.15, bench_gains?5.:.2, bench_gains?5.:.2};
      plans->publish(plan);
    });
    auto observation_timer = io->create_wall_timer(20ms, [&]() {
      Snapshot copy;
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        if (!available) return;
        copy = latest;
      }
      mech_exo_state::msg::RobotState message;
      message.header.stamp = io->now();
      const auto age = static_cast<double>(ns()-copy.row.feedback)*1e-6;
      copy.valid = copy.valid && age >= 0. && age < 60.;
      message.sources[0].source_stamp = rclcpp::Time(
          io->now().nanoseconds()-static_cast<std::int64_t>(age*1e6));
      message.sources[0].received_stamp = message.sources[0].source_stamp;
      message.sources[0].source_age_ms = age;
      message.sources[0].receive_age_ms = age;
      message.sources[0].accepted_samples = static_cast<std::uint64_t>(copy.sample_sequence);
      message.sources[0].valid = copy.valid;
      message.sources[0].sample_time_verified = false;
      message.sources[0].reason = "simulation_local_monotonic_source";
      message.mapping_confirmed = true;  // only the explicit simulator fixture
      message.hardware_freshness_verified = false;
      message.q_valid = message.q_velocity_valid = copy.valid;
      message.alpha_valid = message.alpha_velocity_valid = copy.valid;
      message.q_velocity_condition = std::numeric_limits<double>::quiet_NaN();
      message.q_reason = message.q_velocity_reason = copy.valid ? "native_simulation" : "controller_not_following";
      message.q_closure_residual_m = copy.residual;
      for (int i = 0; i < 2; ++i) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        message.q_position[i] = copy.valid ? copy.row.estimated[i] : nan;
        message.q_velocity[i] = copy.valid ? copy.row.velocity[i] : nan;
        message.alpha_position[i] = copy.valid ? copy.position[1-i]-3.14159265358979323846/4. : nan;
        message.alpha_velocity[i] = copy.valid ? copy.speed[1-i] : nan;
      }
      observations->publish(message);
      ++published;
    });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller.get_node()->get_node_base_interface());
    executor.add_node(io);

    // All loop storage is allocated before measurement. The control thread
    // never waits for the observation mutex or writes files/ROS messages.
    const bool inject = scenario == "loss" || scenario == "duplicate";
    const std::size_t count = static_cast<std::size_t>(std::llround(duration/.005)) + 1;
    std::vector<Row> rows(count);
    double last_generation = 0.;
    std::int64_t last_accept = 0, interruption = 0, receiver_zero = 0;
    bool receiver_latched = false;
    std::size_t written = 0;
    auto next = Clock::now();
    auto previous = ns();
    bool started = false;
    const auto startup_deadline = Clock::now() + 5s;
    Spinner spinner(executor);
    while (written < count && rclcpp::ok()) {
      next += 5ms;
      std::this_thread::sleep_until(next);
      const auto cycle = ns();
      const double simulation_time = static_cast<double>(written)*.005;
      // Loss leaves the interface untouched; duplicate re-delivers the exact
      // same source, sequence and values. Neither refreshes source validity.
      const bool cut = started && inject && simulation_time >= 1.2 && simulation_time < 1.65;
      if (!cut) feedback();
      else {
        if (!interruption) interruption = ns();
        if (scenario == "duplicate") {
          const std::array<double, 8> replay{states[0],states[1],states[2],states[3],
                                            states[4],states[5],states[6],states[7]};
          std::copy(replay.begin(), replay.end(), states);
        }
      }
      controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(
          static_cast<double>(cycle-previous)*1e-9));
      previous = cycle;
      const auto& observed = controller.observation();
      const bool new_command = commands[1] > last_generation && commands[1] == commands[3];
      std::int64_t application = 0;
      if (new_command && !receiver_latched) {
        if (std::abs(commands[0]) > .2+1e-12 || std::abs(commands[2]) > .2+1e-12 ||
            !std::isfinite(commands[0]) || !std::isfinite(commands[2])) {
          receiver_latched = true;
        } else {
          plant->ctrl[actuator[0]] = commands[2];
          plant->ctrl[actuator[1]] = commands[0];
          last_generation = commands[1];
          last_accept = ns();
          application = last_accept;
          started = true;
        }
      }
      if (last_accept && ns()-last_accept >= 60000000) {
        receiver_latched = true;
        if (!receiver_zero) receiver_zero = ns();
      }
      if (receiver_latched) { plant->ctrl[actuator[0]]=0.; plant->ctrl[actuator[1]]=0.; }

      if (!started) {
        if (Clock::now() >= startup_deadline) break;
        continue;
      }
      Row& row = rows[written];
      row.sim_time = simulation_time; row.cycle = cycle; row.feedback = feedback_ns;
      row.application = application; row.compute = controller.compute_seconds()*1e3;
      row.generation = last_generation;
      row.mode = static_cast<int>(observed.mode); row.reason = static_cast<int>(observed.reason);
      row.scale = observed.torque.scale;
      for (int i = 0; i < 2; ++i) {
        row.q[i]=plant->qpos[q[i]]; row.dq[i]=plant->qvel[q[i]];
        row.estimated[i]=observed.state.q[i]; row.velocity[i]=observed.state.dq[i];
        row.applied[i]=plant->ctrl[actuator[1-i]];
      }
      for (int k = 0; k < 8; ++k) plant->qfrc_applied[k]=0.;
      if (simulation_time>=.4 && simulation_time<1.)
        plant->qfrc_applied[q[scenario=="beta" ? 1 : 0]]=.09;
      for (int step = 0; step < 10; ++step) mj_step(model, plant);
      mj_forward(model, plant);
      for (int i = 0; i < 2; ++i) {
        row.force[i] = plant->actuator_force[actuator[1-i]];
        double square = 0.;
        for (int k = 0; k < 3; ++k) {
          const auto difference = plant->site_xpos[3*rod[i]+k]-plant->site_xpos[3*target[i]+k];
          square += difference*difference;
        }
        row.closure=std::max(row.closure,std::sqrt(square));
      }
      {
        std::unique_lock<std::mutex> lock(snapshot_mutex, std::try_to_lock);
        if (lock.owns_lock()) {
          latest.row=row;
          latest.residual=observed.state.residual;
          latest.sample_sequence=sequence;
          latest.valid = observed.refresh && !receiver_latched;
          for (int i=0;i<2;++i) { latest.position[i]=states[i*4]; latest.speed[i]=states[i*4+1]; }
          available=true;
        }
      }
      row.finish=ns();
      ++written;
      // Do not burst to catch up after a missed entire period.
      if (Clock::now() >= next+5ms) next = Clock::now();
    }
    const auto final_reason = controller.observation().reason;
    controller.on_deactivate(lifecycle);
    spinner.stop();
    controller.release_interfaces();
    output << "time_s,cycle_start_ns,feedback_ns,application_ns,finish_ns,psi_rad,beta_rad,dpsi_rad_s,dbeta_rad_s,estimated_psi_rad,estimated_beta_rad,estimated_dpsi,estimated_dbeta,tau104_nm,tau105_nm,scale,closure_end_m,compute_ms,generation,mode,reason,applied104_end_nm,applied105_end_nm\n";
    output << std::setprecision(17);
    for (std::size_t i=0;i<written;++i) {
      const auto& r=rows[i];
      output << r.sim_time << ',' << r.cycle << ',' << r.feedback << ',' << r.application << ',' << r.finish
             << ',' << r.q[0] << ',' << r.q[1] << ',' << r.dq[0] << ',' << r.dq[1]
             << ',' << r.estimated[0] << ',' << r.estimated[1] << ',' << r.velocity[0] << ',' << r.velocity[1]
             << ',' << r.applied[0] << ',' << r.applied[1] << ',' << r.scale << ',' << r.closure
             << ',' << r.compute << ',' << r.generation << ',' << r.mode << ',' << r.reason
             << ',' << r.force[0] << ',' << r.force[1] << '\n';
    }
    int warnings=0; for (const auto& warning : plant->warning) warnings+=warning.number;
    std::ofstream meta(std::string(argv[2])+".meta.json");
    meta << "{\"samples\":" << written << ",\"interruption_ns\":" << interruption
         << ",\"receiver_zero_ns\":" << receiver_zero << ",\"receiver_latched\":" << (receiver_latched?"true":"false")
         << ",\"warnings\":" << warnings << ",\"ros_published\":" << published.load()
         << ",\"ros_observed\":" << seen.load() << ",\"final_reason\":\""
         << mech::mech_controllers::ref01::reason_name(final_reason)
         << "\",\"final_q\":[" << std::setprecision(17) << plant->qpos[q[0]] << ',' << plant->qpos[q[1]]
         << "],\"final_dq\":[" << plant->qvel[q[0]] << ',' << plant->qvel[q[1]]
         << "],\"max_closure_m\":" << (written ? rows[written-1].closure : 0.)
         << ",\"mujoco_runtime\":\"" << mj_versionString() << "\"}\n";
    if (written != count || seen.load() == 0) result=1;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n'; result=1;
  }
  rclcpp::shutdown();
  return result;
}
