#include "mech_simulation/ref01_mujoco_system.hpp"
#include "mech_simulation/ref01_sim_receiver.hpp"
#include "ref01_model.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mech::mech_simulation {
namespace {
constexpr double kPeriod = .005;
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaximumSamples = 24001;
constexpr std::uint64_t kStartupLimit = 4000;  // 20 nominal seconds, no model stepping.
constexpr const char* kJoints[2] = {"motor104_joint", "motor105_joint"};
constexpr const char* kStates[4] = {"position", "velocity", "sample_time_s", "sequence"};
constexpr const char* kCommands[2] = {"effort", "command_generation"};

std::int64_t now_ns() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

template<class Interfaces, std::size_t Size>
bool exact_interfaces(const Interfaces& interfaces, const char* const (&names)[Size]) {
  if (interfaces.size() != Size) return false;
  for (const auto* name : names) {
    if (std::count_if(interfaces.begin(), interfaces.end(),
        [name](const auto& item) { return item.name == name; }) != 1) return false;
  }
  return true;
}

double strict_double(const std::string& value) {
  std::size_t end{};
  const double parsed = std::stod(value, &end);
  if (end != value.size() || !std::isfinite(parsed)) throw std::runtime_error("invalid number");
  return parsed;
}

void open_output(std::ofstream& stream, const std::filesystem::path& path) {
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.open(path, std::ios::out | std::ios::trunc);
  stream << std::setprecision(17);
}

// Writer-only formatting: even a nonfinite plant failure must leave valid JSON.
std::string json_number(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}
}  // namespace

struct Ref01MujocoSystem::Impl {
  struct Row {
    std::uint64_t index{}, read_count{}, write_count{}, sequence{}, generation{};
    std::int64_t cycle_start_ns{}, feedback_ns{}, application_ns{}, finish_ns{};
    double period_arg_ms{}, sim_time_s{};
    std::array<double, 2> q{}, dq{}, effort{}, actual_force{};
    std::array<double, 2> motor_position{}, motor_velocity{}, command_generation{};
    double closure_m{};
    bool source_fresh{}, command_accepted{}, receiver_latched{};
  };
  // Only this frozen value object and rows[0..samples) cross to the writer.
  // The RT path never changes either after complete.store(release).
  struct Final {
    std::size_t samples{};
    std::uint64_t read_count{}, write_count{}, startup_writes{}, warnings{};
    std::int64_t first_read_ns{}, finish_ns{}, receiver_zero_ns{}, interruption_ns{};
    double runtime_s{}, sim_time_s{}, max_closure_m{};
    std::array<double, 2> q{}, dq{};
    bool receiver_latched{};
    Ref01SimReceiver::Fault receiver_fault{Ref01SimReceiver::Fault::None};
    const char* runtime_error{"none"};
  };

  std::string model_path, scenario;
  std::filesystem::path output_dir;
  double duration_s{};
  std::size_t target_samples{}, max_samples{};
  std::unique_ptr<ph_exo::Ref01Model> plant;
  // Indices are [alpha1,alpha2] for model and [psi,beta] for output.
  std::array<int, 2> alpha_pos{}, alpha_vel{}, q_pos{}, q_vel{}, actuator{}, rod{}, target{};
  std::array<double, 8> state{}, duplicate_state{};
  std::array<double, 4> command{};
  Ref01SimReceiver receiver;
  std::vector<Row> rows;
  std::size_t samples{};
  std::uint64_t read_count{}, write_count{}, startup_writes{}, sequence{};
  std::int64_t cycle_start_ns{}, feedback_ns{}, first_read_ns{}, interruption_ns{};
  double period_arg_ms{}, max_closure_m{};
  bool source_fresh{}, configured{}, activated_once{};
  const char* runtime_error{"none"};
  Final final;
  std::atomic<bool> complete{false}, cancel_writer{false};
  static constexpr std::uint32_t kClosed = 0x80000000U;
  std::atomic<std::uint32_t> callback_gate{kClosed};
  std::thread writer;

  // Drain an in-progress callback before lifecycle shutdown touches mjData.
  // The hot side uses atomics only; waiting belongs exclusively to stop().
  struct Visit {
    Impl* owner;
    explicit Visit(Impl& impl) noexcept : owner(nullptr) {
      auto value = impl.callback_gate.load(std::memory_order_acquire);
      while ((value & kClosed) == 0) {
        if (impl.callback_gate.compare_exchange_weak(value, value + 1,
                std::memory_order_acq_rel, std::memory_order_acquire)) {
          owner = &impl;
          return;
        }
      }
    }
    ~Visit() { if (owner) owner->callback_gate.fetch_sub(1, std::memory_order_release); }
    explicit operator bool() const noexcept { return owner != nullptr; }
    Visit(const Visit&) = delete;
    Visit& operator=(const Visit&) = delete;
  };

  ~Impl() { stop(); }

  void stop() noexcept {
    callback_gate.fetch_or(kClosed, std::memory_order_acq_rel);
    while ((callback_gate.load(std::memory_order_acquire) & ~kClosed) != 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    cancel_writer.store(true, std::memory_order_release);
    if (writer.joinable()) writer.join();
    if (plant)
      for (const int index : actuator)
        if (index >= 0 && index < plant->model()->nu) plant->data()->ctrl[index] = 0.;
  }

  void feedback(std::int64_t instant) noexcept {
    auto* data = plant->data();
    feedback_ns = instant;
    ++sequence;
    for (int motor = 0; motor < 2; ++motor) {
      const int alpha = 1 - motor;
      state[4 * motor] = data->qpos[alpha_pos[alpha]] + kPi / 4.;
      state[4 * motor + 1] = data->qvel[alpha_vel[alpha]];
      state[4 * motor + 2] = static_cast<double>(instant) * 1e-9;
      state[4 * motor + 3] = static_cast<double>(sequence);
    }
    duplicate_state = state;
  }

  void set_effort() noexcept {
    // motor104 -> alpha2, motor105 -> alpha1, identical to the validated runner.
    plant->data()->ctrl[actuator[1]] = receiver.effort()[0];
    plant->data()->ctrl[actuator[0]] = receiver.effort()[1];
  }

  std::uint64_t warnings() const noexcept {
    std::uint64_t count{};
    for (int index = 0; index < mjNWARNING; ++index)
      count += static_cast<std::uint64_t>(plant->data()->warning[index].number);
    return count;
  }

  double closure() const noexcept {
    double maximum{};
    const auto* data = plant->data();
    for (int branch = 0; branch < 2; ++branch) {
      double squared{};
      for (int coordinate = 0; coordinate < 3; ++coordinate) {
        const double difference = data->site_xpos[3 * rod[branch] + coordinate] -
                                  data->site_xpos[3 * target[branch] + coordinate];
        squared += difference * difference;
      }
      maximum = std::max(maximum, std::sqrt(squared));
    }
    return maximum;
  }

  bool finite_plant() const noexcept {
    const auto* model = plant->model();
    const auto* data = plant->data();
    for (int i = 0; i < model->nq; ++i) if (!std::isfinite(data->qpos[i])) return false;
    for (int i = 0; i < model->nv; ++i) if (!std::isfinite(data->qvel[i])) return false;
    for (int i = 0; i < model->nu; ++i)
      if (!std::isfinite(data->actuator_force[i])) return false;
    return std::isfinite(data->time);
  }

  void finish(std::int64_t instant) noexcept {
    final.samples = samples;
    final.read_count = read_count;
    final.write_count = write_count;
    final.startup_writes = startup_writes;
    final.warnings = warnings();
    final.first_read_ns = first_read_ns;
    final.finish_ns = instant;
    final.receiver_zero_ns = receiver.zero_ns();
    final.interruption_ns = interruption_ns;
    final.runtime_s = (instant - first_read_ns) * 1e-9;
    final.sim_time_s = plant->data()->time;
    final.max_closure_m = max_closure_m;
    for (int axis = 0; axis < 2; ++axis) {
      final.q[axis] = plant->data()->qpos[q_pos[axis]];
      final.dq[axis] = plant->data()->qvel[q_vel[axis]];
    }
    final.receiver_latched = receiver.latched();
    final.receiver_fault = receiver.fault();
    final.runtime_error = runtime_error;
    complete.store(true, std::memory_order_release);
  }

  void write_files() {
    std::ofstream csv;
    open_output(csv, output_dir / "trace.csv");
    csv << "index,sim_time_s,cycle_start_ns,feedback_ns,application_ns,finish_ns,period_arg_ms,"
           "read_count,write_count,sequence,generation,source_fresh,command_accepted,receiver_latched,"
           "psi,beta,dpsi,dbeta,effort104,effort105,actual_force104,actual_force105,closure_m,"
           "motor104_position,motor105_position,motor104_velocity,motor105_velocity,"
           "command_generation104,command_generation105\n";
    for (std::size_t i = 0; i < final.samples; ++i) {
      const auto& row = rows[i];
      csv << row.index << ',' << row.sim_time_s << ',' << row.cycle_start_ns << ','
          << row.feedback_ns << ',' << row.application_ns << ',' << row.finish_ns << ','
          << row.period_arg_ms << ',' << row.read_count << ',' << row.write_count << ','
          << row.sequence << ',' << row.generation << ',' << row.source_fresh << ','
          << row.command_accepted << ',' << row.receiver_latched << ','
          << row.q[0] << ',' << row.q[1] << ',' << row.dq[0] << ',' << row.dq[1] << ','
          << row.effort[0] << ',' << row.effort[1] << ',' << row.actual_force[0] << ','
          << row.actual_force[1] << ',' << row.closure_m << ','
          << row.motor_position[0] << ',' << row.motor_position[1] << ','
          << row.motor_velocity[0] << ',' << row.motor_velocity[1] << ','
          << row.command_generation[0] << ',' << row.command_generation[1] << '\n';
    }
    csv.close();
    std::ofstream meta;
    open_output(meta, output_dir / "metadata.json");
    meta << "{\n  \"schema\": \"ref01_mujoco_system_v1\",\n"
         << "  \"simulation_only\": true,\n  \"scheduler\": \"controller_manager\",\n"
         << "  \"scenario\": \"" << scenario << "\",\n"
         << "  \"mujoco_version\": \"" << mj_versionString() << "\",\n"
         << "  \"duration_s\": " << duration_s << ",\n"
         << "  \"target_samples\": " << target_samples << ",\n"
         << "  \"max_samples\": " << max_samples << ",\n"
         << "  \"samples\": " << final.samples << ",\n"
         << "  \"read_count\": " << final.read_count << ",\n"
         << "  \"write_count\": " << final.write_count << ",\n"
         << "  \"startup_writes\": " << final.startup_writes << ",\n"
         << "  \"nominal_period_s\": 0.005,\n  \"physics_step_s\": 0.0005,\n"
         << "  \"physics_steps_per_cycle\": 10,\n  \"receiver_ttl_ns\": 60000000,\n"
         << "  \"first_read_ns\": " << final.first_read_ns << ",\n"
         << "  \"finish_ns\": " << final.finish_ns << ",\n"
         << "  \"runtime_s\": " << final.runtime_s << ",\n"
         << "  \"sim_time_s\": " << json_number(final.sim_time_s) << ",\n"
         << "  \"receiver_latched\": " << (final.receiver_latched ? "true" : "false") << ",\n"
         << "  \"receiver_fault\": \"" << Ref01SimReceiver::fault_name(final.receiver_fault) << "\",\n"
         << "  \"receiver_zero_ns\": " << final.receiver_zero_ns << ",\n"
         << "  \"interruption_ns\": " << final.interruption_ns << ",\n"
         << "  \"warnings\": " << final.warnings << ",\n"
         << "  \"max_closure_m\": " << json_number(final.max_closure_m) << ",\n"
         << "  \"final_q\": [" << json_number(final.q[0]) << ',' << json_number(final.q[1]) << "],\n"
         << "  \"final_dq\": [" << json_number(final.dq[0]) << ',' << json_number(final.dq[1]) << "],\n"
         << "  \"runtime_error\": \"" << final.runtime_error << "\",\n"
         << "  \"snapshot_boundary\": \"last measured write; subsequent TTL handling excluded\",\n"
         << "  \"q_timing\": \"before physics step; final_q is after final physics step\",\n"
         << "  \"force_closure_timing\": \"after physics step\"\n}\n";
    meta.close();
    std::ofstream marker;
    open_output(marker, output_dir / "complete.json.tmp");
    marker << "{\"complete\":true,\"samples\":" << final.samples
           << ",\"runtime_error\":\"" << final.runtime_error << "\"}\n";
    marker.close();
    std::filesystem::rename(output_dir / "complete.json.tmp", output_dir / "complete.json");
  }

  void writer_loop() noexcept {
    for (;;) {
      // Complete takes precedence over cancellation so a normal stop cannot
      // discard an already-frozen run. No live model/receiver state is read here.
      if (complete.load(std::memory_order_acquire)) {
        try { write_files(); }
        catch (const std::exception& error) {
          try {
            std::ofstream failure(output_dir / "writer_error.txt");
            failure << error.what() << '\n';
          } catch (...) {}
        }
        return;
      }
      if (cancel_writer.load(std::memory_order_acquire)) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

Ref01MujocoSystem::Ref01MujocoSystem() : impl_(std::make_unique<Impl>()) {}
Ref01MujocoSystem::~Ref01MujocoSystem() = default;

hardware_interface::CallbackReturn Ref01MujocoSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  using Return = hardware_interface::CallbackReturn;
  if (SystemInterface::on_init(info) != Return::SUCCESS) return Return::ERROR;
  try {
    const auto& parameters = info.hardware_parameters;
    if (parameters.at("simulation_only") != "true")
      throw std::runtime_error("simulation_only=true must be explicit");
    if (info.joints.size() != 2) throw std::runtime_error("exactly two motor joints required");
    for (int index = 0; index < 2; ++index) {
      const auto found = std::find_if(info.joints.begin(), info.joints.end(),
          [index](const auto& joint) { return joint.name == kJoints[index]; });
      if (found == info.joints.end() || !exact_interfaces(found->state_interfaces, kStates) ||
          !exact_interfaces(found->command_interfaces, kCommands))
        throw std::runtime_error("unexpected joint interface set");
    }
    impl_->model_path = parameters.at("model_path");
    impl_->output_dir = parameters.at("output_dir");
    impl_->scenario = parameters.at("scenario");
    if (impl_->scenario != "psi" && impl_->scenario != "beta" &&
        impl_->scenario != "loss" && impl_->scenario != "duplicate" && impl_->scenario != "reference")
      throw std::runtime_error("unsupported scenario");
    impl_->duration_s = strict_double(parameters.at("duration_s"));
    if (impl_->duration_s < 2.5 || impl_->duration_s > 120.)
      throw std::runtime_error("duration_s must be 2.5..120");
    const double max_samples = strict_double(parameters.at("max_samples"));
    if (max_samples < 1 || max_samples > kMaximumSamples || std::floor(max_samples) != max_samples)
      throw std::runtime_error("max_samples must be an integer <=24001");
    impl_->max_samples = static_cast<std::size_t>(max_samples);
    impl_->target_samples = static_cast<std::size_t>(std::llround(impl_->duration_s / kPeriod)) + 1;
    if (impl_->target_samples > impl_->max_samples)
      throw std::runtime_error("max_samples is smaller than requested duration");
    if (!std::filesystem::path(impl_->model_path).is_absolute() || !impl_->output_dir.is_absolute())
      throw std::runtime_error("model_path and output_dir must be absolute");
    return Return::SUCCESS;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("Ref01MujocoSystem"), "init: %s", error.what());
    return Return::ERROR;
  }
}

hardware_interface::CallbackReturn Ref01MujocoSystem::on_configure(
    const rclcpp_lifecycle::State&) {
  using Return = hardware_interface::CallbackReturn;
  try {
    auto& impl = *impl_;
    if (impl.activated_once) throw std::runtime_error("single-run hardware must be re-created");
    impl.plant = std::make_unique<ph_exo::Ref01Model>(impl.model_path);
    auto* model = impl.plant->model();
    auto* data = impl.plant->data();
    const char* alpha[2] = {"alpha1", "alpha2"};
    const char* q[2] = {"psi", "beta"};
    const char* actuators[2] = {"alpha1_torque", "alpha2_torque"};
    const char* rods[2] = {"rod_end1", "rod_end2"};
    const char* targets[2] = {"Q1", "Q2"};
    for (int index = 0; index < 2; ++index) {
      const int alpha_id = mj_name2id(model, mjOBJ_JOINT, alpha[index]);
      const int q_id = mj_name2id(model, mjOBJ_JOINT, q[index]);
      impl.actuator[index] = mj_name2id(model, mjOBJ_ACTUATOR, actuators[index]);
      impl.rod[index] = mj_name2id(model, mjOBJ_SITE, rods[index]);
      impl.target[index] = mj_name2id(model, mjOBJ_SITE, targets[index]);
      if (alpha_id < 0 || q_id < 0 || impl.actuator[index] < 0 ||
          impl.rod[index] < 0 || impl.target[index] < 0)
        throw std::runtime_error("REF01 model element missing");
      impl.alpha_pos[index] = model->jnt_qposadr[alpha_id];
      impl.alpha_vel[index] = model->jnt_dofadr[alpha_id];
      impl.q_pos[index] = model->jnt_qposadr[q_id];
      impl.q_vel[index] = model->jnt_dofadr[q_id];
    }
    if (std::abs(model->opt.timestep - .0005) > 1e-12 ||
        std::abs(model->qpos0[impl.q_pos[0]]) > 1e-8 ||
        std::abs(model->qpos0[impl.q_pos[1]] - kPi / 9.) > 1e-8)
      throw std::runtime_error("expected 0.5ms model and home psi=0,beta=20deg");
    for (auto path = impl.output_dir; !path.empty(); path = path.parent_path()) {
      if (std::filesystem::is_symlink(path)) throw std::runtime_error("output_dir must not traverse symlink");
      if (path == path.root_path()) break;
    }
    std::filesystem::create_directories(impl.output_dir);
    for (const char* name : {"trace.csv", "metadata.json", "complete.json", "complete.json.tmp", "writer_error.txt"})
      if (std::filesystem::exists(impl.output_dir / name)) throw std::runtime_error("output files already exist");
    impl.rows.resize(impl.max_samples);  // All bounded sample storage is allocated here.
    impl.plant->reset();
    mj_forward(model, data);
    impl.feedback(now_ns());
    impl.configured = true;
    return Return::SUCCESS;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("Ref01MujocoSystem"), "configure: %s", error.what());
    return Return::ERROR;
  }
}

hardware_interface::CallbackReturn Ref01MujocoSystem::on_activate(
    const rclcpp_lifecycle::State&) {
  using Return = hardware_interface::CallbackReturn;
  auto& impl = *impl_;
  if (!impl.configured || impl.activated_once) return Return::ERROR;
  try {
    impl.activated_once = true;
    impl.command.fill(0.);
    impl.receiver.reset();
    impl.feedback(now_ns());
    impl.cancel_writer.store(false, std::memory_order_release);
    impl.writer = std::thread([&impl]() { impl.writer_loop(); });
    impl.callback_gate.store(0, std::memory_order_release);
    return Return::SUCCESS;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("Ref01MujocoSystem"), "activate: %s", error.what());
    return Return::ERROR;
  }
}

hardware_interface::CallbackReturn Ref01MujocoSystem::on_deactivate(const rclcpp_lifecycle::State&) {
  impl_->stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Ref01MujocoSystem::on_cleanup(const rclcpp_lifecycle::State&) {
  impl_->stop();
  impl_->configured = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn Ref01MujocoSystem::on_shutdown(const rclcpp_lifecycle::State&) {
  impl_->stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Ref01MujocoSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(8);
  for (int joint = 0; joint < 2; ++joint)
    for (int item = 0; item < 4; ++item)
      interfaces.emplace_back(kJoints[joint], kStates[item], &impl_->state[4 * joint + item]);
  return interfaces;
}
std::vector<hardware_interface::CommandInterface> Ref01MujocoSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(4);
  for (int joint = 0; joint < 2; ++joint)
    for (int item = 0; item < 2; ++item)
      interfaces.emplace_back(kJoints[joint], kCommands[item], &impl_->command[2 * joint + item]);
  return interfaces;
}

hardware_interface::return_type Ref01MujocoSystem::read(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  auto& impl = *impl_;
  Impl::Visit visit(impl);
  if (!visit) return hardware_interface::return_type::OK;
  impl.cycle_start_ns = now_ns();
  impl.period_arg_ms = period.seconds() * 1000.;
  ++impl.read_count;
  if (impl.first_read_ns == 0) impl.first_read_ns = impl.cycle_start_ns;
  impl.source_fresh = false;
  // Keep source frozen after completion. Continued controller_manager cycles
  // then exercise stale feedback and the independent receiver TTL off-record.
  if (impl.complete.load(std::memory_order_relaxed)) return hardware_interface::return_type::OK;
  const double sim_time = static_cast<double>(impl.samples) * kPeriod;
  const bool interrupted = (impl.scenario == "loss" || impl.scenario == "duplicate") &&
                            sim_time >= 1.2 && sim_time < 1.65;
  if (interrupted) {
    if (impl.interruption_ns == 0) impl.interruption_ns = impl.cycle_start_ns;
    if (impl.scenario == "duplicate") impl.state = impl.duplicate_state;
  } else {
    impl.feedback(now_ns());
    impl.source_fresh = true;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ref01MujocoSystem::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  auto& impl = *impl_;
  Impl::Visit visit(impl);
  if (!visit) return hardware_interface::return_type::OK;
  ++impl.write_count;
  const bool accepted = impl.receiver.accept(impl.command, now_ns());
  impl.set_effort();
  const auto application_ns = accepted ? now_ns() : 0;
  if (impl.complete.load(std::memory_order_relaxed)) return hardware_interface::return_type::OK;
  if (!impl.receiver.started()) {
    ++impl.startup_writes;
    if (impl.receiver.latched() || impl.startup_writes >= kStartupLimit) {
      impl.runtime_error = impl.receiver.latched() ? "startup_receiver_fault" : "startup_timeout";
      impl.receiver.fail(Ref01SimReceiver::Fault::Runtime, now_ns());
      impl.set_effort();
      impl.finish(now_ns());
    }
    return hardware_interface::return_type::OK;
  }
  auto& row = impl.rows[impl.samples];
  row.index = impl.samples;
  row.sim_time_s = static_cast<double>(impl.samples) * kPeriod;
  row.cycle_start_ns = impl.cycle_start_ns;
  row.feedback_ns = impl.feedback_ns;
  row.application_ns = application_ns;
  row.period_arg_ms = impl.period_arg_ms;
  row.read_count = impl.read_count;
  row.write_count = impl.write_count;
  row.sequence = impl.sequence;
  row.generation = impl.receiver.generation();
  row.source_fresh = impl.source_fresh;
  row.command_accepted = accepted;
  row.receiver_latched = impl.receiver.latched();
  row.effort = impl.receiver.effort();
  auto* model = impl.plant->model();
  auto* data = impl.plant->data();
  for (int axis = 0; axis < 2; ++axis) {
    row.q[axis] = data->qpos[impl.q_pos[axis]];
    row.dq[axis] = data->qvel[impl.q_vel[axis]];
    row.motor_position[axis] = impl.state[4 * axis];
    row.motor_velocity[axis] = impl.state[4 * axis + 1];
    row.command_generation[axis] = impl.command[2 * axis + 1];
  }
  std::fill_n(data->qfrc_applied, model->nv, 0.);
  if (impl.scenario != "reference" && row.sim_time_s >= .4 && row.sim_time_s < 1.)
    data->qfrc_applied[impl.q_vel[impl.scenario == "beta" ? 1 : 0]] = .09;
  for (int step = 0; step < 10; ++step) mj_step(model, data);
  mj_forward(model, data);
  row.actual_force = {data->actuator_force[impl.actuator[1]], data->actuator_force[impl.actuator[0]]};
  row.closure_m = impl.closure();
  impl.max_closure_m = std::max(impl.max_closure_m, row.closure_m);
  const bool plant_failed = !impl.finite_plant() || impl.warnings() != 0;
  if (plant_failed) {
    impl.runtime_error = "plant_nonfinite_or_warning";
    impl.receiver.fail(Ref01SimReceiver::Fault::Runtime, now_ns());
    impl.set_effort();
  }
  row.finish_ns = now_ns();
  ++impl.samples;
  if (impl.samples >= impl.target_samples || plant_failed) impl.finish(row.finish_ns);
  return hardware_interface::return_type::OK;
}

}  // namespace mech::mech_simulation

PLUGINLIB_EXPORT_CLASS(mech::mech_simulation::Ref01MujocoSystem, hardware_interface::SystemInterface)
