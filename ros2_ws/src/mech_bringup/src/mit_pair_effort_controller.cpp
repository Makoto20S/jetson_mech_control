#include "mech_bringup/mit_pair_guard.hpp"

#include <atomic>
#include <chrono>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mech::mech_bringup {
namespace {
using Callback = controller_interface::CallbackReturn;
using Config = controller_interface::InterfaceConfiguration;
using Trajectory = trajectory_msgs::msg::JointTrajectory;
using Trigger = std_srvs::srv::Trigger;
const std::array<std::string, 2> joints{"mit_comm_motor104", "mit_comm_motor105"};
constexpr std::array<const char *, 7> fields{"mit_drive_id", "mit_rx_us", "mit_valid",
    "mit_fault_code", "mit_erpm", "mit_iq_a", "mit_temperature_c"};

std::int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class MitPairEffortController final : public controller_interface::ControllerInterface {
 public:
  Callback on_init() override {
    try {
      for (auto name : {"max_effort_nm", "max_slew_nm_s", "max_abs_erpm", "max_abs_iq_a",
                        "max_temperature_c"}) auto_declare<double>(name, 0.);
      auto_declare<int64_t>("target_ttl_ns", 100000000);
      auto_declare<int64_t>("feedback_ttl_ns", 60000000);
      auto_declare<int64_t>("max_pair_skew_ns", 30000000);
    } catch (const std::exception &) { return Callback::ERROR; }
    return Callback::SUCCESS;
  }

  Config command_interface_configuration() const override {
    Config result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (const auto & joint : joints) {
      result.names.push_back(joint + "/effort");
      result.names.push_back(joint + "/command_generation");
    }
    return result;
  }

  Config state_interface_configuration() const override {
    Config result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
    for (const auto & joint : joints) {
      for (auto field : fields) result.names.push_back(joint + "/" + field);
    }
    return result;
  }

  Callback on_configure(const rclcpp_lifecycle::State &) override {
    auto node = get_node();
    auto value = [&](const char * key) { return node->get_parameter(key).as_double(); };
    auto ns = [&](const char * key) { return node->get_parameter(key).as_int(); };
    limits_ = {value("max_effort_nm"), value("max_slew_nm_s"), value("max_abs_erpm"),
               value("max_abs_iq_a"), value("max_temperature_c"), ns("target_ttl_ns"),
               ns("feedback_ttl_ns"), ns("max_pair_skew_ns")};
    if (!limits_.valid() || node->get_parameter("use_sim_time").as_bool()) return Callback::ERROR;
    sub_ = node->create_subscription<Trajectory>("~/target", rclcpp::QoS(1).durability_volatile(),
      [this](const Trajectory::SharedPtr msg) {
        // Default mutually-exclusive callback group serializes subscription writers.
        const auto epoch = epoch_.load();
        if (!epoch || msg->joint_names.size() != 2 || msg->points.size() != 1 ||
            !std::equal(joints.begin(), joints.end(), msg->joint_names.begin()) ||
            msg->header.stamp.sec <= 0 || msg->header.stamp.nanosec >= 1000000000U) return;
        const auto & p = msg->points.front();
        if (p.effort.size() != 2 || !p.positions.empty() || !p.velocities.empty() ||
            !p.accelerations.empty() || p.time_from_start.sec || p.time_from_start.nanosec ||
            !std::isfinite(p.effort[0]) || !std::isfinite(p.effort[1])) return;
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        const auto received = steady_ns();
        const auto age = get_node()->now().nanoseconds() - stamp;
        if (age < 0 || age >= limits_.target_ttl_ns || received - age <= epoch ||
            stamp <= last_stamp_ || epoch_.load() != epoch) return;
        last_stamp_ = stamp;
        inbox_.writeFromNonRT(MitPairTarget{received - age, {p.effort[0], p.effort[1]}});
      });
    stop_ = node->create_service<Trigger>("~/stop", [this](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      const auto epoch = epoch_.load();
      stop_epoch_.store(epoch);
      response->success = epoch != 0;
      response->message = epoch ? "stop_requested" : "inactive";
    });
    status_ = node->create_service<Trigger>("~/status", [this](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
      const auto mode = mode_.load();
      response->success = epoch_.load() && mode == PairMode::Following;
      response->message = epoch_.load() ? pair_mode_name(mode) : "inactive";
    });
    return Callback::SUCCESS;
  }

  Callback on_activate(const rclcpp_lifecycle::State &) override {
    const auto commands = command_interface_configuration().names;
    const auto states = state_interface_configuration().names;
    if (command_interfaces_.size() != commands.size() ||
        state_interfaces_.size() != states.size()) return Callback::ERROR;
    for (unsigned i = 0; i < commands.size(); ++i)
      if (command_interfaces_[i].get_name() != commands[i]) return Callback::ERROR;
    for (unsigned i = 0; i < states.size(); ++i)
      if (state_interfaces_[i].get_name() != states[i]) return Callback::ERROR;
    const auto now = steady_ns();
    guard_.activate(limits_, now);
    for (unsigned i = 0; i < 2; ++i) {
      const double previous = command_interfaces_[2*i+1].get_value();
      generation_[i] = std::isfinite(previous) ? previous : 0.;
    }
    mode_.store(PairMode::Waiting);
    epoch_.store(now);
    return Callback::SUCCESS;
  }

  Callback on_deactivate(const rclcpp_lifecycle::State &) override {
    epoch_.store(0);
    return Callback::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration & period) override {
    std::array<MitPairAxis, 2> axes;
    for (unsigned i = 0; i < axes.size(); ++i) {
      auto read = [&](unsigned k) { return state_interfaces_[i*fields.size()+k].get_value(); };
      axes[i] = {read(0), read(1), read(2), read(3), read(4), read(5), read(6)};
    }
    const auto epoch = epoch_.load();
    const auto action = (epoch && stop_epoch_.load() == epoch) ? guard_.request_stop() :
        guard_.update(*inbox_.readFromRT(), axes, steady_ns(), period.seconds());
    mode_.store(guard_.mode());
    if (action.refresh) {
      for (unsigned i = 0; i < axes.size(); ++i) {
        command_interfaces_[2*i].set_value(action.effort[i]);
        command_interfaces_[2*i+1].set_value(++generation_[i]);
      }
    }
    return controller_interface::return_type::OK;
  }

 private:
  MitPairLimits limits_{};
  MitPairGuard guard_;
  std::array<double, 2> generation_{};
  std::atomic<std::int64_t> epoch_{0};
  std::atomic<PairMode> mode_{PairMode::Waiting};
  std::int64_t last_stamp_{};
  realtime_tools::RealtimeBuffer<MitPairTarget> inbox_{MitPairTarget{}};
  rclcpp::Subscription<Trajectory>::SharedPtr sub_;
  std::atomic<std::int64_t> stop_epoch_{0};
  rclcpp::Service<Trigger>::SharedPtr stop_;
  rclcpp::Service<Trigger>::SharedPtr status_;
};
}  // namespace mech::mech_bringup

PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::MitPairEffortController,
                       controller_interface::ControllerInterface)
