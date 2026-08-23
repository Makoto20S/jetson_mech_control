#include "mech_bringup/foundation_harness.hpp"

#include <chrono>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mech::mech_bringup {
namespace {

hardware_interface::InterfaceInfo interface(const std::string& name) {
  hardware_interface::InterfaceInfo value;
  value.name = name;
  value.size = 1;
  return value;
}

rclcpp_lifecycle::State state() {
  return rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "foundation_harness");
}

}  // namespace

bool FoundationHarness::configure(std::size_t joint_count) noexcept {
  if (joint_count == 0U || configured_) return false;
  hardware_interface::HardwareInfo info;
  info.name = "foundation_system";
  info.type = "system";
  info.hardware_class_type = "mech_hardware_ros2_control/CompositeSystem";
  for (std::size_t index = 0U; index < joint_count; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(index + 1U);
    joint.type = "joint";
    joint.command_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
    joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                              interface(hardware_interface::HW_IF_VELOCITY),
                              interface(hardware_interface::HW_IF_EFFORT)};
    info.joints.push_back(std::move(joint));
  }
  if (hardware_.on_init(info) != hardware_interface::CallbackReturn::SUCCESS ||
      hardware_.on_configure(state()) !=
          hardware_interface::CallbackReturn::SUCCESS ||
      !limiter_.configure(mech_controllers::BoundedTarget{-1.0, 1.0, 2.0,
                                                           100000000})) {
    return false;
  }
  state_interfaces_ = hardware_.export_state_interfaces();
  command_interfaces_ = hardware_.export_command_interfaces();
  command_name_ = command_interfaces_.front().get_name();
  configured_ = true;
  return true;
}

bool FoundationHarness::activate() noexcept {
  if (!configured_ || active_ || hardware_.on_activate(state()) !=
                                         hardware_interface::CallbackReturn::SUCCESS) {
    return false;
  }
  active_ = true;
  return true;
}

bool FoundationHarness::switch_claim(bool claim) noexcept {
  if (!active_ || claim == claimed_) return false;
  const std::vector<std::string> start = claim ? std::vector<std::string>{command_name_}
                                                : std::vector<std::string>{};
  const std::vector<std::string> stop = claim ? std::vector<std::string>{}
                                               : std::vector<std::string>{command_name_};
  if (hardware_.prepare_command_mode_switch(start, stop) !=
          hardware_interface::return_type::OK ||
      hardware_.perform_command_mode_switch(start, stop) !=
          hardware_interface::return_type::OK) {
    return false;
  }
  claimed_ = claim;
  return true;
}

bool FoundationHarness::set_target(double target,
                                   std::int64_t now_nanoseconds) noexcept {
  return claimed_ && limiter_.submit(target, now_nanoseconds);
}

bool FoundationHarness::cycle(std::int64_t now_nanoseconds,
                              std::int64_t period_nanoseconds) noexcept {
  if (!active_ || !claimed_ || period_nanoseconds <= 0) return false;
  const auto started = std::chrono::steady_clock::now();
  const auto previous = command_interfaces_[0].get_value();
  const auto next = limiter_.update(
      previous, static_cast<double>(period_nanoseconds) / 1000000000.0,
      now_nanoseconds);
  command_interfaces_[0].set_value(next);
  const rclcpp::Time time(now_nanoseconds);
  const rclcpp::Duration period{std::chrono::nanoseconds(period_nanoseconds)};
  const auto wrote = hardware_.write(time, period);
  const auto read = hardware_.read(time, period);
  const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now() - started).count();
  ++metrics_.cycles;
  metrics_.maximum_cycle_nanoseconds =
      std::max(metrics_.maximum_cycle_nanoseconds, elapsed);
  if (wrote != hardware_interface::return_type::OK ||
      read != hardware_interface::return_type::OK) {
    ++metrics_.failures;
    return false;
  }
  return true;
}

bool FoundationHarness::deactivate() noexcept {
  if (!active_ || hardware_.on_deactivate(state()) !=
                      hardware_interface::CallbackReturn::SUCCESS) {
    return false;
  }
  active_ = false;
  claimed_ = false;
  limiter_.clear();
  return true;
}

bool FoundationHarness::cleanup() noexcept {
  if (active_ || !configured_ || hardware_.on_cleanup(state()) !=
                                      hardware_interface::CallbackReturn::SUCCESS) {
    return false;
  }
  configured_ = false;
  state_interfaces_.clear();
  command_interfaces_.clear();
  return true;
}

double FoundationHarness::position() const noexcept {
  return state_interfaces_.empty() ? 0.0 : state_interfaces_[0].get_value();
}

double FoundationHarness::command() const noexcept {
  return command_interfaces_.empty() ? 0.0 : command_interfaces_[0].get_value();
}

}  // namespace mech::mech_bringup
