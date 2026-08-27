#include "mech_controllers/demo_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mech::mech_controllers {

bool TargetLimiter::configure(BoundedTarget limits) noexcept {
  if (!std::isfinite(limits.minimum) || !std::isfinite(limits.maximum) ||
      !std::isfinite(limits.max_slew_per_second) ||
      limits.minimum > limits.maximum || limits.max_slew_per_second <= 0.0 ||
      limits.ttl_nanoseconds <= 0 || limits.hard_ttl_nanoseconds <= 0 ||
      limits.hard_ttl_nanoseconds <= limits.ttl_nanoseconds) {
    return false;
  }
  limits_ = limits;
  target_ = 0.0;
  held_ = 0.0;
  deadline_ = 0;
  configured_ = true;
  has_target_ = false;
  has_held_ = false;
  return true;
}

bool TargetLimiter::submit(double target,
                           std::int64_t now_nanoseconds) noexcept {
  if (!configured_ || !std::isfinite(target) || now_nanoseconds < 0 ||
      now_nanoseconds > std::numeric_limits<std::int64_t>::max() -
                            limits_.hard_ttl_nanoseconds) {
    return false;
  }
  target_ = std::clamp(target, limits_.minimum, limits_.maximum);
  deadline_ = now_nanoseconds + limits_.ttl_nanoseconds;
  has_target_ = true;
  return true;
}

double TargetLimiter::update(double previous, double period_seconds,
                             std::int64_t now_nanoseconds) noexcept {
  if (!configured_ || !std::isfinite(previous) || !std::isfinite(period_seconds) ||
      period_seconds < 0.0) {
    // Invalid input: never invent a value. Hold the last value we know to be
    // valid (seeded lazily from `previous` the first time it is finite) so we
    // never command a silent jump to zero.
    if (!has_held_ && std::isfinite(previous)) {
      held_ = previous;
      has_held_ = true;
    }
    return held_;
  }
  const auto current_stage = stage(now_nanoseconds);
  double result = previous;
  if (current_stage == WatchdogStage::Following) {
    const auto maximum_step = limits_.max_slew_per_second * period_seconds;
    result = previous + std::clamp(target_ - previous, -maximum_step, maximum_step);
  } else {
    // Holding or Expired: freeze at the last valid commanded value rather
    // than continuing to slew toward the (now stale) target.
    result = has_held_ ? held_ : previous;
  }
  held_ = result;
  has_held_ = true;
  return result;
}

void TargetLimiter::clear() noexcept {
  target_ = 0.0;
  held_ = 0.0;
  deadline_ = 0;
  has_target_ = false;
  has_held_ = false;
}

bool TargetLimiter::expired(std::int64_t now_nanoseconds) const noexcept {
  return stage(now_nanoseconds) != WatchdogStage::Following;
}

WatchdogStage TargetLimiter::stage(std::int64_t now_nanoseconds) const noexcept {
  // No target has ever been submitted: there is nothing legitimate to follow,
  // so never invent motion toward the default target_ of 0.0. Treat this the
  // same as a target that went stale at time zero (Holding, then Expired once
  // the hard TTL measured from zero elapses).
  if (!has_target_ || now_nanoseconds < 0) {
    const auto now = std::max<std::int64_t>(now_nanoseconds, 0);
    return now < limits_.hard_ttl_nanoseconds ? WatchdogStage::Holding
                                              : WatchdogStage::Expired;
  }
  if (now_nanoseconds < deadline_) return WatchdogStage::Following;
  const auto hard_deadline =
      deadline_ + (limits_.hard_ttl_nanoseconds - limits_.ttl_nanoseconds);
  if (now_nanoseconds < hard_deadline) return WatchdogStage::Holding;
  return WatchdogStage::Expired;
}

controller_interface::CallbackReturn DemoController::on_init() {
  try {
    auto_declare<std::string>("joint", "joint_1");
    auto_declare<double>("minimum", -1.0);
    auto_declare<double>("maximum", 1.0);
    auto_declare<double>("max_slew_per_second", 1.0);
    auto_declare<int64_t>("ttl_nanoseconds", 4000000);
    auto_declare<int64_t>("hard_ttl_nanoseconds", 6000000);
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_configure(
    const rclcpp_lifecycle::State&) {
  try {
    joint_name_ = get_node()->get_parameter("joint").as_string();
    limits_.minimum = get_node()->get_parameter("minimum").as_double();
    limits_.maximum = get_node()->get_parameter("maximum").as_double();
    limits_.max_slew_per_second =
        get_node()->get_parameter("max_slew_per_second").as_double();
    limits_.ttl_nanoseconds = get_node()->get_parameter("ttl_nanoseconds").as_int();
    limits_.hard_ttl_nanoseconds =
        get_node()->get_parameter("hard_ttl_nanoseconds").as_int();
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (joint_name_.empty() || !limiter_.configure(limits_)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  command_ = 0.0;
  active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_activate(
    const rclcpp_lifecycle::State&) {
  if (command_interfaces_.size() != 1U || state_interfaces_.size() != 1U ||
      command_interfaces_[0].get_name() !=
          joint_name_ + "/" + hardware_interface::HW_IF_POSITION ||
      state_interfaces_[0].get_name() !=
          joint_name_ + "/" + hardware_interface::HW_IF_POSITION) {
    return controller_interface::CallbackReturn::ERROR;
  }
  command_ = state_interfaces_[0].get_value();
  limiter_.clear();
  active_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  limiter_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DemoController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/" + hardware_interface::HW_IF_POSITION}};
}

controller_interface::InterfaceConfiguration
DemoController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/" + hardware_interface::HW_IF_POSITION}};
}

controller_interface::return_type DemoController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (!active_ || command_interfaces_.size() != 1U ||
      time.nanoseconds() < 0 || period.nanoseconds() < 0) {
    return controller_interface::return_type::ERROR;
  }
  if (limiter_.stage(time.nanoseconds()) == WatchdogStage::Expired) {
    return controller_interface::return_type::ERROR;
  }
  command_ = limiter_.update(command_, period.seconds(), time.nanoseconds());
  command_interfaces_[0].set_value(command_);
  return controller_interface::return_type::OK;
}

bool DemoController::set_target(double target,
                                std::int64_t now_nanoseconds) noexcept {
  return active_ && limiter_.submit(target, now_nanoseconds);
}

}  // namespace mech::mech_controllers

PLUGINLIB_EXPORT_CLASS(mech::mech_controllers::DemoController,
                       controller_interface::ControllerInterface)
