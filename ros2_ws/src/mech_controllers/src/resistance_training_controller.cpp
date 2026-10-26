#include "mech_controllers/resistance_training_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mech::mech_controllers {

ResistanceTrainingController::ResistanceTrainingController()
    : SingleJointCommandController("effort", "~/enable") {}

controller_interface::CallbackReturn ResistanceTrainingController::on_init() {
  if (SingleJointCommandController::on_init() != controller_interface::CallbackReturn::SUCCESS)
    return controller_interface::CallbackReturn::ERROR;
  try {
    auto_declare<double>("damping", 0.5);
    auto_declare<double>("torque_limit_nm", 0.1);
    auto_declare<double>("buildup_nm_per_second", 0.5);
    auto_declare<double>("velocity_filter_tau_s", 0.025);
    auto_declare<double>("stop_velocity_rad_s", 0.02);
    auto_declare<double>("start_velocity_rad_s", 0.06);
    auto_declare<double>("direction_confirmation_s", 0.02);
    auto_declare<double>("max_velocity_rad_s", 1.0);
    // No invented mechanical limits: an unconfigured range is invalid.
    auto_declare<double>("position_min_rad", 0.0);
    auto_declare<double>("position_max_rad", 0.0);
    auto_declare<double>("position_margin_rad", 0.02);
    auto_declare<double>("max_update_period_s", 0.006);
    auto_declare<double>("run_limit_s", 5.0);
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ResistanceTrainingController::on_configure(
    const rclcpp_lifecycle::State& state) {
  try {
    auto number = [this](const char* key) { return get_node()->get_parameter(key).as_double(); };
    damping_ = number("damping");
    torque_limit_ = number("torque_limit_nm");
    buildup_ = number("buildup_nm_per_second");
    tau_ = number("velocity_filter_tau_s");
    stop_speed_ = number("stop_velocity_rad_s");
    start_speed_ = number("start_velocity_rad_s");
    direction_confirmation_ = number("direction_confirmation_s");
    max_speed_ = number("max_velocity_rad_s");
    lower_ = number("position_min_rad"); upper_ = number("position_max_rad");
    margin_ = number("position_margin_rad");
    max_period_ = number("max_update_period_s"); run_limit_ = number("run_limit_s");
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (const auto value : {damping_, torque_limit_, buildup_, tau_, stop_speed_,
       start_speed_, direction_confirmation_, max_speed_, lower_, upper_, margin_, max_period_, run_limit_}) {
    if (!std::isfinite(value)) return controller_interface::CallbackReturn::ERROR;
  }
  if (damping_ < 0.0 || torque_limit_ <= 0.0 || buildup_ <= 0.0 || tau_ <= 0.0 ||
      stop_speed_ < 0.0 || start_speed_ < stop_speed_ || start_speed_ >= max_speed_ ||
      direction_confirmation_ < 0.0 || direction_confirmation_ > 0.1 ||
      lower_ >= upper_ || margin_ <= 0.0 || 2.0 * margin_ >= upper_ - lower_ ||
      max_period_ <= 0.0 || max_period_ > 0.006 || run_limit_ <= 0.0 || run_limit_ > 120.0)
    return controller_interface::CallbackReturn::ERROR;
  return SingleJointCommandController::on_configure(state);
}

controller_interface::InterfaceConfiguration
ResistanceTrainingController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name() + "/position", joint_name() + "/velocity"}};
}

controller_interface::CallbackReturn ResistanceTrainingController::on_activate(
    const rclcpp_lifecycle::State& state) {
  clear_motion(); previous_time_ = -1; faulted_ = false;
  return SingleJointCommandController::on_activate(state);
}

controller_interface::return_type ResistanceTrainingController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto result = SingleJointCommandController::update(time, period);
  if (result != controller_interface::return_type::OK) faulted_ = true;
  return result;
}

bool ResistanceTrainingController::valid_limits(const BoundedTarget& limits) const noexcept {
  // The inherited limiter gates permission, not torque; never slew an enable.
  return limits.minimum == 0.0 && limits.maximum == 1.0 &&
         limits.max_slew_per_second >= 1000000.0;
}

bool ResistanceTrainingController::valid_target(double value) const noexcept {
  return value == 0.0 || value == 1.0;
}

bool ResistanceTrainingController::initial_command(double& value) const noexcept {
  if (state_interfaces_.size() != 2U ||
      state_interfaces_[0].get_name() != joint_name() + "/position" ||
      state_interfaces_[1].get_name() != joint_name() + "/velocity") return false;
  const auto q = state_interfaces_[0].get_value();
  const auto v = state_interfaces_[1].get_value();
  // Lifecycle activation is quiescent; it never grants upstream permission.
  value = 0.0;
  return std::isfinite(q) && std::isfinite(v) && lower_ <= q && q <= upper_ &&
         std::abs(v) < start_speed_;
}

void ResistanceTrainingController::clear_motion() noexcept {
  filtered_ = effort_ = 0.0;
  moving_ = false;
  candidate_direction_ = 0; direction_elapsed_ = 0.0;
  started_ = -1;
}

bool ResistanceTrainingController::compute_command(double permission, double dt,
    std::int64_t now, WatchdogStage stage, double& output) noexcept {
  // A fault cannot be cleared by resuming an upstream heartbeat.
  if (faulted_) return false;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > max_period_ || now < 0 ||
      (previous_time_ >= 0 && (now <= previous_time_ ||
          static_cast<double>(now - previous_time_) * 1e-9 > max_period_))) {
    faulted_ = true;
    return false;
  }
  previous_time_ = now;
  const double q = state_interfaces_[0].get_value(), v = state_interfaces_[1].get_value();
  if (!std::isfinite(q) || !std::isfinite(v) || q < lower_ || q > upper_ ||
      std::abs(v) > max_speed_) {
    faulted_ = true;
    return false;
  }
  if (stage != WatchdogStage::Following) {
    // A short permission gap must not restart the monotonic run budget.
    filtered_ = effort_ = 0.0; moving_ = false; output = 0.0;
    candidate_direction_ = 0; direction_elapsed_ = 0.0;
    return true;
  }
  if (permission != 1.0) {
    clear_motion(); output = 0.0;
    return true;
  }
  if (started_ < 0) started_ = now;
  if (static_cast<double>(now - started_) * 1e-9 >= run_limit_) {
    faulted_ = true;
    return false;
  }
  // Keep the signed observer running through short stops. Resetting it at every
  // raw zero crossing repeatedly turned damping into startup force pulses.
  filtered_ += -std::expm1(-dt / tau_) * (v - filtered_);
  const double speed = std::abs(v);
  const int direction = v > 0.0 ? 1 : -1;
  if (speed <= stop_speed_) {
    effort_ = 0.0; moving_ = false;
    candidate_direction_ = 0; direction_elapsed_ = 0.0;
  } else {
    const bool reversal = candidate_direction_ != 0 && direction != candidate_direction_;
    if (direction != candidate_direction_) {
      candidate_direction_ = direction; direction_elapsed_ = dt;
      effort_ = 0.0; moving_ = false;
    } else {
      direction_elapsed_ = std::min(direction_confirmation_, direction_elapsed_ + dt);
    }
    // A real reversal always gets a zero-output sample, even with confirmation
    // explicitly set to zero. Stale signed filter values must never assist it.
    if (reversal || direction_elapsed_ + 1e-12 < direction_confirmation_ ||
        filtered_ * v <= 0.0 || (!moving_ && speed < start_speed_)) {
      effort_ = 0.0; output = 0.0;
      return true;
    }
    moving_ = true;
    // Continuous low-speed fade, and no lagging observer torque larger than the
    // current raw viscous bound. Safety rest/reversal overrides still unload now.
    const double fraction = start_speed_ > stop_speed_
        ? std::clamp((speed - stop_speed_) / (start_speed_ - stop_speed_), 0.0, 1.0)
        : 1.0;
    const double fade = fraction * fraction * (3.0 - 2.0 * fraction);
    const double target = std::min(torque_limit_,
        damping_ * std::min(std::abs(filtered_), speed) * fade);
    effort_ = -std::copysign(std::min(target, std::abs(effort_) + buildup_ * dt), v);
    // Directional soft command cap; absolute feedback bounds above remain hard.
    const double distance = effort_ > 0.0 ? upper_ - q : q - lower_;
    effort_ = std::copysign(std::min(std::abs(effort_), torque_limit_ *
        std::clamp(distance / margin_, 0.0, 1.0)), effort_);
  }
  output = effort_;
  return true;
}

}  // namespace mech::mech_controllers
