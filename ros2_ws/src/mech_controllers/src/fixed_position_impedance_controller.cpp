#include "mech_controllers/fixed_position_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

namespace mech::mech_controllers {

FixedPositionImpedanceController::FixedPositionImpedanceController()
    : SingleJointCommandController("effort", "~/enable") {}

controller_interface::CallbackReturn FixedPositionImpedanceController::on_init() {
  if (SingleJointCommandController::on_init() != controller_interface::CallbackReturn::SUCCESS)
    return controller_interface::CallbackReturn::ERROR;
  try {
    auto_declare<std::string>("command_source", "enable");
    auto_declare<double>("stiffness_nm_per_rad", 15.0);
    auto_declare<double>("damping", 0.5);
    auto_declare<double>("torque_limit_nm", 7.0);
    auto_declare<double>("position_min_rad", 0.0);
    auto_declare<double>("position_max_rad", 0.0);
    auto_declare<double>("position_margin_rad", 0.02);
    auto_declare<double>("activation_velocity_rad_s", 0.05);
    auto_declare<double>("max_velocity_rad_s", 1.0);
    auto_declare<double>("max_update_period_s", 0.006);
    auto_declare<double>("run_limit_s", 5.0);
    auto_declare<bool>("allow_continuous_operation", false);
  } catch (...) { return controller_interface::CallbackReturn::ERROR; }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FixedPositionImpedanceController::on_configure(
    const rclcpp_lifecycle::State& state) {
  try {
    const auto number = [this](const char* key) { return get_node()->get_parameter(key).as_double(); };
    const auto source = get_node()->get_parameter("command_source").as_string();
    if (source != "enable" && source != "training_command")
      return controller_interface::CallbackReturn::ERROR;
    structured_ = source == "training_command";
    stiffness_ = number("stiffness_nm_per_rad"); damping_ = number("damping");
    torque_limit_ = number("torque_limit_nm");
    lower_ = number("position_min_rad"); upper_ = number("position_max_rad");
    margin_ = number("position_margin_rad");
    activation_speed_ = number("activation_velocity_rad_s");
    max_speed_ = number("max_velocity_rad_s");
    max_period_ = number("max_update_period_s"); run_limit_ = number("run_limit_s");
    continuous_ = get_node()->get_parameter("allow_continuous_operation").as_bool();
  } catch (...) { return controller_interface::CallbackReturn::ERROR; }
  for (const double value : {stiffness_, damping_, torque_limit_, lower_, upper_, margin_,
                            activation_speed_, max_speed_, max_period_, run_limit_}) {
    if (!std::isfinite(value)) return controller_interface::CallbackReturn::ERROR;
  }
  if (stiffness_ <= 0.0 || damping_ < 0.0 || torque_limit_ <= 0.0 ||
      lower_ >= upper_ || margin_ <= 0.0 || 2.0 * margin_ >= upper_ - lower_ ||
      activation_speed_ <= 0.0 || activation_speed_ >= max_speed_ ||
      max_period_ <= 0.0 || max_period_ > 0.006 ||
      (continuous_ ? run_limit_ != 0.0 : (run_limit_ <= 0.0 || run_limit_ > 120.0)))
    return controller_interface::CallbackReturn::ERROR;
  const auto result = SingleJointCommandController::on_configure(state);
  training_subscription_.reset();
  if (result == controller_interface::CallbackReturn::SUCCESS && structured_) {
    training_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/training_command", rclcpp::QoS(1),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
          if (!message || message->data.size() != 8U ||
              !message->layout.dim.empty() || message->layout.data_offset != 0U) return;
          std::array<double, 8> frame{};
          std::copy(message->data.begin(), message->data.end(), frame.begin());
          (void)set_training_command(frame);
        });
  }
  return result;
}

controller_interface::InterfaceConfiguration
FixedPositionImpedanceController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name() + "/position", joint_name() + "/velocity"}};
}

bool FixedPositionImpedanceController::initial_command(double& value) const noexcept {
  value = 0.0;
  if (state_interfaces_.size() != 2U ||
      state_interfaces_[0].get_name() != joint_name() + "/position" ||
      state_interfaces_[1].get_name() != joint_name() + "/velocity") return false;
  const double q = state_interfaces_[0].get_value(), v = state_interfaces_[1].get_value();
  return std::isfinite(q) && std::isfinite(v) &&
      lower_ + margin_ <= q && q <= upper_ - margin_ && std::abs(v) <= activation_speed_;
}

controller_interface::CallbackReturn FixedPositionImpedanceController::on_activate(
    const rclcpp_lifecycle::State& state) {
  previous_time_ = started_ = -1; faulted_ = false;
  parameters_locked_ = false;
  const auto result = SingleJointCommandController::on_activate(state);
  if (result == controller_interface::CallbackReturn::SUCCESS)
    anchor_ = state_interfaces_[0].get_value();
  return result;
}

controller_interface::return_type FixedPositionImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto result = SingleJointCommandController::update(time, period);
  if (result != controller_interface::return_type::OK) faulted_ = true;
  return result;
}

bool FixedPositionImpedanceController::valid_limits(const BoundedTarget& limits) const noexcept {
  return limits.minimum == 0.0 && limits.maximum == 1.0 &&
      limits.max_slew_per_second >= 1000000.0;
}

bool FixedPositionImpedanceController::valid_target(double value) const noexcept {
  return value == 0.0 || value == 1.0;
}

bool FixedPositionImpedanceController::set_training_command(
    const std::array<double, 8>& frame) noexcept {
  if (!structured_) return false;
  TargetCommand command;
  command.value = frame[3];
  command.parameters = {frame[0], frame[1], frame[2], frame[4], frame[5], frame[6], frame[7]};
  return set_command(command);
}

bool FixedPositionImpedanceController::valid_command(const TargetCommand& command) const noexcept {
  if (!valid_target(command.value)) return false;
  if (!structured_) return true;
  const auto& p = command.parameters;
  for (double value : p) if (!std::isfinite(value)) return false;
  const double arrival = static_cast<double>(command.arrival_nanoseconds) * 1e-9;
  // Producer runs on this host's monotonic clock; no remote-clock inference.
  if (p[0] != 1.0 || p[1] <= source_sequence_ || p[1] > 9007199254740991.0 ||
      std::floor(p[1]) != p[1] || p[2] < source_time_ || p[2] > arrival ||
      p[2] < static_cast<double>(activation_time()) * 1e-9 || arrival - p[2] > .1 ||
      p[3] < lower_ + margin_ || p[3] > upper_ - margin_ ||
      p[4] <= 0.0 || p[4] > stiffness_ || p[5] < 0.0 || p[5] > damping_ ||
      p[6] <= 0.0 || p[6] > torque_limit_) return false;
  source_sequence_ = p[1]; source_time_ = p[2];
  return true;
}

bool FixedPositionImpedanceController::compute_command(double permission, double dt,
    std::int64_t now, WatchdogStage stage, double& output) noexcept {
  if (faulted_) return false;
  if (!std::isfinite(dt) || dt <= 0.0 || dt > max_period_ || now < 0 ||
      (previous_time_ >= 0 && (now <= previous_time_ ||
       static_cast<double>(now - previous_time_) * 1e-9 > max_period_))) return false;
  previous_time_ = now;
  const double q = state_interfaces_[0].get_value(), v = state_interfaces_[1].get_value();
  if (!std::isfinite(q) || !std::isfinite(v) || q < lower_ || q > upper_ ||
      std::abs(v) > max_speed_) return false;
  // Permission gaps/zero cannot renew this activation's total run budget.
  if (!continuous_ && started_ >= 0 && static_cast<double>(now - started_) * 1e-9 >= run_limit_)
    return false;
  output = 0.0;
  if (stage != WatchdogStage::Following || permission != 1.0) return true;
  if (started_ < 0) started_ = now;
  double reference = anchor_, stiffness = stiffness_, damping = damping_, cap = torque_limit_;
  if (structured_) {
    const auto& p = command_parameters();
    if (static_cast<double>(now) * 1e-9 - p[2] >= .1) return true;
    const std::array<double, 4> parameters{p[3], p[4], p[5], p[6]};
    // A fixed-position training session cannot move its spring anchor or gains
    // silently. Change the plan only after controller deactivation/reactivation.
    if (parameters_locked_ && parameters != locked_parameters_) return false;
    if (!parameters_locked_ && std::abs(p[3] - q) > .01) return false;
    locked_parameters_ = parameters; parameters_locked_ = true;
    reference = p[3]; stiffness = p[4]; damping = p[5]; cap = p[6];
  }
  // CalculateHipTorque_Compliant: zero desired velocity and zero feedforward.
  // No integrator, direction qualification, speed filter or torque slew.
  const double torque = stiffness * (reference - q) - damping * v;
  if (!std::isfinite(torque)) return false;
  output = std::clamp(torque, -cap, cap);
  // Additional directional boundary envelope, separate from the STM32 kernel.
  const double distance = output > 0.0 ? upper_ - q : q - lower_;
  output = std::copysign(std::min(std::abs(output), cap *
      std::clamp(distance / margin_, 0.0, 1.0)), output);
  return true;
}

}  // namespace mech::mech_controllers
