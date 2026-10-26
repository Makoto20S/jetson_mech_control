#pragma once

#include "mech_controllers/single_joint_command_controller.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mech::mech_controllers {

// Host-side STM32 compliant PD kernel. A configured source selects either
// an atomic upper plan or a leased scalar permission with an activation anchor.
class FixedPositionImpedanceController final : public SingleJointCommandController {
 public:
  FixedPositionImpedanceController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // Atomic upper-layer reference/gain/permission contract (schema 1).
  [[nodiscard]] bool set_training_command(const std::array<double, 8>& frame) noexcept;

 private:
  bool needs_activation_time() const noexcept override { return structured_; }
  bool scalar_subscription() const noexcept override { return !structured_; }
  bool valid_command(const TargetCommand&) const noexcept override;
  bool initial_command(double&) const noexcept override;
  bool valid_limits(const BoundedTarget&) const noexcept override;
  bool valid_target(double) const noexcept override;
  bool compute_command(double, double, std::int64_t, WatchdogStage, double&) noexcept override;

  double stiffness_{15.0}, damping_{0.5}, torque_limit_{7.0};
  double lower_{0.0}, upper_{0.0}, margin_{0.02}, anchor_{0.0};
  double activation_speed_{0.05}, max_speed_{1.0}, max_period_{0.006}, run_limit_{5.0};
  std::int64_t previous_time_{-1}, started_{-1};
  bool continuous_{false}, faulted_{false}, structured_{false}, parameters_locked_{false};
  std::array<double, 4> locked_parameters_{};
  mutable double source_sequence_{0.0}, source_time_{0.0};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr training_subscription_;
};

}  // namespace mech::mech_controllers
