#pragma once

#include "mech_controllers/single_joint_command_controller.hpp"

namespace mech::mech_controllers {

// Vendor-independent encoder damping. ~/enable accepts only 0 or 1, NOT torque.
// No publishers, allocations or blocking locks are used in the update hook.
class ResistanceTrainingController final : public SingleJointCommandController {
 public:
  ResistanceTrainingController();
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& state) override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

 private:
  bool initial_command(double& value) const noexcept override;
  bool valid_limits(const BoundedTarget& limits) const noexcept override;
  bool valid_target(double value) const noexcept override;
  bool compute_command(double permission, double dt, std::int64_t now,
                       WatchdogStage stage, double& output) noexcept override;
  void clear_motion() noexcept;

  double damping_{0.0}, torque_limit_{0.1}, buildup_{0.5}, tau_{0.025};
  double stop_speed_{0.02}, start_speed_{0.06}, max_speed_{1.0};
  double direction_confirmation_{0.02}, direction_elapsed_{0.0};
  int candidate_direction_{0};
  double lower_{0.0}, upper_{0.0}, margin_{0.02};
  double max_period_{0.006}, run_limit_{5.0};
  double filtered_{0.0}, effort_{0.0};
  std::int64_t previous_time_{-1}, started_{-1};
  bool moving_{false}, faulted_{false};
};

}  // namespace mech::mech_controllers
