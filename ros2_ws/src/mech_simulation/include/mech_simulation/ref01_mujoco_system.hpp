#pragma once

#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace mech::mech_simulation {

// Software-only ros2_control hardware. No serial, CAN, socket, or motor runtime.
class Ref01MujocoSystem final : public hardware_interface::SystemInterface {
 public:
  Ref01MujocoSystem();
  ~Ref01MujocoSystem() override;
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                      const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mech::mech_simulation
