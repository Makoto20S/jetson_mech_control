#pragma once

#include "mech_controllers/single_joint_command_controller.hpp"

namespace mech::mech_controllers {

class VelocityCommandController final : public SingleJointCommandController {
 public:
  VelocityCommandController();
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

 private:
  [[nodiscard]] bool initial_command(double& value) const noexcept override;
  [[nodiscard]] bool valid_limits(const BoundedTarget& limits) const noexcept override;
};

}  // namespace mech::mech_controllers
