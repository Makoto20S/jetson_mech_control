#include "mech_controllers/position_command_controller.hpp"

namespace mech::mech_controllers {

PositionCommandController::PositionCommandController()
    : SingleJointCommandController("position", "~/target_position") {}

controller_interface::InterfaceConfiguration
PositionCommandController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name() + "/position"}};
}

bool PositionCommandController::initial_command(double& value) const noexcept {
  if (state_interfaces_.size() != 1U ||
      state_interfaces_[0].get_name() != joint_name() + "/position") return false;
  // Preserve position activation: hold the measured position, never invent zero.
  // The shared lifecycle also rejects a non-finite seed.
  value = state_interfaces_[0].get_value();
  return true;
}

}  // namespace mech::mech_controllers
