#include "mech_controllers/velocity_command_controller.hpp"

namespace mech::mech_controllers {

VelocityCommandController::VelocityCommandController()
    : SingleJointCommandController("velocity", "~/target_velocity") {}

controller_interface::InterfaceConfiguration
VelocityCommandController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

bool VelocityCommandController::initial_command(double& value) const noexcept {
  // An internal ramp origin, not a target. Generation does not advance until
  // a fresh explicit target arrives. No position state is needed or trusted.
  value = 0.0;
  return state_interfaces_.empty();
}

bool VelocityCommandController::valid_limits(const BoundedTarget& limits) const noexcept {
  // A fresh zero target must always be representable for deliberate stopping.
  return limits.minimum <= 0.0 && limits.maximum >= 0.0;
}

}  // namespace mech::mech_controllers
