#include "mech_controllers/effort_command_controller.hpp"

namespace mech::mech_controllers {

EffortCommandController::EffortCommandController()
    : SingleJointCommandController("effort", "~/target_effort") {}

controller_interface::InterfaceConfiguration
EffortCommandController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::NONE, {}};
}

bool EffortCommandController::initial_command(double& value) const noexcept {
  // An internal ramp origin, not a target. Generation does not advance until
  // a fresh explicit target arrives. No position state is needed or trusted.
  value = 0.0;
  return state_interfaces_.empty();
}

bool EffortCommandController::valid_limits(const BoundedTarget& limits) const noexcept {
  // A fresh zero target must remove commanded torque; it does not brake motion.
  return limits.minimum <= 0.0 && limits.maximum >= 0.0;
}

}  // namespace mech::mech_controllers
