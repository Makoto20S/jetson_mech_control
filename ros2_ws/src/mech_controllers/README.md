# mech_controllers

Bounded C++ controller package boundary. Controllers do not open CAN sockets
or branch on vendor model names.

## PositionCommandController

The plugin `mech_controllers/PositionCommandController` shapes a single joint's
position command with bounds, slew limits and a monotonic target watchdog.
It claims `position` and `command_generation`, reads `position`, and accepts
`std_msgs/msg/Float64` targets on `~/target_position`. It does not calculate
the motor's impedance torque or configure its gains.

The motor1 instance remains `motor1_position_controller`, so its target topic
remains `/motor1_position_controller/target_position`. Target lifetime defaults
remain 100/106 ms; the independent hardware lease remains 4/6 ms (ADR-017/018).
Activation seeds the held position from finite feedback and requires a new
target; deactivation does not substitute a zero command.

### Migration from DemoController

Change plugin types from `mech_controllers/DemoController` to
`mech_controllers/PositionCommandController`. C++ consumers must include
`mech_controllers/position_command_controller.hpp` and use
`mech::mech_controllers::PositionCommandController`. Rebuild dependent packages
and install into a fresh prefix so obsolete headers and plugin metadata cannot
mask an incomplete migration. The old class/header/plugin name has no alias.
The linkable `mech_controllers` library and separate `mech_controllers_plugin`
registration library retain their names and separation.

Update operator scripts that check the controller type together with the
framework deployment. Keep existing controller instance names, target topics,
parameters and gain settings. The rename preserves position behavior; it does
not authorize another hardware run or certify physical accuracy or timing.

VelocityCommandController and EffortCommandController remain separate future
plugins. Runtime mode switching and a full impedance controller are outside
this position-controller milestone.
