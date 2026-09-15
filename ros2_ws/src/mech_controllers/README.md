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

## VelocityCommandController (T8)

The independent plugin `mech_controllers/VelocityCommandController` claims
`velocity` and `command_generation` and accepts `std_msgs/msg/Float64` on
`~/target_velocity` in rad/s. It claims no state interfaces. The hardware's
feedback validity gate still applies before activation.

`minimum`/`maximum` bound velocity and must include zero. `max_slew_per_second`
bounds the commanded acceleration in rad/s². These are command shaping limits,
not measured acceleration or overspeed protection. Activation starts the internal
ramp at zero, discards previous targets and requires a fresh explicit target
before refreshing generation. It does not measure or assume a stationary shaft;
the operator must confirm rest before activation/recovery.

Position and Velocity share the concrete `SingleJointCommandController` target
mailbox, limiter, monotonic clock, activation epoch and generation pipeline.
Position retains measured-position seeding; Velocity never reads position.
Both use the ADR-018 upstream policy (100/106 ms by default) independently of
the hardware's 4/6 ms lease. Same-value messages refresh upstream lifetime;
NaN/Inf and inactive submissions do not. Messages are dated when their callback
is consumed; Float64 has no source timestamp, so this does not bound upstream
network/queue age. Use a continuously running publisher and executor.

For normal stopping, continuously publish fresh zero velocity targets until the
command ramp reaches zero **and actual speed is confirmed near zero**, then
deactivate. Stale input freezes command and generation at the soft deadline and
returns ERROR at the hard deadline; deactivation stops generation refresh.
These fault paths do not synthesize zero velocity or guarantee mechanical braking.
Recovery requires reactivation and new targets; stale targets are not replayed.

The motor1 example is `motor1_velocity_controllers.yaml`, paired with the
velocity URDF and `motor1_velocity_bringup.launch.py` in `mech_bringup`. It loads
the controller inactive, with ±0.5 rad/s bounds and 0.5 rad/s² acceleration.
There is no runtime mode switch. EffortCommandController (T9) and full impedance
control remain future work.
