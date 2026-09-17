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
There is no runtime mode switch. Full impedance control remains future work.

## EffortCommandController (T9)

The independent plugin `mech_controllers/EffortCommandController` claims `effort`
and `command_generation`, accepts `std_msgs/msg/Float64` on `~/target_effort`
in N*m, and claims no state interfaces. It uses the same bounded target pipeline
as Position and Velocity. Bounds must contain zero; `max_slew_per_second` is
the commanded effort slew in N*m/s. Activation seeds the internal ramp at zero
and requires a fresh target before refreshing generation.

An explicit fresh zero target ramps commanded effort to zero. **Zero effort is
not a stop or brake**: a free shaft may coast. Observe actual rest separately
before deactivation. Target expiry and deactivation stop refreshing commands;
they do not synthesize braking torque. Reactivation discards previous targets
and restarts the ramp from zero. Hardware feedback validity still gates claims.

The motor1 example uses `motor1_effort_controllers.yaml` and
`motor1_effort_bringup.launch.py`, loads inactive, and bounds effort to +/-0.1 N*m
with 0.2 N*m/s slew. In AK3.0 Torque mode the effort feedback is an Iq-derived
equivalent estimate; standard position/velocity fields are unsupported. The
vendor runtime's raw ERPM guard is separate from this generic controller and
must be checked alongside fresh feedback in the bench procedure.
