# Knee control modes

`knee_control_node` is the preferred application entry point. It selects
exactly one strategy at startup; `knee_motor_follow_node` remains an alias for
existing bench scripts. Both names run the same ROS node, whose graph name
remains `knee_motor_follow` for compatibility. Do not launch multiple
motor-output instances against one controller topic: startup mode selection
does not provide a distributed lock across ROS processes.

| Strategy | Input | Output | Selected mode |
| --- | --- | --- | --- |
| `velocity_follow` | `imu` | `preview` | Velocity preview (default) |
| `velocity_follow` | `imu` | `motor` | Mapped motor velocity |
| `velocity_follow` | `motor_feedback_simulation` | `motor` | Unloaded periodic bench |
| `resistance` | `imu` | `preview` | Resistance torque preview |
| `resistance` | `imu` | `motor` | Mapped DM4340 effort, bench-only |

All other combinations fail during startup. Runtime parameter changes are
rejected; restart with a different combination. The shared
`/knee_control/arm` and `/knee_control/reset` services apply to the selected
mode. The legacy `/knee_motor_follow/*` services remain available for velocity
modes so the existing ROS bench entry is unchanged.

## Knee resistance

Select `control_strategy:=resistance` on `knee_control_node` for preview-only
resistance training. It reads `knee/joint_state` and `knee/diagnostics` and publishes
`knee_resistance/preview_effort_nm` and `knee_resistance/diagnostics`. Preview
does not publish to a motor controller or enable hardware. Arm it with
`ros2 service call /knee_control/arm std_srvs/srv/Trigger '{}'`; reset a
latched fault with `/knee_control/reset` after correcting the input.

```bash
ros2 run mech_assist_control knee_control_node --ros-args \
  -p control_strategy:=resistance
```

The virtual knee torque is `-sign(omega) * min(Tmax, B *
max(|omega| - deadband, 0))`. It is dissipative, unlike the STM32 AO+PD
reference follower, which can assist motion. Defaults (`B=0.5 Nm s/rad`,
`Tmax=0.3 Nm`, ten-second trial) are preview fixtures, not clinically validated
training settings. A fresh knee sample and healthy diagnostic are required;
stale/invalid input, out-of-range motion and expiry latch the output to zero.

Explicit `output_mode:=motor` instead publishes to
`/dm4340_effort_controller/target_effort`. It requires
`joint_mapping_verified:=true`, a nonzero measured `motor_to_knee_scale`,
the measured offset, fresh motor JointState feedback, and agreement between
mapped shaft motion and the knee estimate. Stale/disagreeing data latch a
fault, send one zero, then cease target refresh. The motor feedback and
effort command must use the same ROS joint coordinates. For `knee_angle = motor_to_knee_scale * motor_angle + offset`, the
commanded motor effort is `motor_to_knee_scale * knee_resistance_effort` by
virtual work. This handles reversed mounting and transmission ratio; the
published effort is capped at `resistance_motor_max_effort_nm` (default 0.3 Nm,
matching the current effort controller). If the mapped effort would assist
the measured motor motion, the node publishes zero for that tick. The
mapping and effort cap are configuration values, not measured defaults. The
DM4340 hardware configuration must select effort mode; velocity and effort controllers
must never claim the same device concurrently. Neither this node nor preview
starts the hardware lifecycle. Zero effort is not a brake, and no worn
exoskeleton resistance trial is justified without mounted, validated IMUs,
physical joint mapping and a separately verified stop procedure.

This package owns the optional application-level link from the calibrated
dual-IMU knee estimate to the existing DM4340 ros2_control velocity controller.
It does not open CAN, activate hardware, enable the motor, or change the
driver's deployment gates. `mech_knee_kinematics` remains receive-only.

`knee_motor_follow_node` subscribes to `knee/joint_state` and
`knee/diagnostics`. The default `output_mode=preview` publishes only
`knee_motor_follow/preview_velocity`. It uses the offline-tested 0.25 Hz
reference, started from the knee angle at arm, with configurable amplitude
(default 20 deg). Angle error, reference rate and knee rate produce a velocity
target; knee-side and motor-side bounds are each 1.5 rad/s and 1 rad/s^2.
This reference and these gains are experimental, not validated assistance
parameters. There is no foot-pressure gate while insoles are disconnected.

Only `control_strategy=velocity_follow` with `output_mode=motor` publishes on
`/dm4340_velocity_controller/target_velocity` (`std_msgs/Float64`, rad/s).
This velocity mode also requires `joint_mapping_verified=true`, a nonzero measured
`motor_to_knee_scale`, and the corresponding offset in radians. It subscribes
to `/joint_states`, selects `motor_joint_name`, and checks that the mapped
motor position agrees with the fresh knee estimate within 0.15 rad. The
scale converts motor position/velocity to knee position/velocity; it is **not**
automatically the DM4340's internal gearbox ratio. Obtain this mapping from
the actual assembly before selecting motor mode.

For an **unloaded shaft-only** periodic trial without IMUs, set
`output_mode=motor` and `input_mode=motor_feedback_simulation`. This separate
mode takes motor JointState position at startup as a virtual zero, treats shaft
motion relative to it as a simulated knee angle, and never subscribes to IMU
topics. It does not assert an anatomical joint mapping or sensor accuracy.
The default limits are 0.5 rad relative shaft travel and five seconds. An
explicit unloaded-bench profile may configure at most 45 deg, 15 seconds,
and 1.0 rad; the IMU-driven mode retains its original bounds. The
physical driver still requires its own verified bringup settings, activation
and stop procedure; this node does not set or bypass them. This mode must not
be used on a worn exoskeleton.

After the existing DM4340 hardware and velocity controller have been
independently activated, the bench-only application node can be started with:

```bash
ros2 run mech_assist_control knee_motor_follow_node --ros-args \
  -p output_mode:=motor -p input_mode:=motor_feedback_simulation
ros2 service call /knee_motor_follow/arm std_srvs/srv/Trigger '{}'
```

This is not a combined startup script and does not activate or disable the
motor. Stop/deactivate the existing controller and hardware using the
established bench procedure. Never set `*_verified` bringup parameters solely
to make this command start.

The IMU-driven modes require healthy knee diagnostics and fresh knee samples before
`/knee_motor_follow/arm` succeeds. Motor mode additionally requires fresh
motor feedback. Shaft-only simulation synthesizes its knee input from fresh
motor feedback and does not require IMU diagnostics. Invalid, repeated or stale IMU/knee data, unhealthy
diagnostics, stale/invalid motor feedback or position mismatch latch a fault.
The node publishes one zero target and stops refreshing, allowing the existing
controller's command TTL and hardware fault handling to take over. Call
`/knee_motor_follow/reset` and then arm again only after new valid data. Motor
mode defaults to a five-second fault limit; the explicit shaft-only bench
profile may use a longer bound. This is a bounded integration
trial, not continuous wearable assistance. A
zero target or ROS shutdown alone is **not** a physical motor stop guarantee.

For a non-actuating inspection in a sourced ROS 2 workspace:

```bash
ros2 run mech_assist_control knee_motor_follow_node
ros2 topic echo /knee_motor_follow/diagnostics
ros2 topic echo /knee_motor_follow/preview_velocity
```

The motor mode is a code path for staged bench integration, not a ready-to-run
wearable launch. It must run with an independently checked motor-to-joint
mapping, both IMUs fixed and physically calibrated, verified motor-side stop
behavior, and the existing `mech_bringup` gates. This package never sets those
gates or activates `dm4340_system`; do not treat simulated ROS messages as
deployment evidence.
