# mech_bringup

Deployment composition boundary: bring-up harnesses, device probes, and the
AK3.0 force-control runtime wiring. The `Ak30System` plugin
(`mech_bringup/Ak30System`) is the production composition point that connects
`mech_protocol_cubemars` to `mech_hardware_ros2_control`'s `CompositeSystem`;
all three deployment xacro examples load it.

## Components

- **`Ak30ForceControlRuntime`** — the `RuntimePort` implementation that wires
  the AK3.0 force-control session into `CompositeSystem`. It is the first
  production consumer of `command_stage()` (ADR-012's staged watchdog):
  fresh controller writes are submitted while Following, the last valid
  command is frozen through Holding, and Expired faults the system within
  the 3-cycle budget. It never resolves a missing command to `0.0`. Since
  [ADR-014](../../../docs/adr/ADR-014-ak30-submode-command-interfaces.md)
  it maps commands per sub-mode (Position reads position, Velocity reads
  velocity with effort forced to 0, Torque reads effort).
- **`Ak30System`** — the production composition point as a pluginlib
  plugin (`mech_bringup/Ak30System`): it injects the serial→transport→runtime
  chain (`PosixCdcSerialPort` -> `UsbCdcTransport` ->
  `Ak30ForceControlRuntime`) into `CompositeSystem`, and `on_configure` is
  the first device I/O (transport open plus the 0x12 pass-through init). The
  three deployment xacro examples point at it.
- **`Ak30RuntimeParams`** — fail-closed parsing of the URDF `ros2_control`
  hardware parameters into the runtime config. `device_path` is mandatory;
  unknown keys, non-numeric values, and over-budget TTLs reject configure.
  `sub_mode` is an explicit parameter (`position`|`velocity`|`torque`,
  default `position`); the URDF's command interface must match it.
- **`FoundationHarness`** — the Foundation-era simulation harness
  (CompositeSystem + TargetLimiter through exported interfaces).
- **Device probes** (`ak30_torque_probe`, `ak30_position_probe`) — the
  bench-validated bring-up probes, built only with
  `-DMECH_BUILD_DEVICE_PROBES=ON`.
- **Deployment examples** (`config/motor1.urdf.xacro` for Position,
  `config/motor1_torque.urdf.xacro` for Torque,
  `config/motor1_velocity.urdf.xacro` for Velocity,
  `config/motor1_controllers.yaml`, `launch/motor1_bringup.launch.py`) —
  single-motor examples carrying motor1's bench-evidenced parameters,
  one per force-control sub-mode. The position-controller spawner ships
  commented out: uncommenting it arms position commands, which stay
  gated by ADR-006 and per-test owner authorization.

## Position deployment envelope (ADR-019)

`config/motor1.urdf.xacro` requires three hardware parameters in Position
sub-mode: `position_min_rad` and `position_max_rad` (motor1 example: `-12.0`
and `6.0`) bound where the shaft may be commanded to go, and
`position_max_error_rad` (motor1 example: `0.5`) bounds how far a command may
sit from the latest usable feedback position. The second one is a force limit,
not a travel limit: in Position sub-mode the applied torque is
`Kp * (command - measured)`, so with motor1's `Kp = 1 N*m/rad` a `0.5 rad`
error bound is a `0.5 N*m` ceiling on the torque any controller can request.
For scale, static friction on the unloaded shaft is about `0.2 N*m`. Missing,
non-finite, `min >= max` or non-positive values reject the hardware at
initialization (`on_init`), before any device I/O. The other sub-modes do not
require these parameters, but the set is all-or-nothing everywhere: once any
one of the three appears, all three must be present and valid or `on_init`
rejects the hardware.

The check runs in the AK3.0 runtime before submission and never clamps. The
absolute bounds are inclusive and are checked on every Position command; the
error bound is evaluated only on cycles where the feedback sample was judged
usable, so a command is never compared against a stale or absent number. A
violation drops the command — no frame carrying that value is transmitted —
latches the runtime, emits the `PositionEnvelope` telemetry reason, and makes
`read()` fail every cycle until `configure()`/`start()` runs. Claim
cancellation does not clear the latch. Because the gate sits in the shared
hardware layer, it binds **every** controller that claims the joint, including
upstream `ros2_control` controllers that the project did not write; that is the
point of it. The post-latch behaviour is the same one the T9 overspeed latch
showed (see [T9 effort deployment](#t9-effort-deployment)): ros2_control moves
the hardware into its error state and rejects STRICT deactivation there, so
bench tooling must take post-latch rest evidence from a fresh passive
observation. See
[ADR-019](../../../docs/adr/ADR-019-hardware-position-envelope.md); its status
is Proposed and no bench evidence for the envelope exists yet.

## T8 velocity deployment

Use the separate `launch/motor1_velocity_bringup.launch.py` with
`config/motor1_velocity.urdf.xacro` and `config/motor1_velocity_controllers.yaml`.
It serializes the broadcaster and velocity spawners and loads
`motor1_velocity_controller` **inactive**. Launching still opens the physical
transport; inactive does not make this an offline launch. The offline test
evaluates the actual xacro/launch parameters without starting any nodes.

The controller consumes `/motor1_velocity_controller/target_velocity`
(`std_msgs/msg/Float64`, rad/s); its example bounds are ±0.5 rad/s, with a
0.5 rad/s² command ramp. The URDF uses Kp=0, Kd=1, and the runtime forces t_ff=0.
For an approved future bench run: stop the other deployment, verify current
feedback and shaft rest, activate using a STRICT switch, verify ACTIVE, then
send fresh zero targets before any bounded nonzero ramp. Keep publishing at
20–50 Hz. This configuration does not switch the deployed position operator.

Normal stopping requires continued fresh zero targets through the ramp and
confirmation of actual near-zero speed before deactivation. Target loss or
deactivation stops refresh/transmission; it does not promise mechanical braking.
Velocity mode supplies evidenced velocity/effort feedback, **not position**;
the position state placeholder must not be used to infer displacement or rest.
The launch therefore omits robot_state_publisher. Initialization is independent
of that position state; hardware feedback validity is still required to claim.

See [controller semantics](../mech_controllers/README.md) for target lifetime,
activation and recovery. Real velocity-controller acceptance remains a separate
authorized bench task; offline frame mapping does not establish physical limits.

## T9 effort deployment

`launch/motor1_effort_bringup.launch.py` pairs `motor1_torque.urdf.xacro` with
`motor1_effort_controllers.yaml`, serializes the spawners and loads the effort
controller inactive. It opens the physical transport when executed; constructing
the launch description in an offline test does not. The fixed Torque profile
uses Kp=Kd=0 and only the effort field. No hot mode switching is supported.

Targets arrive on `/motor1_effort_controller/target_effort` as Float64 in N*m.
Example limits are +/-0.1 N*m and 0.2 N*m/s, with upstream TTL100/106 ms and
hardware lease4/6 ms. Normal zero return must continue publishing through the
ramp and verify coast-down using fresh raw ERPM; neither zero effort nor
deactivation guarantees a mechanical stop. Physical force accuracy needs
independent metrology; the exported effort is estimated from Iq and Kt.

Torque deployments require an explicit finite positive `torque_max_abs_erpm`
hardware parameter (motor1 example:300). The AK3.0 runtime checks raw signed
ERPM from accepted feedback before submitting pending torque. Exceeding the
absolute limit latches a host fault and stops submission, including when a
later sample in the same drain is below the limit. Claim cancellation cannot
clear that fault. Feedback loss remains subject to the60 ms validity window.
This guard is sample-based and stops sending; it does not brake, guarantee a
physical speed bound or replace local emergency power removal.

Optional feedback telemetry includes `raw_erpm_available` and `raw_erpm`, tied
to accepted RX sequence/time. These are device diagnostics, not a promotion of
Torque mode's unsupported canonical velocity. Tools must reject missing/stale
diagnostics and never use placeholder velocity/position as rest evidence.

Bench acceptance on motor1 (2026-09-17, unloaded shaft, Kp=Kd=0): a zero-effort
1 s trial completed the full activate/zero/rest/deactivate cycle, and a
+0.2 N*m trial with a 5000 ERPM ceiling echoed 0.21-0.22 N*m while static
friction held the shaft, then accelerated at roughly 28 rad/s^2 after breakaway
and latched the overspeed guard within 0.2 s. Pure torque with no damping and
no load has no steady speed, so any raw ceiling below the motor's own limit
ends such a run; a sustained visible spin needs a damping term or a load.

Once the overspeed latch fails `read()` closed, ros2_control moves the hardware
into its error state. A subsequent STRICT deactivation of the effort controller
is rejected there ("Not acceptable command interfaces combination") and the
controller's update returns errors until the manager is shut down; submission
has already stopped and the shaft coasts. Bench tooling must therefore obtain
its post-latch rest evidence from a fresh passive observation instead of from
the deactivation acknowledgement. Physical force accuracy still needs metrology.

## Safety boundary

No default build or test opens a serial device or sends a CAN frame. All
tests run against `mech_simulation::FakeTransport` with injected clocks.
Real-device activation is gated by [ADR-006](../../../docs/adr/ADR-006-conditional-can0-deployment.md)
(still Proposed) and the G0–G3 evidence gates; every real-motor run needs
the owner's explicit per-test authorization.
