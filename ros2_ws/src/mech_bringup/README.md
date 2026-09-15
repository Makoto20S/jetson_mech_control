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

## Safety boundary

No default build or test opens a serial device or sends a CAN frame. All
tests run against `mech_simulation::FakeTransport` with injected clocks.
Real-device activation is gated by [ADR-006](../../../docs/adr/ADR-006-conditional-can0-deployment.md)
(still Proposed) and the G0–G3 evidence gates; every real-motor run needs
the owner's explicit per-test authorization.
