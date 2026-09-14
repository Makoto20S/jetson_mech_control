# mech_bringup

Deployment composition boundary: bring-up harnesses, device probes, and the
AK3.0 force-control runtime wiring that connects `mech_protocol_cubemars` to
`mech_hardware_ros2_control`'s `CompositeSystem`.

## Components

- **`Ak30ForceControlRuntime`** — the `RuntimePort` implementation that wires
  the AK3.0 force-control session (Position sub-mode) into `CompositeSystem`.
  It is the first production consumer of `command_stage()` (ADR-012's staged
  watchdog): fresh controller writes are submitted while Following, the last
  valid command is frozen through Holding, and Expired faults the system
  within the 3-cycle budget. It never resolves a missing command to `0.0`.
- **`Ak30RuntimeParams`** — fail-closed parsing of the URDF `ros2_control`
  hardware parameters into the runtime config. `device_path` is mandatory;
  unknown keys, non-numeric values, and over-budget TTLs reject configure.
- **`FoundationHarness`** — the Foundation-era simulation harness
  (CompositeSystem + TargetLimiter through exported interfaces).
- **Device probes** (`ak30_torque_probe`, `ak30_position_probe`) — the
  bench-validated bring-up probes, built only with
  `-DMECH_BUILD_DEVICE_PROBES=ON`.
- **Deployment example** (`config/motor1.urdf.xacro`,
  `config/motor1_controllers.yaml`, `launch/motor1_bringup.launch.py`) —
  single-motor Position-sub-mode example carrying motor1's bench-evidenced
  parameters. The position-controller spawner ships commented out:
  uncommenting it arms position commands, which stay gated by ADR-006 and
  per-test owner authorization.

## Safety boundary

No default build or test opens a serial device or sends a CAN frame. All
tests run against `mech_simulation::FakeTransport` with injected clocks.
Real-device activation is gated by [ADR-006](../../../docs/adr/ADR-006-conditional-can0-deployment.md)
(still Proposed) and the G0–G3 evidence gates; every real-motor run needs
the owner's explicit per-test authorization.
