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
  it maps commands per sub-mode (Position reads position and optional desired
  velocity/feedforward effort, Velocity reads velocity with effort forced to 0,
  Torque reads effort).
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
sit from the latest usable feedback position. This bounds the nominal
proportional term `Kp * (command - measured)`: with motor1's `Kp = 1 N*m/rad`,
0.5 rad means a 0.5 N*m proportional-term budget. It does not bound total
torque, which also includes the Kd velocity-error term and optional feedforward.
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
remains Proposed; the historical Kp4 latch is recorded below, and no new
combined-command hardware acceptance is claimed.

### What the error bound costs this deployment

The absolute bounds are unchanged from what the controller YAML already
enforced. The error bound is new, and at motor1's shipped `Kp = 1` it is a real
limit on how far a bench move may reach, not a formality:

- At `Kp = 1` the shaft lags far behind the command. On the 2026-09-05
  progressive bench route a `+10 deg` target moved the shaft `0.2 deg` while
  the command itself completed, and the move settled at `9.8 deg` of error
  (`0.171 rad`); the `+30 deg` run settled at `15.7 deg` (`0.274 rad`). Source:
  `docs/planning/README.md` §3, 阶段 2 ("位置步进 +2°/+5° 与 +30° 全程
  （Kp=1 摩擦稳态误差）").
- `PositionCommandController` ramps its command open-loop at
  `max_slew_per_second: 2.0 rad/s`, regardless of where the shaft actually is.
  It does not wait for the shaft to catch up.

Neither settled error above would trip the bound on its own — both are under
`0.5 rad`. The transient is what does: while the ramp is in flight the command
runs away from a shaft that has barely moved, so command-minus-measured error
peaks well above the error the move eventually settles at. Under the shipped
`position_max_error_rad = 0.5`, a single target much farther than `0.5 rad`
(`29 deg`) from the current position is therefore expected to exceed the bound
part way through the ramp and latch the hardware before the shaft reaches the
target, after which STRICT deactivation will be refused until the controller
manager is restarted (the T9 finding above). This is a prediction from the ramp
shape and the `Kp = 1` bench numbers; the distinct Kp4 trial below later
observed an envelope latch. The `2.0 rad/s` slew rate is likewise not reachable under
this envelope at `Kp = 1`: with `Kd = 1` and zero commanded velocity, holding
`2 rad/s` would need more than `2 rad` of error.

The `0.5 rad` value is the owner-approved one and is kept, because it is the
proportional-term budget (`0.5 N*m` at `Kp = 1`), not a travel budget. Bench procedures
must still plan each move as if it were one: treat `29 deg` as the ceiling on a
single target displacement — the T10 trajectory's `0.1745 rad` step is inside.

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

## Standard controller deployment (JTC)

`motor1_trajectory_bringup.launch.py` loads the upstream
`joint_trajectory_controller/JointTrajectoryController` using
`config/motor1_trajectory_controllers.yaml` and the Position hardware xacro.
Install `ros-humble-joint-trajectory-controller` on the target host first.
The state broadcaster spawns first; JTC then loads **inactive**. Starting the
launch still opens the physical transport and requires the bench gates below.

JTC claims only `motor1_joint/position`, with position-only state feedback;
it does not claim `command_generation`. This is ADR-017's weak tier: at the
configured 500 Hz, every authorized manager cycle refreshes the hardware
command and can produce a frame. A controller that stays active but stops
writing fresh targets is invisible to that freshness check and may keep its
last target indefinitely. A stalled manager still expires the hardware lease.
This is the accepted weak-tier risk recorded as R23 in the MVP plan.

ADR-019's hardware envelope checks the absolute target and its error from
usable feedback before submission; it applies regardless of controller type.
An offending command is dropped and latched, never clipped into range. The
shipped bounds are `[-12, 6] rad` and a `0.5 rad` maximum error. This is a
command boundary, not a physical stop guarantee or an upstream-target watchdog.
After a latch, obtain fresh passive rest evidence: STRICT deactivation can be
refused while the hardware is in its error state.

The offline integration test loads the actual upstream plugin through
`ControllerManager`, with `CompositeSystem`, the AK3 runtime, and
`FakeTransport`; no serial port is opened. It covers measured-position hold,
a bounded out-and-back trajectory, continued hardware cycling after
controller deactivation, and the envelope's frame rejection and latch.
Humble JTC 2.54 queues a hold in `on_activate()` and writes its command
interface on the next `update()`. To cover the intervening hardware write,
CompositeSystem initializes each new weak position claim from the feedback
already accepted in that cycle (ADR-017). This applies again after release
and reacquisition; it never seeds strong claims, velocity, effort, release-only
or rejected switches. The shipped JTC parameter
`set_last_command_interface_value_as_state_on_activation: false` also makes
JTC's internal trajectory use measured state. Neither mechanism strengthens
the weak-tier silence watchdog or guarantees a physical stop.
A separate persistent Jetson bench operator now exposes
`./trajectory run [Kp=6] [Kd=1]`, plus `check` and `observe`, from the deployment
root. It observes a fresh center per run, keeps the same trajectory timing, and
accepts Kp 0..18 / Kd 0..5. The tuning error envelope is 0.3 rad for Kp <= 10
and 1.8/Kp rad above 10 (0.1 rad at Kp=18). This bounds only the nominal
proportional term, not total torque. Deployment validation is not physical
Kp=6 tracking evidence. See the deployed Chinese operating guide.

Follow-up gain trial (2026-09-18): the owner authorized Kp=4/Kd=1 with the same
trajectory. The temporary candidate tightened position error to 0.075 rad,
retaining a nominal 0.3 N*m proportional-term ceiling (not a total torque bound).
At about 2 s, reference displacement was 5.0 degrees but feedback displacement
was 0.7 degrees. PositionEnvelope (reason 12) latched; the round trip did not
complete. STRICT deactivation was refused after the latch, as described above.
A fresh passive run confirmed rest, zero motor commands, and port release.

Motor1 bench evidence (2026-09-18): an owner-authorized JTC action completed
+0.1745 rad in 4 s, held its target for 1 s, and returned the target in 4 s;
STRICT deactivation, fresh post-stop rest, fault-free feedback and port release
passed. Actual feedback moved only about 0.6 degrees from the start and ended
about 0.5 degrees above it: action success is functional interchangeability
evidence, not accurate trajectory tracking. Two earlier strict zero-motion
hold attempts aborted on 40 ERPM; the owner waived that prerequisite to proceed,
so a successful two-second zero-motion hold is not claimed. ADR-019 remains
Proposed pending owner acceptance; the follow-up Kp=4 trial above observed a
physical position-error envelope latch.

Later diagnostic trials used Kp=18/Kd=5 and the same 10-degree displacement,
with two repeats each of 4 s and 2 s legs. Outbound speed coefficient of
variation fell from 0.453–0.461 to 0.245–0.260 with faster motion, while absolute
speed standard deviation increased from about 17–18 to 21–22 ERPM. An
angle-correlated current/voltage component near a 3.214-degree output period
persisted. Cross-speed voltage/angle fits with a roughly 26 ms relative lag
reached R² 0.960–0.979; neither the fit nor the period identifies a unique
sensor, commutation, encoder or mechanical root cause. Host writes were near
2 ms apart; this does not establish actual CAN delivery or device timing.

All four diagnostic actions completed, but their immediate post-deactivation
rest checks failed; separate fresh passive checks subsequently confirmed rest,
zero motor commands and port release. UART telemetry changed mode about 3 s
after the last host transmission, so deactivation is not proof of immediate
rest or torque disable. Precise tracking and full stopping behavior remain
unqualified.

On 2026-09-21 the owner deferred further jerk/root-cause investigation until
it recurs in practical use, and authorized committing the lifecycle fix and
updating PR #20. This deferral does not claim that jerk is fixed, that stopping
is qualified, or that ADR-019 is accepted. Historical lifecycle validation ran
414 tests each on local Humble, native ARM64 and ASan/UBSan with no failures.
Diagnostic tools and raw captures remain local and are not part of this PR.

## Position desired velocity and feedforward extension

The hardware Position path accepts explicitly declared `position+velocity`,
`position+effort`, and `position+velocity+effort` bundles. The owning controller
writes position [rad], desired velocity [rad/s] and feedforward effort [N*m]
together before each hardware update; configured Kp/Kd are appended by the
adapter. This does not implement a gravity model or change drive calibration.

`position_max_abs_velocity_rad_s` and `position_max_abs_feedforward_nm` default
to zero. A declared auxiliary interface requires its explicit positive bound;
both must be finite and within wire ranges. An invalid auxiliary command
rejects/latches the whole tuple without transmitting an earlier pending target.
This emits `PositionTuple` (reason 13); existing reason numbers are unchanged.
Existing position envelope, feedback freshness and command deadlines still
apply. These limits do not bound total drive torque or physical speed.

Examples (software-only; values are not approved hardware settings):

- `config/motor1_position_velocity.urdf.xacro` with
  `config/motor1_position_velocity_trajectory_controllers.yaml` exposes P+V for
  the installed upstream Humble JTC. Existing position-only files are unchanged.
- `config/motor1_position_velocity_effort.urdf.xacro` exposes the full P+V+E
  bundle for a compatible single controller. **Humble JTC cannot command this
  bundle** because its effort interface must be used alone. No production
  full-tuple controller is shipped by this extension. Offline controller-manager
  tests exercise it through a test controller and decode actual fake frames.

Whole bundles must be claimed/released together. Deploy one owner per tuple;
aggregate hardware switch callbacks cannot identify controller ownership.
New claims clear old auxiliary values, and weak Position takeover seeds the
accepted position. Strong generation applies to the whole tuple; weak owners
remain unable to signal target staleness. Change declared bundles only through
inactive reconfiguration. No new real experiment or persistent deployment is
authorized by these examples; await owner approval after the PR.

## Safety boundary

No default build or test opens a serial device or sends a CAN frame. All
tests run against `mech_simulation::FakeTransport` with injected clocks.
Real-device activation is gated by [ADR-006](../../../docs/adr/ADR-006-conditional-can0-deployment.md)
(still Proposed) and the G0–G3 evidence gates; every real-motor run needs
the owner's explicit per-test authorization.
