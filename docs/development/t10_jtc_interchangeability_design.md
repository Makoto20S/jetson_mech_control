# T10: Standard Controller Interchangeability and Hardware Position Envelope

- **Status:** Approved design (owner, 2026-09-17); PR-1 implemented; PR-2 offline verification complete (412 tests each on local, ASan/UBSan and native ARM64), bench pending
- **Date:** 2026-09-17
- **Scope:** `mech_bringup` runtime/deployment, `mech_hardware_ros2_control` weak-position takeover, `docs/adr`, `docs/planning`; bench tools stay ignored under `tmp/`
- **Owner decisions recorded here:** validate `joint_trajectory_controller` first; add a fail-closed hardware position envelope before any standard controller drives the motor; envelope defaults `[-12, 6] rad` / `0.5 rad`, bench overlay `+/-20 deg` / `0.3 rad`, first trajectory `+10 deg` legs of 4 s; two stacked PRs.

## 1. Purpose

The project's founding requirement is that standard `ros2_control` controllers
can drive this hardware directly (ADR-017 status rationale;
`02_architecture_and_interfaces.md:255`). Three in-house controllers
(Position, Velocity, Effort) are accepted on the motor1 bench, but no upstream
controller has ever been instantiated against `CompositeSystem`, offline or on
hardware. T10 closes that gap with `joint_trajectory_controller` (JTC) and, at
the same time, implements the hardware-side final bound that
`02_architecture_and_interfaces.md` §10 item 4 planned and no slice delivered.

## 2. Facts the design rests on

| Fact | Evidence |
|---|---|
| A controller that claims only the motion interface runs in ADR-017's weak tier: every authorized manager cycle counts as a refresh, so the hardware watchdog never sees "the controller went silent". | ADR-017 Decision 4; `composite_system.cpp` `bool fresh = authorized;`; `WeakTierControllerKeepsSendingWhileSilent` |
| The hardware path applies no clamp or slew to position targets. ADR-019 rejects absolute-position/error-bound violations before the wire codec also checks its `+/-12.56 rad` range. | `Ak30ForceControlRuntime::submit_stored()`; `ak30_force_wire.hpp` |
| In Position sub-mode the applied torque is `Kp * (command - position)` with `Kp = 1 N*m/rad`, `Kd = 1`; static friction is about `0.2 N*m`; `0.2 N*m` produced about `28 rad/s^2` after breakaway on 2026-09-17. | `motor1.urdf.xacro:37-38`; T9 bench README in `tmp/t9_effort/physical-runs/` |
| `submit_stored()` checks the pending command against the feedback sample already accepted by `read()` this cycle; ADR-016 validates feedback freshness before submission. | `Ak30ForceControlRuntime::submit_stored()` and `read()` |
| JTC 2.54 queues a measured hold in `on_activate()` but first writes the command interface in the next `update()`. The manager switch cycle writes hardware before that update. | upstream source and failing real-plugin integration test; ADR-017 takeover amendment below |
| JTC 2.54.0 is installed on both development hosts; CI resolves ROS packages through `rosdep` from `package.xml`. | 2026-09-18 package verification; `docker/ros_humble_jammy/Dockerfile` |

## 3. Deliverable 1 (PR-1): hardware position envelope

### 3.1 ADR-019

`docs/adr/ADR-019-hardware-position-envelope.md`, status Proposed at
implementation, Accepted by the owner after bench evidence. It records:

- Decision: the AK3.0 runtime rejects, without clamping or sending, any
  Position sub-mode command outside `[position_min_rad, position_max_rad]` or
  farther than `position_max_error_rad` from the latest usable feedback
  position; a violation latches like `torque_max_abs_erpm` and is cleared only
  by the runtime lifecycle, never by claim cancellation.
- Physical meaning: `position_max_error_rad * Kp` is the largest torque the
  hardware layer lets any controller apply. With the shipped `Kp = 1`,
  `0.5 rad` means `0.5 N*m`.
- Consequences: ADR-016/ADR-017 unchanged; the weak-tier watchdog gap stays an
  accepted risk and is finally entered in `03_mvp_delivery_plan.md` §12; the
  §6 "command watchdog" acceptance row is annotated "strong tier only".
- Review triggers: any Kp change, any load on the shaft, any second joint.

### 3.2 Runtime parameters (`Ak30RuntimeParams`)

| Param | Type | Rule |
|---|---|---|
| `position_min_rad` | double | finite; required in Position sub-mode |
| `position_max_rad` | double | finite; `> position_min_rad`; required in Position sub-mode |
| `position_max_error_rad` | double | finite; `> 0`; required in Position sub-mode |

Handling in other sub-modes mirrors how `torque_max_abs_erpm` is handled
outside Torque today (the implementer reads that code path and matches it;
no silent ignore). Missing or invalid values fail `on_init`.

### 3.3 Runtime check

In `Ak30ForceControlRuntime::submit_stored()`, Position sub-mode only, after
the fresh-command gate and before `session_.submit()`:

1. Let `p = pending_[0].position`. The absolute bounds are checked
   unconditionally: violation if `p < min` or `p > max` (bounds inclusive).
2. Use the feedback sample already accepted by `read()` this cycle, never a
   second snapshot that could become stale between checks. Treat
   `|p - accepted_position| > position_max_error_rad` as a violation.
   ADR-016 fails closed on stale or absent feedback before submission.
3. On violation: set `position_envelope_latched_ = true`, drop the pending
   command, `capture_error(FeedbackTelemetryReason::PositionEnvelope, now)`,
   return `false`. `read()` keeps returning `false` while latched, exactly as
   the torque latch does. No frame with the violating value ever leaves.

`FeedbackTelemetryReason::PositionEnvelope` is appended after
`TorqueSpeedUnavailable` so existing numeric reason codes keep their values.

### 3.4 Deployment

`config/motor1.urdf.xacro` gains the three params with defaults
`-12.0`, `6.0`, `0.5`. The absolute bounds match the existing controller YAML
so PositionCommandController behaviour is unchanged (the error bound is a new
limit on that deployment; see ADR-019 Consequences); the error bound is the
new hardware ceiling. The persistent Jetson position release is not
re-deployed by this PR.

### 3.5 Tests

- `test_ak30_runtime_params.cpp`: required/finite/ordering rules, other
  sub-mode handling.
- `test_ak30_runtime.cpp`: inclusive bounds; error bound both signs; no
  evaluation without usable feedback; latch survives claim cancellation and a
  subsequent in-range command; telemetry reason emitted; frame counter does
  not increase on violation.
- `test_deployment_files.cpp`: position variant carries the params; velocity
  and torque variants do not.
- `test_controller_manager_integration.cpp`: a weak-tier `WriterController`
  commanding out of envelope latches the hardware and transmits nothing for
  that command.
- Full local, ASan/UBSan and native ARM64 suites green before publication.

## 4. Deliverable 2 (PR-2): JTC deployment and offline proof

### 4.1 Deployment variant

- `config/motor1_trajectory_controllers.yaml`: `joint_state_broadcaster` plus
  `motor1_trajectory_controller` of type
  `joint_trajectory_controller/JointTrajectoryController`, `joints:
  [motor1_joint]`, `command_interfaces: [position]`, `state_interfaces:
  [position]` (velocity state is not relied on; if Humble JTC refuses
  position-only state the implementer adds `velocity` and records why),
  `allow_partial_joints_goal: false`,
  `allow_nonzero_velocity_at_trajectory_end: false`, `open_loop_control:
  false`, tolerances left at upstream defaults unless the offline test shows
  they must change. `update_rate: 500` as in the other variants.
- `launch/motor1_trajectory_bringup.launch.py`: same shape as the velocity
  and effort launches, serialized spawners, JTC loaded `--inactive`.
- The URDF is the position variant from PR-1; no new xacro.
- `mech_bringup/package.xml`: `exec_depend` and `test_depend` on
  `joint_trajectory_controller` so rosdep installs it in CI and on hosts.

### 4.1a Weak-position takeover (owner approved 2026-09-18)

The real JTC test exposed a switch-cycle gap: JTC activation queues a hold but
has not yet written the command interface when hardware `write()` runs.
`CompositeSystem::perform_command_mode_switch()` therefore seeds every newly
started weak position claim from `states_[index].position`, the feedback
already accepted by the current cycle. The full switch is validated before
any seed; first activation and release/reclaim both use this rule. Strong
claims, velocity, effort, release-only and rejected switches do not seed.
Generation baselines, cancellation, feedback validity, and weak-tier silence
refresh remain unchanged (ADR-017 Decision 4).

The deployment also sets
`set_last_command_interface_value_as_state_on_activation: false`, so JTC's
own internal trajectory starts from measured state. That parameter alone
cannot fill the switch-cycle gap. The seed is a measured hold, not a new
upstream target or a physical stop guarantee.

### 4.2 Offline integration test (real upstream plugin)

New test in `mech_bringup` using `ControllerManager` + `CompositeSystem` +
`FakeTransport`, loading the upstream JTC by its plugin name:

1. Activation holds: after STRICT activation the first frames carry the
   current feedback position (within float tolerance) and no other value.
2. Trajectory execution: a two-point trajectory (`+0.1745 rad` over 4 s,
   back over 4 s) yields a monotone, bounded-step sequence of position frames
   that ends at the start position; frames are produced every manager cycle
   (weak tier), which the test asserts explicitly as documented behaviour.
3. Deactivation stops frames.
4. Envelope: a trajectory whose target exceeds `position_max_error_rad`
   from the feedback position latches the hardware and no frame carries the
   violating value; the controller manager reports the error state.
5. Existing `WriterController` weak-tier tests remain.

Simulated time is not frozen (project rule: real clock for freshness
behaviour); the test drives read/update/write on the real clock as the
existing manager tests do.

### 4.3 Documentation

- `mech_bringup/README.md`: new "Standard controller deployment (JTC)"
  section stating the weak-tier cadence (one frame per manager cycle), the
  envelope, and what the bench proved.
- `docs/planning/README.md` §3 row for T10; §12 risk entry for the weak-tier
  gap; §6 watchdog row annotated.
- ADR-019 status moves to Accepted only after the bench run.

## 5. Bench procedure (owner-authorized, per run)

Tools under `tmp/t10_trajectory/` (ignored), following the T8/T9 pattern:
hash-pinned candidate bound to its directory, policy/runner/simulated-ROS
rehearsal on both hosts, default check that never opens the port.

Candidate overlay: `position_min/max_rad = center +/- 0.349 rad` where
`center` is the rest position recorded by the most recent passive
observation and is baked into the hashed candidate; `position_max_error_rad
= 0.3`; `Kp = 1`, `Kd = 1` unchanged. Before any execute run the tool
re-reads the rest position and refuses to proceed if it differs from
`center` by more than `0.087 rad` (5 deg), so a shaft that was moved by hand
requires a new candidate rather than silently shifting the envelope.

Run shape, each step separately authorized:

1. Passive observation, controller inactive; record rest position.
2. Activate JTC (STRICT, retry against the ADR-016 gate as
   `tools/bench/activate_position_controller.sh` does); confirm the commanded
   position equals the feedback position and the shaft does not move for
   2 s; deactivate. This alone proves "upstream controller drives the
   hardware" with zero motion.
3. Trajectory `+0.1745 rad` in 4 s, hold 1 s, back in 4 s via
   `FollowJointTrajectory`; monitor `/joint_states` error (abort above
   `0.25 rad`), feedback age (abort above 60 ms), fault, and the JTC action
   result; deactivate; confirm rest.
4. Optional, separately authorized: a deliberately out-of-envelope goal to
   observe the latch on hardware. Post-latch rest evidence comes from a fresh
   passive observation (T9 finding: STRICT deactivation is rejected in the
   hardware error state).

Expected physics: at `Kp = 1` a 2.5 deg/s ramp needs only `0.02-0.04 N*m`,
below static friction, so stick-slip (creep, catch, jump) is the likely
motion pattern. That is a gain-tuning observation, not a JTC or framework
defect, and does not fail acceptance.

## 6. Acceptance criteria

- Offline: PR-1 and PR-2 suites green locally, under sanitizers and on ARM64;
  CI green with JTC resolved by rosdep.
- Bench: steps 1-3 complete with `fault = 0`, no envelope latch, JTC action
  succeeds, STRICT deactivation confirmed, port released; evidence archived
  under `tmp/t10_trajectory/physical-runs/`.
- Governance: ADR-019 Accepted by the owner; risk register updated; PR-2
  merged after bench acceptance and tagged.

## 7. Out of scope

Persistent Jetson operator for JTC; `forward_command_controller`; MoveIt;
closing the weak-tier watchdog gap (ADR-017 alternative G); the
ros2_control behaviour that rejects STRICT deactivation in the hardware
error state; Kp retuning for smooth low-speed tracking (may follow as its
own bench task).

## 8. Prerequisites and host changes

- Workstation: `sudo apt install ros-humble-joint-trajectory-controller`
  (owner runs it; no passwordless sudo here).
- Jetson: the same single package via `apt install`, which the standing
  "specific package only, never upgrade" rule permits; owner authorization
  recorded before it is run.
