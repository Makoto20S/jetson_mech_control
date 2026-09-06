# AK3.0 Hardware-Plugin Wiring Slice (Position sub-mode)

- **Date:** 2026-09-06
- **Branch / PR:** `feat/ak30-hardware-plugin-wiring`, one PR against protected `main` (merge-commit only, two required checks).
- **Scope:** the stage-3 software slice defined by `docs/planning/README.md` §3 and `docs/development/ak30_force_control_adapter_design.md` §10 — wire `mech_protocol_cubemars`'s `Ak30ForceControlSession` into `CompositeSystem` via the existing `RuntimePort` boundary, add the first production consumer of `command_stage()` (ADR-012), and deliver a deployment example.
- **Owner decisions (2026-09-06):**
  1. **Position sub-mode only.** `CompositeSystem` exports a `position` command interface and `position/velocity/effort` state interfaces, which is exactly the Position sub-mode's shape (Kp/Kd impedance control). Torque/Velocity command interfaces would change the composite interface shape = canonical contract change = ADR first (adapter_contract_v1.md item 7); they are explicitly deferred to a later slice.
  2. **Full deployment example:** single-joint URDF + controllers.yaml + launch file in `mech_bringup`.
- **Out of scope:** any real-device command (fresh per-test authorization + ADR-006 Decision 7 boundary apply regardless), Torque/Velocity command interfaces, the servo profile, B14 investigation, any change to `CompositeSystem`/`mech_control_core`/`mech_protocol_cubemars` public interface semantics.

## Global constraints

- No new ADR is required for this slice: it consumes only frozen contracts (`RuntimePort`, `Ak30ForceControlSession`, `command_stage`). If implementation appears to need a contract change, stop and file the ADR first.
- No CAN, serial, vcan, or device access in code or tests. All tests use `mech_simulation::FakeTransport`/`FakeSerial`-style doubles and injected clocks; no test opens `/dev/ttyACM*`.
- No new packages: `context_check.py`'s six-package `EXPECTED_PACKAGES` map and every package's internal dependency set stay unchanged. All new runtime code lives in `mech_bringup` (already depends on both `mech_hardware_ros2_control` and `mech_protocol_cubemars`; it is the documented deployment composition boundary).
- Reject, never clamp. Fail closed at configure. Non-blocking active path, `noexcept`, no allocation on the active path.
- `-Wall -Wextra -Wpedantic -Werror`; C++17.
- One commit per task, Conventional Commits.

## Architecture

```
CompositeSystem (unchanged)
  └─ RuntimePort (unchanged boundary)
      └─ Ak30ForceControlRuntime  [new, mech_bringup]
          ├─ injected: Transport& (UsbCdcTransport in production; FakeTransport in tests)
          ├─ injected: now() clock (steady_clock in production; test clock in tests)
          └─ Ak30ForceControlSession (Position sub-mode, Kp/Kd gains)
```

Per control cycle (ros2_control order is `read -> update -> write`):

- `write(commands)`: store the latest finite positions as pending commands (CompositeSystem already rejects NaN/Inf on every element, claimed or not).
- `read(states)`:
  1. `now = clock_()`.
  2. `stage = session.command_stage(now)` — **the production watchdog consumer this slice exists to add**:
     - `Following` → submit the pending (or held) command with `generation+1`, `deadline = now + 2*period`.
     - `Holding` → re-submit the last valid command unchanged. Never synthesize `0.0` (ADR-012 Decision 3); never advance `last_command_time_` artificially — the stage advances on the real clock, so Holding naturally ends in Expired within the ≤3-cycle budget.
     - `Expired` → return `false`, which routes through CompositeSystem's existing `fault_latched_` ERROR path.
  3. Drain `transport.try_receive()` (bounded) → `session.process(frame, now)`.
  4. Fill `CanonicalState` from `session.snapshot(now)`. With the Position sub-mode, `evidenced_state_fields()` is all-true, so position/velocity/effort are all populated.

Wait — ordering note resolved during planning: `read()` runs *before* `write()` in a ros2_control cycle, so on each cycle `read()` submits the command stored by the *previous* `write()`. This mirrors the probe loop (submit → drain → snapshot) and the existing harness's read-before-write ordering test (`FirstCycleReadsStaleStateBeforeCommandTakesEffect`). First activated cycle has no pending command: submit nothing, only drain/sample; the first `Following` submit happens on the next cycle. This keeps `read()` from inventing a zero command on the first cycle — the exact defect ADR-012 Decision 3 forbids.

## File structure

| File | Responsibility |
|---|---|
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_force_runtime.hpp` | `Ak30ForceControlRuntime : RuntimePort` + config struct |
| `ros2_ws/src/mech_bringup/src/ak30_force_runtime.cpp` | Implementation |
| `ros2_ws/src/mech_bringup/test/test_ak30_runtime.cpp` | Runtime + wiring tests (FakeTransport, injected clock) |
| `ros2_ws/src/mech_bringup/config/motor1.urdf.xacro` | Single-joint URDF with `ros2_control` block |
| `ros2_ws/src/mech_bringup/config/motor1_controllers.yaml` | controller_manager + demo position controller |
| `ros2_ws/src/mech_bringup/launch/motor1_bringup.launch.py` | Composition entry (loads URDF + controllers) |
| `ros2_ws/src/mech_bringup/test/test_deployment_files.cpp` | URDF/YAML structure validation (no launch) |
| `docs/development/plans/2026-09-06-ak30-hardware-plugin-slice.md` | This plan |

## Tasks

1. **[plan]** Write this plan document. Commit.
2. **[runtime TDD]** Add `test_ak30_runtime.cpp` failing tests: lifecycle (configure/start/stop), following-submit, holding-freeze-not-zero, expired-faults, WouldBlock-retry-not-fault, feedback-decode fills states, stale-snapshot handling, clock-backwards safety. Implement `ak30_force_runtime.{hpp,cpp}` to green. Update `CMakeLists.txt` (lib source + test). Commit.
3. **[parameters]** Failing tests for `Ak30ForceRuntimeConfig::from_hardware_parameters()` (fail-closed on missing/invalid; TTL defaults 4 ms/6 ms; hard-TTL ceiling 6 ms; sub-mode pinned to Position; gains mapped to Kp/Kd). Implement. Commit.
4. **[integration]** Failing tests wiring `CompositeSystem.set_runtime(Ak30ForceControlRuntime)` end-to-end: claim → write → read round-trip through FakeTransport (echo feedback), 100× lifecycle repeat, watchdog three-stage behavior at 2 ms period boundaries (4,000,999/4,001,000 and 6,000,999/6,001,000 ns pins), non-finite rejection parity. Implement/adjust. Commit.
5. **[deployment]** URDF/controllers.yaml/launch + `test_deployment_files.cpp` structure checks (parse the xacro-generated XML via a plain-XML golden check — no xacro execution in CI; validate controller names, joint name, hardware parameters presence, interface lists). Commit.
6. **[closing]** README updates (mech_bringup package README; planning README §3 next-slice row status; design doc §10 last bullet marked closed with pointer to the consumer). Full verification: 6-package build/test, sanitizers (verify `-fsanitize` in the CMake cache), `context_check.py`, `check_adrs.py`, `git diff --check`, docs link check. Push via proxy 7890, open PR. Commit.

## Verification

- `MECH_OUTPUT_ROOT=/tmp/ak30-plugin MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh` — 6 packages, 0 failures.
- `bash tools/ci/run_sanitizers.sh /tmp/ak30-plugin-san` — 0 sanitizer reports, flags verified in cache.
- `python3 tools/ci/context_check.py`, `python3 tools/ci/check_adrs.py`, `git diff --check` — PASS.
- Watchdog boundary tests pin 4,000,999/4,001,000 and 6,000,999/6,001,000 ns (same standard as `test_ak30_force_session.cpp`).

## Deviations from this plan, found during execution

- **Task 2's Holding semantics tightened beyond the plan's first sketch.** The plan sketched "Holding → re-submit the last valid command unchanged". During test design this proved wrong: every accepted submit advances the session's `last_command_time_`, so re-submitting during Holding would reset the stage clock and keep the watchdog in Following forever, masking a dead controller. The implemented semantics: a command is submitted only when the controller wrote a fresh one (`write()` sets a fresh flag); a stale stored command is never re-submitted, so the stage escalates through Holding to Expired on real time — which is what ADR-012's "freeze the last valid command" means on the wire (the device keeps executing the last accepted frame; the runtime sends nothing new).
- **Session's `Expired` conflates "never commanded" with "stale".** `command_stage()` returns Expired when no command was ever accepted, which would fail the first cycle after activation. The runtime distinguishes the two with a `submitted_once_` flag: only a previously-accepted command aging past the hard TTL is a watchdog failure; the never-commanded state sends nothing (and never invents 0.0).
- **`StaleFeedbackYieldsZeroStatesWithoutFault` needed a shortened `feedback_ttl`.** With the default TTLs (command 4/6 ms, feedback 6 ms), a feedback frame always ages out inside the command's hard window, so staleness cannot be isolated from watchdog expiry; the test uses `feedback_ttl = 2 ms` — a legal configuration the runtime merely forwards to the session.
- **`read()` ordering fixed as plan described**: each cycle's `read()` submits the command stored by the previous `write()` (ros2_control's read → update → write ordering). The first activated cycle sends nothing.
- **xmllint note:** the URDF ships as `.xacro`; CI validates it as plain XML (`xmllint --noout`) plus programmatic structure checks in `test_deployment_files.cpp` — no xacro execution in CI, per the plan.
