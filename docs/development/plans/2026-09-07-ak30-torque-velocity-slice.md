# AK3.0 Torque/Velocity Command-Interface Slice

- **Date:** 2026-09-07
- **Branch / PR:** `feat/ak30-torque-velocity-command-interfaces`, stacked PR against PR #11's branch (`feat/ak30-hardware-plugin-wiring`); once PR #11 merges, the PR retargets `main` (merge-commit only, two required checks).
- **Scope:** extend `CompositeSystem`'s canonical interface shape so a joint's single command interface may be `position`, `velocity`, or `effort`, wire the AK3.0 force-control Velocity and Torque sub-modes through `Ak30ForceControlRuntime`, and deliver torque/velocity deployment variants. Contract change is **ADR-first**: ADR-014 is filed `Proposed` before implementation and requires owner approval before merge.
- **Owner decision (2026-09-07):** "Torque/Velocity 命令接口切片：扩展 CompositeSystem 接口形状，这个准备搞一下吧" — proceed with the slice; PR #11 stays unmerged for now, so this slice stacks on it.
- **Out of scope:** any real-device command (fresh per-test authorization + ADR-006 Decision 7 apply regardless), the servo profile, Torque/Velocity ros2 controllers (`DemoController` stays position-only; a torque/velocity controller is a later slice), B14 investigation, vendor follow-ups, any change to `mech_protocol_cubemars`/`mech_control_core`/`mech_controllers`.

## Global constraints

- **ADR-014 first** (adapter_contract_v1.md item 7): the `CanonicalCommand`/`CompositeSystem` interface-shape change is a canonical contract change; the ADR is filed `Proposed` in this slice's second commit, before any code commit, and owner approval must precede merge (ADR-013 path, not ADR-012's retroactive path).
- No CAN, serial, vcan, or device access in code or tests. All tests use `mech_simulation::FakeTransport` and injected clocks; no test opens `/dev/ttyACM*`.
- No new packages: `context_check.py`'s six-package `EXPECTED_PACKAGES` map and every package's internal dependency set stay unchanged. All AK3.0-specific code stays in `mech_bringup`; `CompositeSystem` stays vendor-neutral (its accepted single-command-interface set is `{position, velocity, effort}` by name, with no sub-mode concept).
- Reject, never clamp. Fail closed at configure. Non-blocking active path, `noexcept`, no allocation on the active path.
- `-Wall -Wextra -Wpedantic -Werror`; C++17.
- One commit per task, Conventional Commits.

## Architecture

```
CompositeSystem  [extended: joint's one command interface ∈ {position, velocity, effort}]
  └─ RuntimePort (signature unchanged; CanonicalCommand gains velocity/effort fields)
      └─ Ak30ForceControlRuntime  [extended: per-sub-mode command mapping]
          └─ Ak30ForceControlSession (unchanged; sub_mode from config)
```

Per sub-mode:

| Sub-mode | URDF command interface | `CanonicalCommand` field consumed | Session gains consumed | Wire frame fields emitted |
|---|---|---|---|---|
| Position (existing) | `position` | `position` | kp + kd | KP KD POS (VEL/effort as zero feedforward) |
| Velocity (new) | `velocity` | `velocity` | kd | KD VEL (kp=0; effort forced 0) |
| Torque (new) | `effort` | `effort` | — | TRQ only (kp=kd=pos=vel=0) |

Safety semantics carried over from the bench-proven probe patterns:

- **Velocity mode forces `effort = 0` in the runtime** — the wire's effort field rides along as feedforward torque `t_ff` in every sub-mode (`to_device_command` always copies it), so the runtime must zero it, pinning the probe discipline ("Kd path bounds both torque and speed") in the production path rather than trusting the caller.
- **Torque mode ignores position/velocity** — `to_device_command` returns after writing only `torque_nm`; the runtime maps only `effort` and leaves the rest at `CanonicalDeviceCommand{}`'s zeros.
- **Watchdog semantics unchanged** (ADR-012): Following submits fresh writes, Holding freezes without re-submitting, Expired faults within the ≤3-cycle budget, and a missing command is never resolved to `0.0` — which on a velocity interface would command full stop and on an effort interface zero torque; on Position it would command a move to calibrated zero. None are invented.
- **State interfaces stay `[position, velocity, effort]` for all three sub-modes** — the B4 honesty gate lives in the session/codec (`evidenced_state_fields()`), which zero-fills unevidenced fields (Torque: effort only; Velocity: velocity+effort); ADR-009's "raw values never masquerade as effort" semantics are untouched.

## File structure

| File | Change |
|---|---|
| `docs/adr/ADR-014-ak30-submode-command-interfaces.md` | New ADR (`Proposed`) |
| `tools/ci/check_adrs.py` | Registry entry for ADR-014 |
| `docs/adr/README.md` | Index row + provenance note |
| `docs/planning/02_architecture_and_interfaces.md` | ADR status table row |
| `docs/planning/07_framework_bootstrap_plan.md` | ADR status table row |
| `docs/planning/05_decisions_and_open_questions.md` | ADR navigation row (convention) |
| `docs/planning/README.md` | §3 stage-table row update |
| `docs/adr/ADR-003-composite-system-interface.md` | Amendment pointer to ADR-014 |
| `ros2_ws/src/mech_hardware_ros2_control/include/mech_hardware_ros2_control/composite_system.hpp` | `CanonicalCommand` += `velocity`/`effort`; per-joint command-interface name storage |
| `ros2_ws/src/mech_hardware_ros2_control/src/composite_system.cpp` | Shape validation, export, finiteness, claims, LoopbackRuntime |
| `ros2_ws/src/mech_hardware_ros2_control/test/test_composite_system.cpp` | New shape/rejection/loopback tests |
| `ros2_ws/src/mech_bringup/include/mech_bringup/ak30_runtime_params.hpp` | `sub_mode` parameter + `expected_command_interface_name()` |
| `ros2_ws/src/mech_bringup/src/ak30_runtime_params.cpp` | Parse `sub_mode ∈ {position, velocity, torque}` |
| `ros2_ws/src/mech_bringup/src/ak30_force_runtime.cpp` | Per-sub-mode write validation + command mapping (Velocity: effort=0) |
| `ros2_ws/src/mech_bringup/test/test_ak30_runtime_params.cpp` | sub_mode parse matrix |
| `ros2_ws/src/mech_bringup/test/test_ak30_runtime.cpp` | Per-sub-mode runtime tests (golden frames, NaN per field, watchdog parity) |
| `ros2_ws/src/mech_bringup/test/test_ak30_system_integration.cpp` | Torque/Velocity CompositeSystem end-to-end |
| `ros2_ws/src/mech_bringup/config/motor1_torque.urdf.xacro` | New deployment variant (`sub_mode=torque`, effort command interface) |
| `ros2_ws/src/mech_bringup/config/motor1_velocity.urdf.xacro` | New deployment variant (`sub_mode=velocity`, velocity command interface) |
| `ros2_ws/src/mech_bringup/launch/motor1_bringup.launch.py` | `sub_mode` launch argument selects the xacro |
| `ros2_ws/src/mech_bringup/test/test_deployment_files.cpp` | Per-variant structure checks (all spawners stay commented) |
| `docs/development/ak30_force_control_adapter_design.md` | §10 last bullet closed with ADR-014 pointer |
| `docs/development/plans/2026-09-07-ak30-torque-velocity-slice.md` | This plan |

## Tasks

1. **[plan]** Write this plan document. Commit.
2. **[ADR]** File ADR-014 as `Proposed` with the five CI-enforced registrations (ADR file, `check_adrs.py` registry, `docs/adr/README.md`, planning-02/07 status rows) plus convention sync (planning-05 nav row, planning README §3 row, ADR-003 amendment pointer). `check_adrs.py` must report 10 decisions (8 Accepted, 2 Proposed). Commit.
3. **[composite TDD]** Failing tests in `test_composite_system.cpp`: effort/velocity-shaped joints pass `on_init` and export the right names; two command interfaces per joint rejected; unknown command-interface name rejected; velocity/effort finiteness rejection + fault latch; LoopbackRuntime mirrors velocity/effort. Implement the `CompositeSystem` extension to green. Commit.
4. **[runtime + params TDD]** Failing tests: `sub_mode` parse matrix (`position|velocity|torque`, invalid rejected, default position); runtime write validates the consumed field per sub-mode; Velocity submit emits a frame encoding the commanded velocity with effort=0; Torque submit emits a frame encoding the commanded effort; per-field NaN rejection; watchdog three-stage parity across sub-modes. Implement to green. Commit.
5. **[integration]** Failing tests in `test_ak30_system_integration.cpp`: Torque- and Velocity-shaped `HardwareInfo` claim → write → read round-trips through `FakeTransport`; NaN rejection parity; watchdog ERROR + `fault_latched()` surfacing per sub-mode. Implement/adjust. Commit.
6. **[deployment]** `motor1_torque.urdf.xacro` and `motor1_velocity.urdf.xacro` (params round-trip through `Ak30RuntimeParams`; command-interface name matches `expected_command_interface_name(sub_mode)`); launch `sub_mode` argument; `test_deployment_files.cpp` structure checks for all three variants with the spawner-stays-commented invariant. Commit.
7. **[closing]** Planning README §3 row; design doc §10 bullet closed; this plan's Deviations section filled. Full verification (below). Push via proxy 7890, open the stacked PR. Commit.

## Verification

- `MECH_OUTPUT_ROOT=/tmp/ak30-tv MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh` — 6 packages, 0 failures (baseline 236 tests; count grows).
- `bash tools/ci/run_sanitizers.sh /tmp/ak30-tv-san` — 0 sanitizer reports, `-fsanitize` verified in the CMake cache.
- `env -i PATH=/usr/bin:/bin /usr/bin/python3 tools/ci/context_check.py`; `python3 tools/ci/check_adrs.py` — 10 decisions (8 Accepted, 2 Proposed); `git diff --check`; `xmllint --noout` on the new xacro files; docs relative-link check.
- Golden-frame expectations recomputed independently from the L07 ranges (±12.56 rad / ±40 rad/s / ±15 N·m, kp 0–500, kd 0–5) via the `((1<<bits)-1)` packing — never copied from a manual example row.

## Deviations from this plan, found during execution

- **Torque golden 0.2 N·m first computed as `08 1A`, actually `F8 1A`.** The first recompute script printed the correct bytes for the 2.0 N·m cross-check but the 0.2 row was mis-derived once (byte 6 is `((t_i & 0xF) << 4) | ((t_i >> 8) & 0xF)` = 0xF8, not 0x08); caught by the integration test's byte-for-byte comparison, fixed by re-running the independent recompute before consulting any implementation. This is exactly why the goldens are recomputed rather than copied from any single source.
- **`known_command_interface` had to become per-joint.** The plan sketched "known_command_interface/claim model adapted"; the concrete change is that the accepted command-interface *name* is now stored per joint at `on_init` (`joint_command_interface_names_`), and `known_command_interface` compares against that joint's own name instead of the global `HW_IF_POSITION` constant. The per-joint claim model (one bool per joint) needed no change - it was already per-joint, not per-interface.
- **`exactly_interfaces` lost its command parameter rather than gaining a list.** The exact-order list now applies to state interfaces only; the command side is "exactly one interface whose name is in the accepted set" (`is_command_interface_name`), because the name's meaning is per-joint configuration.
- **LoopbackRuntime mirrors velocity/effort additively rather than per-kind branching.** The first sketch branched on the joint's command kind; the shipped form adds the commanded velocity to the ramp step and assigns commanded effort, which is deterministic and value-independent without the runtime knowing each joint's declared kind - and stays correct for the position case because a position joint's velocity/effort members stay 0.0 by construction.
- **The old `RejectsInvalidInterfacesAndStrictSwitchConflicts` "effort command interface rejected" assertion had to change meaning.** Effort is now a *legal* command interface; the rejection test now uses an unknown name ("acceleration") plus new dedicated tests pin that two interfaces and zero interfaces per joint are also rejected. The negative space moved from "only position" to "exactly one of the three".
- **`WriteValidatesConsumedFieldPerSubMode` deliberately accepts ignored-member garbage.** A non-finite position in Torque mode does not fault the runtime write because the Torque device never sees it (CompositeSystem still rejects non-finite values in *exported* members before the runtime; here the garbage is written directly into the runtime port, the path only a custom RuntimePort caller could take). Pinned as intentional in the test.
