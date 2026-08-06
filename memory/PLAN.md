# Project Plan

> Last updated: 2026-08-06T19:28:00+08:00

## Overall Goal

- Deliver a reusable Jetson C++/ros2_control CAN framework that can control supported motors and read supported sensors through device-specific protocol adapters while preserving standard controller interfaces.

## Current Milestone

- Deliver `Foundation v0.1` without real hardware: reproducible Git/CI, pure C++ core, deterministic simulation, SocketCAN/vcan integration, thin ros2_control adapter and a simulated end-to-end demo.
- Recovery baseline: project-memory points to local FND-001; the verified Foundation handoff remains historical context, not authorization for remote or hardware actions.

## P0 — Next Critical Tasks

- [x] GOV-001: Establish the AI collaboration continuity baseline.
  - Outcome: Added root `AGENTS.md`, the detailed SOP and `manifests/ai_skills.yaml`; project memory is mandatory and automatically updated/validated after every new project task, while handoff remains event-triggered and is not created for ordinary task completion.
  - Validation: Initial Markdown/YAML, remote-ref and memory/handoff checks passed on 2026-08-03; rule reconciliation and cross-document search completed on 2026-08-06, with the current memory validator run recorded below.

- [x] GOV-002: Reconcile automatic project-memory checkpoints and event-triggered handoffs.
  - Outcome: User-confirmed policy is reflected in `AGENTS.md`, the collaboration SOP, planning references and the approved skill manifest; implementation remains gated on explicit user confirmation.
  - Validation: Cross-document rule search, Markdown table smoke and YAML parse passed; `validate_memory.py` returned errors=0 and warnings=0 on 2026-08-06.

- [x] FND-000: Decide repository and asset policy.
  - Purpose: Prevent the first commit from embedding wrong ownership or supplier assets.
  - Scope: repo name/private remote, code license, binary/PDF/archive policy, `memory/` tracking and branch protection timing.
  - Outcome: D1–D5 all accepted on 2026-08-06; GitHub target is confirmed as `Makoto20S/jetson_mech_control`, and baseline commit/push authorization was supplied.
  - Validation: FND-000 status/confirmation record, asset inventory and Markdown structure reviewed; checkpoint validator pending.
  - Dependencies: None for local FND-001; creating the currently absent private GitHub repository remains a separately authorized external action.

- [ ] FND-001: Initialize the private Git repository and planning baseline.
  - Purpose: Create a reliable source of truth before implementation.
  - Scope: establish `main`, `.gitignore`, README/AGENTS, asset/source manifests and the reviewed `manifests/ai_skills.yaml`; create the authorized baseline commit, then create/push the confirmed private remote only with explicit external-action authorization.
  - Completion criteria: clean clone contains plans/memory/AI workflow and pinned skill provenance but not `CubeMars/`, `.codex/`, `tmp/`, binaries or build output.
  - Progress: Local `main`, reviewed index, repository-level author identity and `origin` are ready; commit/push are authorized, but authenticated API verification shows the target GitHub repository has not been created.
  - Validation: `git status`, branch, HEAD, remote and staged-path review.
  - Dependencies: FND-000.

- [ ] FND-002/FND-003: Establish Ubuntu 22.04/Humble build manifest, five-package workspace and CI.
  - Purpose: Make every later interface change reproducibly buildable.
  - Scope: `mech_control_core`, `mech_simulation`, `mech_hardware_ros2_control`, `mech_controllers`, `mech_bringup`; rosdep/colcon/lint/unit jobs and portable context validation.
  - Completion criteria: clean checkout builds/tests in a pinned Ubuntu/Humble environment and validates required memory/AI entry files without user-specific absolute paths.
  - Validation: actual CI logs and `colcon test-result --verbose`.
  - Dependencies: FND-000/FND-001.

- [ ] FND-004: Convert core architecture decisions into ADRs.
  - Purpose: Freeze ownership and interface direction before writing runtime code.
  - Scope: ADR-001/002/003/004/005/006/009 with status, consequences, validation and triggers.
  - Completion criteria: Each ADR is Accepted or explicitly Proposed; no hidden default remains.
  - Validation: ADR lint/review.
  - Dependencies: FND-001.

- [ ] FND-005 through FND-009: Implement and test pure C++ contracts and deterministic BusRuntime simulation.
  - Purpose: Build the vendor-independent core.
  - Scope: frame/time/status, schema/capability, fake clock/transport, router/filter conflicts, snapshots, command lease and single-writer runtime.
  - Completion criteria: no-sleep tests deterministically cover stale/TTL/drop/duplicate/reorder/queue/fault behavior.
  - Validation: unit/property/sanitizer tests without ROS or hardware.
  - Dependencies: FND-002/FND-003/FND-004.

- [ ] RSP-001/FND-010: Decide and implement SocketCAN reuse boundary.
  - Purpose: Reuse mature pieces without putting DDS or foreign thread ownership in the control loop.
  - Scope: compare `ros2_socketcan` low-level APIs with minimal Linux RAW SocketCAN; implement chosen non-blocking adapter and vcan tests.
  - Completion criteria: license, filter, error frame, timestamp, allocation/thread and ARM64 criteria are evidenced in an ADR.
  - Validation: vcan integration and ARM64 build.
  - Dependencies: FND-004/FND-005/FND-009.

- [ ] FND-011 through FND-015: Complete simulated ros2_control path and Foundation RC.
  - Purpose: Prove the complete controller-to-bus-to-state architecture before vendor adapters.
  - Scope: loopback profile/device, composite SystemInterface, bounded demo controller, lifecycle/switch/fault tests, 500 Hz/1 kHz measurements and AdapterContract v1.
  - Completion criteria: every Definition of Done item in `07_framework_bootstrap_plan.md` section 11 has actual evidence.
  - Validation: CI, 30 min simulation, lifecycle/switch repetitions, sanitizers and ARM64 clean build.
  - Dependencies: FND-006/FND-009/FND-010.

## P1 — Important Tasks

- [ ] Freeze INT-001 adapter template, then split CubeMars and HI12 work.
  - Purpose: Let device owners work in parallel without changing core/controller semantics.
  - Scope: codec/session package template, golden tests, config/capability, simulator and registry contract.
  - Completion criteria: A reference adapter can be copied and renamed without modifying core public APIs.
  - Validation: template package build/test and architecture dependency check.
  - Dependencies: `v0.1.0-foundation` RC.

- [ ] Capture read-only identity/configuration evidence for both AKE60-8 units and both HI12 devices.
  - Purpose: Bind vendor adapters to actual firmware/configuration.
  - Scope: motor version/CAN/AppParams/McParams; HI12 PNAME/APP_VER/protocol/node/bitrate/profile.
  - Completion criteria: each device has a versioned evidence record; remaining questions are exact and supplier-addressable.
  - Validation: hashes, matched documentation and later passive candump.
  - Dependencies: Physical access; does not block Foundation.

- [ ] Implement CubeMars and HI12 adapters with evidence-backed profiles.
  - Purpose: Add real devices through the frozen adapter boundary.
  - Scope: AK V3 servo/force and selected HI12 protocol codec/session/golden/simulator.
  - Completion criteria: adapter tests pass without changing compatible controllers or core semantic contracts.
  - Validation: golden, fuzz, vcan/simulation, then gated G0~G2 evidence.
  - Dependencies: INT-001 and device evidence.

- [ ] Decide the initial single-`can0` deployment profile.
  - Purpose: Confirm two motors plus two HI12 can coexist.
  - Scope: common bitrate, IDs, termination, load, response latency and error coupling.
  - Completion criteria: ADR accepts single bus or requires second interface/profile change.
  - Validation: static budget, G1 passive capture and bus statistics.
  - Dependencies: all device configs/adapters.

- [ ] Run gated hardware bring-up and 500 Hz performance validation.
  - Purpose: Verify real device behavior and the current two-motor operating target.
  - Scope: G0 identity, G1 passive bus, G2 read-only motor feedback, G3 constrained command/measurement, timing/load/freshness statistics.
  - Completion criteria: Planning acceptance metrics are met with archived raw evidence.
  - Validation: 30-minute nominal/stress runs, candump, timing histograms and fault matrix.
  - Dependencies: Hardware authorization and completed safety/measurement gates.

## P2 — Later Improvements

- [ ] Compare the 1 kHz controller_manager profile with the 500 Hz baseline.
  - Purpose: Enable 1 kHz only when measured control benefit and timing margin justify it.
  - Completion criteria: An ADR records controller, motor-I/O and bus-rate choices from nominal/stress data.
  - Validation: Actual `dt`, command-to-wire, state age, CAN load and closed-loop comparison.
  - Dependencies: Stable 500 Hz system.

- [ ] Extend to additional motor/sensor brands and the future STM32 node.
  - Purpose: Exercise the planned codec/session/capability extension boundary.
  - Completion criteria: New devices work through adapters without changing compatible C++ controllers.
  - Validation: Protocol golden frames, simulator/vcan tests and gated hardware evidence.
  - Dependencies: Device specifications and stable core APIs.

## Dependencies

- FND-000 project decisions, GitHub target and commit/push authorization are confirmed; local `main`, staged-path review and `origin` configuration are complete. Creating the absent private repository remains the only remote setup decision.
- FND-001 must review and track the prepared approved-source manifest; until a clone contains it, the repository SOP remains the portable fallback.
- Ubuntu 22.04/ROS 2 Humble build/CI environment for all implementation evidence.
- Current CubeMars documentation is reviewed; live motor/HI12 evidence is required only for P1 integration.
- Approved CAN topology, termination and G0~G3 only for hardware stages.

## Risks

- New machines or AI environments may lack the two skills; the approved repositories have no tags and moving `main` may differ from the manifest, so installations must report drift and retain the documented fallback.
- Initial Git commit may accidentally include the nested `CubeMars/` repository or vendor binaries if ignore/asset policy is late.
- Premature vendor packages or generic interfaces may create abstractions with no tested consumer.
- Windows editing results may be mistaken for Ubuntu/Humble build evidence.
- `ros2_socketcan` reuse may conflict with the required single-writer/thread/timestamp boundary.
- Custom AKE60-8 firmware may differ from AK3.0 V3.2 in frame format, scaling, encoder source or watchdog behavior.
- Both HI12 units may use the same default node ID or a bitrate incompatible with the motors.
- Standard effort has an AKE60-8 candidate (`Kt = 0.7382 N*m/A`) but may be wrong if the custom units changed the relevant parameters.
- Enabling optional `0x2A` at full motor feedback rate may push the current single-bus profile beyond the 50% average-load target.
- Single-bus load or fault coupling may force a second CAN interface.

## Open Questions

- Whether Codex should create the confirmed `Makoto20S/jetson_mech_control` private GitHub repository now; NAS mirror target and concrete reviewers remain undecided.
- Portable installer/CI wrapper and whether onboarding should verify installed skill contents against the manifest commits.
- Exact pinned Ubuntu/Humble container/runner and ARM64 CI path.
- Whether RSP-001 selects a wrapped `ros2_socketcan` low-level API or a minimal direct RAW SocketCAN adapter.
- Exact custom identifiers, driver hardware, firmware and current AK3.0 command profile of both AKE60-8 units.
- CAN visibility of both encoders and which source supplies `0x29/0x2A` position.
- Whether standard AKE60-8 Kt/ranges remain valid for the custom units.
- Stable motor feedback rate, command watchdog and neutral behavior.
- Both HI12 protocols, IDs, bitrates and output profiles.
- Measured Jetson 500 Hz/1 kHz timing and command-to-wire behavior.

## Deferred

- Six-motor real integration, final two-bus topology, STM32 firmware/AFE, learning control, automatic firmware updates and safety certification remain outside the current milestone.
