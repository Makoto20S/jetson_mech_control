# Project Plan

> Last updated: 2026-08-07T12:00:00+08:00

## Overall Goal

- Deliver a reusable Jetson C++/ros2_control CAN framework that controls supported motors and reads supported sensors through device-specific adapters while preserving vendor-independent controller interfaces.

## Current Milestone

- Deliver `Foundation v0.1` without real hardware. FND-000/FND-001 are committed; FND-002/FND-003 are implemented and native-Humble validated in the uncommitted working tree, with Docker/GitHub Actions execution still pending.
- The project is intentionally paused after the 2026-08-07 Memory/documentation checkpoint. Historical handoffs are context only and do not authorize remote, CAN, Jetson, or hardware actions.

## P0 — Next Critical Tasks

- [x] GOV-001/GOV-002: Establish the AI continuity and checkpoint policy.
  - Outcome: Root `AGENTS.md`, collaboration SOP, approved skill manifest, mandatory project-memory checkpoints, and event-triggered handoffs are established.
  - Validation: Documentation/YAML and Memory/handoff validation passed in the recorded governance tasks.

- [x] FND-000/FND-001: Establish repository, asset, private remote, and clean-clone baseline.
  - Outcome: D1–D5 accepted; `main` is synchronized at committed HEAD `c0f9306`, and supplier/session/build assets are excluded.
  - Validation: Staged-path, secret, whitespace, private-remote, remote-HEAD and clean-clone checks passed.

- [ ] FND-002/FND-003: Finalize the Ubuntu 22.04/Humble manifest, five-package workspace, and CI baseline.
  - Purpose: Make every later interface change reproducibly buildable.
  - Scope: Review the current uncommitted manifest, digest-pinned Docker image, workflow, five packages, build script and portable context check; execute CI/container evidence and then decide on commit/push.
  - Current outcome: Exactly five planned packages exist; official `rosdep` and explicit `ROSDEP_COMMAND=rosdepc` native WSL runs each built five packages and passed 30/30 tests; context checks pass.
  - Completion criteria: A clean committed checkout builds/tests through the pinned Docker/GitHub Actions path, required context files are validated, and reviewed files contain no host-specific paths or forbidden assets.
  - Validation remaining: Docker image build or GitHub Actions logs, followed by clean-checkout review. ARM64 is a later Foundation RC gate, not required to close this bootstrap issue.
  - Dependencies: User instruction to resume and review/commit; Docker-capable or GitHub Actions environment.

- [ ] FND-004: Convert core architecture decisions into ADRs.
  - Purpose: Freeze ownership and interface direction before runtime implementation.
  - Scope: ADR-001/002/003/004/005/006/009 with status, consequences, validation, and review triggers.
  - Completion criteria: Each ADR is Accepted or explicitly Proposed with no unresolved contradiction across planning documents.
  - Validation: Link/status/table checks and architecture review.
  - Dependencies: Resume instruction; preferably FND-002/FND-003 review complete.

- [ ] FND-005 through FND-009: Implement core types, deterministic fakes, routing, leases, snapshots, and BusRuntime.
  - Purpose: Establish the vendor-independent runtime contract before device work.
  - Completion criteria: Boundary/config/time/stale/TTL/queue/fault tests pass deterministically without ROS, sleep, CAN, or hardware.
  - Validation: Unit/property tests and sanitizer jobs.
  - Dependencies: FND-003/FND-004.

- [ ] RSP-001/FND-010: Select and implement the SocketCAN transport.
  - Purpose: Reuse only code that fits the single-writer, non-blocking, timestamped runtime boundary.
  - Completion criteria: License, filter, error-frame, timestamp, allocation/thread and ARM64 criteria are evidenced in an ADR and vcan tests.
  - Dependencies: FND-004/FND-005/FND-009.

- [ ] FND-011 through FND-015: Complete simulated ros2_control path and Foundation RC.
  - Purpose: Prove the controller-to-bus-to-state architecture before vendor adapters.
  - Completion criteria: The Foundation Definition of Done has actual simulation, lifecycle, performance, sanitizer and ARM64 evidence; AdapterContract v1 is frozen.
  - Dependencies: FND-006/FND-009/FND-010.

## P1 — Important Tasks

- [ ] Freeze INT-001 adapter template, then split CubeMars and HI12 work.
  - Completion criteria: A reference codec/session/config/simulator package can be adapted without changing core/controller public semantics.
  - Dependencies: `v0.1.0-foundation` RC.

- [ ] Capture read-only identity/configuration evidence for both AKE60-8 units and both HI12 devices.
  - Completion criteria: Each device has a versioned evidence record with exact remaining supplier questions.
  - Dependencies: Physical access; does not block Foundation.

- [ ] Implement CubeMars and HI12 adapters with evidence-backed profiles.
  - Completion criteria: Golden, negative, fuzz and simulation/vcan tests pass without changing compatible controllers or core semantics.
  - Dependencies: INT-001 and device evidence.

- [ ] Decide and validate the initial single-`can0` deployment profile through G0–G3.
  - Completion criteria: Bitrate, identifiers, termination, load, latency, fault coupling, safety and measurement evidence accept a single bus or require a second interface.
  - Dependencies: Completed adapters, device evidence and explicit hardware authorization.

## P2 — Later Improvements

- [ ] Compare the 1 kHz controller-manager profile with the measured 500 Hz baseline.
  - Completion criteria: An ADR records controller, motor-I/O and bus rates using actual timing/load/closed-loop data.
  - Dependencies: Stable 500 Hz system.

- [ ] Extend to additional motor/sensor brands and the future STM32 node.
  - Completion criteria: New devices work through adapters without changing compatible C++ controllers.
  - Dependencies: Stable core APIs and device specifications.

## Dependencies

- Use `Ubuntu-22.04` explicitly for the current x86_64 Humble build evidence; the separate `Ubuntu` WSL distribution is not provisioned for ROS.
- Official `rosdep` is the portable default. `ROSDEP_COMMAND=rosdepc` is a documented host override only after that host's mirror sources/cache are configured.
- Use a Linux filesystem output root through `MECH_OUTPUT_ROOT` for WSL builds; DrvFS output is not a reliable build path.
- Docker/GitHub Actions execution, then ARM64 and vcan environments, are required at their respective gates.
- Live CubeMars/HI12 evidence and G0–G3 are required only for later device integration.

## Risks

- The working tree is entirely uncommitted for FND-002/FND-003; broad staging could include unintended assets unless the explicit path/context review is repeated.
- Structural CI/Docker review can be mistaken for executed CI evidence; current Docker/Actions status is explicitly unrun.
- WSL distribution ambiguity or DrvFS output can create false failures/hangs.
- A host-specific `rosdepc` mirror must not silently replace official `rosdep` as the repository/CI standard.
- Premature vendor packages or generic interfaces may create untested abstractions.
- Custom AKE60-8 and HI12 identities, scaling, watchdogs, IDs and common-bus behavior remain unknown.

## Open Questions

- Whether the pinned Docker image and GitHub Actions workflow pass from a clean committed checkout.
- NAS mirror target, concrete reviewers, CODEOWNERS and branch-protection activation after runnable CI.
- Portable skill installation/content verification against manifest commits.
- Whether RSP-001 selects a wrapped `ros2_socketcan` API or minimal Linux RAW SocketCAN.
- Exact custom AKE60-8 firmware/configuration/encoder sources and both HI12 protocol/ID/bitrate profiles.
- Measured Jetson ARM64 500 Hz/1 kHz timing and command-to-wire behavior.

## Deferred

- No FND-004 or later work proceeds until the user gives a new instruction.
- Docker/CI, ARM64, vcan, SocketCAN, CAN enablement, hardware commands, six-motor integration, STM32, learning control, firmware updates and safety certification are not performed in this pause checkpoint.
