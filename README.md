# jetson_mech_control

Reusable C++/ros2_control foundation for Jetson-based electromechanical
systems, with vendor-specific CAN devices isolated behind codec and session
adapters.

## Current status

FND-000 through FND-015, RSP-001/RSP-002 and INT-001 are implemented on a
pinned Ubuntu 22.04/ROS 2 Humble build boundary. FND-004A passed a native
Jetson ARM64 smoke test. The architecture baseline is seven Accepted ADRs plus
one Proposed one, ADR-006, which gates real single-channel deployment.
ADR-012 records the command watchdog, capability-reporting and remote-frame
contract changes that came out of the RC code review; it was accepted on
2026-08-31 and constrains interface semantics only — it does not relax any
device-activation gate.

Note that `v0.1.0-foundation-rc1` tags `a056492`, which predates the RC code
review and still contains the defects listed below. It is a historical marker,
not the recommended commit.

That review found and fixed defects the RC test suite had passed straight
through, including an out-of-bounds write in the ros2_control claim tracking, a
bus runtime that latched a permanent fault on transient write backpressure, and
a USB-CDC transport that would emit a frame addressed to a different logical
bus. Timing and hardware results are still not claimed. No CAN interface is
enabled and no motor command is sent by the current Foundation work.

Start with:

1. [`AGENTS.md`](AGENTS.md) for the repository collaboration contract.
2. [`CONTRIBUTING.md`](CONTRIBUTING.md) for the shared-versus-local boundary and contribution workflow.
3. [`docs/planning/README.md`](docs/planning/README.md) for the planning index.
4. [`docs/adr/README.md`](docs/adr/README.md) for the FND-004 architecture decisions and their status.
5. [`docs/planning/07_framework_bootstrap_plan.md`](docs/planning/07_framework_bootstrap_plan.md) for the active Foundation sequence.
6. [`docs/archive/README.md`](docs/archive/README.md) only when historical, non-normative planning inputs are needed.
7. GitHub Issues/Milestones for shared task state. Developers may use an ignored local `memory/` directory through the approved `project-memory` skill.

After FND-004A passes, the exact tested commit will receive the annotated
`fnd-004a-passed` milestone tag and `main` will be protected. FND-005 and later
work must use short-lived task branches and Pull Requests; repository owners
use branches in this repository, while external contributors may use forks.

## Repository and asset boundary

This is an internal, private research repository. Read [`LICENSE-or-INTERNAL-LICENSE.md`](LICENSE-or-INTERNAL-LICENSE.md) and [`manifests/assets.yaml`](manifests/assets.yaml) before adding third-party or experimental material. The independent `CubeMars/` supplier repository, raw supplier archives/documents, generated output, and large captures are intentionally excluded.

The private GitHub target is `Makoto20S/jetson_mech_control`. Repository state
comes from Git and GitHub; personal AI/session memory is intentionally not
tracked.

## Foundation workspace

The current packages are `mech_control_core`, `mech_simulation`,
`mech_hardware_ros2_control`, `mech_controllers`, and `mech_bringup`. CubeMars,
HI12, and other vendor adapters are deferred until the core adapter contract is
frozen and device evidence is available.

Build and test instructions, including the optional `rosdepc` host override
and WSL output-directory guidance, are in [`ros2_ws/README.md`](ros2_ws/README.md).
