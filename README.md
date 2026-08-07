# jetson_mech_control

Reusable C++/ros2_control foundation for Jetson-based electromechanical
systems, with vendor-specific CAN devices isolated behind codec and session
adapters.

## Current status

FND-002/FND-003 are implemented in the current working tree: a pinned
Ubuntu 22.04/ROS 2 Humble build boundary, five package skeletons, and a minimal
CI/context workflow. Native x86_64 Humble builds pass all 30 skeleton tests
with both the default official `rosdep` and an explicitly selected `rosdepc`
wrapper. Docker/GitHub Actions, ARM64, vcan, timing, and hardware evidence are
still pending. No CAN interface is enabled, no motor command is sent, and no
Jetson system configuration is changed from this workspace.

Start with:

1. [`AGENTS.md`](AGENTS.md) for the repository collaboration contract.
2. [`docs/planning/README.md`](docs/planning/README.md) for the planning index.
3. [`docs/planning/07_framework_bootstrap_plan.md`](docs/planning/07_framework_bootstrap_plan.md) for the active Foundation sequence.
4. [`memory/MEMORY.md`](memory/MEMORY.md), [`memory/STATE.md`](memory/STATE.md), and [`memory/PLAN.md`](memory/PLAN.md) for recoverable project context.

## Repository and asset boundary

This is an internal, private research repository. Read [`LICENSE-or-INTERNAL-LICENSE.md`](LICENSE-or-INTERNAL-LICENSE.md) and [`manifests/assets.yaml`](manifests/assets.yaml) before adding third-party or experimental material. The independent `CubeMars/` supplier repository, raw supplier archives/documents, generated output, and large captures are intentionally excluded.

The private GitHub target is `Makoto20S/jetson_mech_control`, and the local
`origin` points to that repository. See `memory/STATE.md` for the current
commit, repository-creation, and push status.

## Foundation workspace

The current packages are `mech_control_core`, `mech_simulation`,
`mech_hardware_ros2_control`, `mech_controllers`, and `mech_bringup`. CubeMars,
HI12, and other vendor adapters are deferred until the core adapter contract is
frozen and device evidence is available.

Build and test instructions, including the optional `rosdepc` host override
and WSL output-directory guidance, are in [`ros2_ws/README.md`](ros2_ws/README.md).
