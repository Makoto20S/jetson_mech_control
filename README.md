# jetson_mech_control

Reusable C++/ros2_control foundation for Jetson-based electromechanical
systems, with vendor-specific CAN devices isolated behind codec and session
adapters.

## Current status

The project is at the planning and repository-baseline stage. Foundation v0.1
is intentionally hardware-independent: no CAN interface is enabled, no motor
command is sent, and no Jetson system configuration is changed from this
workspace. ROS 2 Humble, vcan, timing, and ARM64 claims must be verified in the
corresponding Ubuntu/ARM64 environment.

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

## Planned Foundation packages

The first implementation packages are `mech_control_core`, `mech_simulation`,
`mech_hardware_ros2_control`, `mech_controllers`, and `mech_bringup`. CubeMars,
HI12, and other vendor adapters are deferred until the core adapter contract is
frozen and device evidence is available.
