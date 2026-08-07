# Foundation ROS 2 workspace

The workspace contains only the five Foundation v0.1 packages planned in
`docs/planning/07_framework_bootstrap_plan.md`:

- `mech_control_core`
- `mech_simulation`
- `mech_hardware_ros2_control`
- `mech_controllers`
- `mech_bringup`

From Ubuntu 22.04 with ROS 2 Humble installed, run from the repository root:

```bash
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
bash tools/ci/build_workspace.sh
```

For a host where rosdep dependencies were already provisioned by an external
image or package manager, use `MECH_SKIP_ROSDEP=1`. CI and the pinned Docker
image leave this unset so dependency resolution remains part of the check.

The project-standard resolver is the official `rosdep`, which remains the
default in CI and Docker. On a host already configured to use the compatible
`rosdepc` mirror wrapper, select it explicitly:

```bash
rosdepc update --rosdistro humble
ROSDEP_COMMAND=rosdepc \
MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build \
bash tools/ci/build_workspace.sh
```

When the source tree is mounted from Windows into WSL, place generated output
on the Linux filesystem to avoid DrvFS metadata latency:

```bash
MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build \
bash tools/ci/build_workspace.sh
```

This workspace is hardware-independent. The build script does not enable CAN,
open a device, or modify Jetson configuration.
