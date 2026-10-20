# ROS 2 workspace

The workspace contains eleven packages. See the Chinese
[code structure guide](../docs/development/code_structure.md) for ownership,
data flow and the distinction between offline and live validation.

- `mech_control_core`
- `mech_simulation`
- `mech_hardware_ros2_control`
- `mech_controllers`
- `mech_bringup`
- `mech_protocol_cubemars` — the CubeMars AK3.0 force-control adapter. It is
  offline-only: it opens no device and performs no real device access.
- `mech_protocol_damiao` — DM4340 MIT codec/session, used by the separate
  `Dm4340System` velocity/effort hardware plugin in `mech_bringup`.
- `mech_knee_kinematics` — dual-IMU calibration/estimation and sensor-only tools.
- `mech_assist_control` — selectable follow/resistance policies; preview by default.
- `mech_protocol_ctrboard` — receive-side decoder for the STM32 sensor stream.
- `mech_ctrboard_bridge` — receive-only SocketCAN/USB-CDC ROS 2 publisher for two IMUs
  and two 20-point plantar-pressure samples.

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

Building and automated testing are hardware-independent. Runtime launch files
may open devices and must be reviewed separately. The build script does not enable CAN,
open a device, or modify Jetson configuration.
