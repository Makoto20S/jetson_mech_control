# ROS 2 workspace

The workspace contains the five Foundation v0.1 packages planned in
`docs/planning/07_framework_bootstrap_plan.md`, plus one protocol adapter:

- `mech_control_core`
- `mech_simulation`
- `mech_hardware_ros2_control`
- `mech_controllers`
- `mech_bringup`
- `mech_protocol_cubemars` — CubeMars AK3.0 force-control and servo-position codecs/sessions.
  The protocol package performs no device I/O; real device composition lives
  in `mech_bringup`. See [servo tools](../tools/servo/README.md).

From Ubuntu 22.04 with ROS 2 Humble installed, run from the repository root:

```bash
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
bash tools/ci/build_workspace.sh
```

For a host where rosdep dependencies were already provisioned by an external
image or package manager, use `MECH_SKIP_ROSDEP=1`. CI resolves dependencies when building the pinned Docker image and uses
`MECH_SKIP_ROSDEP=1` for the subsequent test container.

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

The build/test path uses fake devices and synthetic PTYs, not real motor ports.
Dependency resolution can install packages; it does not configure CAN or motor firmware.

## CI and reproducible container checks

From the repository root on a Linux Docker host:

```bash
docker build --shm-size=2g --build-arg MECH_BUILD_ONLY=1 \
  -f docker/ros_humble_jammy/Dockerfile -t jetson-mech-control:test .
docker run --rm --shm-size=2g \
  --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  -e MECH_SKIP_ROSDEP=1 jetson-mech-control:test \
  bash /workspace/tools/ci/build_workspace.sh
```

The first step compiles the image; only the second completes test validation.
`MECH_BUILD_ONLY=1` is an explicit build-only option, not a passing test result.
Without that option, the shared script builds and tests as before. The test
container has no device or host-directory mounts. Its scheduling permissions
allow the controller to request FIFO scheduling while keeping the production
6 ms hard deadline; shared-runner success is not a real-time guarantee.
The 2 GiB shared-memory allocation covers the production bounded trace budget.

For permanent deployments using symlink-install, build at the final source,
build and install paths. Moving only the install directory can break links.
A source clone is not a complete servo deployment: configuration, calibration
and root-level operator entrypoints require separate installation. Unified
installation remains a [planned task](../docs/planning/03_mvp_delivery_plan.md#93-后续待办完整框架安装与配置初始化).
