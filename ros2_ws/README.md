# ROS 2 workspace

The workspace contains the five Foundation v0.1 packages planned in
`docs/planning/07_framework_bootstrap_plan.md`, two protocol adapters, and one
receive-only sensor bridge:

- `mech_control_core`
- `mech_simulation`
- `mech_hardware_ros2_control`
- `mech_controllers`
- `mech_bringup`
- `mech_protocol_cubemars` — CubeMars AK3.0 force-control and servo-position codecs/sessions.
  The protocol package performs no device I/O; real device composition lives
  in `mech_bringup`. See [servo tools](../tools/servo/README.md).
- `mech_protocol_ctrboard` — STM32 telemetry reassembly, CRC validation, and
  sensor payload decoding without device I/O.
- `mech_ctrboard_bridge` — SocketCAN/USB-CDC ROS 2 publication for CtrBoard IMU
  and plantar-pressure data, including timeout and per-field validity diagnostics.

From Ubuntu 22.04 with ROS 2 Humble installed, run from the repository root:

```bash
source /opt/ros/humble/setup.bash
rosdep update --rosdistro humble
bash tools/ci/build_workspace.sh
```

For a host where rosdep dependencies were already provisioned by an external
image or package manager, use `MECH_SKIP_ROSDEP=1`. CI resolves dependencies when building the pinned Docker image and uses
`MECH_TEST_ONLY=1` for the subsequent test container.

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

The normal build/test job runs on native x86_64 and ARM64 GitHub-hosted runners,
using `ubuntu-24.04` and `ubuntu-24.04-arm` hosts. The compiler and ROS environment
remain inside the pinned Ubuntu 22.04/Humble image. GitHub's
[runner reference](https://docs.github.com/en/actions/reference/runners/github-hosted-runners)
lists the ARM64 labels; 22.04 host labels are being retired according to the
[official announcement](https://github.com/actions/runner-images/issues/14254).
This host update does not migrate the project to ROS Jazzy or change Jetson.

Both architectures build all eight packages and execute the software test suite.
Host and image architecture guards reject mismatches; no QEMU emulation or
Jetson/self-hosted runner is configured. Each architecture has a separate cache
key and evidence artifact. A failure on one platform does not cancel the other
platform's diagnostics. The existing x86 build check name is preserved. When
updating branch protection, select the ARM64 and sanitizer checks as well;
workflow edits alone do not change repository protection settings.

The portable CI job checks repository context and ADRs, all Git-visible Markdown
local links/heading anchors, and Python/Bash syntax before the ROS build starts.
These checks do not import scripts or launch devices. Run them locally with:

```bash
python3 tools/ci/context_check.py
python3 tools/ci/check_docs.py
python3 tools/ci/check_script_syntax.py
python3 -m unittest discover -s tools/ci -p 'test_*.py' -v
```

The link checker covers inline links/images and defined references, ignores code
examples, and does not fetch external URLs. Local targets must exist in the
Git-visible repository; links to ignored personal files fail. Heading checks
cover Markdown headings, duplicate heading suffixes and explicit HTML anchors.
Syntax checks parse files only; behavioral tests remain in the ROS test suite.

From the repository root on a Linux Docker host:

```bash
docker build --shm-size=2g --build-arg MECH_BUILD_ONLY=1 \
  -f docker/ros_humble_jammy/Dockerfile -t jetson-mech-control:test .
docker run --rm --shm-size=2g \
  --cap-add=SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  -e MECH_TEST_ONLY=1 jetson-mech-control:test \
  bash /workspace/tools/ci/build_workspace.sh
```

The first step compiles the image; only the second completes test validation.
`MECH_BUILD_ONLY=1` is an explicit build-only option, not a passing test result.
`MECH_TEST_ONLY=1` runs tests on the existing build without resolving dependencies
or compiling again. It requires a complete test-enabled build and matching input
hashes recorded by the build script; changed source/tools or missing outputs fail
before testing. Rebuild instead of deleting or editing the stamp. Build-only and
test-only modes are mutually exclusive. Without either option, the shared script
still builds and tests as before. The test
container has no device or host-directory mounts. Its scheduling permissions
allow the controller to request FIFO scheduling while keeping the production
6 ms hard deadline; shared-runner success is not a real-time guarantee.
The 2 GiB shared-memory allocation covers the production bounded trace budget.

The Docker dependency stage copies all eight package manifests and resolves rosdep
before copying source. Documentation/source edits therefore reuse dependency
layers. CI imports a BuildKit cache via a SHA-pinned builder and cache actions;
cache keys include the Dockerfile, dependency manifest, package manifests and
commit. Each export uses a fresh directory to avoid retaining unreferenced blobs.
Cache hits never replace the test stage. A cold/missing cache still builds normally;
actual timing and cross-run reuse require a Linux Docker/Actions run to measure.

Each CI build job uploads `mech-ci-evidence-<arch>-<run-id>-<attempt>` for 14 days,
including failed runs. Download it from that Actions run's Artifacts section.
It contains the plain image-build output, test console, available colcon logs,
per-package test XML/CTest reports, and ROS logs that use the configured log
directory. `collection.json` records container exit/OOM state and unavailable
directories. Collection success is not test success; the original build/test
exit status still fails the job. The container is removed only after collection
and the upload attempt. No source/build binaries or complete environment dump
are included.

`runner-platform.json` records the verified host and image platform.
`test-summary.json` lists available XML reports, actual GTest/XML skips,
unrun cases, failures and malformed/missing reports. It remains available for
partial runs; report availability alone does not prove full test completion.
The normal build collector does not request sanitizer-only proof files.

If image construction fails, only the image-build console and collection status
are available; there is no test container to copy. Optional directories can be
absent when execution stops early or tests use their own temporary log paths.
Cancellation, runner loss or the overall job timeout can prevent finalization;
artifact retention is best effort in those cases, not a complete evidence guarantee.

### Address and undefined-behavior sanitizers

The independent `ASan / UBSan build and tests` job builds all eight packages in
Debug mode and runs the same software test suite with AddressSanitizer and
UndefinedBehaviorSanitizer. It uses the pinned `test-source` Docker target and
its dependency layers, then compiles inside a separate container. It does not
reuse release binaries or mount host/device directories. Both this job and the
normal build/test job must pass; a passing release build alone is insufficient.

On an already provisioned Humble host, run:

```bash
bash tools/ci/run_sanitizers.sh /tmp/mech-sanitizers
```

The script verifies sanitizer flags in every package's CMake cache and actual
compilation commands before testing. Missing instrumentation or disabled tests
fail the run. `sanitizer-build.json` records the verified flags and compilation
counts. Test errors retain their exit status, with test-result reporting attempted
even if the test command fails. CI uploads a separate
`mech-sanitizer-evidence-<run-id>-<attempt>` artifact for 14 days, including
consoles, available XML/CTest/ROS logs and instrumentation evidence.

Leak detection remains optional (`MECH_ASAN_DETECT_LEAKS=1` on a compatible
standalone host); CI defaults to `0`, while ASan and UBSan stay enabled with
`halt_on_error=1`. The uninstrumented Humble controller manager preloads the
compiler's ASan runtime only when launched with `MECH_ASAN_RUNTIME`, supplied
by the sanitizer script. Python, colcon and spawners do not receive a global
preload. The existing upstream Humble `rclcpp` allocator
`new_delete_type_mismatch` exception applies to the manager integration tests,
including the servo manager; it does not disable other ASan checks or UBSan.

Read the XML and detailed console as well as `colcon test-result`: the installed
Humble summary can omit GTest `result="skipped"` counts. A missing vcan interface
skips its optional cases; the position-only JTC branch also has a documented
conditional skip. Sanitizer success does not establish device or real-time
performance.

For permanent deployments using symlink-install, build at the final source,
build and install paths. Moving only the install directory can break links.
A source clone is not a complete servo deployment: configuration, calibration
and root-level operator entrypoints require separate installation. Unified
installation remains a [planned task](../docs/planning/03_mvp_delivery_plan.md#93-后续待办完整框架安装与配置初始化).
