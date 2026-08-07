# Current Project State

> Last updated: 2026-08-07T13:26:44+08:00
> Repository: D:\Work\jetson
> Branch: main
> HEAD: current `main`; FND-002/FND-003 implementation baseline `ee4c64c`; verify the latest checkpoint with `git rev-parse HEAD`
> Working tree: clean after the FND-002/FND-003 commit and push

## Current Objective

- FND-002/FND-003 are complete: the reproducible Ubuntu 22.04/ROS 2 Humble dependency baseline, five-package workspace, context validator, shared build script, and CI/Docker path are committed and verified.

## Current Status

- FND-000/FND-001 remain complete; FND-002/FND-003 are committed and pushed at `ee4c64c`.
- The repository now contains a pinned Jammy/Humble dependency manifest, digest-pinned multi-architecture ROS base image, minimal GitHub Actions workflow, portable context validator, shared build/test script, and exactly five planned ROS 2 package skeletons.
- Ubuntu evidence comes from WSL distribution `Ubuntu-22.04`, not the separate `Ubuntu` distribution. The verified environment is Ubuntu 22.04.5 LTS, x86_64, user `makoto`, ROS 2 Humble, GCC/G++ 11.4, CMake 3.22.1, Python 3.10.12, and rosdep/rosdepc 0.26.0.
- Official `rosdep` remains the CI/Docker/default resolver. A configured host may select the compatible mirror wrapper explicitly with `ROSDEP_COMMAND=rosdepc`; both complete paths have now built and tested the workspace successfully.
- No CAN interface was enabled, no device was opened, no motor command was sent, no Jetson configuration was changed, and no CubeMars/HI12 adapter was created.

## Completed Recently

- Added `manifests/dependencies.json` and the explicit empty VCS manifest `manifests/dependencies.repos`; pinned `ros:humble-ros-base-jammy` to an immutable manifest digest and recorded amd64/arm64 platform digests.
- Added `.dockerignore`, the pinned Humble Dockerfile, and `.github/workflows/foundation.yml`; checkout actions use a full commit SHA and the workflow runs the portable context check before building the image.
- Created `mech_control_core`, `mech_simulation`, `mech_hardware_ros2_control`, `mech_controllers`, and `mech_bringup` with C++17 marker libraries, GTests, CMake lint and XML lint. Controllers depend on the vendor-independent core, not the hardware package.
- Added `tools/ci/context_check.py` and `tools/ci/build_workspace.sh`; the latter supports Linux-filesystem output via `MECH_OUTPUT_ROOT` and explicit resolver selection via `ROSDEP_COMMAND` while defaulting to `rosdep`.
- Successfully updated the mirrored dependency index with `rosdepc update --rosdistro humble`; `rosdepc check --from-paths ros2_ws/src --ignore-src --rosdistro humble` reported all system dependencies satisfied.
- Completed full native Humble build/test runs with both official `rosdep` and `ROSDEP_COMMAND=rosdepc`: each finished five packages and 30 tests with zero errors, failures, or skips.

## In Progress

- FND-002/FND-003 implementation is committed in `ee4c64c` and the completion checkpoint is on `main`; no uncommitted project changes remain.
- GitHub Actions supplied the clean-checkout context and pinned-container build evidence; local Docker execution remains unavailable.
- ARM64, vcan, SocketCAN, performance, sanitizers, and hardware validation are intentionally outside this checkpoint and remain future tasks.

## Modified Files

- `manifests/dependencies.json`, `manifests/dependencies.repos`, `.dockerignore`, `docker/ros_humble_jammy/`, `.github/workflows/foundation.yml`
  - Change: Define the pinned Ubuntu/Humble dependency and CI/container boundary.
  - Reason: Make Foundation builds reproducible without floating images or supplier assets.
  - Status: Implemented, committed in `ee4c64c`, and pushed.
  - Validation: Portable context job and pinned Humble Docker build job passed in Actions run `31150054330`.
- `ros2_ws/README.md`, `ros2_ws/src/mech_*`
  - Change: Add the five planned package skeletons, dependency boundaries, marker tests and host instructions.
  - Reason: Establish the smallest hardware-independent ROS workspace before runtime code.
  - Status: Implemented, committed in `ee4c64c`, and pushed.
  - Validation: `colcon list` found exactly five packages; two current full build/test runs passed.
- `tools/ci/build_workspace.sh`, `tools/ci/context_check.py`
  - Change: Add reusable context/build/test automation, Linux output-root support, and explicit `rosdep`/`rosdepc` selection.
  - Reason: Keep CI standard on official rosdep while supporting the configured WSL mirror path.
  - Status: Implemented, committed in `ee4c64c`, and pushed.
  - Validation: Context check and both dependency-resolver build paths passed.
- `README.md`, `docs/planning/README.md`, `docs/planning/07_framework_bootstrap_plan.md`, `memory/MEMORY.md`, `memory/STATE.md`, `memory/PLAN.md`
  - Change: Replace stale planning-only/FND-001 status with the current FND-002/FND-003 implementation and evidence boundary.
  - Reason: Preserve an accurate pause checkpoint and documented build entry.
  - Status: Updated for FND-002/FND-003 completion and committed on `main`.
  - Validation: Final text/context/diff checks and Memory validator passed.

## Validation Results

- `python tools/ci/context_check.py` on Windows and `python3 tools/ci/context_check.py` in Ubuntu previously passed: portable context, pinned dependency manifest, and five ROS package skeletons validated.
- `rosdepc update --rosdistro humble` in `Ubuntu-22.04` on 2026-08-07 completed against the configured mirror and updated `/home/makoto/.ros/rosdep/sources.cache`.
- `rosdepc check --from-paths ros2_ws/src --ignore-src --rosdistro humble` on 2026-08-07 returned “All system dependencies have been satisfied” with exit code 0.
- `ROSDEP_COMMAND=rosdepc MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build-rosdepc bash tools/ci/build_workspace.sh` on 2026-08-07: five packages built; 30 tests, 0 errors, 0 failures, 0 skipped.
- `MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build-rosdep bash tools/ci/build_workspace.sh` on 2026-08-07: official rosdep installed/resolved dependencies; five packages built; 30 tests, 0 errors, 0 failures, 0 skipped.
- A prior `MECH_SKIP_ROSDEP=1` WSL run also completed five packages and 30/30 tests, proving the build independently of resolver-index availability.
- `MECH_OUTPUT_ROOT=/tmp/jetson-mech-control-build-fnd002-final MECH_SKIP_ROSDEP=0 bash tools/ci/build_workspace.sh` in `Ubuntu-22.04` on 2026-08-07 installed official rosdep dependencies, built 5 packages, and passed 30 tests with 0 errors/failures/skips.
- GitHub Actions run `31150054330` for commit `ee4c64c`: portable context check and pinned Humble Docker build both completed successfully.
- Docker image build on the local Windows/WSL host, ARM64, vcan, CAN, performance, sanitizer and real-hardware checks: not run; later milestone constraints remain.

## Current Problems

- There are two WSL distributions: `Ubuntu` lacks ROS, while `Ubuntu-22.04` is the verified Humble environment. Future commands must name `Ubuntu-22.04` explicitly.
- Generating `build/install/log` on `/mnt/d` caused severe DrvFS I/O blocking; use `MECH_OUTPUT_ROOT` under the WSL Linux filesystem such as `/tmp`.
- Local Docker/Podman is unavailable; the remote GitHub Actions Docker job is the executed container evidence.
- NAS target, concrete CODEOWNERS reviewers, branch protection activation, and portable skill-content verification remain undecided.

## Blockers

- No blocker remains for FND-002/FND-003. FND-004 is the next planned task.
- Real motor/IMU integration remains blocked by device evidence and G0–G3, independently of Foundation.

## Unverified Assumptions

- The recorded container manifest/platform digests are the reviewed immutable baseline; amd64 Docker execution was verified by GitHub Actions, while arm64 execution remains unverified.
- The two AKE60-8 custom motors and two HI12 devices are planned to share `can0`; compatibility remains unverified.
- Actual motor firmware/configuration, encoder source, standard AKE60-8 scaling applicability, both HI12 identities, and physical bus timing remain unknown.

## Failed Approaches

- Invoking the separate WSL distribution `Ubuntu` produced false missing-ROS results; explicit `wsl -d Ubuntu-22.04` is required.
- Direct WSL output under `/mnt/d/Work/jetson` left CMake processes in uninterruptible I/O waits; Linux-filesystem output under `/tmp` completed normally.
- Earlier official rosdep updates timed out while traversing remote/future distribution indexes. The configured `rosdepc` mirror with `--rosdistro humble` completed; after that cache was populated, both resolver commands passed.

## Immediate Next Action

- Begin FND-004 by converting the approved architecture decisions into linked Accepted/Proposed ADR documents; do not start CAN or hardware work implicitly.
