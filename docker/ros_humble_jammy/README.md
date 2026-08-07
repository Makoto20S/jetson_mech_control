# Ubuntu 22.04 / ROS 2 Humble build image

This Dockerfile is the Foundation v0.1 build and unit-test boundary. Its base
image is pinned by digest and supports both `linux/amd64` and `linux/arm64`.
The manifest and platform digests are recorded in
[`../../manifests/dependencies.json`](../../manifests/dependencies.json).

Building the image runs
[`../../tools/ci/build_workspace.sh`](../../tools/ci/build_workspace.sh). The
build does not enable CAN, require a device, or modify the host configuration.
