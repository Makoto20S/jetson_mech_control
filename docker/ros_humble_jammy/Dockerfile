# Foundation v0.1 CI/build image. The multi-architecture manifest digest is
# recorded in manifests/dependencies.json; do not replace it with a floating tag.
FROM ros:humble-ros-base-jammy@sha256:7bea3d9aa2483d3ca34c8e30d921b79273b0913bd7dc64bebf51d082b5d107e4

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

WORKDIR /workspace
COPY . /workspace

# The ros-base image intentionally contains the runtime, not the complete
# build/test toolchain. Keep this list aligned with `base_image_tools` in the
# dependency manifest so a clean image can run the shared workspace script.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep resolves only package.xml keys from the pinned Jammy/Humble base.
# The same build/test script is used by CI and matching native hosts.
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && bash /workspace/tools/ci/build_workspace.sh \
    && rm -rf /var/lib/apt/lists/*
