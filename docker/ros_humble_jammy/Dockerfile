# Foundation v0.1 CI/build image. The multi-architecture manifest digest is
# recorded in manifests/dependencies.json; do not replace it with a floating tag.
FROM ros:humble-ros-base-jammy@sha256:7bea3d9aa2483d3ca34c8e30d921b79273b0913bd7dc64bebf51d082b5d107e4 AS dependencies

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

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

# Only dependency manifests invalidate this layer; source/docs arrive later.
COPY ros2_ws/src/mech_control_core/package.xml /opt/mech-dependencies/mech_control_core/package.xml
COPY ros2_ws/src/mech_simulation/package.xml /opt/mech-dependencies/mech_simulation/package.xml
COPY ros2_ws/src/mech_hardware_ros2_control/package.xml /opt/mech-dependencies/mech_hardware_ros2_control/package.xml
COPY ros2_ws/src/mech_controllers/package.xml /opt/mech-dependencies/mech_controllers/package.xml
COPY ros2_ws/src/mech_protocol_cubemars/package.xml /opt/mech-dependencies/mech_protocol_cubemars/package.xml
COPY ros2_ws/src/mech_bringup/package.xml /opt/mech-dependencies/mech_bringup/package.xml
RUN apt-get update \
    && rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /opt/mech-dependencies --ignore-src \
        --rosdistro "${ROS_DISTRO}" -r -y \
    && rm -rf /var/lib/apt/lists/*

FROM dependencies AS test-source
WORKDIR /workspace
COPY . /workspace

# Sanitizers build/test this source-only target inside the diagnostic container.
FROM test-source AS workspace
ARG MECH_BUILD_ONLY=0
RUN MECH_SKIP_ROSDEP=1 MECH_BUILD_ONLY="${MECH_BUILD_ONLY}" \
    bash /workspace/tools/ci/build_workspace.sh
