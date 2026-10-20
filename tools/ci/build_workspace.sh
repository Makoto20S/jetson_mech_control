#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
OUTPUT_ROOT="${MECH_OUTPUT_ROOT:-${REPO_ROOT}}"
ROSDEP_COMMAND="${ROSDEP_COMMAND:-rosdep}"
BUILD_ONLY="${MECH_BUILD_ONLY:-0}"
TEST_ONLY="${MECH_TEST_ONLY:-0}"

if [[ ! "$BUILD_ONLY" =~ ^[01]$ || ! "$TEST_ONLY" =~ ^[01]$ ||
      ( "$BUILD_ONLY" == 1 && "$TEST_ONLY" == 1 ) ]]; then
  echo "ERROR: MECH_BUILD_ONLY and MECH_TEST_ONLY must be 0/1 and mutually exclusive" >&2
  exit 2
fi

mkdir -p "${OUTPUT_ROOT}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash is missing" >&2
  echo "Use the pinned Ubuntu 22.04/ROS 2 Humble image or a matching host." >&2
  exit 2
fi

# Humble's generated setup scripts read optional variables before assigning
# defaults, so source them with nounset temporarily disabled.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
SYSTEM_PATH="/opt/ros/${ROS_DISTRO}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

for required_command in colcon; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "ERROR: required command is missing: ${required_command}" >&2
    exit 2
  fi
done

if [[ "$TEST_ONLY" == "1" ]]; then
  /usr/bin/python3 "${SCRIPT_DIR}/workspace_stamp.py" verify \
    --root "$REPO_ROOT" --output "$OUTPUT_ROOT"
fi

if [[ "$TEST_ONLY" != "1" && "${MECH_SKIP_ROSDEP:-0}" != "1" ]]; then
  if ! command -v "${ROSDEP_COMMAND}" >/dev/null 2>&1; then
    echo "ERROR: dependency resolver is missing: ${ROSDEP_COMMAND}" >&2
    echo "Set ROSDEP_COMMAND=rosdepc for a configured rosdepc host, or set" >&2
    echo "MECH_SKIP_ROSDEP=1 only on an already provisioned host." >&2
    exit 2
  fi
  "${ROSDEP_COMMAND}" install \
    --from-paths "${REPO_ROOT}/ros2_ws/src" \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    -r \
    -y
fi

if [[ "$TEST_ONLY" != "1" ]]; then
  env PATH="${SYSTEM_PATH}" colcon --log-base "${OUTPUT_ROOT}/log" build \
  --base-paths "${REPO_ROOT}/ros2_ws/src" \
  --build-base "${OUTPUT_ROOT}/build" \
  --install-base "${OUTPUT_ROOT}/install" \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DBUILD_SHARED_LIBS=ON
  /usr/bin/python3 "${SCRIPT_DIR}/workspace_stamp.py" write \
    --root "$REPO_ROOT" --output "$OUTPUT_ROOT"
fi

# CI builds the image first, then runs the same tests with SYS_NICE so the
# production controller can request SCHED_FIFO without weakening deadlines.
if [[ "$BUILD_ONLY" == "1" ]]; then
  exit 0
fi

env PATH="${SYSTEM_PATH}" colcon --log-base "${OUTPUT_ROOT}/log" test \
  --base-paths "${REPO_ROOT}/ros2_ws/src" \
  --build-base "${OUTPUT_ROOT}/build" \
  --install-base "${OUTPUT_ROOT}/install" \
  --event-handlers console_direct+

env PATH="${SYSTEM_PATH}" colcon test-result \
  --test-result-base "${OUTPUT_ROOT}/build" \
  --verbose

if [[ "${MECH_STABILITY_REPETITIONS:-0}" != "0" ]]; then
  MECH_OUTPUT_ROOT="$OUTPUT_ROOT" bash "$SCRIPT_DIR/run_stability.sh"
fi
