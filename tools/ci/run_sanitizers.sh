#!/usr/bin/env bash
set -euo pipefail

OUTPUT_ROOT="${1:-/tmp/mech-foundation-sanitizers}"
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
DETECT_LEAKS="${MECH_ASAN_DETECT_LEAKS:-0}"

if [[ "${DETECT_LEAKS}" != "0" && "${DETECT_LEAKS}" != "1" ]]; then
  echo "MECH_ASAN_DETECT_LEAKS must be 0 or 1" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
SYSTEM_PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

env PATH="${SYSTEM_PATH}" colcon --log-base "${OUTPUT_ROOT}/log" build \
  --base-paths "${REPO_ROOT}/ros2_ws/src" \
  --build-base "${OUTPUT_ROOT}/build" \
  --install-base "${OUTPUT_ROOT}/install" \
  --merge-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_CXX_FLAGS=-fsanitize=address,undefined\ -fno-omit-frame-pointer \
    -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined \
    -DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined

ASAN_OPTIONS="detect_leaks=${DETECT_LEAKS}:halt_on_error=1" \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
PATH="${SYSTEM_PATH}" \
colcon --log-base "${OUTPUT_ROOT}/log" test \
  --base-paths "${REPO_ROOT}/ros2_ws/src" \
  --build-base "${OUTPUT_ROOT}/build" \
  --install-base "${OUTPUT_ROOT}/install" \
  --merge-install \
  --event-handlers console_direct+

env PATH="${SYSTEM_PATH}" colcon test-result \
  --test-result-base "${OUTPUT_ROOT}/build" --verbose
