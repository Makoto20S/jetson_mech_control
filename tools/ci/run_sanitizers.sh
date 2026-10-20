#!/usr/bin/env bash
set -euo pipefail

OUTPUT_ROOT="${1:-/tmp/mech-foundation-sanitizers}"
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
DETECT_LEAKS="${MECH_ASAN_DETECT_LEAKS:-0}"

if [[ "${DETECT_LEAKS}" != "0" && "${DETECT_LEAKS}" != "1" ]]; then
  echo "MECH_ASAN_DETECT_LEAKS must be 0 or 1" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble setup is missing" >&2
  exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
SYSTEM_PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# Only uninstrumented controller-manager children preload this runtime, not
# Python/colcon or every utility in the process tree.
MECH_ASAN_RUNTIME="$(env PATH="${SYSTEM_PATH}" c++ -print-file-name=libasan.so)"
if [[ ! -f "$MECH_ASAN_RUNTIME" ]]; then
  echo "ERROR: compiler ASan runtime is missing" >&2
  exit 2
fi
export MECH_ASAN_RUNTIME

env PATH="${SYSTEM_PATH}" colcon --log-base "${OUTPUT_ROOT}/log" build \
  --base-paths "${REPO_ROOT}/ros2_ws/src" \
  --build-base "${OUTPUT_ROOT}/build" \
  --install-base "${OUTPUT_ROOT}/install" \
  --merge-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS=-fsanitize=address,undefined\ -fno-omit-frame-pointer \
    -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined \
    -DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined \
    -DBUILD_SHARED_LIBS=ON

/usr/bin/python3 "${REPO_ROOT}/tools/ci/check_sanitizer_build.py" \
  --root "$REPO_ROOT" --output "$OUTPUT_ROOT"

test_exit=0
ASAN_OPTIONS="detect_leaks=${DETECT_LEAKS}:halt_on_error=1" \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
PATH="${SYSTEM_PATH}" \
colcon --log-base "${OUTPUT_ROOT}/log" test \
  --base-paths "${REPO_ROOT}/ros2_ws/src" \
  --build-base "${OUTPUT_ROOT}/build" \
  --install-base "${OUTPUT_ROOT}/install" \
  --merge-install \
  --event-handlers console_direct+ || test_exit=$?

result_exit=0
env PATH="${SYSTEM_PATH}" colcon test-result \
  --test-result-base "${OUTPUT_ROOT}/build" --verbose || result_exit=$?

# Keep both diagnostics and the original failure when colcon test itself fails.
if [[ "$test_exit" != 0 ]]; then exit "$test_exit"; fi
if [[ "$result_exit" == 0 && "${MECH_STABILITY_REPETITIONS:-0}" != "0" ]]; then
  ASAN_OPTIONS="detect_leaks=${DETECT_LEAKS}:halt_on_error=1" \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  MECH_OUTPUT_ROOT="$OUTPUT_ROOT" bash "$REPO_ROOT/tools/ci/run_stability.sh"
fi
exit "$result_exit"
