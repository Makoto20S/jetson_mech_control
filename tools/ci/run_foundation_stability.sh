#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTPUT_ROOT="${MECH_OUTPUT_ROOT:-/tmp/mech-foundation-stability}"
SECONDS_TO_RUN="${MECH_STABILITY_SECONDS:-1800}"

if ! [[ "${SECONDS_TO_RUN}" =~ ^[1-9][0-9]*$ ]]; then
  echo "MECH_STABILITY_SECONDS must be a positive integer" >&2
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
  --packages-up-to mech_bringup \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3

END_TIME=$(( $(date +%s) + SECONDS_TO_RUN ))
RUNS=0
while (( $(date +%s) < END_TIME )); do
  ctest --test-dir "${OUTPUT_ROOT}/build/mech_bringup" \
    --output-on-failure -R mech_bringup_test
  RUNS=$(( RUNS + 1 ))
done

echo "Foundation simulated stability passed: ${RUNS} runs in ${SECONDS_TO_RUN}s"
