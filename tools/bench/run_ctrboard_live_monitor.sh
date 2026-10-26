#!/usr/bin/env bash
# Start the receive-only CtrBoard bridge and terminal monitor as one workflow.
set -euo pipefail

device_path="${1:-/dev/ttyACM0}"
duration_s="${2:-0}"
ros_distro="${ROS_DISTRO:-humble}"

if [[ ! "${duration_s}" =~ ^[0-9]+$ ]]; then
  echo "ERROR: duration_s must be zero or a whole number of seconds" >&2
  exit 2
fi
if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "ERROR: ROS 2 setup is missing: /opt/ros/${ros_distro}/setup.bash" >&2
  exit 3
fi

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ros_distro}/setup.bash"
if [[ -n "${MECH_INSTALL_SETUP:-}" ]]; then
  if [[ ! -f "${MECH_INSTALL_SETUP}" ]]; then
    echo "ERROR: MECH_INSTALL_SETUP does not exist: ${MECH_INSTALL_SETUP}" >&2
    exit 3
  fi
  # shellcheck disable=SC1090
  source "${MECH_INSTALL_SETUP}"
fi
set -u

readonly script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly monitor="${script_dir}/watch_ctrboard_sensors.py"

if ! ros2 pkg prefix mech_ctrboard_bridge >/dev/null 2>&1; then
  echo "ERROR: mech_ctrboard_bridge is not in the ROS environment" >&2
  echo "Set MECH_INSTALL_SETUP to the workspace install/setup.bash path." >&2
  exit 3
fi
if [[ ! -f "${monitor}" ]]; then
  echo "ERROR: monitor script is missing: ${monitor}" >&2
  exit 3
fi
if [[ ! -c "${device_path}" || ! -r "${device_path}" || ! -w "${device_path}" ]]; then
  echo "ERROR: device is missing or inaccessible: ${device_path}" >&2
  exit 3
fi
if ros2 node list 2>/dev/null | grep -qx '/stm32_ctrboard'; then
  echo "ERROR: /stm32_ctrboard is already running" >&2
  exit 3
fi

launch_pid=""
wait_for_group_exit() {
  local process_group="$1"
  local attempts="$2"
  for _ in $(seq 1 "${attempts}"); do
    if ! kill -0 -- "-${process_group}" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

cleanup() {
  if [[ -z "${launch_pid}" ]]; then
    return
  fi
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    if ! wait_for_group_exit "${launch_pid}" 50; then
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
      if ! wait_for_group_exit "${launch_pid}" 20; then
        kill -KILL -- "-${launch_pid}" 2>/dev/null || true
      fi
    fi
  fi
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

launch_log="$(mktemp /tmp/ctrboard-live-launch.XXXXXX.log)"
setsid ros2 launch mech_ctrboard_bridge ctrboard_usb_cdc.launch.py \
  device_path:="${device_path}" >"${launch_log}" 2>&1 &
launch_pid=$!

ready=0
for _ in $(seq 1 50); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "ERROR: STM32 bridge exited during startup" >&2
    sed -n '1,120p' "${launch_log}" >&2
    exit 4
  fi
  if ros2 node list 2>/dev/null | grep -qx '/stm32_ctrboard'; then
    ready=1
    break
  fi
  sleep 0.2
done
if [[ "${ready}" -ne 1 ]]; then
  echo "ERROR: /stm32_ctrboard was not discovered within 10 seconds" >&2
  sed -n '1,120p' "${launch_log}" >&2
  exit 4
fi

echo "Live CtrBoard data. Press Ctrl+C to stop."
if [[ "${duration_s}" -eq 0 ]]; then
  python3 "${monitor}"
else
  set +e
  timeout --signal=INT --kill-after=2s "${duration_s}s" python3 "${monitor}"
  monitor_status=$?
  set -e
  if [[ "${monitor_status}" -ne 0 && "${monitor_status}" -ne 124 &&
        "${monitor_status}" -ne 130 ]]; then
    exit "${monitor_status}"
  fi
fi
