#!/usr/bin/env bash
# Start only the CtrBoard receive path, knee estimator, and dashboard.
set -euo pipefail

device_path="${1:-/dev/ttyACM0}"
ros_distro="${ROS_DISTRO:-humble}"
if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "ERROR: ROS 2 ${ros_distro} is not installed" >&2
  exit 3
fi
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ros_distro}/setup.bash"
if [[ -n "${MECH_INSTALL_SETUP:-}" ]]; then
  # shellcheck disable=SC1090
  source "${MECH_INSTALL_SETUP}"
fi
set -u
for package in mech_ctrboard_bridge mech_knee_kinematics; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "ERROR: ${package} is not built in the current ROS environment" >&2
    exit 3
  fi
done
if [[ ! -c "${device_path}" || ! -r "${device_path}" || ! -w "${device_path}" ]]; then
  echo "ERROR: CAN gateway channel is missing or inaccessible: ${device_path}" >&2
  exit 3
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: no graphical DISPLAY; start this on the Jetson desktop" >&2
  exit 3
fi
if ros2 node list 2>/dev/null | grep -qx '/stm32_ctrboard'; then
  echo "ERROR: CtrBoard bridge is already running; do not open the CAN gateway twice" >&2
  exit 3
fi

bridge_pid=""
knee_pid=""
cleanup() {
  for pid in "${knee_pid}" "${bridge_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 -- "${pid}" 2>/dev/null; then
      kill -INT -- "-${pid}" 2>/dev/null || true
      for _ in $(seq 1 30); do
        if ! kill -0 -- "-${pid}" 2>/dev/null; then
          break
        fi
        sleep 0.1
      done
      if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null || true
      fi
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

setsid ros2 launch mech_ctrboard_bridge ctrboard_usb_cdc.launch.py \
  device_path:="${device_path}" &
bridge_pid=$!
for _ in $(seq 1 50); do
  if ! kill -0 "${bridge_pid}" 2>/dev/null; then
    echo "ERROR: CtrBoard bridge exited" >&2
    exit 4
  fi
  if ros2 node list 2>/dev/null | grep -qx '/stm32_ctrboard'; then
    break
  fi
  sleep 0.2
done
if ! ros2 node list 2>/dev/null | grep -qx '/stm32_ctrboard'; then
  echo "ERROR: CtrBoard bridge did not start within 10 seconds" >&2
  exit 4
fi

setsid ros2 run mech_knee_kinematics knee_kinematics_node &
knee_pid=$!
ros2 run mech_knee_kinematics ctrboard_can_dashboard
