#!/usr/bin/env bash
set -euo pipefail

duration="${1:-12}"
device="${2:-/dev/ttyACM0}"
if ! [[ "$duration" =~ ^[0-9]+$ ]] || (( duration < 5 || duration > 120 )); then
  echo 'duration must be an integer from 5 to 120 seconds' >&2
  exit 2
fi
if [[ ! -r "$device" || ! -w "$device" ]]; then
  echo "sensor device is not readable/writable: $device" >&2
  exit 2
fi

bridge_pid=''
knee_pid=''
shadow_pid=''
cleanup() {
  for pid in "$shadow_pid" "$knee_pid" "$bridge_pid"; do
    if [[ -n "$pid" ]]; then kill -INT -- "-$pid" 2>/dev/null || true; fi
  done
  for pid in "$shadow_pid" "$knee_pid" "$bridge_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT INT TERM

setsid ros2 launch mech_ctrboard_bridge ctrboard_usb_cdc.launch.py "device_path:=$device" &
bridge_pid=$!
setsid ros2 run mech_knee_kinematics knee_kinematics_node &
knee_pid=$!
setsid ros2 run mech_knee_kinematics shadow_assist_node &
shadow_pid=$!

sleep 3
echo 'CtrBoard:'
timeout 8s ros2 topic echo --once /ctrboard/diagnostics
echo 'Knee:'
timeout 8s ros2 topic echo --once /knee/diagnostics
echo 'Shadow assist:'
timeout 8s ros2 topic echo --once /assist_shadow/diagnostics
echo 'Virtual effort:'
timeout 8s ros2 topic echo --once /assist_shadow/virtual_effort_nm
sleep "$duration"
