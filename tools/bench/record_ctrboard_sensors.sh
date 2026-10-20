#!/usr/bin/env bash
# Start the repository's receive-only STM32 bridge and preserve a bounded
# sensor capture. The caller must source ROS 2 and the built workspace first.
set -euo pipefail

usage() {
  cat >&2 <<USAGE
usage: $(basename "$0") <output_dir> [duration_s] [device_path]

  output_dir   new rosbag2 output directory
  duration_s   whole seconds to record (default 30)
  device_path  Livelybot USB-CDC channel (default /dev/ttyACM0)
USAGE
}

output_dir="${1:-}"
duration_s="${2:-30}"
device_path="${3:-/dev/ttyACM0}"

if [[ -z "${output_dir}" ]]; then
  usage
  exit 2
fi
if [[ ! "${duration_s}" =~ ^[0-9]+$ ]] || [[ "${duration_s}" -lt 1 ]]; then
  echo "ERROR: duration_s must be a whole number of seconds >= 1" >&2
  usage
  exit 2
fi
if [[ -e "${output_dir}" ]]; then
  echo "ERROR: output path already exists: ${output_dir}" >&2
  exit 2
fi
if [[ ! -c "${device_path}" ]]; then
  echo "ERROR: USB-CDC device is not present: ${device_path}" >&2
  exit 3
fi
if [[ ! -r "${device_path}" || ! -w "${device_path}" ]]; then
  echo "ERROR: USB-CDC device is not readable and writable: ${device_path}" >&2
  exit 3
fi

for command in ros2 setsid timeout sha256sum; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "ERROR: required command is unavailable: ${command}" >&2
    exit 3
  fi
done
if ! ros2 pkg prefix mech_ctrboard_bridge >/dev/null 2>&1; then
  echo "ERROR: mech_ctrboard_bridge is not in the sourced ROS environment" >&2
  exit 3
fi
if ros2 node list 2>/dev/null | grep -qx '/stm32_ctrboard'; then
  echo "ERROR: /stm32_ctrboard is already running" >&2
  exit 3
fi

readonly output_parent="$(dirname -- "${output_dir}")"
readonly output_name="$(basename -- "${output_dir}")"
mkdir -p "${output_parent}"
output_dir="$(cd -- "${output_parent}" && pwd)/${output_name}"
readonly launch_log="${output_dir}.launch.log"
readonly record_log="${output_dir}.record.log"

launch_pid=""
bag_pid=""
wait_for_exit() {
  local pid="$1"
  local attempts="$2"
  for _ in $(seq 1 "${attempts}"); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.1
  done
  return 1
}

stop_process() {
  local pid="$1"
  if ! kill -0 "${pid}" 2>/dev/null; then
    wait "${pid}" 2>/dev/null || true
    return
  fi
  kill -INT "${pid}" 2>/dev/null || true
  if ! wait_for_exit "${pid}" 50; then
    kill -TERM "${pid}" 2>/dev/null || true
    if ! wait_for_exit "${pid}" 20; then
      kill -KILL "${pid}" 2>/dev/null || true
    fi
  fi
  wait "${pid}" 2>/dev/null || true
}

wait_for_process_group_exit() {
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

stop_process_group() {
  local process_group="$1"
  if kill -0 -- "-${process_group}" 2>/dev/null; then
    kill -INT -- "-${process_group}" 2>/dev/null || true
    if ! wait_for_process_group_exit "${process_group}" 50; then
      kill -TERM -- "-${process_group}" 2>/dev/null || true
      if ! wait_for_process_group_exit "${process_group}" 20; then
        kill -KILL -- "-${process_group}" 2>/dev/null || true
      fi
    fi
  fi
  wait "${process_group}" 2>/dev/null || true
}

cleanup() {
  if [[ -n "${bag_pid}" ]]; then
    stop_process "${bag_pid}"
  fi
  if [[ -n "${launch_pid}" ]]; then
    stop_process_group "${launch_pid}"
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch mech_ctrboard_bridge ctrboard_usb_cdc.launch.py \
  device_path:="${device_path}" >"${launch_log}" 2>&1 &
launch_pid=$!

node_ready=0
for _ in $(seq 1 50); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    echo "ERROR: STM32 bridge exited during startup" >&2
    sed -n '1,160p' "${launch_log}" >&2
    exit 4
  fi
  if ros2 node list 2>/dev/null | grep -qx '/stm32_ctrboard'; then
    node_ready=1
    break
  fi
  sleep 0.2
done
if [[ "${node_ready}" -ne 1 ]]; then
  echo "ERROR: /stm32_ctrboard was not discovered within 10 seconds" >&2
  sed -n '1,160p' "${launch_log}" >&2
  exit 4
fi

if ! timeout --signal=INT --kill-after=2s 10s \
    ros2 topic echo --once /ctrboard/timestamp_ms std_msgs/msg/UInt32 \
    >/dev/null 2>&1; then
  echo "ERROR: no live STM32 timestamp sample arrived within 10 seconds" >&2
  sed -n '1,160p' "${launch_log}" >&2
  exit 4
fi

topics=(
  /imu/imu1/data
  /imu/imu2/data
  /imu/imu1/euler_deg
  /imu/imu2/euler_deg
  /fsr/left/raw
  /fsr/right/raw
  /fsr/left/total
  /fsr/right/total
  /ctrboard/sensor_status
  /ctrboard/timestamp_ms
)

echo "Recording ${#topics[@]} STM32 sensor topics for ${duration_s}s"
ros2 bag record --storage sqlite3 -o "${output_dir}" "${topics[@]}" \
  >"${record_log}" 2>&1 &
bag_pid=$!
sleep "${duration_s}"
kill -INT "${bag_pid}" 2>/dev/null || true
if ! wait_for_exit "${bag_pid}" 100; then
  echo "ERROR: ros2 bag record did not stop within 10 seconds" >&2
  kill -TERM "${bag_pid}" 2>/dev/null || true
  wait "${bag_pid}" 2>/dev/null || true
  bag_pid=""
  exit 5
fi
if ! wait "${bag_pid}"; then
  echo "ERROR: ros2 bag record did not stop cleanly" >&2
  sed -n '1,160p' "${record_log}" >&2
  exit 5
fi
bag_pid=""

if [[ ! -f "${output_dir}/metadata.yaml" ]]; then
  echo "ERROR: rosbag2 metadata was not created" >&2
  exit 5
fi
if ! compgen -G "${output_dir}/*.db3" >/dev/null; then
  echo "ERROR: rosbag2 database was not created" >&2
  exit 5
fi

cp -- "${launch_log}" "${output_dir}/bridge.log"
cp -- "${record_log}" "${output_dir}/record.log"
ros2 bag info "${output_dir}" | tee "${output_dir}/validation.txt"
sha256sum "${output_dir}"/*.db3 >"${output_dir}/SHA256SUMS"

echo "Capture complete: ${output_dir}"
