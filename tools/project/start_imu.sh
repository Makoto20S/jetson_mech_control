#!/usr/bin/env bash
# Explicit sensor launch only; never called by restore/build/test.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
device="${1:-/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if08}"
motor_config="${2:-$root/config/parallel_exo/servo_pair.local.json}"
out="$root/.local/ros/imu"
test -f "$out/install/local_setup.bash" || { echo 'Build the imu profile first.' >&2; exit 2; }
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$out/install/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR="$out/ros_logs"
python3 -B "$root/tools/project/check_imu_device.py" "$device" "$motor_config"
exec ros2 launch mech_ctrboard_bridge ctrboard_usb_cdc.launch.py device_path:="$device"
