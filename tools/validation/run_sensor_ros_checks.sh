#!/usr/bin/env bash
# Synthetic ROS graphs only: no serial port, CAN interface or motor launch.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# != 1 || ! -f "$1" ]]; then
  echo "Usage: $0 /absolute/path/to/install/setup.bash" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "$1"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-63}"
export ROS_LOCALHOST_ONLY=1

knee_pid=''
cleanup() {
  if [[ -n "$knee_pid" ]]; then
    kill -INT "$knee_pid" 2>/dev/null || true
    wait "$knee_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

python3 -m mech_knee_kinematics.node --ros-args \
  -p static_seconds:=0.5 -p motion_seconds:=1.5 -p flexion_sign:=1 &
knee_pid=$!
timeout 60s python3 "$script_dir/check_knee_ros_graph.py"
cleanup
knee_pid=''

for check in check_shadow_ros_graph.py check_gait_shadow_ros_graph.py \
             check_architecture_ros_graph.py check_exoskeleton_ros_graph.py; do
  timeout 60s python3 "$script_dir/$check"
done
