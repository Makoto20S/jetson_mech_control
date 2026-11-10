#!/usr/bin/env bash
# Build/test or explicitly launch the receive-only state node. No devices or commands.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
# The SSH sync entry holds this same advisory lock during remote builds.
mkdir -p "$root/.local"
if [[ "${EXO_SYNC_LOCK_HELD:-0}" != 1 ]]; then
  exec 9>"$root/.local/sync.lock"
  flock -n 9 || { echo 'Another sync/build/test is active.' >&2; exit 2; }
fi
test ! -e "$root/.local/sync/incomplete.json" || { echo 'An interrupted sync needs recovery before building.' >&2; exit 2; }
action="${1:-build}"
profile="${2:-imu}"
case "$action" in build|test|run-state) ;; *) echo 'Usage: ros_workspace.sh build|test|run-state imu|state|servo_imu|all [launch arguments]' >&2; exit 2;; esac
case "$profile" in
  state) selection=(--packages-up-to mech_exo_state);;
  imu) selection=(--packages-up-to mech_ctrboard_bridge);;
  servo_imu) selection=(--packages-up-to mech_bringup mech_ctrboard_bridge mech_exo_state);;
  all) selection=();;
  *) echo 'Unknown profile' >&2; exit 2;;
esac
test -f /opt/ros/humble/setup.bash || { echo 'Install ROS 2 Humble on Ubuntu 22.04 first.' >&2; exit 2; }
# Do not inherit an old robot release overlay from the invoking terminal.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
if [[ "$action" == run-state && "$profile" == imu ]]; then
  echo 'run-state requires state, servo_imu or all profile' >&2; exit 2
fi
if [[ "$profile" != imu ]]; then
  source "$root/tools/project/mujoco_ros_env.sh"
fi
out="$root/.local/ros/$profile"
export PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR="$out/ros_logs"
export COLCON_HOME="$out/colcon_home" XDG_CACHE_HOME="$root/.local/cache"
export TMPDIR="$root/.local/tmp"
mkdir -p "$out" "$ROS_LOG_DIR" "$COLCON_HOME" "$XDG_CACHE_HOME" "$TMPDIR"
cd "$root"
if [[ "$action" == build ]]; then
  colcon --log-base "$out/log" build --base-paths "$root/ros2_ws/src" \
    --build-base "$out/build" --install-base "$out/install" \
    --executor sequential "${selection[@]}" --cmake-args \
    -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=RelWithDebInfo
else
  test -f "$out/install/local_setup.bash" || { echo 'Build this profile first.' >&2; exit 2; }
  source "$out/install/local_setup.bash"
  if [[ "$action" == run-state ]]; then
    shift 2
    exec ros2 launch mech_exo_state ph_exo_state.launch.py "$@"
  fi
  colcon --log-base "$out/log" test --base-paths "$root/ros2_ws/src" \
    --build-base "$out/build" --install-base "$out/install" \
    --executor sequential "${selection[@]}" --return-code-on-test-failure
  colcon test-result --test-result-base "$out/build" --verbose
fi
