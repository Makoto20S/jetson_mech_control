#!/usr/bin/env bash
# Installed as root-level servo-range / servo-status / servo-control / servo-move symlinks.
set -euo pipefail
mode=$(basename -- "$0")
root=$(cd -- "$(dirname -- "$0")" && pwd)
case "${1:-}" in
  -h|--help|help)
    cat <<'HELP'
双电机伺服工具（104 / 105）
  ./servo-range               手动右端、左端标定；默认向内预留 10 度
  ./servo-range --margin-deg 10
  ./servo-status              实时显示角度、ERPM、电流、温度和反馈状态
  ./servo-status --duration 10 限时监视
  ./servo-move                交互输入两台电机目标角度与移动时间，显示位置反馈
  ./servo-move check          离线校验运动配置，不打开串口
  ./servo-control             使用已标定限位加载框架，轨迹控制器保持 inactive
  ./servo-status check        离线校验配置和实际插件解析，不打开串口
  ./servo-control check       离线校验已标定运动配置，不打开串口
配置：config/servo_pair.json。标定和监视不发送电机运动命令。
一次运行一个入口；运行前退出已有电机控制程序。
HELP
    exit 0 ;;
esac
release="${root}/servo-current"
config="${root}/config/servo_pair.json"
if [[ ! -f "${release}/output/install/setup.bash" ]]; then
  echo "缺少部署 servo-current/output/install/setup.bash" >&2; exit 2
fi
if [[ ! -f "$config" ]]; then
  echo "缺少配置 $config" >&2; exit 2
fi
# This lock also prevents replacing bounds while a control launch uses them.
exec 9>"${root}/.servo-tools.lock"
if ! flock -n 9; then
  echo "另一个伺服工具正在运行，请先退出。" >&2; exit 2
fi
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
set +u
source /opt/ros/humble/setup.bash
source "${release}/output/install/setup.bash"
set -u
operator="${release}/src/tools/servo/operator.py"
reader="${release}/output/install/mech_bringup/lib/mech_bringup/servo_feedback_reader"
if [[ "${1:-}" == check && "$mode" != servo-control && "$mode" != servo-move ]]; then
  shift
  exec /usr/bin/python3 "$operator" check --config "$config" --reader "$reader" "$@"
fi
case "$mode" in
  servo-range)
    exec /usr/bin/python3 "$operator" calibrate --config "$config" --reader "$reader" "$@" ;;
  servo-status)
    exec /usr/bin/python3 "$operator" monitor --config "$config" --reader "$reader" "$@" ;;
  servo-move)
    if [[ "${1:-}" == check ]]; then
      shift
      exec /usr/bin/python3 "${release}/src/tools/servo/motion.py" --config "$config" --run-dir "${root}/runs/servo" --check "$@"
    fi
    exec /usr/bin/python3 "${release}/src/tools/servo/motion.py" --config "$config" --run-dir "${root}/runs/servo" "$@" ;;
  servo-control)
    if [[ $# != 0 && "${1:-}" != check ]]; then echo "servo-control 不接受运动参数；使用 --help 查看说明。" >&2; exit 2; fi
    mkdir -p "${root}/runs/servo"
    urdf=$(mktemp "${root}/runs/servo/control-XXXXXXXX.urdf")
    /usr/bin/python3 "$operator" generate-urdf --config "$config" --output "$urdf"
    "$reader" --urdf "$urdf" --check
    if [[ "${1:-}" == check ]]; then exit 0; fi
    exec ros2 launch "${release}/src/tools/servo/servo_pair.launch.py" urdf:="$urdf" ;;
  *) echo "请通过 servo-range、servo-status、servo-control 或 servo-move 运行。" >&2; exit 2 ;;
esac
