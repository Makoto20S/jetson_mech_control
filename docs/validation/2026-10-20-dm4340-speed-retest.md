# DM4340 空载速度复测（2026-10-20）

## 范围与依据

用户当次确认离体固定空载台架、/dev/ttyACM1、现场可立即断电，并确认启动要求的验证全部完成。四项 verified 参数依据用户声明设置，不构成本轮对断联停机、关节映射、板版本和位速率的独立验证。

使用既有隔离构建 `/home/nvidia/ctrboard_ros_diag_20261016/ros2_ws/install`，不是当前未提交源码的重新构建。ROS_DOMAIN_ID=42。未修改设备参数。启动出现无法启用 FIFO 实时调度警告；本次不作实时性验收。

## 执行与结果

- 初始独立状态查询：位置 2.468 rad，速度 -0.002 rad/s，状态 0。
- 启动 backend=dm4340、enable_on_activate=true、velocity_kd=2、velocity_feedforward_nm=0.4；硬件和速度控制器初始 inactive。
- `python3 tools/bench/run_dm4340_ros_motion.py --preflight` 成功。
- `python3 tools/bench/run_dm4340_ros_motion.py --execute --feedforward-five-second --operator-supervised` 返回 0：目标 +0.8 rad/s 持续 5 秒，随后零目标 2 秒。
- 位置 2.4680 -> 6.3560 rad，位移 3.8880 rad；首秒后均速 0.7738 rad/s（2001 样本），峰值 0.7937 rad/s，结束速度 -0.0024 rad/s。
- 事后 preflight：硬件 inactive、控制器 inactive。关闭 launch 成功。
- 独立 `dm4340_enable_bench /dev/ttyACM1 --status-only`：状态 0，位置 6.356 rad，速度 -0.002 rad/s，力矩反馈 0.062 Nm，MOS/转子温度 30/29 C。
- fuser 无端口占用；无 ros2_control_node 残留。

本次空载单向速度复测通过，不代表穿戴闭环、精度认证或断联停机独立验收。原始日志：本地 `/home/nvidia/dm4340_retest_20261020/`（launch.log、trial.log、final_status.log）。

## 下一步

在正式同步当前源码并完成回归前，不将本结果推广到新增力矩模式或传感器闭环。
