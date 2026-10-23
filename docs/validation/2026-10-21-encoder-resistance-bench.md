# 2026-10-21 无 IMU 编码器阻力台架

## 范围与授权

EXO-ARCH-001 无 IMU 阻力入口，单台达妙离体固定空载、无人体连接。
用户当次确认现场断电人员、安全夹具、/dev/ttyACM1、板版本/CAN位速率/轴方向单位不变，
并授权 Kp=Kd=0、B=0.5 N·m·s/rad、死区0.02 rad/s、力矩≤0.2 N·m、
速度≤1 rad/s、相对位移≤0.5 rad、5秒。未修改任何电机寄存器。
电机侧超时 verified 沿用本会话用户已自行验证的声明，不是本轮独立断联测量。

## 执行与结果

- 本机临时端口 ACL 由用户授予；初始独立状态查询：status=0、位置0.073 rad、速度−0.002 rad/s。
- ARM64 当前隔离安装，ROS_DOMAIN_ID=89；使用既有力矩 backend，管理器/力矩控制器初始inactive。
- `tools/bench/run_encoder_resistance.py --execute --operator-supervised` 启用硬件、力矩控制器、应用arm，完成5秒窗口，再停用控制器及硬件。
- 循环采样2621次（包含重复取样，不代表独立反馈包数或频率）；位置范围均为0.07267109178301645 rad；速度绝对值峰值0.0073260073260073 rad/s；应用阻力目标峰值0 N·m。
- 没有超过死区的实测轴运动；runner返回1，`meaningful_resistance_command=false`。不宣称非零阻力、方向或手感验收通过，不自动重试或增大参数。
- 管理器正常退出，独立查询status=0、位置0.073 rad、速度0.002 rad/s；端口无占用。
- 原始日志：本机 `/home/nvidia/mech_attitude_validation/encoder_resistance_real/` 的 manager.log、trial.log、final_status.log；日志不提交。

## 软件补充验证

新增执行器复用统一应用节点和既有管理器服务，无直接厂商帧实现。
`check_encoder_resistance_runner.py` 使用模拟管理器验证非零耗散目标、5秒正常停用、反馈断流失败后的停用清理与一次归零；两项通过。
模拟反馈不作为真实力矩/失能证据。真实硬件最后的失能结论来自独立查询。

## 后续唯一验收点

需现场配合完成死区外的安全、缓慢输出端转动，观察非零反向目标并反馈手感。
实际力矩数值精度需要独立测量；仅凭命令或电机估算反馈不能证明0.2 N·m精度。
