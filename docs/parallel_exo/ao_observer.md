# PH-EXO：ContraKin AO 观察节点（EXO-AO-001）

用户选择 ContraKin，作为 PH-EXO 独立步态周期估计候选。正式实现位于既有 `mech_exo_state`，没有新增控制器或电机出口。

## 边界与数据

`sensor_msgs/Imu.angular_velocity`（rad/s）→ `ao_core.py` → `GaitPhase`。默认输入 `/imu/imu1/data` 的X轴，输出 `/ph_exo/gait_phase`。输入对应哪条大腿必须由现场确认；原始步态CSV的左右标签不自动证明在线 imu1/imu2 的安装归属。

`valid` 表示运动幅值、模型残差和频率稳定性门控通过；`phase_reference_verified` 恒为 false。内部振荡相位未经触地或解剖坐标校准，不能作为已验收的步态相位，也不能单凭valid启动助力。当前无任何目标力矩发布。

标准库数值实现衍生自 ContraKin-RT commit `6ff1be354055674c926d60b3e2ce4d6810a8b3c3` 的 `src/ao_estimator.py`。来源 https://github.com/ldevillez/ContraKin-RT 。原方程的近零分母保护移到除法之前；增加5ms内部子步、频率0.2–3Hz限制、显式rad/s归一化、有限性与时间检查、4096项窗口上限和状态门控。原实现与本保护版的初始化和积分不同，历史基准不能直接代表保护版性能。

LGPL-2.1许可随 `licenses/ContraKin-LGPL-2.1.txt` 保存，衍生文件有来源与修改声明；该许可证条目仅说明AO衍生模块，不改变其他模块授权。

## 启动（只订阅，不启动设备）

先按项目流程Windows验证、同步，然后 `Sync-Orin.ps1 build -Profile state`。在Orin开发副本加载 `/opt/ros/humble/setup.bash` 与 `.local/ros/state/install/local_setup.bash`：

```bash
ros2 launch mech_exo_state ao_observer.launch.py \
  input_topic:=/imu/imu1/data output_topic:=/ph_exo/gait_phase \
  axis:=x input_scale_rad_s:=1.0 input_offset_rad_s:=0.0
```

scale/offset须来自明确的预校准，默认1/0只用于软件探针，不代表穿戴标定。节点参数启动后冻结，修改需重启。`ao_node.py`还支持启动参数 `timeout_s`（默认0.1，允许0.05–1秒）。核心最大相邻时间间隔默认0.05秒；超出会重置学习并等待重新收敛。

节点每个新输入更新一次，推荐100Hz输入；回调发布估计，10ms定时器另发布新鲜度/状态，因此100Hz输入时输出约200Hz，重复发布不代表新采样。QoS为BEST_EFFORT、depth=1。接收重复源时间不增加accepted_samples，不刷新接收年龄；过期/未来源时间、不可用角速度、非有限值均无效。输入断流由单调接收年龄与源时间年龄双检查。核心保持输入时钟域，超时用最后源时间加单调接收时差检查。

STOPPED窗口约0.6秒，重锁至少3秒稳定运动并结合残差/频率条件；这些是估计门控，不是机器人保护停机期限。硬件独立保护不由本节点替代。

## 可复现软件验证

- Windows：给PYTHONPATH加 `ros2_ws/src/mech_exo_state/scripts`，运行 `python ros2_ws/src/mech_exo_state/test/test_ao_core.py`。
- Orin：`Sync-Orin.ps1 test -Profile state`；在隔离 `ROS_DOMAIN_ID`、`ROS_LOCALHOST_ONLY=1` 和项目同步锁下运行：

```bash
python3 -B tools/project/ao_software_validation.py \
  --output .local/validation/<唯一标识>/observer --duration 22
```

该夹具启动独立AO进程，以合成大腿角速度检查启动、静止、重启、重复帧、断流和NaN。`--load-only --duration 90`提供独立100Hz输入负载，供与现有 `test_ref01_manager.py` 仿真闭环比较。无设备接口，所有输出按项目归档流程核验回Windows后清理Orin对应临时记录。

源码编译、合成锁定、真实IMU波形回放、Orin调度与真实步态相位精度分别验收；最后一项仍需更长、带独立步态事件标注的数据。
