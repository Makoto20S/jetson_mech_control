# PH-EXO · 104、105 的 MIT 纯力矩接口

Orin 向控制器提交两台电机各自的目标力矩。驱动接口采用 CubeMars AK3.0 扩展帧，`sub_mode=torque`。每个控制帧都带 Kp、Kd、目标位置、目标速度和前馈力矩；本工程把前四项设为名义零，仅改变前馈力矩。位置和速度项在这组零增益下不产生跟踪力矩。电流环在驱动器内闭合，其 PI 参数不随这些帧下发；ROS 保留驱动器返回的 Iq。只启动状态接收器时，它订阅 ROS 消息，不发送电机命令。

## 控制器入口

| 用途 | ROS 接口 | 行为 |
|---|---|---|
| 零目标通信检查 | `/mit_comm/zero_effort_<ID>/target_effort` | `mit_communication.py` 支持 104、105 或双轴。每个控制器的上下限都为 0，只能发名义零。 |
| 成对给定力矩 | `MitPairEffortController` 的私有话题 `~/target`，类型 `trajectory_msgs/msg/JointTrajectory` | 一条消息、一个点、轴序固定为 `[mit_comm_motor104, mit_comm_motor105]`。`effort=[τ104, τ105]`，单位 N·m；位置、速度、加速度数组留空，`time_from_start` 为零。 |
| 逐轴小力矩台架 | `/ph_pulse_<本次标识>/pulse/target` | `mit_axis_pulse.py` 使用成对接口。仅被测轴取 ±0.10 N·m，另一轴取 0。两路在同一控制更新周期写入 `effort` 和 `command_generation`，各自经独立 USB-CAN 总线发送。 |

双轴控制器每次激活后等待新目标。发布者用 Orin 的 ROS 系统时钟填写严格递增的 `header.stamp`。轴序错误、值非有限、未来、过期或重复的目标均不刷新期限；控制器收到合法目标后按主机单调时钟判断新鲜度。台架配置的上层目标期限为 60 ms，底层命令期限 / 硬期限为 4 / 6 ms。[控制器实现](../../ros2_ws/src/mech_bringup/src/mit_pair_effort_controller.cpp) · [台架配置](../../tools/project/mit_axis_pulse.py)

## 总线与报文

| 电机 | 逻辑总线与稳定设备路径后缀 | Orin → 电机，MIT 力控 | 电机 → Orin，原始反馈 | Orin → 电机，失能 |
|---|---|---|---|---|
| 104 | 2，`-if02` | `0x0868` | `0x2968` | `0x0F68` |
| 105 | 1，`-if00` | `0x0869` | `0x2969` | `0x0F69` |

三类报文都使用 Classic CAN 扩展 ID。MIT 控制和原始反馈各有 8 字节。失能的 DLC 为 0，没有数据区。接线依据稳定设备路径，`ttyACM` 编号可能随枚举顺序变化。[总线配置](../../config/parallel_exo/servo_pair.ref01.json) · [失能编码](../../ros2_ws/src/mech_protocol_cubemars/src/ak30_servo_wire.cpp)

**下发。** 64 位按高位到低位依次为 `Kp[12] | Kd[12] | P_des[16] | V_des[12] | T_ff[12]`。当前纯力矩模式的前四个编码值固定为 `0x000、0x000、0x7FFF、0x7FF`。两轴前馈力矩使用项目负责人确认的 ±15 N·m 编码档。限幅及斜率作用在编码前，台架可用幅值为 0.10 N·m。软件速度编码档为 ±40 rad/s，启用非零速度目标前仍需逐机确认。[编码实现](../../ros2_ws/src/mech_protocol_cubemars/src/ak30_force_wire.cpp) · [设备映射](../../ros2_ws/src/mech_protocol_cubemars/src/ak30_mapping.cpp)

| 请求力矩 (N·m) | 8 字节数据区，从 DATA0 到 DATA7 | 力矩码 | 按量程反解 (N·m) |
|---:|---|---:|---:|
| 0.00 | `00 00 00 7F FF 7F F7 FF` | 2047 | −0.003663 |
| +0.10 | `00 00 00 7F FF 7F F8 0D` | 2061 | +0.098901 |
| −0.10 | `00 00 00 7F FF 7F F7 F1` | 2033 | −0.106227 |

力矩字段将目标映射到 0–4095 后向下取整，精确零点位于 2047 和 2048 之间。表中反解值只描述发送字段。力矩目标分别沿两台电机的命令坐标给出；计算机构共同偏转时按 `Δψ=(Δθ104−Δθ105)/2` 处理，104 编码器增大对 ψ 为正，105 增大对 ψ 为负。

**反馈。** 以下为 `0x29` 功能码的同一帧。状态节点以 `mit_motors[0]` 保存 104、`mit_motors[1]` 保存 105。

| 字节 | 解码 | `/ph_exo/state` 每轴字段 |
|---|---|---|
| DATA0–1 | 有符号大端 16 位 × 0.1 ° | `position_deg`，电机输出角度 |
| DATA2–3 | 有符号大端 16 位 × 10 ERPM | `erpm`，电气转速 |
| DATA4–5 | 有符号大端 16 位 × 0.01 A | `iq_a`，驱动器反馈的 q 轴电流 |
| DATA6 | 有符号 8 位，单位 °C | `temperature_c`，驱动板温度 |
| DATA7 | 无符号 8 位，原样保留 | `fault_code`，`0x00` 正常；失能操作后按对应轴的 `0x77` 核对应答 |

每轴另有 `has_sample、valid、reason、rx_sequence、host_rx_us、age_ms`。默认允许接收年龄 60 ms、双轴接收时差 30 ms。重复广播同一帧不会刷新年龄。`joints.effort` 在 MIT 状态链留空；轴端实测力矩需要独立测力。反馈帧不带目标序号、设备采样时间或执行时间。[ROS 状态字段与有效性](realtime_state.md)

## 运行、撤力与失能

- **频率。** `controller_manager` 的配置周期是 2 ms（500 Hz），原始反馈的配置周期是 20 ms，逐轴脉冲工具按 10 ms 间隔发布新目标。统计实际频率时分别计数新目标、成功写出和新的反馈帧。
- **正常撤力。** 当前脉冲依次给 1.0 s 名义零、0.34 s 单轴脉冲、1.2 s 回零观察，给力和回零斜率均为 0.50 N·m/s。核对最后一次成功 MIT 写入为零码、Iq 回到基线附近和速度回落；只有后续独立失能帧及该轴 `0x77` 应答才计为该轴确认失能。
- **保护停止。** 双轴控制器检查成对反馈和目标期限；脉冲监督另检查相对本段起始角的位移（1°）、速度（100 ERPM）、Iq（0.5 A）及板温（0 < T < 60 °C）。`~/stop` 请求锁存当前激活周期；若正在跟随，更新线程尝试给两轴各写一次名义零，随后停止续发。`~/status` 可查询控制器阶段。是否成功写出以每轴发送记录核对，退出后各发一次失能，缺失应答的轴记为未知。[保护实现](../../ros2_ws/src/mech_bringup/include/mech_bringup/mit_pair_guard.hpp)
- **观察空闲状态。** 单独启动 `/ph_exo/state` 接收器不会开启 MIT 周期发送。`header.stamp` 更新只说明状态节点在发布。判断过去一段时间下发过什么，应查看该轴的 `transport_result`、CAN ID、成功标志与数据区；判断有没有新反馈，应查看 `rx_sequence` 和 `age_ms`。零力矩、停止续发、收到失能应答是三个不同事件。

## 验证与记录

在 Windows 仓库根目录可运行以下无设备检查。

```text
python -m unittest discover -s tools/project -p test_mit_communication.py -v
python tools/project/mit_communication.py --check
python tools/project/mit_axis_pulse.py --check
```

使用开发副本的 `servo_imu` overlay 做 Orin 软件回归时，`test_mit_pair_pty.py` 使用两条私有 PTY；脉冲工具的 `--rehearse` 演练正常撤力，`--rehearse-trip` 注入位移越界。脉冲演练须指定 `--axis`、`--sign` 及 `.local/validation/` 下全新的 `--output` 目录。PTY 的幅值、虚拟应答和停止时序按软件验证记录。[本地至 Orin 工作流](local_orin.md) · [PTY 回归入口](../../tools/project/test_mit_pair_pty.py)

现场测试在可见 SSH 终端进行，核对开发 overlay、设备独占、机构支撑、无人穿戴及可断电条件。零目标工具 `mit_communication.py --execute` 支持 `--motors`、`--duration`（最长 20 s）和独立输出目录。它在 `/mit_comm`、ROS 域 173 下先做 2 s 被动检查，再激活零目标控制器；每 200 ms 异步检查接口归属，响应期限 1 s。逐轴工具 `mit_axis_pulse.py --execute` 使用独立命名空间和 ROS 域 163。两个工具均限本机通信；脉冲工具在自己的 `dynamic_joint_states` 上观察反馈，另起状态接收器时须匹配 ROS 域及 `mit_topic`。[零目标工具](../../tools/project/mit_communication.py) · [状态源配置](../../ros2_ws/src/mech_exo_state/config/ph_exo_mit_state.yaml)

每次运行生成配置、被动记录、控制器日志、逐轴收发 trace 及失能应答。零目标工具写 `summary.json`，逐轴工具写 `result.json`，原始记录为 `bus<总线>_motor<ID>.jsonl`。等待进程退出且 trace 写盘后，按[结果归档流程](local_orin.md#测试结果归档)迁回 Windows，核对完整性与实际写入结果。

历史台架结果、故障段记录和时延统计在 Windows 的 `docs/parallel_exo/reports/` 查阅。
