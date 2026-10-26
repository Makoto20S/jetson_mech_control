# DM4340 单机／双机位置阻抗控制

任务：EXO-ARCH-001。实现状态：本地软件接入；离体逐轴启停及Kp=5加力复测已执行；随后1 N·m名义预算试验触发反馈保护停机；新诊断版本逐轴9秒复测无保护触发但仅微动；随后2 N·m预算1号转约2.38°但返回残差约2.21°，2号未进入目标测试，位置跟踪未通过验收，ADR-021 保持 Proposed。

当前只保留本文的原电机端position阻抗入口；试验性[主机侧阻抗](host_impedance_stm32_port.md)
已于2026-10-22按用户要求移除，其历史试验记录不作为当前可执行接口。

## 控制方法与架构

复用已有 CubeMars 力控位置模式的架构方式，达妙使用自己的 MIT 编码与会话，
不复用 CubeMars 报文字节，不新增厂商专用上层控制器。

```text
上层位置目标（输出轴／关节 rad，按已验证映射）
  → 通用 PositionCommandController（限幅、斜率、目标时效）
  → Dm4340System（position + command_generation 成组 claim）
  → Dm4340Runtime（一个网关写者、逐轴会话、租约、整组故障停止）
  → Dm4340MitSession（映射、实际／目标角度、偏差与最终 TX 校验）
  → 达妙 MIT 电机端位置弹簧和速度阻尼
```

电机端名义控制律为 `τm = Kp × (qm_ref − qm) − Kd × vm`。
本切片只导出 **position**，速度参考与力矩前馈均为零，增益在配置时固定；
不是在线可变阻抗，也不是新的主机力矩算法。原有速度和阻力／力矩模式保留。
同一网关支持两个位置模式电机，各自目标、增益、映射和包络独立；
不支持本实例内混合命令模式，两个 Float64 目标也不提供原子同步轨迹语义。

`q = scale × qm + offset`，目标反向映射到电机坐标。
Kp 的单位是电机坐标 N·m/rad，Kd 为 N·m·s/rad；直接编码到设备。
非单位比例时不能把 Kp/Kd 当作关节刚度／阻尼直接使用：按理想虚功映射，
关节等效刚度／阻尼分别为 Kp/scale² 与 Kd/scale²，实际传动还需标定。
负 scale 支持反向映射与关节坐标边界检查。
MIT 字段有量化误差，速度参考零的编码也有量化偏差；这是名义模型，不是力矩计量结果。

## 参数与保护

| 参数 | 位置模式示例 | 作用 |
|---|---:|---|
| dm4340_command_mode | position | 启动时选择接口；不能在运行中只改 Kp 切换模式 |
| position_kp | 1 | 电机端位置弹簧刚度；必须 >0 且 ≤500 |
| position_kd | 0.2 | 电机端阻尼；必须 >0 且 ≤5 |
| max_target_error_rad | 0.3 rad（约17.2°） | 目标与最新反馈最大允许偏差，按量化后的目标复查 |
| position_min_rad / position_max_rad | −12.5 / +12.5 rad | 协议轴示例包络，必须替换为实际机构允许行程；不是人体膝角范围 |
| position_guard_margin_rad | 0.02 rad | 实际角度进入边界缓冲区时拒绝更向外的位置目标，发送前重检 |
| max_feedback_velocity_rad_s | 2 rad/s（电机坐标） | 反馈超速使会话锁存，整组停止 |
| max_feedback_torque_nm | 5 N·m（电机坐标） | 反馈力矩超界锁存；同时作为配置名义力矩预算 |
| max_command_velocity_rad_s / max_command_effort_nm | 0 / 0 | 禁止位置模式夹带其他命令；velocity_feedforward_nm 也必须为0 |
| max_slew_per_second | 0.2 rad/s | 通用控制器的位置目标变化速率，不是设备硬速度限制 |
| minimum / maximum（控制器） | −12.5 / +12.5 rad | 上层目标裁剪；硬件仍独立拒绝自身包络外目标，不裁剪后继续发送 |
| target_ttl / target_hard_ttl | 100 / 106 ms | 上游位置目标时效；相同目标也需持续刷新 |
| command_ttl / command_hard_ttl | 4 / 6 ms | 控制器到硬件命令租约；保持原保护，不因位置保持而放宽 |
| feedback_ttl / query_period | 40 / 20 ms | 反馈时效／逐轴查询周期；双机查询沿用错开发送 |
| enable_on_activate | false | 默认不自动使能；硬件和选定控制器均默认 inactive |

配置必须满足：

```text
Kp × max_target_error_rad / abs(scale)
  + Kd × max_feedback_velocity_rad_s ≤ max_feedback_torque_nm
```

默认 scale=1 时名义最坏项为 `1×0.3 + 0.2×2 = 0.7 N·m`。
这是反馈有效、控制模型成立时的保守配置预算，**不是电机内部硬力矩限流器**，
不能保证两次反馈之间没有力矩峰值，也不提供制动距离保证。
反馈越界、反馈过期、失能状态、位置目标越界／偏差过大、租约超时或传输异常均停止整组。
故障后不得自动重试；生命周期恢复与新的目标必须显式进行。
位置模式只在真实使能反馈下准备报文，最终发送前再次核对使能、最新位置偏差和向外方向。
通用控制器激活以测得位置初始化，不把零当作当前位置；没有显式上游目标不发送运动帧。
停用仍走原有零增益中性帧、失能及生命周期读回确认，不把保持位置当作失能。

## 首次反馈拒绝诊断与后续台架候选

会话在清空无效控制状态之前，固定大小地锁存首个拒绝帧；运行时复制该记录，
使停止／会话清理不会覆盖首因。下次 runtime start 才清空运行时诊断。
诊断不参与控制、不恢复命令权限，也不把被拒绝数值发布成有效关节状态。
生产插件在首次 read/write 故障时记录轴索引、电机ID、CAN ID／DLC、原因、
解码是否成功、原始电机位置／映射位置、速度、力矩、到达年龄及配置阈值。
原始帧保存在固定结构中；不在正常反馈路径逐帧写日志或扩展容器。

| reason | 含义 |
|---|---|
| route / decode | 路由不匹配，或帧格式／长度／字段解码失败；未解码量显示 NaN |
| device_status / latched_fault | 设备报告故障，或会话已有锁存故障 |
| velocity / torque | 超过对应反馈阈值；两项同时超限时主原因按原校验顺序取速度，两项标志均保留 |
| time_order / expired | 到达时间逆序／来自未来／早于激活，或帧年龄达到反馈TTL |
| position | 映射后位置非有限数或超出下层硬边界 |

状态码1表示已使能，不能单凭此码解释为何拒绝。新诊断无法恢复历史试验丢失的帧。
离线字节注入确认：速度编码2067为0.095238 rad/s，下一档2068为0.100122 rad/s，
会越过0.1 rad/s阈值；这是可复现边界行为，不证明历史试验正是该原因。

以下为新诊断版本**已执行的逐轴受限台架参数**，仅用于本次scale=1的输出轴映射；
不是仓库默认值。两轴9秒时序完成但仅微动，位置跟踪未通过；
详见[实测记录](../validation/2026-10-21-dm4340-position-impedance.md#新诊断版本的逐轴受限实机复测)。

| 参数 | 本轮值 | 目的 |
|---|---:|---|
| Kp / Kd | 10 / 1 | 保持刚度，增加速度阻尼 |
| 目标偏移 / 目标斜率 | +0.07 rad / 0.03 rad/s | 缩小步幅并降低指令推进速率 |
| 最大目标误差 / 反馈速度保护 | 0.08 rad / 0.2 rad/s | 与增益成套核定；指令斜率低于速度保护 |
| 反馈力矩保护 / 相对位移边界 | 1 N·m / ±0.2 rad | 保留反馈停机和位移保护 |
| 时间 | 原位1秒→偏移4秒→返回4秒，逐轴 | 给限斜率目标足够往返时间 |

名义预算为 `10×0.08 + 1×0.2 = 1 N·m`，静止目标弹簧项约0.7 N·m。
零速度参考、零前馈及现有4/6ms命令租约、40ms反馈时效不变。
后续重复试验仍要求核定当次现场条件和运动参数；不得自动重试、自动升级力度，
也不能以名义预算宣称有电机内部1 N·m硬限流或制动距离保证。

用户随后要求提高至2 N·m，临时配置Kp=20、Kd=2，反馈力矩保护2 N·m，
其余目标／斜率／速度／偏差／位移边界及9秒时序沿用上表，名义预算为2 N·m。
1号实际跨度2.382°、反馈力矩峰值0.814 N·m，返回仍偏离起点2.208°；
进入2号前的原位核对因此拒绝，2号未发送位置目标，两台最终独立确认失能。
这不是两轴位置验收通过，也不是实测2 N·m；
详见[2 N·m试验记录](../validation/2026-10-21-dm4340-position-impedance.md#用户要求提高至2-nm后的逐轴试验)。

返回记录已进一步[分析并生成调参候选](../validation/2026-10-21-dm4340-return-analysis.md)：
返回目标已发出，末段持续约−0.79 N·m而角度不变，物理阻滞根因尚未确认。
新离线候选Kp40/Kd2、目标+0.035rad、斜率0.015rad/s、偏差0.04rad，
反馈速度0.2rad/s／力矩2N·m及其他保护保留，预算仍2N·m。
无设备回环已通过；尚未实机运行，不能保证消除静态残差。

## 配置入口

沿用 `motor_backend_bringup.launch.py`，选择 `backend:=dm4340` 或 `dm4340_dual`，
并设置 `dm4340_command_mode:=position`。加载硬件插件可能打开设备，
因此本文不把启动命令作为无需现场授权的离线预览。

单机参数为 `position_kp`、`position_kd`、`max_target_error_rad`；
双机为 `motor1_position_kp`、`motor2_position_kp` 等逐轴前缀参数。
双机原有 `motorN_joint_scale`、`motorN_joint_offset_rad`、
`motorN_position_min_rad/max_rad` 与边界缓冲参数继续生效。
所有原有启动证据标志默认 false，不为接通位置模式而自动置为通过。

控制器及目标接口：

| 部署 | 控制器 | 目标 topic（std_msgs/Float64，rad） |
|---|---|---|
| 单机 | dm4340_position_controller | /dm4340_position_controller/target_position |
| 双机1号 | dm4340_motor1_position_controller | /dm4340_motor1_position_controller/target_position |
| 双机2号 | dm4340_motor2_position_controller | /dm4340_motor2_position_controller/target_position |

## 验证与后续验收

软件回归覆盖 MIT 独立字节向量、负比例映射、失能拒绝、边界／偏差 TX 重检、
逐轴目标路由、非法目标整组停止、原租约边界、插件接口以及 launch 实际 xacro 展开。
真实 controller_manager 测试加载两个通用位置控制器，通过生产 Dm4340System 和
USB-CDC transport、模拟串口检查逐轴 MIT 位置／增益帧、无目标静默与释放后不重放。
测试使用模拟设备，不代表电机已经运行或力矩准确。

下一项实机验收是离体固定空载的小幅位置目标、逐轴再双轴跟踪；
需要另行确认当次参数和现场条件，测定位置方向、响应、过冲、停止及失能读回。
此前速度往返成功不代表本模式通过，尚不进入穿戴测试。

本轮具体软件结果见[位置阻抗验证记录](../validation/2026-10-21-dm4340-position-impedance.md)。

## 恢复原生MIT后的复测配置（2026-10-22）

用户要求移除主机阻抗，恢复原生position模式并进行实机检测。此次准备Kp30／Kd4、
目标+0.05rad、参考斜率0.015rad/s、原位1s→偏移6s→返回6s，先1号再2号。
目标偏差0.055rad、反馈速度0.2rad/s、反馈力矩2.5N·m、每轴新原点±0.2rad。
名义配置预算`30×0.055+4×0.2=2.45N·m≤2.5N·m`；2.5不是电机内部硬力矩限流。
下一轴前仍核对两轴距原点≤0.005rad且速度≤0.05rad/s，不自动重置原点、重试或放宽保护。
原生位置模式没有主机总力矩斜率参数，不沿用被移除模式的20N·m/s参数。
实际结果见[原生恢复复测记录](../validation/2026-10-22-native-position-restore.md)。

静止段后续核对见[PD与机构阻滞分析](../validation/2026-10-22-native-pd-static-analysis.md)：
量化不足以单独解释残差，尚未唯一确定机械根因。下一轮仅K80/Kd4单轴诊断候选，
步幅0.03rad、斜率0.008rad/s、最大偏差0.035rad，反馈保护仍4N·m；名义预算3.6N·m。
K120／K160为后续条件档，不是自动升档或已获运动授权；默认配置未变。

[Kp80单轴复测](../validation/2026-10-22-native-position-k80-bench.md)已完成：
到位0.539°、返回0.350°仍超0.286°判据；两台最终独立失能，未继续2号或升档。
新增默认关闭的`command_trace_snapshot_dir`观测参数复用有界记录器，
实际末端MIT参考已到目标量化值；使用方式、容量、证据边界和测试见复测记录。

后续[Kp80双机同时测试](../validation/2026-10-22-native-position-k80-dual-bench.md)在约2.9秒
因1号反馈力矩4.44N·m超过4N·m保护而整组停止，未完成往返；两台最终独立失能。
TX与拒绝反馈估算差异尚未定位，下一步核对原始反馈，不自动加力或放宽保护。

## 输出限幅与反馈停机的边界（2026-10-22核对）

用户要求设置输出力矩上限，并询问是否可移除反馈停机。当前原生MIT位置帧没有独立的
总输出力矩限幅字段；固定Kp/Kd在驱动器内部参与运算。仅截断前馈不能限制弹簧与阻尼项。
本轮仅核对代码与协议资料，未移除反馈保护、设置设备参数、切换模式或启动电机。
输出上限数值与实现路径仍待用户确定，不能把现有4N·m反馈保护写成“已设置输出上限”。

现有可确认入口与尚缺证据：

| 入口 | 已确认含义 | 不可直接推导的结论 |
| --- | --- | --- |
| MIT的torque字段 | 前馈命令；与位置／速度PD共同作用 | 限制该字段不等于限制总输出 |
| PMAX／VMAX／TMAX | 当前codec与旧STM32用于量化编码／反馈解码的范围 | 未证实TMAX对所接固件全部PD输出做硬限幅，不能直接将28改成4 |
| OC_Value | 官方SDK标注过流故障阈值 | 不是已验证的平滑电流饱和上限 |
| 主机PD后截断 | 旧STM32存在`limit_torque`；本项目历史主机阻抗已按用户要求移除 | 不能不经选择就恢复旧实现或悄悄改变控制模式；软件命令限幅也非物理力矩保证 |
| 力位混合模式 | 官方SDK有独立`control_pos_force`，位置／速度与电流标幺字段，CAN偏移0x300 | 所接固件支持性、字段是否限定最大电流、基准电流及输出轴N·m换算未核实，现有position后端未实现 |

[官方SDK源码](https://github.com/dmBots/motor-sdk/blob/main/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py)
中`controlMIT`、`change_limit_param`、`control_pos_force`为本轮线上核对来源；
这是当前SDK接口证据，不是本设备固件读回。旧STM32本地参考为
`/home/nvidia/mech_attitude_validation/stm32_dm_reference/User/dm_motor_drv.c`及`exocontrol.c`。
后续已确定两轴请求各8N·m，优先保留原生模式，查对应固件的驱动限流文档、
电流基准与扭矩映射，不能用映射范围或过流故障参数冒充输出限幅。


用户随后指定两轴目标输出上限各8N·m，尚未设置到设备或当前原生MIT路径；
没有因此将4N·m反馈停机阈值改为8N·m。当前安全实现分为：
整机`robot/supervisor.py`按每轴`max_command`先对策略命令做对称饱和再施加斜率和位置保护；
当策略模式为effort时，它的单位是N·m。effort下层runtime/session另按
`max_command_effort_nm`在effort runtime对有限命令饱和，内部session仍拒绝越界；
这是后续抗阻训练软件补全，不改变原生position总PD的能力。
原生位置台架直接走PositionCommandController，发送的是位置／增益，未经过该整机effort饱和链路，
因此不能靠设置`max_command=8`限制驱动器内部PD总力矩。
下一步核实驱动限流能力，不能将请求值写成已生效配置。


后续按用户强调的底层限幅方向，优先保留原生MIT、核实驱动侧限流；
不恢复已移除主机阻抗。完整任务和完成条件见
[EXO-ARCH-001-TQ](../planning/exo-arch-001-torque-safety.md)。
若原生模式不支持所需限幅，再评审替代；8N·m仍未设置生效。


## 双机Kp80反馈阈值配置更新（2026-10-22）

用户明确选择仅将两轴反馈超限停机阈值从4改为8N·m，不启动电机。
新离线候选位于 `/home/nvidia/mech_attitude_validation/native_position_k80_dual_feedback8/`：
`candidate_inactive.urdf` 两轴 `max_feedback_torque_nm=8`，`profile.json`、
`prepare.py`、`prepare_authorized.py` 和 `trial.py` 的对应阈值均一致。
判定仍为 `abs(feedback_torque)>8` 时整组停机并锁存；这不是输出饱和限幅。
Kp80、Kd4、目标0.03rad、斜率0.008rad/s、速度0.2rad/s、位移0.2rad及偏差0.035rad不变。
候选默认禁止使能，原点为离线占位；未生成当前运动授权或实际运行URDF。
旧 `native_position_k80_dual_bench` 保留当时4N·m配置和失败记录。
通用Xacro默认值未修改；驱动器8N·m输出限幅仍未实现、未验证。
已执行离线生成、两轴参数／脚本一致性检查及四个脚本语法编译，未进行CAN、寄存器或运动操作。
下一步先补齐首拒绝反馈的原始RX证据，核对与TX估算不一致的原因，再安排新的实机验收。


反馈8N·m候选已完成一次实机尝试，在原位阶段因速度0.364>0.2rad/s停止，
两台最终独立失能。随后按用户要求另建反馈速度2rad/s离线候选，未实机重跑；
详见[复测与新候选记录](../validation/2026-10-22-native-position-feedback8-bench.md)。
