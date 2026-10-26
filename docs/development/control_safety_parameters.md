# 分层控制安全校验与参数表

适用当前 EXO-ARCH-001：无 IMU 电机轴阻力/轴运动助力、达妙 position/velocity/effort 运行时。
这是源码实现与示例配置清单，不是整套穿戴安全认证，不授予设备操作权限。
下表数值为当前文件值；配置覆盖后以实际加载值为准。单位角度为rad，速度rad/s，力矩N·m。
供应商编码范围、设备物理极限和人体活动范围三者不同，不能相互替代。

双机Kp80专用离线候选的反馈停机阈值已按用户要求改为8N·m；
下表通用示例默认值不变，详见[配置更新](dm4340_position_impedance.md#双机kp80反馈阈值配置更新2026-10-22)。

## 1. 保护位置和故障行为

| 层级 | 入口 | 保护与异常动作 |
|---|---|---|
| 应用输入 | robot_node.py | 拒绝重复/未来/过旧时间戳、缺轴、重复轴名、NaN/Inf；锁存故障 |
| 统一监督 | robot/supervisor.py、safety.py | 完整轴集合、健康/标定、位置/速度边界、命令幅值/斜率、软限位、时间和做功预算；越界/失效锁存，命令无效 |
| 路由 | robot/routing.py | schema/序号/期限、模式/轴顺序、映射、映射后幅值；拒绝则上层故障 |
| 通用控制器 | mech_controllers | 上游目标时效、幅值/斜率、generation；不把旧上游目标永久刷新成新命令 |
| 达妙session | dm4340_mit_session.cpp | 有效反馈、故障状态、实际角度硬范围、速度/力矩反馈范围、模式与命令范围、方向性硬边界缓冲区；拒绝候选帧 |
| 达妙运行时 | dm4340_runtime.cpp | 收帧失败、session拒绝、反馈/命令过期、丢使能、发送失败等触发总线运行时故障，清pending、撤销授权、尝试中性帧和失能 |
| 最终发送 | session pending_target_still_authorized / Runtime authorize_send | 真正TX前再次检查最新反馈、方向、授权、generation及期限；排队旧命令不能越过这些检查 |
| 生命周期 | Dm4340System / confirm_disabled | 停用/关闭时进行有界失能读回，未确认返回错误；生命周期inactive本身不是失能证据 |

应用故障时至多发布一次零目标并停止刷新，不直接操作硬件生命周期。
`/exoskeleton/fault_stop` 是新增锁存软件停止服务；reset后仍需新反馈及显式arm。
它不等于硬接线急停，不会单凭服务返回success宣称实际失能或机械静止。
普通disarm允许再次显式arm，fault_stop/异常则先reset；运行中不允许reset或修改参数。

## 2. 应用层位置、速度和力矩

配置：`mech_assist_control/config/motor_resistance_preview.yaml`、`motor_assist_preview.yaml`。

| 参数 | 阻力预览 | 助力预览 | 作用 |
|---|---:|---:|---|
| position_min / position_max | −3.14 / +3.14 | 同左 | 绝对轴角硬范围；目前是软件示例，不是人体限位 |
| max_excursion_rad | 0.5（约28.65°） | 0.3（约17.19°） | 相对本次arm的位置；与绝对范围取交集，不改编码器零点 |
| position_soft_margin_rad | 0.1（约5.73°） | 同左 | 接近有效边界时线性收窄向外命令上限；向内仍受其他限幅但不受此方向缩减 |
| max_velocity | 1.0 | 0.5 | 实际轴速度越界锁存；它不是速度保证值 |
| max_command | 0.2 | 0.1 | 输出轴力矩命令上限 |
| max_slew | 0.5 N·m/s | 0.2 N·m/s | 普通力矩变化率；撤销、死区归零和位置保护可优先降低命令 |
| run_limit_s | 5 | 3 | 到时退出active，不自动重启 |
| output_mode | preview | preview | 只发布预览，不发布控制器力矩目标 |
| allow_motor_output | false | false | 真实输出需显式许可；预览launch强制false |

上次实机阻力脚本独立指定绝对范围−12.5～12.5、相对位移0.5、力矩0.2、速度1、运行5秒。
此脚本值与预览例子不同；此次未改该历史实机配置或启动它。
软限位在上层斜率之后执行，但下层独立斜率与设备惯性仍会影响实际停止，不能代替底层硬保护。

## 3. 策略专用参数

| 参数 | 当前值 | 作用 |
|---|---:|---|
| damping | 0.5 N·m·s/rad | 阻力系数；此前建议0.8未实际采用 |
| velocity_deadband | 0.02 rad/s | 阻力静止死区；死区内归零 |
| assist_gain | 0.3 N·m·s/rad | 同向轴运动助力增益，不能判断人的意图 |
| assist_deadband | 0.04 rad/s | 助力起效死区 |
| assist_taper_speed | 0.3 rad/s | 从此速度开始降低助力 |
| assist_cutoff_speed | 0.5 rad/s | 达到或超过即故障锁存，防止持续放大运动 |
| assist_work_budget_j | 0.05 J | 单次arm所有轴的指令做功估计预算；超过即锁存，重新arm清零 |

阻力检查 `τ·ω≤0`；若斜率残留会变成助力则故障。助力反转先撤掉旧方向力矩。
做功预算是 `dt × Σ max(|τ_previous|,|τ_now|) × |ω_now|` 的离散估计，
不是实测能量或被动性证明。编码器助力可能在松手后持续运动，不能宣称松手即停。

## 4. 应用时效、消息和映射（当前代码常量/启动参数）

| 参数或校验 | 当前值 | 作用 |
|---|---|---|
| ROS应用timer | 0.02 s | 50Hz监督/命令计算，非硬实时保证 |
| 编码器源stamp/到达新鲜度 | ≤0.15 s | 递增、非未来且未过旧；持续无反馈则锁存 |
| 估计关节源/健康消息时效 | ≤0.3 s | 估计模式需对应健康/标定生产者有效 |
| sample_timeout | 编码器0.15s，估计0.3s | 监督层独立检测状态过期 |
| health_timeout | 0.3 s | 健康记录时效；诊断更新不能刷新旧关节样本 |
| max_tick_gap | 0.1 s | 监督循环间隔越界即故障 |
| command_ttl | 0.1 s | 应用命令信封本机单调时钟期限 |
| schema_version / sequence | 1 / 严格递增 | 路由拒绝未知schema和重放命令 |
| joint_mapping_verified | 默认false | motor模式必须显式确认映射；不能代替实际测量 |
| joint_scales / joint_offsets | 默认0 / 0 | 未配置scale不允许输出；编码器模式必须scale=1、offset=0且规范轴名等于实际轴名 |
| max_actuator_command | 默认0.3；按设备显式指定 | 映射后输出上限；超限拒绝，不静默截断 |
| motor反馈映射角差 | ≤0.15 rad | 电机反馈与上层状态映射后的差异检查 |
| motor反馈映射速度差 | ≤0.5 rad/s | 速度映射一致性检查 |
| 目标发布者数量 | 必须恰好1 | 有其他发布者争用即故障；不是DDS身份认证 |
| contact_names / verified | 接触策略需明确且新鲜 | 没有鞋垫时不能假设双脚着地；当前阻力/轴助力不要求接触 |

## 5. 通用控制器和硬件租约

配置来源：`mech_bringup/config/dm4340_velocity_controllers.yaml`、`dm4340_velocity.urdf.xacro`。

| 参数 | 当前值 | 作用 |
|---|---:|---|
| controller_manager.update_rate | 500 Hz | 目标周期2ms；不证明实际调度达标 |
| velocity controller minimum/maximum | −1.5 / +1.5 rad/s | 控制器速度目标限幅 |
| effort controller minimum/maximum | −0.3 / +0.3 N·m | 控制器力矩目标限幅；应用可设置更小值 |
| controller max_slew_per_second | 1.0（对应模式单位/s） | 控制器独立斜率限制 |
| target_ttl_nanoseconds | 100000000（100ms） | 上游目标软期限 |
| target_hard_ttl_nanoseconds | 106000000（106ms） | 上游目标硬期限，不再继续维持有效命令刷新 |
| control_period_ns | 2000000（2ms） | 下层租约校验使用的配置周期 |
| command_ttl_ns | 4000000（4ms） | controller→runtime/session命令软期限 |
| command_hard_ttl_ns | 6000000（6ms） | 下层命令硬期限；新目标不能掩盖已发生的过期 |
| command_generation | 严格递增有效代次 | 区分控制器刷新与硬件空循环；模式接口整组claim/release |
| authorized / fresh | 必须匹配claim与新目标 | 未授权不发运动命令，撤销时清除pending；fresh=false不伪造新租约 |

Float64跨进程入口只传数值，尚未携带应用信封的源序号/绝对截止时间；
控制器按本机回调到达时刻计时。不能把应用路由的序号检查宣称成跨进程重放保护。

## 6. 达妙下层参数（独立于上层算法）

这些是当前离体配置值，绝不能直接套用为外骨骼人体活动范围。

| 参数 | 当前值 | 作用 |
|---|---:|---|
| position_min_rad / position_max_rad | −12.5 / +12.5 rad | 实际反馈映射轴角硬范围；超出即session锁存并使运行时故障停止 |
| position_guard_margin_rad（新增必填） | 0.02 rad（约1.15°） | 到硬边界距离≤此值时，拒绝向外速度/力矩及更向外的位置目标；TX前按新反馈重检，向内/零仍允许。必须>0且小于范围一半 |
| joint_scale / joint_offset_rad | 1 / 0 | joint=motor×scale+offset；方向判定使用映射后坐标，支持负scale |
| max_target_error_rad | 0.3 rad | session位置候选目标相对反馈的误差上限；DM位置模式已接通ROS接口，量化目标及最终TX均复查；尚无本模式实机验收 |
| max_command_velocity_rad_s | velocity模式1.5，effort模式0 | 速度候选上限；非零仅用于速度模式 |
| max_command_effort_nm | effort模式0.3；特殊supervised bench为0.1 | runtime关节力矩饱和上限；session防御拒绝，编码按映射后的限值向内约束 |
| max_feedback_velocity_rad_s | 2 | 实际原始电机速度越界锁存；非映射关节速度 |
| max_feedback_torque_nm | 5 | 原始电机估计力矩越界锁存，不等于独立测力器精度 |
| feedback_ttl_ns | 40000000（40ms） | 超过有效反馈期限触发停止 |
| query_period_ns | 20000000（20ms） | 主动状态查询周期；必须短于反馈期限 |
| kp / kd | effort均0；velocity Kp=0、Kd默认3（launch可配） | 力矩模式禁止附加位置/速度保持；模式与增益不一致拒绝配置 |
| velocity_feedforward_nm | 默认0 | velocity可选前馈；effort要求0，不把速度前馈当力矩验收 |
| limit_position_rad / limit_velocity_rad_s / limit_torque_nm | 12.5 / 10 / 28 | SDK协议编码参考范围，必须有本设备证据声明，不是人体/机构安全极限 |
| confirm_disabled 总等待/重发 | 300ms / 30ms | 生命周期非RT路径清旧反馈后重复失能/查询，匹配状态0才确认 |
| enable_on_activate | 默认false | 防止配置/启动即自动使能，需显式启动许可 |
| board_version_verified / can_bitrate_verified | 默认false | 板版本、位速率证据闸门；不能用串口baud代替CAN速率 |
| motor_side_timeout_verified | 默认false | 必须有设备侧保护证据；不是本机TTL，也不由寄存器0自动推定 |
| firmware/ID/map/limits evidence | 显式声明且配置有效 | 识别固定固件范围、总线、帧ID和单位；未知不能猜测为通过 |

越界或候选方向拒绝在运行时触发失能尝试，不实现自动往回拉，也不在故障后自动重启。
本轮用户未授权真实电机试验，因此没有改设备RAM/Flash/寄存器。
软件零命令存在协议量化误差，撤销命令、失能状态和机械静止是三种不同事实。

## 7. CubeMars和其他未闭合项

- CubeMars AK3.0 force velocity/effort路径的位置反馈不可靠/不可用；不能用占位position伪造角度保护，也不能声称与达妙具有同样的位置能力。
- CubeMars servo位置session已有独立反馈和目标范围检查，使用其专用profile与参数；不是DM参数表可直接套用。未改共享core/控制器品牌边界。
- 温度：当前DM wire仅保留raw_byte_6/7，尚未固定温度物理语义、阈值和有效性；主机温度保护未实现。供应商故障码拒绝不等于独立温度阈值保护。
- 电流、电压、硬件急停、制动器、机械限位、双通道安全监控、人体意图/接触判定、独立角度传感器和通信身份认证：不是当前实现的全套认证能力。
- 六关节整机原子停机、异构同网关、跨进程版本化命令、非RT应用死亡后的设备侧停机和掉电/重力保持，均需各自验收，不能由单轴离线测试替代。
- 当前硬角度范围过宽；真正上外骨骼前必须测量并将设备侧配置收紧到机构允许范围，验证停机距离和失能后的机构行为。不要使用预览默认值直接穿戴。

## 8. 软件验证与实机验收区分

新增回归直接调用session/runtime，不依赖上层监督：两端方向拦截、负scale、已准备命令遇新边界反馈撤权、实际越界锁存、velocity/effort运行时撤销和失能帧。
保留已有CRC/帧型/ID、反馈期限、租约期限、claim、背压、生命周期失能读回和模拟ROS回归。
此文档与测试只证明指定软件路径。实机需分别验证方向/角度标定、极限位置停止距离、反馈断流/应用退出/主机失效、失能后的重力与机械风险。

2026-10-21本轮ARM64验证：7个C++包build通过，colcon test-result共747项、应用包85项，均0errors/0failures/0skips。
bringup完整324项包含模拟ROS/PTY生命周期，达妙协议23项；其余共用包与CubeMars回归也通过。
文档478链接、125Python/12Bash语法和context基线通过，git diff --check通过。
日志位于本机 `/home/nvidia/mech_attitude_validation/safety_*`；本轮没有运行实机操作或远程CI。

## 达妙位置阻抗扩展

位置模式复用下层实际角度硬范围，并增加量化目标方向／偏差及使能状态的最终TX复查。
每轴Kp=1、Kd=0.2、最大目标误差0.3rad的示例及名义力矩预算，见
[位置阻抗参数与限制](dm4340_position_impedance.md)。仅软件验证，未实机运行。


## 输出限幅与反馈保护后续任务

当前高优先级任务为[EXO-ARCH-001-TQ](../planning/exo-arch-001-torque-safety.md)。
两轴各±8N·m是用户请求的驱动输出目标，尚未生效，不能用来覆盖本表示例或既有实机保护值。
实施分开处理驱动饱和、主机命令限制与异常停机，并统一单位／轴映射及证据状态。
原生MIT尚无已核实的独立输出限幅；达到正常上限不应直接当异常停机，
反馈异常保护也不能因有名义限幅而删除。阈值余量需有反馈／机械依据，不采用随意固定百分比。


2026-10-22抗阻训练软件更新：effort入口在runtime逐轴饱和，内部session仍拒绝越界，
最终力矩编码按映射限值向内约束；独立反馈／位置／速度／通信保护保留。
新增双轴滤波抗阻训练与默认预览，详见[抗阻训练说明](../development/resistance_training.md)。
原生position内部总PD限幅及驱动电流限幅缺口不因此消除。

## 可选抗阻快环

新增通用 `ResistanceTrainingController`：借用位置/速度状态，在500 Hz目标更新周期计算有界反向力矩；
Python只发送50 Hz许可。原有应用抗阻路径保留，快环需显式选择。
参数、`resistance_training_fast`入口、许可TTL与下层保护边界见[抗阻快环说明](resistance_fast_loop.md)。
软件及无设备回环已验证，新路径实机阻力与抖动尚未验收。


2026-10-22快环抖动修正：新增20ms同方向确认（静止/反向重新计时），
低速区连续衰减并按原始速度收窄粘性命令，保留既有上限、角度、TTL和租约保护。
这属于算法抗抖，不替代驱动器硬限流，也不代表连续时间机械被动性已证明。
[修正验证记录](../validation/2026-10-22-resistance-chatter-fix.md)区分固定轨迹回放与实机验收。

## 固定参考位置阻抗补充（2026-10-22）

用户重新选择STM32 PD内核后，新增独立 `FixedPositionImpedanceController`，
不恢复已移除的主机轨迹阻抗。它用纯effort模式，每次更新先计算PD再限幅，
默认K15/D0.5/命令±7N·m，候选下层命令7/反馈保护8N·m。
绝对位置、方向边界、下层通信与反馈保护保持；无速度抗阻的静止卸力/方向等待/力矩缓升。
参考点只在明确生命周期激活时捕获，静止偏离仍回拉；许可0不重置运行预算或参考点。
完整参数与激活/锁定条件见[固定位置阻抗](fixed_position_impedance.md)，首次1号有界窗口已通过非零命令与独立失能核对，双机持续及穿戴尚未实机验收。
