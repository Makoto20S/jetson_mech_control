# 固定参考位置阻抗：STM32 PD 内核移植

## 本轮范围与参考证据

EXO-ARCH-001：用户明确选择以前 STM32 main 调用的
`CalculateHipTorque_Compliant`，本次新增通用
`mech_controllers/FixedPositionImpedanceController`，保留速度抗阻与原生 MIT 位置控制。
这是用户重新指定的固定参考模式；[旧主机轨迹阻抗](host_impedance_stm32_port.md)仍为已移除的历史实现。

源代码核对基线为用户提供的 C620--Demo-- 仓库本地检出
`3211a7ae90e6a0960baea20d7f3cf3c893a2c95e`：

- `User/exocontrol.c:48`：`CalculateHipTorque_Compliant`，位置弹簧加速度阻尼，最后饱和。
- `User/exocontrol.h`：实际函数采用 `IMP_STIFFNESS_HIP=15`、`IMP_DAMPING_HIP=0.5`、`HIP_MAX_TORQUE=7`。
- `User/dm_motor_ctrl.c`：两轴内部 MIT `kp_set=kd_set=0`，主机计算结果进入 `tor_set`。
- `Core/Src/main.c`：TIM3 回调读取两轴反馈、调用规划器和上述 PD，再发送两轴命令。

必须区分：STM32 main 的步态规划器会更新位置参考，整个 main 并非始终固定位置。
本次按用户选定的固定位置行为，只移植实际PD内核。兼容台架在激活时捕获参考；
规范上层入口在arm时给出参考和参数，见文末合同。
未移植步态规划、启动自动使能、传感器依赖或 STM32 的中断/外设实现。
声明的 STM32 `CONTROL_FREQ` 不作为已测定中断频率；Jetson 经理标称为500 Hz，首次1号实机已完成有界窗口，详见验证记录；双机回拉与穿戴尚未验收。

## 控制行为

每轴独立捕获 `q_ref`，每次 update 计算：

```text
tau_raw = K * (q_ref - q) - D * velocity
tau_command = clamp(tau_raw, -torque_limit_nm, +torque_limit_nm)
```

目标速度及前馈均为0；无积分项。源代码 `prev_error` 仅存储误差，不参与下一次计算。
Jetson 使用 double，离线参考使用 STM32 风格 float 比较，允许浮点舍入误差。
例如偏离0.1 rad且停住，K15产生1.5 N·m持续回拉；0.5 rad时原始7.5 N·m被限到7 N·m。

固定位置模式像弹簧：远离参考点时抵抗，回参考点时可能顺着运动方向输出力矩。
因此不套用速度抗阻的 `tau*velocity<=0`、静止卸力、方向等待、速度低通或加力斜率。
这是弹簧能量返回，不是纯耗能的粘性阻尼，也不是穿戴助力或意图识别算法。

## 分层与生命周期

```text
上层明确许可（~/enable，Float64 0/1，建议50 Hz）
  -> 每轴通用 C++ 固定位置 PD（经理标称500 Hz）
  -> effort + command_generation
  -> 共用 ros2_control hardware runtime
  -> effort 限幅、绝对角度/方向、反馈/租约检查
  -> 达妙 MIT 纯力矩字段（内部Kp=Kd=0）
```

控制器只依赖通用 position/velocity 状态和 effort/generation 命令，不读取 CAN ID 或厂商协议。
双轴复用同一总线拥有者，各自计算，不承诺跨轴原子目标更新。

激活前要求反馈有限、近似静止、处在绝对范围内且距边界至少一个保护余量。
激活不授予输出许可。首次有效 `enable=1` 开始单调时钟运行预算。
`enable=0` 与许可软过期均输出0，不重设参考点或运行预算；硬过期、超时、反馈异常锁定故障。
重新发许可不能解锁，必须明确停用/重新激活并重新取得当前参考点。
停用本身沿用资源/底层生命周期停止流程，不将 ROS active/inactive 标签视为设备失能证据。

控制器 ERROR 时不再刷新命令世代，由底层原有租约与异常处理停机；没有删除底层保护。
状态接口不携带采样时间，真实反馈新鲜度与组停仍由现有下层运行时负责。

## 参数与边界

| 参数 | 控制器默认 / 双DM4310候选 | 作用 |
|---|---|---|
| `stiffness_nm_per_rad` | 15 / 15 N·m/rad | 位置偏差的回拉刚度 |
| `damping` | 0.5 / 0.5 N·m·s/rad | 速度阻尼，目标速度为0 |
| `torque_limit_nm` | 7 / 7 N·m | 计算结果的对称命令饱和，匹配STM32 |
| `position_min_rad/max_rad` | 默认0/0拒绝配置 / 加载两轴实测文件 | 绝对反馈角度校验；不自行猜测人体角度 |
| `position_margin_rad` | 0.02 / 测量文件余量 rad | 靠近边界时，仅减小向外命令上限 |
| `activation_velocity_rad_s` | 0.05 / 0.05 rad/s | 捕获参考点前的静止检查 |
| `max_velocity_rad_s` | 1 / 测量文件声明的编码VMAX | 有限反馈有效范围，不控制目标转速，不改寄存器 |
| `max_update_period_s` | 0.006 / 0.006 s | 周期与真实单调时间间隔上限 |
| `run_limit_s` | 5 / 5 s，配置范围(0,120] | 每次激活的有界窗口；许可间隙计入预算 |
| `minimum/maximum` | 必须显式0/1 | 上层许可范围，不是力矩范围 |
| `max_slew_per_second` | 必须≥1000000 | 只用于许可，避免许可被缓变；不是力矩斜率 |
| `target_ttl_nanoseconds` | 100000000 | 上层许可软有效期；过期清零且不刷新世代 |
| `target_hard_ttl_nanoseconds` | 106000000 | 硬有效期，返回ERROR并锁定 |
| 下层命令上限 | 候选7 N·m | 与PD命令上限同步，纯力矩模式有效 |
| 下层反馈力矩保护 | 候选8 N·m | 反馈异常停机，独立于命令饱和 |

方向性边界包络在 PD 饱和后额外执行，与 STM32 内核对照时测试点位于边界余量之外。
力矩命令上限不是驱动电流硬限流或校准后的实际输出力矩保证；量化、通讯和机械动态仍存在。
实测角度参考失效时仍须恢复有效标定，不以固定参考捕获代替机械限位标定。

## 入口与验证

离线检查默认不开设备、不构造节点：

```bash
ros2 launch mech_bringup dm4310_dual_bringup.launch.py fixed_position_impedance:=true
```

可设置 `impedance_run_limit_s`；该入口与三种 resistance profile 互斥，只接受 effort 模式。
设备路径分支保留既有证据闸门，硬件和两轴控制器默认inactive，无自动许可发布或使能。
上层现有 resistance policy 并非本模式的完整应用集成，不能不改语义就套用旧实机脚本。
首次台架已采用独立固定参考许可/记录流程，1号约4.8秒窗口通过，最终独立确认两台失能；
2号与双机回拉仍需后续授权与验证。

无设备的真实插件检查：

```bash
python3 tools/validation/check_resistance_fast_ros.py --controller fixed-position --output-dir /absolute/new/external/evidence
```

它只能实例化 `mock_components/GenericSystem`，验证加载、约500 Hz世代刷新、TTL与故障锁定；
非零位置/速度、静止回拉、正负饱和及边界行为由 loaned interface 单元测试覆盖。
验证结果见[本轮记录](../validation/2026-10-22-fixed-position-impedance-port.md)。

## 双机持续运行入口

用户在首次1号实机后反馈“可以”，进一步要求一个包含使能/失能的指令，并明确两台同时运行。
新增 `tools/bench/run_fixed_position_impedance.py`：独立已审核只读工具核对已知ID1/2及参数，
两轴静止50ms后各自捕获固定参考，共享使能、同时许可；Ctrl+C清零、停用控制器/硬件，
关闭manager后再次独立查询两台status0。失败提示立即断电，不自动重试/恢复。
当前主机包装命令保存在 `/home/nvidia/mech_attitude_validation/start_dual_fixed_impedance.sh`，
绑定本次实际安装与已审核只读工具；这是本机运行入口，不是其他机器的依赖路径标准。

持续运行需两个显式控制器参数同时成立：`allow_continuous_operation=true` 且 `run_limit_s=0`。
单独0值仍拒绝；默认模式仍为5秒、上限120秒。新增工具负责配置持续模式，原launch默认不改变。
持续模式只移除时长退出，仍保留许可TTL100/106ms、6ms最大周期、绝对位置与方向包络、
有限速度解码有效范围、命令7N·m与反馈8N·m保护、原有下层租约/故障锁定。
两轴使用K15/D0.5；无相对位移限制、无倒计时，必须全程现场值守、离体固定支撑无人体连接。

启动时保持两轴静止，看到 `ARMED BOTH` 后再操作安全夹具；回到参考点时会产生同向回拉，
不按纯速度抗阻解释。Ctrl+C结束可能需要数秒完成独立查询；清理期间再次Ctrl+C不会中断停机流程。
强杀进程/系统断电无法由Python保证清理，独立硬件保护与现场断电能力不能由本工具代替。
不记录无限增长的高频反馈或原始trace，仅维护有界最新状态和峰值摘要。

开发/复核时必须用 `--shadow`；此模式只构造GenericSystem，并且不调用只读工具或访问电机。
支持mock专用 `--shadow-duration-s` 自动结束，真实持续模式只能主动退出或保护退出。
本次只验证mock的双机生命周期与Ctrl+C清理，未执行双机持续真实运行或长时稳定性验收。

## 统一上层训练模式（2026-10-23）

按用户确认，统一上层新增策略名 `impedance_training`，中文为“位置阻抗训练”。
新的通用训练预览入口默认选择此模式：

```bash
ros2 launch mech_assist_control motor_training_preview.launch.py
```

该入口只启动上层预览，不启动manager、IMU、CAN或自动arm。
其配置为 `dm4310_impedance_training_preview.yaml`，默认5秒有界监督、两轴7N·m候选，
配置角度仅为离线fixture，真实运行必须加载当前有效实测范围，不能把示例±12.5rad当作机构限位。
历史 `resistance_training` 策略和明确命名的resistance启动入口仍为速度抗阻，未悄悄改变其行为。

规范路径现为：编码器状态→上层arm捕获参考并下发K/D/命令上限→
`~/training_command`完整帧→C++实时PD→hardware/runtime独立保护。
上层不重复计算PD，诊断 `reference_capture=upper_arm`，预览values仍是未使用的零占位。
后端须显式选择 `command_source=training_command`；旧 `~/enable` 兼容台架继续以激活时位置为参考。
两种入口互斥；上层不自动启用设备或切换控制器，独立台架工具不变。

字段、参数天花板、首次参考检查、会话内参数固定、超时及未迁移策略限制，统一见
[上层计划与实时控制合同](training_command_contract.md)。
本轮仅软件集成和无设备验证；不把此前1号独立台架结果扩展为统一上层实机验收。
