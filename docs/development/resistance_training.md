# 双电机抗阻训练（EXO-ARCH-001-TQ）

本模式包含软件、离线验证及有界台架试验记录；默认预览不启动电机，不代表穿戴或负载验收。
无需IMU，使用两轴编码器速度；坐标是电机输出轴，不自动等同人体关节角。
原有 `resistance` 策略及其配置保留，新增 `resistance_training` 策略。

## 原理与STM32对应

参考用户指定的历史 `Makoto20S/C620--Demo--` 仓库dm分支固定提交
`3211a7ae90e6a0960baea20d7f3cf3c893a2c95e`：
本地 `/home/nvidia/mech_attitude_validation/stm32_dm_reference/User/exocontrol.c`
中的 `ResistiveTrainingPlanner`（223行附近）先低通双轴速度，再计算
`T=-D_VIRTUAL*velocity`，通过 `limit_torque` 截断后发送。
该源码头文件的B=1、ALPHA_V=0.15、力矩上限7属于历史配置，不作为本机硬件许可。
该提交main.c的定时回调实际调用CalculateHipTorque_Compliant；未找到对
ResistiveTrainingPlanner的调用，不能把函数存在当作此版本已运行抗阻的证据。

Jetson按现有分层复用：

```text
双轴 /joint_states → ExoskeletonNode → ResistanceTrainingPolicy
  → ExoskeletonSupervisor（力矩饱和、斜率、位置与时效保护）
  → CommandRouter → 两个通用EffortCommandController
  → 单个Dm4340Runtime（逐轴命令饱和）→ 两个MIT session → 共享USB-CAN
```

策略采用 `alpha=1-exp(-dt/tau)`，滤波速度为
`vf += alpha*(v-vf)`；阻力目标为
`-B*vf`，监督器按每轴上限截断。原始速度死区只负责静止卸力，
不再从移动速度中扣除死区值；保留原始方向判断，避免滤波滞后产生助力。
与STM32固定alpha不同，滤波时间常数不随Jetson调度间隔变化。
不增加位置弹簧：原生MIT帧Kp=Kd=0，只发送受限力矩字段。

原始速度进入死区，立即清滤波和力矩命令；反向时先清零旧方向阻力，再按斜率建立新反向阻力。
训练策略的斜率只限制阻力增大；目标阻力变小时立即跟随目标，避免减速后保留过大的旧阻力。
这避免滤波／斜率残留使阻力变成助力；其余非法数据、越界或失联仍锁存故障。
该保证针对当前观测与主机命令，非连续时间被动性、机械制动或断电制动保证。

## 初版参数

配置：`mech_assist_control/config/dual_motor_resistance_training_preview.yaml`。

| 参数 | 两轴初值 | 作用 |
| --- | --- | --- |
| `training_damping` | [1.0,1.0] N·m·s/rad | 每轴独立阻尼，转得越快阻力越大 |
| `training_velocity_filter_tau_s` | 0.05 s | 一阶速度滤波时间常数 |
| `velocity_deadband` | 0.02 rad/s | 静止噪声区，不输出训练阻力 |
| `max_command`／`max_actuator_command` | [0.2,0.2] N·m | 应用及路由的逐轴阻力上限；超限饱和 |
| `max_slew` | [0.5,0.5] N·m/s | 阻力建立／常规变化斜率；静止和反向卸力优先 |
| `max_velocity` | [1,1] rad/s | 应用反馈超速故障阈值，不是速度控制目标 |
| `max_excursion_rad` | 0.5 rad | 相对arm位置的允许位移；不是训练往返目标 |
| `position_min/max` | ±12.5 rad示例 | 协议坐标范围，实机应替换为测得的机构安全范围 |
| `position_soft_margin_rad` | 0.1 rad | 接近边界减少向外命令；保留向内制动力矩 |
| `run_limit_s` | 5 s | 一次软件窗口到期退出active，不自动重启 |
| 编码器样本有效期 | 0.15 s | 断流／过期不维持旧阻力 |
| 应用周期 | 0.02 s | 50Hz策略更新；底层独立500Hz执行 |
| `output_mode`／`allow_motor_output` | preview／false | 默认只预览，不创建电机命令发布器 |

例如稳定速度+0.2rad/s时，B=1产生约−0.18N·m；−0.2rad/s产生约+0.18N·m。
更高速度会使初版命令饱和在±0.2N·m，不能把8N·m底层能力理解为默认训练强度。

## 力矩模式下层补全

Dm4340Runtime对有限effort命令按每轴 `max_command_effort_nm` 饱和，
然后才做坐标映射及session编码；NaN/Inf、混合模式字段、过期和故障不转成限幅命令。
session独立拒绝越界的内部命令，最终12位力矩编码向限值内侧约束，
避免负向边界经SDK截断量化后略超限。非1或负scale按电机坐标检查。
比半个量化步还小、无法表示任何值的非零力矩上限在配置时拒绝。
零命令的MIT解码仍可能有约−0.00684N·m量化偏差，不能宣称物理零力矩。

双机抗阻配套选择 `dm4340_effort_profile:=training`：下层命令／控制器限幅±0.2N·m，
反馈力矩保护8N·m、速度保护2rad/s，默认inactive。控制器斜率100N·m/s，
在500Hz下可在一周期撤掉0.2N·m；正常阻力建立由应用0.5N·m/s约束。
这避免二级慢斜率将旧方向阻力滞留；仍存在观测、传输和驱动延迟。

双机硬件入口另有显式 `dm4340_effort_profile:=torque8`，仅允许
`backend:=dm4340_dual dm4340_command_mode:=effort`：
控制器上限与下层命令上限均±8N·m，反馈力矩停机阈值8N·m，控制器斜率1N·m/s，
速度反馈保护2rad/s，硬件／控制器仍初始inactive。
默认standard仍为原先±0.1N·m，不自动放大已有配置。
应用初版0.2N·m上限与底层8N·m上限是不同层次，最终受所有上限约束。
8N·m是软件发送命令上限，不是经验证的驱动电流限制或物理输出保证。
反馈超限、位置越界、超速、TTL、总线故障和停用读回保护全部保留。

## 预览入口与验收

安装当前工作区后，运行 `dual_motor_resistance_training_preview.launch.py`，
输入完整双轴 `/joint_states`，显式调用 `/exoskeleton/arm` 后可在
`/exoskeleton/preview_command` 和 `/exoskeleton/command_envelope` 观察阻力目标。
该launch不启动硬件或管理器。`disarm`撤销输出许可，`fault_stop`锁存停止；
应用服务成功不等于真实电机失能，设备状态必须另行独立核对。

离线测试覆盖双轴独立阻尼／饱和、不同dt下滤波一致性、双向反复动作、静止卸力、
重新arm清状态、过期与时长退出、参数负例、ROS输出冲突和默认预览无电机发布器。
底层FakeTransport覆盖双轴不同限值、负scale、巨大有限请求饱和、NaN拒绝、
最终MIT Kp/Kd为0，以及±上限的编码不越界。
后续实机需重新确认机构范围和低力矩测试包络，先验证反向阻力／停止卸力，再评估负载；
不以人体穿戴或徒手抓轴代替台架验收。


首次双机5秒实机窗口完成启停，两台最终独立失能，但速度均低于死区，
非零抗阻尚未验收。详见[实机记录](../validation/2026-10-22-dual-resistance-training-bench.md)。

## 普通DM4310：每轴2 N·m命令上限

新增独立`dm4310_resistance_training_2nm_preview.yaml`和同名preview启动入口，
保留4340旧配置。应用命令及执行器上限均为2；硬件启动选项
`resistance_training_2nm:=true`同步控制器/底层为±2 N·m，保持纯effort。
默认预览不连接硬件，硬件候选仍inactive；详细参数、当前量程、实测角度接入和
启动前待验证项见[DM4310说明](dm4310_position_limits.md#当前设备换算与2-nm抗阻候选2026-10-22)。

2026-10-22后续用户选择加强阻力：上述独立DM4310预览B=4、
`enforce_excursion_limit=false`，关闭起点相对位移保护而保留实测绝对角度边界；
默认其他配置仍启用相对保护。此为2 N·m阶段记录；后续8 N·m流程见下节。

## DM4310每轴8 N·m命令上限候选（2026-10-22）

用户将上限从2改为8。新增独立dm4310_resistance_training_8nm_preview.yaml及
同名预览入口；原2 N·m配置保留。选择硬件候选的离线检查：

```bash
ros2 launch mech_bringup dm4310_dual_bringup.launch.py resistance_training_8nm:=true
```

2nm/8nm选项互斥，仅允许effort模式，默认check_only=true不开设备。
应用max_command/max_actuator_command、通用控制器minimum/maximum及下层
max_command_effort_nm同步为±8。控制器斜率4000 N·m/s允许从8卸至0在标称
500 Hz一拍完成；首次实机应用建立斜率为0.5 N·m/s，不把4000当训练加力斜率；新版候选见下节。
首次8 N·m实机配置B=4、滤波0.05秒、死区0.02 rad/s、应用速度1 rad/s、相对位移开关false，
实测绝对位置保护保留。用户明确同意后，8 N·m选项同步反馈停机阈值为8 N·m；
下层要求命令上限≤反馈阈值，旧8/5组合无效，现已在离线入口提前拒绝。
8是命令饱和边界，不承诺可持续输出实测8，也不代表电机电流硬限幅。

MIT有独立tau字段：tau_request=Kp*(q_ref-q)+Kd*(v_ref-v)+tau_ff。
本抗阻模式由Jetson计算tau_ff，底层prepare_effort写入{0,0,0,0,effort*scale}。
Kp/Kd为0关闭位置/速度PD项，并不会清除tau_ff；只有三项都为0才是零力矩请求。
本轮离线codec输入Kp=Kd=0、tau=−1，输出码kp=0/kd=0/tau=1842，按实测
TMAX10反算−1.003663 N·m，量化误差不代表实际轴输出计量。

提高上限不直接提高低速阻力：旧公式B4在稳定0.1 rad/s时目标约0.32 N·m；
应用0.5 N·m/s建立斜率从0到8需至少16秒（还受策略目标和其他保护约束）。
5秒窗口从0开始最多建立约2.5 N·m，B4/速度1限制下稳态目标也不超过3.92。
12项启动测试、3项真实插件初始化测试及ARM64构建通过。获准的8/8实机尝试
在零力矩倒计时阶段因2号速度1.282 rad/s超过1 rad/s退出，尚未进入抗阻；最终两台独立读回失能。
按用户最新要求，新台架候选取消人为倒计时：使能后仅等待新鲜、有效且静止的50 ms反馈窗口
（最多1秒），随后立即激活控制器并arm，打印“抗阻已启动”。力矩从零按0.5 N·m/s建立，
不是突然施加8 N·m。该流程已做语法和调用顺序检查，尚未实机复测；
参数、证据及限制见[8 N·m实机记录](../validation/2026-10-22-dm4310-resistance-8nm.md)。


## STM32响应对齐与短动作修复（尚未实机验证）

无倒计时实机完整运行5秒：1号应用命令峰值0.2100 N·m，发送帧反算峰值0.20757 N·m；
2号未产生有效动作。操作者反馈未感到阻力，不能认定效果通过。
旧0.5 N·m/s建立斜率叠加50 ms滤波，是短动作输出很弱的明确软件因素，
还不能由此排除实际驱动输出或机械因素。

新版独立8 N·m预览配置保留B4、命令/反馈上限8、速度保护1、实测绝对角度和无倒计时流程，
仅将滤波时间常数改为0.0061531294秒（对应STM32在1 kHz下alpha=0.15），
应用建立斜率改为20 N·m/s；其余2 N·m配置参数保留。
共享训练策略改用STM32的T=-B*vf，死区仅作静止开关；监督器支持训练策略立即减小阻力。
这不是将Jetson调度频率改为1 kHz，也不是原封不动复制STM32的反向滤波残留。

稳定速度0.1/0.5 rad/s的新目标分别约0.4/2 N·m，仍不会固定输出8 N·m。
同样0.5 rad/s、200 ms模拟动作，0.5斜率仅能建立0.1 N·m，新20斜率可接近2 N·m。
该对比是确定性离线输入，不是实机轨迹重放或已测轴力矩。
72项策略/监督/位置/ROS检查通过（首次环境缺overlay跳过10项ROS，加载overlay后10项全部通过），
ARM64应用构建及已安装配置检查通过。新参数必须按新的完整包络启动实机，不能复用已消费的旧授权。


## 平滑抗阻候选：减小低速开关抖动

上述6.15 ms/20 N·m/s响应方案两次实机后，操作者反馈阻力偏弱且有抖动。
复测实际编码命令峰值仅0.803/0.403 N·m，且存在约0.801/0.400 N·m的相邻下降。
命令跳变可能贡献抖动，尚无同步反馈证明因果；原响应配置保留作对照。

新增`dm4310_resistance_training_smooth_preview.launch.py`和同名YAML，默认仅预览：

| 参数 | 候选值 | 作用 |
| --- | --- | --- |
| training_damping | 每轴8 N·m·s/rad | 同样速度下提高阻力目标 |
| training_velocity_filter_tau_s | 0.025秒 | 比6.15 ms方案更平滑 |
| training_start_velocity_rad_s | 0.06 rad/s | 超过启动阈值才建立阻力，抑制低速噪声开关 |
| velocity_deadband | 0.02 rad/s | 下降至停止阈值即卸力，形成滞回 |
| max_slew | 每轴8 N·m/s | 放缓阻力建立，下降/静止/反向仍优先卸力 |
| max_command / max_actuator_command | 每轴8 N·m | 命令饱和边界，非恒定阻力 |
| max_velocity | 每轴1 rad/s | 超速退出保护不变 |

新ROS参数`training_start_velocity_rad_s=0`默认跟随原死区，旧配置行为保留。
非零设置必须不小于停止死区。启动后，0.02~0.06 rad/s仍保持运动状态；
从静止开始低于0.06 rad/s不建立力矩，这是为减少噪声牺牲部分低速灵敏度。
静止立即清滤波、撤力；反向先清零，禁止沿旧方向保留力矩。
B8在稳定0.1 rad/s时目标约0.8 N·m，0.5 rad/s约4 N·m，实际仍受所有限幅约束。
新候选仅完成78项离线/ROS检查、ARM64构建及安装预览检查，尚无平滑效果实机验收。
实测绝对边界必须由测量文件加载，源预览宽边界不得用于真实设备。


## 已发现的实时性边界

平滑20秒实机完整运行，但操作者仍反馈阻力弱/抖动，尚未验收。
本轮每轴实际发送约500.054 Hz、最大成功发送间隔约2.114 ms，无大于4 ms间隔；
然而应用robot_node固定20 ms定时，阻力计算只有标称50 Hz，编码力矩变化的中位间隔约20 ms。
因此上层50 Hz目标加下层500 Hz重发不是500 Hz抗阻闭环。原6.15 ms滤波参数也不能改变这个事实。

现已新增可选通用C++抗阻控制器，在每次控制更新计算阻尼，Python保留50 Hz许可监督。
ARM64构建、软件回归和无设备ROS回环已通过；旧路径保留，新路径尚未实机验收。
参数、入口和保护边界见[快环说明](resistance_fast_loop.md)，不通过增大B或删除保护替代验证。
