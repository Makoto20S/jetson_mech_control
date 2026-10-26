# STM32 主机侧阻抗移植（EXO-ARCH-001）

> 历史设计：2026-10-22已按用户要求移除该主机阻抗实现和启动入口，保留[原生MIT位置阻抗](dm4340_position_impedance.md)。下述接口仅为历史记录。

状态：软件移植与离线验收完成；实机首测1号触发速度保护，位置跟踪／返回尚未验收。旧电机端位置阻抗保留。

## 来源与差异

按用户指定的 [C620--Demo-- dm 分支固定提交](https://github.com/Makoto20S/C620--Demo--/tree/3211a7ae90e6a0960baea20d7f3cf3c893a2c95e)
核对，算法在 `User/exocontrol.c` 的 `CalculateHipTorque_Compliant`，
参数在 `User/exocontrol.h`，调用在 `Core/Src/main.c` 的定时器回调；
MIT 发送和初始化分别在 `User/dm_motor_drv.c`、`User/dm_motor_ctrl.c`。
此处独立实现控制数学和接口，没有复制供应商 HAL、驱动或整套工程。

```text
τraw = τff + K × (qref − q) − D × v
τbounded = clamp(τraw, −τmax, +τmax)
```

STM32 实际使用 `IMP_STIFFNESS_HIP=15`、`IMP_DAMPING_HIP=0.5`，
`HIP_MAX_TORQUE=7 N·m`；不是文件中另一个未被该函数使用的80/2宏。
调用处前馈为0，MIT位置、速度、Kp、Kd初始化均为0，仅发送计算后的力矩。
`prev_error`只赋值，没有积分或误差差分；也没有在该函数中做摩擦／重力补偿。

| 项目 | STM32 参考 | 原 Jetson position 入口 | 新 Jetson impedance 入口 |
|---|---|---|---|
| 力矩计算位置 | 主控定时回调 | 电机内置MIT环 | 通用C++控制器update |
| 下发内容 | Kp=Kd=0，力矩命令 | 位置、固定Kp/Kd，前馈0 | Kp=Kd=0，限幅／斜率后的力矩 |
| 刚度／阻尼 | 15 / 0.5 | 部署配置的电机坐标增益 | 可配置关节坐标增益，示例15 / 0.5 |
| 力矩限制 | 主控±7 N·m | 名义增益预算＋反馈保护，非输出硬限流 | 主控显式力矩裁剪＋独立底层命令／反馈限制 |
| 调度 | 代码声明1 kHz、TIM3回调 | manager配置500 Hz，内环设备执行 | manager配置500 Hz，实际调度须另测 |
| 轨迹 | 参考工程步态规划 | 上游位置目标 | 显式新鲜位置目标，未移植步态／自动启动 |

主控和电机端PD在理想的相同坐标、参数、反馈条件下数学等价。
更换计算位置本身不能保证克服静摩擦或消除返回残差，也不能证明旧公式写错。
新实现让力矩限幅、斜率和反馈依赖留在可测试的主控代码内。

**协议量程不能照搬**：参考初始化的PMAX/VMAX/TMAX为12.5/30/10，
当前DM4340部署为12.5/10/28，参考注释还涉及DM4310。
这是源码配置差异，不是本轮对实物寄存器的确认；实际设备型号、固件和量程必须匹配。
不把参考工程的7 N·m、绝对轨迹偏置、自动使能、Flash写入或恒返回0的故障函数迁入。

## 架构与接口

```text
显式上游位置目标（关节rad）
  → ImpedanceCommandController（每轴一个，500Hz配置）
      实测q/v → 位置目标限幅/斜率 → PD+固定前馈
      → 力矩裁剪 → 力矩斜率 → effort + command_generation
  → Dm4340System / Dm4340Runtime（已有effort模式，共享网关）
  → Dm4340MitSession（已有映射/边界/时效/反馈及最终发送保护）
  → MIT：原生Kp=Kd=0，torque=映射后的主控力矩
```

实现位于 `mech_controllers/impedance_command_controller.*`，不判断品牌，不打开串口。
只有支持可靠位置／速度反馈、纯力矩命令和已有保护的后端才能组合；
当前测试覆盖达妙双机，不表示所有CubeMars模式都有这些反馈能力。
位置、速度、力矩控制器保持原行为，共用目标信箱、激活代次、限幅器和单调时钟租约。
基类新增参考到输出变换钩子，原三种控制器仍为恒等变换。

- 订阅 `~/target_position`，`std_msgs/msg/Float64`，坐标为已配置的关节rad。
- 读取 `joint/position`、`joint/velocity`，写 `joint/effort` 和 `joint/command_generation`。
- 激活以当前测量位置作为斜坡起点、力矩从0开始；没有新目标不刷新发送授权。
- 参数只在configure时读取，运行中改参数不会在线调节控制律；需停用后重新配置。
- 双轴独立配置目标／增益，仍共享一条总线与整组故障停止，不承诺两个Float64目标原子同步。
- `q=scale×qm+offset`时，下层按既有虚功映射发送`τm=scale×τjoint`。
  新刚度／阻尼在关节坐标；旧原生MIT增益在电机坐标，非单位scale时不能直接相互拷贝。
- 阻尼速度参考固定为0，与该STM32函数一致。没有积分、摩擦补偿、在线变阻抗或人体意图识别。

## 参数与故障行为

以下为仓库单、双机YAML保守示例，不是已批准的运动包络；原2 N·m位置模式试验参数未自动套用。

| 参数 | 示例值／单位 | 作用 |
|---|---|---|
| stiffness | 15 N·m/rad | 关节坐标位置误差产生的弹簧力矩 |
| damping | 0.5 N·m·s/rad | 对实测速度施加反向阻尼 |
| feedforward_effort | 0 N·m | 固定前馈，绝对值不得大于max_effort |
| max_effort | 0.1 N·m | 计算力矩双向裁剪，不是电机硬件限流 |
| max_effort_slew_per_second | 1 N·m/s | 对裁剪后的力矩限制每周期变化，避免启用／换向跳变 |
| minimum / maximum | −12.5 / +12.5 rad | 上游位置目标裁剪；协议轴示例，不能直接作为穿戴关节范围 |
| max_slew_per_second | 0.03 rad/s | 位置参考变化率，独立于力矩斜率；不保证实际速度 |
| max_position_error | 0.3 rad | 限斜率后的参考与实测位置偏差超界返回ERROR，非裁剪后继续 |
| max_feedback_velocity | 2 rad/s | 本控制器关节坐标超速检测；底层电机坐标速度保护仍独立 |
| target_ttl_nanoseconds | 100000000（100 ms） | 目标失效软期限：冻结力矩与generation，不再跟随反馈计算新力矩 |
| target_hard_ttl_nanoseconds | 106000000（106 ms） | 硬期限返回ERROR，底层独立租约故障停组 |
| 下层command_ttl / hard_ttl | 4 / 6 ms | 控制器发送租约；上层冻结不能把设备授权永久续期 |
| 下层feedback_ttl / query_period | 40 / 20 ms | 反馈有效期与查询；控制器标量接口本身不携带源时间戳 |
| 下层max_command_effort_nm | 双机0.1 N·m；单机标准0.3 N·m | 每轴电机坐标独立拒绝过大命令，须覆盖映射后的主控上限且符合实物包络 |

所有算法参数必须有限，K/D非负，力矩／斜率／偏差／速度阈值为正。
位置或速度非有限、实际位置越界、偏差／速度超界、计算溢出都返回ERROR且不刷新generation。
Holding仍检查反馈，但不会产生新的力矩；Expired不写新命令。
当前Humble管理器可能在update返回ERROR后仍显示控制器active；不能用该标签证明设备状态。
实际停组由底层租约故障和失能读回判断，故障后恢复目标发布不能自动恢复电机。
恢复必须显式处理生命周期与故障、重新核验反馈，并提交新目标。
反向行程、绝对行程、边界方向、反馈故障、传输故障及停组／失能读回继续由既有下层承担。
改变controller限值不会修改底层限值，两套坐标与限制必须共同审核。
零力矩、目标失效或退出控制器不等于机械制动；PD控制也不保证静止到位。

## 部署与验收

已有 `motor_backend_bringup.launch.py` 新增选择值 `dm4340_command_mode:=impedance`：
底层仍传入 `command_mode:=effort`，加载通用阻抗插件，原生Kp/Kd为0。
支持 `backend:=dm4340` 或 `backend:=dm4340_dual`，默认硬件／控制器inactive、
`enable_on_activate=false`，所有设备证据闸门保持原值和校验。
加载物理launch可能打开设备，因此本轮测试仅评估launch/xacro，不运行该物理入口。

单机控制器名 `dm4340_impedance_controller`；双机为
`dm4340_motor1_impedance_controller`、`dm4340_motor2_impedance_controller`。
各自接收其私有 `target_position` 话题。未加入50Hz整机Python策略的POSITION命令路由，
当前上游须显式提供该通用控制器的位置目标；不能声称穿戴整机闭环已完成。

离线测试包含公式、正负饱和、独立位置／力矩斜率、实测位置初始化、
无目标无授权、TTL冻结／过期、反馈故障、错误参数及重新激活不重放；
另用真实controller_manager、生产Dm4340System和假的串口，验证双轴独立力矩、
MIT原生Kp/Kd为0、单网关、释放失能和目标丢失停组。
这些验证不能代表实际机构／摩擦／电流环／Jetson调度精度已验收。

完整构建、首轮失败与修正、最终测试结果见
[10月22日软件验收记录](../validation/2026-10-22-stm32-host-impedance-port.md)。

[10月22日授权实机首测](../validation/2026-10-22-host-impedance-bench.md)：
K20/D2、2N·m包络下1号偏移阶段反馈速度0.202686rad/s超过0.2上限而停组，
返回和2号目标未执行；最终独立确认两台失能。未自动重试或升阈值。

[起动速度上冲分析](../validation/2026-10-22-host-impedance-onset-analysis.md)提出
K30／D4、参考斜率0.015rad/s、力矩上限2.5N·m、总力矩斜率20N·m/s候选；
斜率同时影响增力／卸力，默认参数未变，未自动执行。

## 后续独立实现

用户后续重新选择STM32实际PD内核，新增[固定参考位置阻抗](fixed_position_impedance.md)。
该实现不恢复本文历史轨迹与力矩斜率；本文原有实机结果不自动适用于新模式。
