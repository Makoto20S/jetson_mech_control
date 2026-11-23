# PH-EXO 上层阻抗接口

更新：2026-11-17，Asia/Shanghai。实现位于既有 `mech_assist_control`，由
`exoskeleton_control_node` 的 `policy: ph_exo_fixed_pose` 分支接入。
原调参脚本、面板和原有其他策略保留；没有新增独立管理进程。

## 调用关系

上层任务 / 调参工具 → 现有管理节点 → `FixedPosePlan` 或 `ImpedanceRef` → 原生阻抗控制器 → MIT 接口。
 固定、指定目标、连续往返共用原节点和启停服务；没有新增控制器或会话管理器。
电机反馈 → 原生控制器的 5 ms 控制周期，同时经状态节点形成 `/ph_exo/state`。
管理节点以 10 ms（100 Hz）周期生成/续发参考并监督状态。原生控制器仍每 5 ms 用最新反馈计算力矩。
 轨迹模块 `mech_assist_control.robot.ph_exo_reference` 只计算 q/dq，不计算阻抗力矩，
 不读取文件或调参历史。独立调参脚本复用此模块，保留原专用启动与编码器预检。

本分支运行在 Orin，与控制器共用单调时钟。电脑端通过既有工具连接；
不把电脑的单调时间戳作为 Orin 的计划时间。ROS 调用无需终端输入 START；
原台架脚本的确认方式不受影响。

## 服务

下列名称以节点无命名空间为例；添加命名空间时统一前缀。服务类型均为 `std_srvs/srv/Trigger`。

| 服务 | 行为 |
| --- | --- |
| `/exoskeleton/impedance/start` | 检查指定控制器/硬件/状态，捕获当前 ψ、β，按所选固定/目标/往返模式续发参考 |
| `/exoskeleton/impedance/stop` | 撤销续发，停用目标控制器及硬件，执行对应后端的停止确认 |
| `/exoskeleton/impedance/emergency_stop` | 锁存软件急停，采用相同的目标停止和失能确认流程 |
| `/exoskeleton/impedance/reset` | 停止确认且后台处理完成后返回调试；不发送力矩，不自动使能 |
| `/exoskeleton/impedance/status` | 即时返回与状态话题相同的 JSON |

`start/stop/emergency_stop` 的 `success=true` 表示请求已接收。
用状态话题中的 `state` 判断完成；不要把服务接收当作电机已进入保持或已经失能。

典型流程：

```text
debug → start → preparing → starting → running
running → stop / emergency_stop / 状态失效 → stopping → stopped
stopped → reset → debug
停止无法确认 → unconfirmed（reset 返回失败）
```

重新开始必须再次调用 `start`，使用新鲜反馈重新捕获姿态。停止未确认时不自动重试；
解决具体故障后可显式重新请求停止，成功确认后再 reset。

## 状态

| 话题 | 类型 / 用途 |
| --- | --- |
| `/ph_exo/state` | 既有 `mech_exo_state/RobotState`；电机位置/速度、ψ/β、有效性和来源时间 |
| `/exoskeleton/impedance/state` | `std_msgs/String` JSON；管理状态 |
| `/exoskeleton/impedance/controller_state` | 原样转发既有 `ph_exo_interfaces/ImpedanceStatus`；实际控制模式、目标力矩、姿态、故障 |

新增 JSON 使用简短字段：

| 字段 | 含义 |
| --- | --- |
| `state`, `reason` | 当前阶段及原因 |
| `id`, `seq` | 本轮标识、计划序号 |
| `q_ref`, `dq_ref` | 最近一次发布的 [ψ, β] 参考，rad / rad/s；调试状态为 null |
| `ref`, `source` | 本轮 fixed/target/cycle 与 fixed_pose_v1/dynamic_ref_v1 |
| `stop_ok` | 所需停止步骤已确认 |
| `disable_ok` | [104,105] 的失能报文确认 |
| `backend`, `sim` | 后端与是否仅软件 |
| `ctrl`, `hw` | 明确指定的控制器与硬件组件 |

MuJoCo 后端正常停止时，`stop_ok=true`，`disable_ok=[false,false]`，
因为没有真实驱动器应答。PTY 的应答是模拟器应答；真实失能 ACK 也不等于独立测量的机械静止。
已有公共 ROS 消息保持其字段和单位，避免破坏现有订阅者。

## 配置与启动

默认配置：`ros2_ws/src/mech_assist_control/config/ph_exo_upper.yaml`。
新增参数使用 `hold.` 前缀及短字段：

| 参数 | 用途 |
| --- | --- |
| `hold.k` | [ψ,β] 刚度，N·m/rad |
| `hold.b` | [ψ,β] 阻尼，N·m·s/rad |
| `hold.tau` | [104,105] 目标力矩上限，N·m |
| `hold.cm`, `hold.ctrl` | controller_manager 与目标控制器的绝对 ROS 名称 |
| `hold.hw` | 目标硬件组件名称数组 |
| `hold.backend` | `simulation`、`mit_pty` 或 `mit` |
| `hold.state` | RobotState 输入话题 |
| `hold.disable` | MIT 独立失能 URDF 的绝对路径 |
| `hold.log` | MIT 失能记录目录的绝对路径 |
| `hold.source` | 启动时选 `fixed_pose_v1`（原默认）或 `dynamic_ref_v1`，与底层配置一致 |
| `hold.profile` | 默认空字符串沿用原低力矩软件范围；显式 `ref01_supported_v1` 使用已存在的样机范围 |
| `hold.ref` | `fixed` 捕获后保持；`target` 平滑到 q 后保持；`cycle` 连续往返 |
| `hold.q` | 指定目标 [ψ,β]，rad；模型范围仍为 ψ±45°、β −8～45° |
| `hold.target` | cycle 时 false=用启动姿态为基准，true=先到 hold.q 再往返 |
| `hold.amp` | [ψ,β] 有符号往返幅度，rad；ψ±45°以内、β±53°以内，至少一轴非零，端点必须在模型范围 |
| `hold.period` | 每轴完整往返周期，秒，1～60 |
| `hold.move` | 指定目标的平滑过渡时间，秒，1～60 |

默认仅软件，K=[2.4,2.0]、B=[0.18,0.15]、每台力矩上限 0.2 N·m。
这是原软件验证配置，不改原调参脚本的参数。
只在 debug 状态允许修改 `hold.k/b/tau/ref/target/q/amp/period/move`；每轮启动后固定。
 地址、后端、source/profile 在启动节点时配置，不在线切换。当前合同 K≤30、B≤1、每台力矩≤10 N·m。
 默认软件范围仍是原低增益值；候选配置 `ph_exo_cycle.yaml` 显式选样机范围 K=[20,20]、B=[0.3,0.3]、
 tau=[10,10]，幅度示例 [3°,−2°]、周期 [6,4] 秒。加载该配置不会启动控制。
 电流6 A和相对编码器位移±90°仍归 MIT 硬件配置，ROS上层只提交目标力矩上限；
 配置上层不能扩大硬件限幅。MuJoCo没有物理电流反馈。

 ψ围绕基准做正负两侧往返，符号决定首向；β从基准到基准+幅度再返回。两轴周期可不同，0幅度表示该轴保持。
 采用五次曲线，位置、速度、加速度连续；先保留1秒启动参考捕获，再过渡/往返。
 上层没有保持时长，运行至显式停止、急停或原有故障停发。尚未加入在线改增益、外部步态规划等功能。

在已有管理器、目标控制器和状态节点启动后，加载本节点：

```bash
ros2 launch mech_assist_control ph_exo_upper.launch.py config:=/absolute/path/ph_exo_upper.yaml
```

该 launch 自身不启动硬件，也不自动调用 start。上层应用创建 Trigger 客户端调用上述服务，
并订阅两个状态话题即可。命令行等价示例：

```bash
ros2 service call /exoskeleton/impedance/status std_srvs/srv/Trigger '{}'
ros2 service call /exoskeleton/impedance/start std_srvs/srv/Trigger '{}'
ros2 service call /exoskeleton/impedance/emergency_stop std_srvs/srv/Trigger '{}'
ros2 service call /exoskeleton/impedance/reset std_srvs/srv/Trigger '{}'
```

接入现有 ROS 图须满足：

- 目标控制器与管理器必须同一命名空间，并在启动/停止前校验插件类型。
- 目标控制器是既有 `Ref01ImpedanceController`，配置 `command_source` 与 `hold.source` 一致；
  初始为 inactive，同一参考话题只有一个发布者和一个控制器订阅者。
  固定模式可走任一入口；target/cycle 需要 dynamic_ref_v1。动态参考经 `~/ref` 发送已有
  `ImpedanceRef{id,seq,t,on,q,dq,k,b,tau}`，不新增 ROS 消息。外部程序读取 state/controller_state，
  不额外占用受检查的控制参考话题。不同时运行第二个直接参考发布器。
- `hold.hw` 恰好覆盖 `motor104_joint`、`motor105_joint`；
  其他控制器不能占用这两个关节的命令接口。
- 状态映射已核对，位置和速度均有效。MuJoCo 使用 JointState；
  MIT 使用状态节点的原生 MIT 来源及有效主机接收时间、速度换算。
- MIT 模式的失能文件沿用已有 `Ak30ServoSystem` 失能描述；
  与目标组件的 ID、端口、逻辑总线逐项一致，并先通过 `servo_disable --check`。
- 真实 MIT 还要求 `allow_motor_output=true`、`joint_mapping_verified=true`，
  以及底层已核对的实机配置。这些布尔值不能替代配置和现场确认。

停止只操作指定控制器和两个硬件组件，不结束共享 controller_manager。
先停用控制器，再关闭目标硬件的通信，最后独立发送失能并检查两个应答。
普通停用后的空闲周期不再触发额外提交；错误转到 UNCONFIGURED 时释放端口，
使独立失能能够取得端口。正常 5 ms 提交与原超时规则不变。

## 软件验证入口与边界

`tools/project/test_ph_exo_upper_ros.py` 在隔离 ROS 域/命名空间运行实际管理节点、
官方 controller_manager、原生控制器和 MuJoCo。MIT 用测试新建的私有 PTY，拒绝物理端口。
测试覆盖两轴扰动恢复、运行中急停、状态中断、reset 不续发、显式新一轮启动，
以及缺少失能应答时拒绝 reset。报告和原始结果回传 Windows。

本轮未进行实机启停或力控验收。纯 MuJoCo 硬件是原有单轮验证夹具；
其 stop 后重跑需重建该软件夹具，不能把 reset 解释为重置模型或自动重启。
MIT 的生命周期恢复由私有 PTY 场景单独验证。该场景显式模拟失能后的外部支撑，不作为无支撑机构或实机恢复性能验收。

验证结果见 2026-11-14 接入报告（项目本地记录 upper_impedance_20261114.md）。

现有电脑调参界面已接入上述服务；默认ROS模式，原脚本以 -Legacy 对照保留。详见 [调参界面](bench_panel.md)。


## 连续参考的软件启动与调用

在已 source Humble、MuJoCo 环境和 ref01_native overlay 的 Orin 开发目录：

```bash
ros2 launch mech_assist_control ph_exo_cycle_sim.launch.py output:="$PWD/.local/validation/cycle_sim_$(date +%Y%m%d_%H%M%S)"
```

这是专用无设备组合：仅 MuJoCo 硬件插件、状态节点、inactive 阻抗控制器和现有上层节点。
上层等待显式 start；电脑 ROS 调参入口可连接它。原固定姿态 launch 和 YAML 保持可用。
MuJoCo 插件仍是原30秒软件采样夹具；这不是产品保持时长，软件重复实验应重建该夹具。
接现有实机 ROS 图时仅加载 `ph_exo_upper.launch.py config:=...`，由已有硬件入口负责电机配置；
本次未创建或执行实机自动启动入口。

程序可通过 `/exoskeleton_control/set_parameters_atomically` 一次提交上述 `hold.*` 参数，
再调用已有 `/exoskeleton/impedance/start`；停止、急停、reset 及状态订阅方式不变。
例如基于启动姿态往返：ref=cycle、target=false、amp=[0.05235988,−0.03490659]、period=[6,4]。
返回固定姿态时显式设 ref=fixed。ROS调参客户端在每次开始前完整提交参考参数，
留空目标/幅度会清除上一轮的可选模式，避免旧往返设置被意外沿用。
页面仍填角度；客户端统一换算成ROS的弧度，时间戳在Orin生成。

软件封装验证见 2026-11-17 封装记录（项目本地记录 continuous_package_20261117.md）。
此前用户已接受的样机波动结论继续保留，原数值门槛和失败标志不改写；实机验证按用户要求暂缓。


### 参考到期后的恢复

2026-11-18已补齐故障后控制器停用和硬件控制权释放：参考到期仍按原门槛停发，停止确认及双轴失能回执完成后，可调用reset回到debug。reset不重新发令，下一轮必须显式start。缺失失能回执仍保持unconfirmed。

Windows与Orin私有MIT/MuJoCo验证及原因见修复记录（项目本地记录 expiry_reset_20261118.md）。带界面的私有PTY长轮模拟反馈错相仍单独跟踪；未做实机验证。

## ψ 助力边界（普通阻抗，2026-11-22 样机采纳）

普通阻抗按实际 ψ 判界。模型计算范围为 ±60°，参考及助力范围为 ±45°，正解浮点容差 1e-8 rad。越界当周期将 ψ 阻抗贡献清零。β 阻抗和两轴重力在全程正常计算，再映射到两台电机。超过计算范围或反馈失效沿用原故障逻辑。

从任一侧回界的首个有效控制帧取 ψ 权重 0。从该帧起按单调时间在 200 ms 内将权重平滑恢复至 1，曲线为 6u⁵−15u⁴+10u³（u 为已过去时间占渐入时长的比例，限于 0～1）。再次越界会重新清零。正常在界内启动维持原启动行为。权重仅作用于 ψ 的刚度和阻尼合力矩；两台电机的总目标仍随当前速度、β 和重力变化。

Windows 复现：使用项目 Python 执行 `tools/project/verify_psi_assist.py --output .local/validation/psi_assist_new`，可取得 100 ms、200 ms 两种时长的双侧编码器回放和 MuJoCo 外力闭环逐帧 CSV 及 summary.json。

原生 ROS 控制器使用普通阻抗会话的默认 200 ms 参数，固定姿态和动态参考共用此路径。Python 对照控制器采用相同默认值。100 ms 为软件对照用时。

台架验证在每轴 K = [20, 30] N·m/rad、B = [0.3, 1.0] N·m·s/rad、每台电机目标上限 10 N·m 下进行。以下数字来自同一控制周期的 104/105 总线 MIT 目标力矩，单位 N·m。

| 手推回界方向 | 回界首帧变化绝对值（104 / 105） | 其后 200 ms 内最大单帧变化（104 / 105，含首帧） |
|---|---|---|
| 正侧 | 0.000 / 0.000 | 0.029 / 0.051 |
| 负侧 | 0.007 / 0.007 | 0.037 / 0.022 |

两次完整回界分别对应实机记录 `ref01_bench_20261122_145940_b4b6804d` 和 `ref01_bench_20261122_150102_07e3ee2a`。同两轮向外越界时出现约 0.36～0.37 N·m 的单周期力矩变化。采纳范围为样机普通阻抗的 ψ 边界切换。自动接近 ±43° 的跟踪误差另行处理。Windows 本机复算文件位于 `.local/validation/psi_ramp_20261122/bench_analysis/`。

## β 轴范围（2026-11-23 更新）

| 范围 | 当前工程行为 | 设计状态 |
|---|---|---|
| 实测姿态计算范围 | 所选模型分支中，C++ 和 Python 正解支持 −11°～+45°，含端点。Python 动力学接受同域的实测姿态。 | −11° 为软件计算余量。按实际编码器的 φ 核对装配分支，避免折返后误报有效。 |
| 参考范围 | 参考生成器及原生计划校验允许 β 目标为 −8°～+45°。 | 规划目标的限制单独管理。 |
| 助力范围（普通阻抗） | β 上侧按两电机反馈得到的 φ 判界。φ 达到 90° 时，正向 β 阻抗力矩为零；负向回程阻抗按原值计算。 | 对应当前模型 β≈+30.48°。β 下侧沿用已有模型有效性检查。 |

当前几何在 β=−11° 时对应 φ≈8.72691°，两台编码器平均角为 126.27309°。台架已扫到的 φ=30° 对应模型 β≈−6.29°。理论折返点为 β≈−11.13666°、φ≈4.46310°。−15° 来自目测；标定和几何依据见零点与几何核对（项目本地记录 beta_zero_geometry_20261122.md）。

**现行处理。** 普通阻抗已经启动时，−11°≤β<−8° 仍更新状态、动力学和目标力矩。台架从当前位置捕获启动参考时，先检查 β 在参考域内。实测 β 越过计算域或编码器落在错误分支时，正解报告无效，普通阻抗会话进入模型故障并停止产生新的双电机目标。反馈回到正确分支后，状态估计可恢复；已故障的会话需按原停止、复位和显式启动流程重新进入控制。离线回归覆盖两台编码器均为 126.2° → 126.3° → 126.2° 及直接跳到折返点另一侧的输入。

**β 上侧的方向限制。** 每个控制周期取 φ = 135° − (θ104 + θ105)/2。φ ≤ 88° 时正向 β 阻抗全量输出。在 88°～90° 之间，取 u=(90°−φ)/2°，正向 β 阻抗权重为 u²(3−2u)；φ ≥ 90° 时权重为零。进入和离开过渡区使用同一条连续曲线，φ=89° 时权重为 0.5。这个 2° 过渡宽度为软件初值。负向 β 阻抗权重始终为 1。这里的方向按刚度项和阻尼项相加后的广义力矩判断。

β 限制作用于普通阻抗的固定目标和动态参考，位于叠加重力、映射两台电机和执行原力矩限幅之前。φ=90° 等价于编码器平均角 45°。重力、外力和惯性仍可使机构越过该点。参考接口仍可接受原范围内的目标，目标超出助力边界时控制器不会继续向外助推。Windows 对照用当前模型把输出姿态换算成 φ。上侧软件用例覆盖 ψ=±12°、φ=87°～91° 往返、正负刚度力矩和纯阻尼力矩。
