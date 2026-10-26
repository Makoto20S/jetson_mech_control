# DM4340 双电机单总线部署（EXO-ARCH-001）

## 范围与结构

复用现有 `Dm4340System → Dm4340Runtime → 两个 Dm4340MitSession`，
两轴共用一个 USB-CDC transport、一个总线所有者和一个 controller_manager。
控制器仍为通用 PositionCommandController / VelocityCommandController / EffortCommandController，不增加品牌分支。
单机 `backend:=dm4340` 保持兼容；双机选择 `backend:=dm4340_dual`。

本入口已完成软件部署、离线验证及记录范围内的双机离体空载短时试验，
不是穿戴配置，长期稳定性与负载验收须独立取证。
两个关节只是输出轴坐标，不预设左右腿、传动比或人体零位。

```text
上层策略/测试目标
  ├─ motor1 通用控制器 ─ joint1 ┐
  └─ motor2 通用控制器 ─ joint2 ┴─ Dm4340System
                                      └─ 单一共享 runtime / USB-CAN
                                           ├─ session1 → 电机1
                                           └─ session2 → 电机2
```

## 配置及证据

新增配置：`dm4340_dual.urdf.xacro` 与 `dm4340_dual_controllers.yaml`。

| 项目 | 电机1默认值 | 电机2默认值 | 含义 |
|---|---|---|---|
| joint | dm4340_motor1_joint | dm4340_motor2_joint | ROS 轴名 |
| command_can_id | 1 | 2 | MIT 命令标准 CAN ID |
| motor_id | 1 | 2 | 反馈负载中的电机 ID |
| feedback_can_id | 0 | 0 | 反馈标准 CAN ID |
| firmware_id | 5010107 | 5010107 | 当前已支持的 DM4340 固件基线，待逐台核对 |
| joint_scale | 1 | 1 | 电机反馈到关节坐标的比例/方向 |
| joint_offset_rad | 0 | 0 | 坐标零偏 |
| position_min_rad / max | -12.5 / 12.5 | -12.5 / 12.5 | 协议范围示例，不是机构允许行程 |
| position_guard_margin_rad | 0.02 | 0.02 | 下层边界方向保护缓冲区 |

命令与负载 ID 1/2 已在2026-10-21真实查询中应答，当前两台反馈 CAN ID 均为0。
默认配置已按本轮读回修正；共享反馈ID通过负载电机ID分流。
历史 ID 来源是用户提供的 STM32 仓库
[C620--Demo-- / dm 分支固定快照](https://github.com/Makoto20S/C620--Demo--/blob/3211a7ae90e6a0960baea20d7f3cf3c893a2c95e/User/dm_motor_ctrl.c)：
初始化行24/39为1/2，接收路由行204/205为4/5，均使用 FDCAN1。
`mst_id=0` 注明未使用；默认 master 分支不是这一份达妙代码。
4/5是历史软件期望，已被本轮实际反馈0/0取代。
历史代码的 DM4310 编解码范围不沿用；本配置仍使用 DM4340 的
位置±12.5 rad、速度±10 rad/s、力矩±28 N·m 协议缩放。

每轴上述参数均可经 launch 覆盖，参数名为 `motor1_<项目>` / `motor2_<项目>`；
例如 `motor2_feedback_can_id:=5` 仅适用于之后核对确为5的设备；不自动修改设备 ID。
固件参数只声明已核对身份，改变数字不会自动获得新固件兼容能力。

共享参数沿用单机入口：设备路径显式传入；逻辑总线1、控制500 Hz、查询每轴50 Hz（双轴相隔10ms发送）、
反馈有效期40 ms、下层命令TTL4 ms/硬截止6 ms。两机必须使用同一命令模式。
板版本示例4.8.8，CAN配置1 Mbit/s；部署前须核对实际网关能力与总线负载。

| 模式/参数 | 数值 | 作用 |
|---|---|---|
| velocity 控制器及下层目标限幅 | ±0.3 rad/s | 双机初始低速范围 |
| velocity_kd | 默认3，可选(0,3] | MIT 速度阻尼增益 |
| velocity_feedforward_nm | 默认0，可选[0,0.5] | 速度模式共享前馈；首次双机保持0 |
| effort 控制器及下层限幅 | ±0.1 N·m | 双机初始力矩范围；Kp=Kd=0 |
| controller max_slew_per_second | 1 | 目标变化速率，按对应模式单位/s |
| controller TTL / hard TTL | 100 / 106 ms | 上游目标停止刷新后失效 |
| max_feedback_velocity_rad_s | 2 | 每轴反馈超速触发故障 |
| max_feedback_torque_nm | 5 | 每轴反馈力矩超限触发故障 |
| Kp | 0 | 两种模式均不施加位置弹簧 |

完整安全语义见[安全参数表](control_safety_parameters.md)。
实际位置上下限必须换成每轴独立测量得到的安全值，不能拿协议范围用于穿戴。
下层位置、速度、力矩、反馈过期或命令故障触发整组停止与失能发送；
这不等于机械瞬间静止或独立硬件急停。停用必须两台均有新鲜失能状态读回才报告成功。
释放单个控制器只撤销该轴；故障时则整组联停。生命周期使能/失能管理报文逐台错开发送，错误退出也须核对失能。

## 使用入口

完全离线展开（不访问端口）：

```bash
source /opt/ros/humble/setup.bash
source <工作区安装目录>/setup.bash
xacro $(ros2 pkg prefix mech_bringup)/share/mech_bringup/config/dm4340_dual.urdf.xacro
```

实机前先逐台确认固件、命令/反馈 ID、输出轴方向和单位，再确认两台接在同一网关的同一路 CAN；
两路 CAN 不能使用本单总线入口。核对端点、终端电阻和新鲜反馈，按 G0–G3 完成当次现场确认。
不要运行历史 STM32 程序作为只读探测，它包含写模式、保存和使能逻辑。

完成核对后使用现有 `motor_backend_bringup.launch.py`，指定：

- `backend:=dm4340_dual device_path:=/dev/ttyACM1`
- `dm4340_command_mode:=velocity` 或 `effort`
- 每轴实测 ID、映射、位置上下限及缓冲区参数。
- `board_version_verified`、`can_bitrate_verified`、`joint_mapping_verified`、
  `motor_side_timeout_verified` 仅在两台均符合现有契约时声明为true；默认false。
  本实现不读写超时寄存器，也不改变既有用户验证记录。
- `enable_on_activate:=false` 为默认；启动后硬件和两台选定模式控制器均 inactive，
  不自动激活。后续使能属于另行明确授权的实机步骤。

`supervised_effort_bench` 是已有单机专用 profile，双机入口会拒绝它。
不要启动两个单机 launch 来控制同一个串口。

目标话题（Float64，两个模式只加载其中一种）：

| 轴 | 速度目标 | 力矩目标 |
|---|---|---|
| 1 | /dm4340_motor1_velocity_controller/target_velocity | /dm4340_motor1_effort_controller/target_effort |
| 2 | /dm4340_motor2_velocity_controller/target_velocity | /dm4340_motor2_effort_controller/target_effort |

`/joint_states` 按两个 joint 名识别轴。独立目标话题不是多轴原子同步事务，
本轮不宣称严格同步步态控制、实时性或穿戴助力验收。

## 验证与实机顺序

离线覆盖真实 launch→xacro→双轴接口及 YAML 对应关系、默认 inactive、两种模式、
单机兼容、参数字符拒绝；runtime 验证历史1/4与2/5组合同时匹配CAN ID和负载ID，
并复用独立命令、重复ID拒绝、单机故障整组联停测试。
生产插件通过 FakeSerial/实际 USB-CDC codec 验证两机使能、同端口单次打开、
两台失能读回，以及第二台拒绝失能时不能误报停用成功。

设备接入后另行执行：

1. 核对两台身份和只读反馈，记录实际映射、范围，不自动扫描/改ID。
2. 单轴受限动作，另一轴无运动目标，分别确认方向与反馈分流。
3. 双轴受限动作；确认目标独立、反馈新鲜和端口唯一所有者。
4. 有界验证反馈丢失/边界等故障响应，以及两台最终失能读回。

离线通过不代替上述实机证据；具体运动参数在当次测试前确定。

### 本轮软件结果（2026-10-21）

Jetson ARM64 / ROS 2 Humble 原生依赖链7包构建通过；最终 bringup 重建通过。
`colcon test --packages-select mech_bringup`：326项，0错误、0失败、0跳过；
单独运行启动配置检查12项全部通过（同一启动检查也已纳入 bringup 测试）。
文档链接、脚本语法、context基线和 `git diff --check` 通过。
这是本地未提交工作区的软件结果，没有远程CI或双电机实机证据。

### 双机真实只读查询（2026-10-21）

现场确认与端口临时授权后，复用生产 PosixCdcSerialPort/UsbCdcTransport/达妙codec，
对已知ID1、2各发送一次0x7FF状态查询（0xCC），不使能、不发MIT运动帧、不读写配置寄存器。
两台反馈CAN ID均0，负载ID分别1/2，状态均0（失能）。按DM4340缩放解码：
电机1位置-1.115244 rad、速度-0.002442 rad/s；电机2位置0.072671 rad、速度-0.007326 rad/s。
该次没有重新核对固件版本或做计量校准；不扩大为固件身份、双机运动或停机验收。
有界查询结束后关闭端口。原始日志保存在本机验证目录，不提交设备日志。

实际接入发现管理报文集中发送丢反馈，修复后双机使能复测及逐轴正向空载运动已通过；
另已通过一次双机同时独立空载运动；精确同步、负载和穿戴仍未验收。详见
[双机台架记录](../validation/2026-10-21-dm4340-dual-bench.md)。

## 位置阻抗模式

新增 `dm4340_command_mode:=position`，加载两台通用 PositionCommandController，
每轴使用独立位置目标、Kp/Kd及位置偏差包络。参数、控制律与待实机验收项见
[位置阻抗说明](dm4340_position_impedance.md)。本模式不沿用速度长测的实机验收结论。
