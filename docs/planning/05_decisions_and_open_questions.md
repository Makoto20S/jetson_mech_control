# 已确认决策与待确认项

> 更新日期：2026-08-03
> 作用：记录总体规划完成后的讨论结论；与早期规划冲突时，以本文件中标明的最新决定为准。
> 边界：本文件只更新设计与验收语义，不代表真实设备、固件、CAN 总线或物理力矩已经验证。

> FND-004 已完成：正式规范状态见 [ADR 索引](../adr/README.md)。本文件继续保存讨论背景和实机待确认项；与 ADR 冲突时，以 ADR 为准。

## 1. 证据标记

- **规划决定**：当前同意采用的架构或工程基线，后续可由测试或 ADR 修订。
- **用户确认事实**：项目成员明确确认的设备背景或任务边界；仍不替代每台实机配置证据。
- **用户提供（待独立确认）**：由项目成员描述、但尚未用铭牌、配置导出、抓包或测量复核的现场信息。
- **资料事实**：可由当前本地手册直接支持的结论。
- **部分确认**：通用资料或基型已确认，但定制实机仍有影响实现的参数未锁定。
- **待确认项**：会影响协议、接口、性能或验收，仍需供应商资料或现场证据。

## 2. 当前硬件与部署上下文

| 项目 | 当前记录 | 性质 |
|---|---|---|
| 电机 | 当前有两台基于 AKE60-8 的定制驱控一体电机，加装 CubeMars 双编码器；结构重新设计不属于软件任务 | 用户确认事实 |
| IMU | 当前有两台 HI12；协议、节点 ID、位速率和输出 profile 尚未读取 | 用户提供（待独立确认） |
| CAN | 当前四台设备计划共用 Jetson 的 `can0`，暂无第二路接口 | 用户提供（待独立确认） |
| 断能 | 已有遥控继电器切断电机电源，并由项目方说明此前已验证 | 用户提供（待独立确认） |
| 未来平台 | 新 Jetson 到位后再评估第二路 CAN 和最终物理拓扑 | 用户提供（待独立确认） |

## 3. 已澄清的架构决定

### 3.1 核心边界

当前规范：[ADR-001](../adr/ADR-001-core-boundary.md)、[ADR-002](../adr/ADR-002-bus-runtime-ownership.md) 和 [ADR-003](../adr/ADR-003-composite-system-interface.md) 均为 Accepted。

**规划决定**：继续采用“纯 C++ 传输/协议/时间/设备核心 + 薄 ros2_control 复合 `SystemInterface`”。`SystemInterface` 导出标准状态和命令接口，但不包含厂商 CAN 位域或直接阻塞收发逻辑。

**规划决定**：每条物理 CAN 总线由一个 `BusRuntime` 协调接收、路由、发送调度、时间戳、错误状态和命令租约。控制器不打开 SocketCAN，也不构造厂商 CAN 帧。

### 3.2 CubeMars servo 与力控/MIT-like

当前规范：[ADR-004](../adr/ADR-004-fixed-protocol-profile.md) 为 Accepted；接受的是配置期固定和失败关闭规则，不表示实机协议代际已经确认。

**规划决定（2026-08-03 修订）**：框架分别实现并测试 `AK V3 servo extended`、`AK V3 force-control extended` 和 `legacy MIT standard-frame`。当前 AKE60-8 只把前两个作为候选；legacy profile 仅用于明确匹配的旧固件。设备在 `on_configure` 时固定协议代际、active command profile 和接口 claim，ACTIVE 期间不自动猜测或混发。

**资料事实**：最新且明确列出 AKE60-8 的 AK3.0 V3.2 手册中，servo 使用扩展帧并提供电流、速度、位置等驱动器内部闭环命令；力控/MIT-like 同样使用扩展帧，控制模式 ID 为 8，payload 顺序为 `Kp, Kd, position, velocity, torque`。手册称两者使用无需切换。旧 Arduino demo 的标准帧 `position, velocity, Kp, Kd, torque` 属于冲突的旧代际参考，不能用于当前 codec。详见 `06_cubemars_material_review.md`。

### 3.3 C++ 控制器与驱动器内部闭环

**规划决定**：驱动器继续负责 FOC 和高速电流环。简单恒位置或恒速度场景中，C++ 控制器产生位置/速度目标，驱动器内部位置环或速度环执行目标。

**规划决定**：阻抗、滑模、PID、学习策略融合等关节级算法由 Jetson 的 C++ 控制器读取电机与 IMU 状态，计算目标位置、速度或力矩。Python 不直接拥有电机命令接口。

### 3.4 最小恒定力矩命令 demo

当前规范：[ADR-009](../adr/ADR-009-effort-semantic-gate.md) 为 Accepted；定制实机标准 `effort` 映射仍需 OQ-01/OQ-04 和 G0–G3 证据。

**规划决定**：恒定力矩命令只是验证控制框架的最小 demo，不是项目最终控制目标。它用于贯通并观察“C++ controller `update()` -> ros2_control 接口 claim -> `SystemInterface::write()` -> device session -> `BusRuntime` -> CAN -> 状态与诊断”整条链路。

**规划决定**：demo 控制器在每个 `update()` 周期持续写入同一、可配置且受限的目标。下面数值只用于解释接口，不是默认值或验收固定值：

```text
joint/effort command = 2.0 N*m
```

`SystemInterface::write()` 对该目标执行 finite、限幅、slew、mode、状态新鲜度和租约检查，再交给 CubeMars device session；device session 按所选 profile 映射为 AK V3 servo 电流/Iq 命令或 AK V3 force-control torque 字段，最后由 `BusRuntime` 发送 CAN 帧。

**规划决定**：demo 从零目标开始，按配置的 slew 进入目标，并能回到零；控制器不直接循环发送 CAN 帧。验收重点是目标值、generation、deadline、限幅、协议映射、发送与反馈/诊断可追踪，命令停止刷新时旧命令必须在 TTL 到期后失效。

**部分确认**：V3.2 已给出标准 AKE60-8 的 `Kt = 0.7382 N*m/A`、`T = Kt * Iq`，并把 T 定义为输出端输出扭矩。这已经形成标准 `joint/effort` 的候选映射；但定制双编码器实机是否保持相同参数仍须用每台 `.AppParams`/`.McParams` 或供应商确认锁定。外部计量用于验证物理输出精度，是与最小框架 demo 分开的验收项。

### 3.5 控制频率

时间与 freshness 语义以 Accepted 的 [ADR-005](../adr/ADR-005-monotonic-time-freshness.md) 为准；单 `can0` 部署以 Proposed 的 [ADR-006](../adr/ADR-006-conditional-can0-deployment.md) 为准，当前频率预算不是激活证据。

| 场景 | controller_manager | 电机命令/反馈 | HI12 | 性质 |
|---|---:|---:|---:|---|
| 模拟与首次硬件 bring-up | 100~200 Hz | 100~200 Hz | 100 Hz | 规划决定 |
| 当前两电机正常 MVP | **500 Hz** | 目标 **500 Hz** | 100 Hz，必要时 200 Hz | 规划决定 |
| 1 kHz 性能实验 | 1 kHz | 先保持 500 Hz，再按总线实测决定 | 100/200 Hz | 规划决定 |
| 单总线六电机设计包络 | 按控制需求配置 | 200~250 Hz 初始预算 | 独立重算 | 有依据的推断 |

**规划决定**：`200 Hz` 不是当前两电机系统的最终正常频率；它是 bring-up 或六电机单总线的保守档。当前两电机的正常控制循环目标为 `500 Hz`，代码和配置必须支持 `1 kHz` 测试。

**规划决定**：控制循环、CAN 命令、设备反馈和 IMU 输出是独立频率。若控制循环为 1 kHz 而反馈为 500 Hz，控制器允许读取同一完整快照两次，但必须保留源时间、递增 age，并使用实际 `dt`。

**资料事实 + 规划目标（受 ADR-006 Proposed 约束）**：AK V3.2 的 `0x29` 状态反馈可配置 1–2000 Hz，因此 500 Hz 不是设备手册上限。500 Hz 是当前两电机候选 profile 的正常目标；两电机基础反馈加两 HI12 的保守估算约 41.6%，但该计算不证明单 `can0` 已可激活。若两电机都按 500 Hz 启用额外 `0x2A` 位置帧，占用约升至 53.6%，不再满足平均 50% 目标；`0x2A` 必须按需显式启用并重新验证 deployment。

### 3.6 新 CAN 设备扩展

**规划决定**：新增其他品牌电机或传感器时，优先只增加协议 codec、device session、capability、配置、标准单位映射和测试。若新设备能提供相同的标准 `position`、`velocity`、`effort` 或 IMU 接口，现有 C++ 控制器、`SystemInterface`、controller_manager 和实验流程不修改。

**规划决定**：厂商专用字段使用独立原始/诊断接口，不通过相似名称伪装为标准 SI 接口。

### 3.7 Foundation-first 实施顺序

**规划决定（2026-08-03）**：可以在缺少两台电机和两台 HI12 实机配置的情况下开始基础框架。当前里程碑改为 `Foundation v0.1`：Git/CI、纯 C++ core、transport abstraction、fake/vcan、BusRuntime、模拟 device session、薄 ros2_control `SystemInterface` 和模拟 demo controller。

**规划决定**：实机资料只阻塞真实 adapter 激活、最终缩放/编码器来源、`joint/effort` 锁定和物理性能验收，不阻塞 core、模拟器、生命周期和接口测试。Foundation 由项目负责人统一搭建；`AdapterContract v1` 冻结后，再把 CubeMars、HI12 和 HIL 分工实施。详细顺序见 `07_framework_bootstrap_plan.md`。

## 4. CubeMars 供应商资料审查状态

**已完成**：供应商给出的完整 `CubeMars/` 资料库已经完成软件相关检索。已找到 AKE60-8 对应的 AK3.0 V3.2 手册、标准 AK54 驱动板资料、AK V3.2.0 CubeMarsTool、CubeMars 双编码器通用参数，以及 servo/force-control 报文定义。详细采用/拒绝矩阵和 SHA-256 见 `06_cubemars_material_review.md` 与 `04_source_register.md`。

**明确排除**：不再检索或要求定制结构、2D/3D、测试工装资料。AKE60-8 驱动板归档中只有 STEP 文件，也不影响软件结论。

**资料包中没有**：两台实机的 `.AppParams`/`.McParams`、实机固件版本、AKE60-8 专用固件/协议映射、CAN 双编码器字段说明、command watchdog、DBC/EDS 或 Linux/SocketCAN SDK。

**下一步证据**：先分别连接两台电机，用匹配的 CubeMarsTool 只读并保存连接/版本页、基础设置页、`.AppParams` 和 `.McParams`；不执行参数识别、写入、恢复默认或升级。若导出仍不能回答，再向技术支持只问双编码器 CAN 来源、定制参数是否沿用标准 AKE60-8、watchdog 和终端电阻四类具体问题。

## 5. 仍待确认的问题

| ID | 当前状态 | 待确认项 | 所需证据 | 影响 |
|---|---|---|---|---|
| OQ-01 | **部分解决** | 基型已确认为 AKE60-8；两台定制件号、驱动板和固件仍未知 | 每台连接/版本截图、序列号和配置导出 | 锁定协议代际、缩放和范围 |
| OQ-02 | **资料部分解决** | V3.2 支持同固件 servo 与 force-control 且无需切换；实机是否为该代际未验证 | 实机固件版本、V3.2 golden frame/抓包 | codec 选择、claim 和 lifecycle |
| OQ-03 | **资料部分解决** | 通用 21-bit 内环/15-bit 外环已知；`0x29/0x2A` 使用哪个编码器、能否 CAN 同读两者未知 | 供应商明确帧说明、实机配置/抓包 | 标准 `joint/position` 来源 |
| OQ-04 | **资料部分解决** | 标准 AKE60-8 的 Kt、范围和输出端定义已知；定制版是否不变未知 | `.AppParams`/`.McParams` 与供应商确认 | 能否锁定标准 `effort` 映射 |
| OQ-05 | **协议范围已解决，性能待测** | 手册允许 1–2000 Hz；定制实机 500 Hz 的稳定性和时序仍未知 | 配置导出、抓包和 30 min 统计 | 500 Hz 有效闭环带宽 |
| OQ-06 | **未解决** | 命令丢失后的 watchdog/neutral 行为 | 固件资料和后续受控断包试验 | 命令租约和故障设计 |
| OQ-07 | **未解决** | 两台 HI12 的协议、节点、位速率和输出 profile | PNAME/APP_VER、逐台配置读取和抓包 | 单 `can0` 可行性 |
| OQ-08 | **未解决** | CubeMars 与 HI12 能否在 `can0` 使用同一位速率且无 ID 冲突 | 全设备配置和总线预算 | 当前物理拓扑 |
| OQ-09 | **未解决** | Jetson 500 Hz/1 kHz 周期与 command-to-wire 性能 | nominal/stress 原始时序统计 | 是否需要 RT 内核或第二总线 |
| OQ-10 | **部分解决** | 标准命令语义已收敛；固定命令与定制实机物理输出力矩的误差未知 | 实机参数；需要时使用外部计量 | 对外宣称和物理验收 |

## 6. 变更规则

**已完成（FND-004）**：核心边界已转为 ADR-001/002/003/004/005/006/009；六项为 Accepted，ADR-006 因缺少当前四设备单 `can0` 的决定性证据保持 Proposed。后续配置 schema 和运行时验收由 FND-005～FND-015 实现。

供应商资料或现场证据与本文件或 ADR 冲突时，不静默覆盖；先停止受影响的真实 profile，更新对应待确认项、记录证据来源，并按 ADR 的重审触发修改正式决策。
