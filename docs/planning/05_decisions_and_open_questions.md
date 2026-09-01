# 决策导航与待确认项

> 最近收敛：2026-08-19
> 作用：把正式 ADR、现场上下文和仍会阻塞设备接入的问题放在一个短入口中。
> 边界：本文件不复制 ADR 决策正文，也不代表真实设备、CAN 拓扑或物理力矩已经验证。

## 1. 正式决策导航

当前规范状态见 [ADR 索引](../adr/README.md)：

| ADR | Status | 当前边界 |
|---|---|---|
| [ADR-001](../adr/ADR-001-core-boundary.md) | Accepted | 纯 C++ 核心与薄 ros2_control 适配层 |
| [ADR-002](../adr/ADR-002-bus-runtime-ownership.md) | Accepted | 每条物理 CAN 总线只有一个进程内 `BusRuntime` 写者 |
| [ADR-003](../adr/ADR-003-composite-system-interface.md) | Accepted | Foundation/MVP 使用配置驱动的复合 `SystemInterface` |
| [ADR-004](../adr/ADR-004-fixed-protocol-profile.md) | Accepted | configure 期固定协议代际和 active command profile，ACTIVE 期间不猜测或混发 |
| [ADR-005](../adr/ADR-005-monotonic-time-freshness.md) | Accepted | 单调时钟管理 freshness/TTL，源时间与到达时间独立保存 |
| [ADR-006](../adr/ADR-006-conditional-can0-deployment.md) | Proposed | 当前单物理通道及其 transport backend 只是等待设备、能力和总线证据的条件式 profile |
| [ADR-009](../adr/ADR-009-effort-semantic-gate.md) | Accepted | 标准 `effort` 必须经过物理语义证据闸门 |

Accepted 只接受对应文件的架构或语义，不等于 ARM64、vcan、真实 CAN、设备兼容、500 Hz 或物理力矩精度已经验证。任何冲突以 ADR 状态和正文为准。

## 2. 当前现场上下文

| 项目 | 当前记录 | 证据性质 |
|---|---|---|
| 电机 | 两台基于 AKE60-8 的定制驱控一体电机，加装 CubeMars 双编码器；结构设计不属于软件任务 | 用户确认；实机配置待证 |
| 电机资料 | **2026-09-01 更正（[ADR-013](../adr/ADR-013-ak30-protocol-baseline.md)）**：适用手册为 L07（AK3.0 V3.2.0），力控与伺服扩展帧都支持，先实现力控扩展帧 | 用户确认事实 + 规划决定，见 [06](06_cubemars_material_review.md) |
| HighTorque 资料 | ROS2 SDK 是集成参考；`hightorque_fdcan` 是 USB CDC raw-CAN transport 参考，不是电机 codec | 资料事实 + 规划决定，见 [04](04_source_register.md)、[06](06_cubemars_material_review.md) |
| HI12 | 当前有两台；标准 CAN 交付通常为 J1939，但每台交付协议、ID、位速率和输出 profile 尚未读取 | 用户提供 + 待确认 |
| CAN transport | **用户确认（2026-08-23）：电机将接入高擎通用盒子**（`company/4.png`，7路CAN功率板独立形态）的 XT30(2+2) 电源+CAN 通道；Jetson 经 Type-C USB CDC（7×`/dev/ttyACM`，每通道一个串口）收发原始帧。两台 HI12 接盒子通道或独立适配器**均为候选，未定**。USB-CDC transport 由此升级为主部署路径候选，SocketCAN 转为测试路径与备选。关键未知：每通道 nominal 波特率固件固定且数值未知（见 §5.2 问题 1/2） | 用户确认拓扑意向 + 资料事实；ADR-006 仍 Proposed，激活仍需全部证据 |
| 断能 | 项目方说明已有遥控继电器断能并曾验证 | 用户提供；G3 前仍需当前书面证据 |
| 项目平台基线 | 目标为 Jetson Orin ARM64 / Ubuntu 22.04 / ROS 2 Humble | 构建基线；FND-004A 尚未验证当前目标机 |
| 实际目标机 | `P3767-0000` Orin NX 16GB 模组 + **HZHY HYAI-311UAV 第三方载板**（2026-08-23 实物核验推翻此前"标准 P3768 参考载板"记录）；**2026-08-23 已完成迁移**：JetPack 6.2 / L4T R36.4.3 / Ubuntu 22.04.5 / 内核 5.15.148-tegra，`nvidia-l4t-*` 已 hold、ROS 2 Humble 已装 | 已确认事实（刷机 + 首启验收 + 加固均完成，见升级教程 §12.0/§14.0）；FND-004A 平台前置已满足，烟测待执行 |

供应商参数、Jetson 状态和硬件状态在本轮未重新读取时不得写成当前事实。

## 3. 已确认的实施解释

- **协议选择（2026-09-01 修订）：** 当前电机区分 `AK3.0 force-control (extended, mode ID 8)` 与 `AK3.0 servo extended (modes 0-6,15,16)` 两个 profile，先实现前者；AK V3 profile 作为补充证据集保留。配置期绑定，ACTIVE 期间禁止自动探测、混发或热切换；详细协议证据见 [06](06_cubemars_material_review.md)。
- **Transport 选择：** `RawCanFrame`/`BusRuntime` 下同时保留 Fake、SocketCAN 和 HighTorque USB-CDC backend。**2026-08-23 起 USB-CDC 为主部署路径候选**（用户确认电机接通用盒子通道；每盒子通道即一个 `/dev/ttyACM` 设备，与 ADR-002 单写者一一对应），SocketCAN 为测试路径（vcan）与备选实链路。USB-CDC backend 必须以注入式串口、CDC CRC/批量帧 golden vectors 和明确能力失败语义验证，不能直接把供应商 `canport` 类放进 SystemInterface；缺失能力（无时间戳、错误上报需自行实现、波特率固件固定）必须显式声明为 unavailable/固定值，不得伪造。
- **控制职责：** 驱动器负责 FOC、高速电流环和所选模式的内部闭环；Jetson C++ 控制器读取标准状态并产生有界目标。Python 不拥有电机命令接口。
- **最小 demo：** 恒定命令只验证 controller → hardware → session → BusRuntime → CAN → state/diagnostics 链路。目标必须可配置、有限、受限、带 slew/TTL 且可回零；它不是最终控制算法或物理力矩精度验收。
- **频率：** 当前两电机候选正常目标为 500 Hz，HI12 初始 100 Hz；100～200 Hz 用于模拟/首次 bring-up；代码支持 1 kHz 控制循环实验。控制循环、命令、反馈和 IMU 是独立频率，重复快照不得伪装成新样本。
- **标准 effort：** 标准 AKE60-8 的 `Kt = 0.7382 N*m/A` 与输出端公式只是定制实机候选。只有实机固件、方向、减速比、字段和配置证据闭合后才暴露标准 `effort`；否则使用明确的 current/raw 接口。
- **设备扩展：** 新 CAN 设备优先只增加 codec、device session、capability、配置和测试，不为品牌修改 core/controller 公共语义。
- **实施顺序：** 缺少实机配置不阻塞 Foundation。**平台迁移已于 2026-08-23 完成**（HZHY 镜像 + `l4t_initrd_flash`，JetPack 6.2 / Ubuntu 22.04.5；ROS 2 Humble 已装），FND-004A 已无平台阻塞，是当前下一个可执行闸门。FND-004A 通过后才开始 FND-005，Foundation RC/AdapterContract v1 冻结后再接入真实 CubeMars/HI12 adapter。执行记录见 [升级教程](../development/jetson_orin_nx_jetpack6_upgrade_guide.md)。

架构上下文见 [02](02_architecture_and_interfaces.md)，量化验收与总线预算见 [03](03_mvp_delivery_plan.md)，当前任务顺序见 [07](07_framework_bootstrap_plan.md)。

## 4. 仍待确认的问题

| ID | 当前状态 | 待确认项 | 所需证据 | 影响 |
|---|---|---|---|---|
| OQ-01 | 部分解决 | 基型为 AKE60-8；两台定制件号、驱动板和固件未知 | 每台连接/版本记录、序列号和只读配置导出 | 协议代际、缩放和范围 |
| OQ-02 | 资料部分解决 | **L07 适用性已由项目负责人 2026-09-01 确认并有驱动板证据**；每台实际固件、active profile、CAN ID/反馈设置（含单圈模式与 `0x2A` 的 Flash 持久状态）仍未知 | 只读固件/配置记录、两套 L07 golden frame 与后续被动证据 | codec、claim、lifecycle |
| OQ-03 | 资料部分解决 | `0x29` 的编码器来源仍未知——L07 在力矩与转速处明写「输出端」，位置处未写；`0x2A` 来源及能否同读两者也未知 | 供应商帧说明、配置和后续被动证据 | `joint/position` 来源 |
| OQ-04 | 资料部分解决 | 标准 Kt/范围已知；定制版是否保持不变未知 | `.AppParams`/`.McParams` 与供应商确认 | 标准 `effort` 映射 |
| OQ-05 | 协议范围解决，性能待测 | **L07 反馈协议范围为 1–2000 Hz，上限不再是协议约束而是总线预算约束（500 Hz 为当前目标）**；实际设备稳定性、transport latency 和 USB batching 未知 | 配置、离线/注入测试、后续抓包与 30 min 统计 | 有效闭环带宽 |
| OQ-06 | 资料部分解决（2026-08-23） | 失控保护机制已证实存在（上位机应用设置：失控时间/失控刹车电流（原引证 L02 p18 已随基线切换作废，L07 侧对应章节待核对），见 [06 §4.4.3](06_cubemars_material_review.md)）；出厂默认值与启用状态未知（截图为 0，疑似默认关闭），能否经 CAN 配置未知 | 逐台读取"应用功能"页并截图（G0）；§5.1 问题 1；G3 后受控断包试验验证实际行为 | 命令租约与进程崩溃安全 |
| OQ-07 | 未解决 | 两台 HI12 的协议、节点、位速率和输出 profile | PNAME/APP_VER、逐台配置读取和抓包 | HI12 接入方案（盒子通道或独立适配器） |
| OQ-08 | 问题重塑（2026-08-23） | 原"四设备共享单总线的共同位速率"问题因电机确认接盒子分通道而降级；**新关键问题：盒子每通道 nominal 波特率由固件固定、数值未记载、无配置命令**——电机通道能否跑 Classic CAN 1 Mbps、HI12（默认 500 kbps，可改 125k~1M）能否上盒子通道均未知 | 高擎书面答复（§5.2 问题 1/2）+ 实物板卡固件核对 + G1 抓包 | ADR-006 状态、HI12 接入方案 |
| OQ-09 | 未解决 | Jetson 500 Hz/1 kHz 周期与 command-to-wire 性能 | nominal/stress 原始时序 | RT 内核/第二总线需求 |
| OQ-10 | 部分解决 | 标准命令语义已收敛；定制实机物理输出误差未知 | 实机参数和必要的外部计量 | 对外宣称与物理验收 |

## 5. 供应商问题清单（2026-08-23 资料审查产出）

以下问题在 2026-08-23 对 `company/` 全部供应商资料的系统审查后仍无法从资料闭合，需分别向供应商书面确认。答复应作为证据登记（含日期与原文），并按第 4 节对应 OQ 项收口。

### 5.1 致 CubeMars（电机）

背景：项目电机为基于 AKE60-8 的定制驱控一体双编码器电机；用户已确认 AK2.0 V1.0.18 手册（L02，已于 2026-09-01 更正为 L07）的 CAN 协议适用。

1. **失控保护默认值与配置路径**：V1.0.18 手册第 18 页"应用功能"含"失控时间(ms)"与"失控刹车电流(A)"。两参数的出厂默认值是多少？默认是否启用？除 CubeMarsTool 外能否通过 CAN 报文配置？
2. **运控/MIT 模式归一化常量**：手册第 43 页示例代码使用固定常量（P ±12.5 rad、V ±30 rad/s、T ±18 N·m、Kp 0-500、Kd 0-5），而第 42 页各型号物理范围差异很大。我们的定制 AKE60-8 机型固件在 MIT 编码/解码时使用哪一套 MIN/MAX 常量？
3. **型号覆盖确认**：V1.0.18 第 42 页型号表不含 AKE60-8。定制机固件对应该手册的哪个型号行/参数集？其规格（含编码器位数：V1.0.18 写 14-bit 单圈，AK V3.2 双编码器资料写 21-bit 内环/15-bit 外环）以哪份为准？
4. **编码器来源**：CAN `0x29` 反馈的位置字段来自内环编码器、外环编码器还是固件融合值？可否经 CAN 同时读取两个编码器？
5. **波特率**：定制机 CAN 波特率是否严格固定 1 Mbps？是否存在任何受支持的其它档位？
6. **终端电阻**：驱动板是否内置 120 Ω 终端？默认状态与配置方法？

### 5.2 致高擎 HighTorque（通用盒子 / 7路CAN功率板）

背景：项目计划将电机接入通用盒子（`company/4.png`，即 7路CAN主控盒子内功率板的独立形态）的 XT30(2+2) 电源+CAN 通道，Jetson 经 Type-C USB CDC（7×`/dev/ttyACM`）收发原始 CAN 帧（`MODE_FDCAN_PASS`）。

1. **仲裁段波特率（最关键）**：每路 CAN 通道的 nominal（仲裁段）波特率是多少？Classic CAN 帧模式下总线速率是多少？能否按通道配置？说明书"FDCAN波特率：5Mbps"我们理解为数据段速率，请确认。
2. **第三方 Classic CAN 设备兼容性**：通道能否挂接非高擎的 Classic CAN 设备——具体为固定 1 Mbps 的 CubeMars 电机与默认 500 kbps 的 HiPNUC HI12？
3. **错误上报语义**：`MODE_FDCAN_MOTOR_STATE (0x0F)` / `MODE_FDCAN_MOTOR_STATE2 (0x11)` 的完整字段定义与错误码表？（示例库定义了命令码但未实现解析。）
4. **板卡与固件身份**：交付板卡的准确型号/硬件版本？当前固件版本？`hightorque_fdcan` 示例要求固件 ≥4.8.8，而 SDK 文档截图显示 v4.6——版本体系如何对应？固件升级渠道与包（`MODE_BOOTLOADER 0x0D`）如何获取？
5. **许可证**：`hightorque_fdcan` 示例源码无 LICENSE 与版权头。我们能否在内部项目中参考其协议结构自行实现？可否获得书面许可或正式协议文档？
6. **协议文档**：USB CDC 协议（帧头 0xF7、CRC8/CRC16、命令码表）是否有正式版本化文档？（本地 PDF 为飞书 wiki 快照。）协议是否在任何固件版本中提供接收时间戳？
7. **板载 YESENSE IMU**：经什么接口输出？若不使用能否禁用？
8. **上电时序**：说明书建议"先通电功率板，再通主控"；用 Jetson 替代 RK3588 作为 USB 主机时，推荐的上电/枚举时序与断连恢复行为是什么？

## 6. 下一证据动作

1. ~~平台迁移~~ **已完成（2026-08-23）**：刷机、加固与 ROS 2 Humble 安装均已执行并验收，记录见升级教程。
2. **当前下一步：执行 FND-004A**——只验证目标 Jetson 上的 clean clone、context、依赖和五包 build/test，不读取或修改设备。同时可并行把 §5 两份供应商问题清单发出。
3. Foundation 后的 G0 取证先分别只读保存两台电机的连接/版本页、基础设置、`.AppParams` 和 `.McParams`；不得执行参数识别、写入、恢复默认或升级。
4. 两台 HI12 必须逐台记录身份、交付协议、节点、位速率和输出 profile，再评估合并总线。
5. 配置证据仍不能回答编码器来源、watchdog、定制 Kt 或终端时，再向供应商提出精确问题。
6. 任何证据与 ADR 冲突时，停止受影响的真实 profile，更新本表并按 ADR review trigger 重审；不得静默改写或继续激活。
