# ADR-006：条件式单物理通道部署与双总线架构边界

- **Decision ID:** ADR-006
- **Status:** Proposed
- **Date:** 2026-08-07
- **Owner:** 项目负责人（批准）+ Integration/HIL owner（证据）
- **Scope:** 当前两台定制 AKE60-8 候选设备、两台 HI12、可选 transport backend 和未来最多两条物理 CAN 的 deployment profile

## Status rationale / 状态依据

架构应使用逻辑总线、可替换 transport backend，并保留两条物理总线的能力，这一方向已经明确；但“当前四台设备可安全共用一个物理 CAN 通道”仍缺少逐台位速率、节点/ID、输出 profile、transport 能力、仲裁和实测负载/错误证据。因此本 ADR 保持 `Proposed`，任何真实单通道激活都被阻止。

转为 `Accepted` 至少需要：逐台 G0 身份/配置；两台 HI12 协议、节点和位速率；完整 RX/TX ID/format/filter 表；共同位速率与终端证据；所选 backend/通信板的准确型号、固件、通道、bitrate 所有权和错误/队列能力；按实际 profile 计算并实测的平均/峰值占用、仲裁响应、queue/drop/error；以及 G1 被动总线和评审记录。个人 Memory 或旧 handoff 不能替代这些证据。

**（2026-09-02 增补，项目负责人裁定）本 ADR 约束的是「多设备共用一条物理通道」这一 deployment profile —— 即当前候选的两台 AKE60-8 加两台 HI12。它不约束「单台电机、空载空转、专用总线」的取证台架。** 依据有二：本 ADR 的 Scope 行本身就把范围写为该 deployment profile；且下文 Decision 第 6 条已把「分配独立物理总线」列为合法出路。此前 Status rationale 的「任何真实单通道激活都被阻止」一句字面覆盖过宽，容易被读成连取证台架也一并阻止 —— **该句的实际意图是阻止*最终部署 profile* 的激活，不是阻止受控取证。** 台架的具体边界见 Decision 第 7 条。这一增补不放宽 G0–G3：台架仍逐条受闸门约束，缩小的是 G1 的冲突分析量与带宽计算量，不是闸门本身。


## Context / 上下文

最近一次已记录的 Jetson 快照只观察到 `can0`，现场意图是两台电机与两台 HI12 共总线；该硬件状态本轮未重验，第二物理通道也未确认。`company/hightorque_fdcan` 又提供了 USB-CDC raw CAN transport 的参考，但准确通信板、固件、通道数、nominal bitrate 配置和错误/时间戳能力尚未闭合，不能把示例存在等同于 transport 已可部署。

**（2026-09-01 按 [ADR-013](ADR-013-ak30-protocol-baseline.md) 重算）** 按 1 Mbit/s 经典 CAN 最坏位填充预算（8 字节扩展帧 160 bit、4 字节 120 bit），当前第一目标 **AK3.0 力控**的两电机 500 Hz 加两台 HI12 100 Hz 约 **41.6%**。力控命令固定 8 字节扩展帧、反馈按 `0x29` 8 字节计，帧长与切换前的「8B 命令 + `0x29`」场景相同，**因此基线切换没有使带宽变差，频率目标不需要下调**。若额外启用 `0x2A` 位置帧则升至 53.6%，**超过 50% 平均目标，故 500 Hz 下不得启用**；两电机 1 kHz 场景为 73.6%，拒绝。六电机包络为 200–250 Hz（含两台 HI12 时 48.0%）。完整场景表见 [06 §5](../planning/06_cubemars_material_review.md)。

~~原文按 L02（AK2.0）servo-extended 与 motion-control/MIT-standard 帧型估算，得 41.6% 与 36.6%~~ —— 帧型基线已变更，数字按上文重算。无论数字如何，静态预算只是 profile-specific 估算，不证明交付设备具有共同位速率、无 ID 冲突、可接受仲裁延迟或稳定错误行为。

## Decision / 决策

若本提案转为 Accepted，将采用以下边界；在此之前实现只能保守验证配置并拒绝真实激活：

1. core、codec、controller 只引用 logical bus；deployment 把逻辑总线显式映射到一个具有稳定身份的物理通道。该通道可由 SocketCAN、经验证的 HighTorque USB-CDC backend、`vcan` 或 fake 提供，协议常量不硬编码 `can0` 或 `/dev/ttyACM*`。
2. 当前单物理通道是一个条件式 deployment profile，不是架构事实，也不预先选定 transport backend。四台设备只有在共同位速率、唯一 ID/节点、明确帧格式/filter、终端、backend capability 和负载/故障验收全部通过后才可激活。
3. nominal 目标为每总线平均占用不高于 50%、任意 1 秒峰值不高于 60%，RX dropped/overrun 与未解释 bus error/bus-off 为零；占用率之外还必须验证 command-to-wire、仲裁最坏响应和 queue 行为。
4. **（2026-09-01 按 [ADR-013](ADR-013-ak30-protocol-baseline.md) 改写）** AK3.0 力控与伺服扩展帧必须分别预算和验收；ACTIVE 期间不探测、混发或热切换。`0x2A` 可选位置反馈帧、高于设备配置限值的反馈率（L07 的协议上限为 1–2000 Hz，远高于 L02 的 500 Hz，因此上限不再是天然约束，实际约束来自总线预算）、1 kHz 电机 I/O、更多电机或 STM32，都必须作为独立 profile 重算并实测后才可启用。
5. backend 必须显式报告其 Classic/FD、标准/扩展帧、最大 DLC、bitrate 配置、filter、时间戳、错误状态和队列能力；不支持或未知的能力必须失败关闭，不得伪造成统一 transport 保证。
6. 任一条件不通过时，动作是增加经验证的第二个隔离物理通道、改用能力充分的 backend、分配独立物理总线或降低/调整 profile；不得假装已有 `can1` 或可用 USB 通道，也不得盲改未知设备配置来迎合单总线。
7. **（2026-09-02 新增）单电机取证台架 profile。** 为解决映射证据缺口（`direction_sign`、`0x29` 位置编码器来源、力控反馈帧是否为 `0x29`），允许一个与最终部署 profile 分离的受控台架，边界如下，任何一条不满足即退回离线仿真：
   - **组成**：一台 AKE60-8，**空载**（输出轴不接任何负载或工装），加 Jetson 与通用盒子。不含 HI12，不含第二台电机。
   - **拓扑（2026-09-02 晚实测更正）**：~~Jetson CAN → 通用盒子 CAN → 电机 CAN，三节点同一条物理总线~~ —— **该描述错误，已作废。** 通用盒子是 **USB-CDC ↔ CAN 网关**，不是总线上的一个 CAN 节点。正确拓扑为 **Jetson ──USB Type-C──> 通用盒子 ──XT30(2+2)──> 电机**（`4.3.4 通用盒子硬件接口说明` §2.3：「通过USB接口连接至电脑」，且 XT30(2+2) 同时承载 24-50V 电源与 CANx_L/CANx_H）。**Jetson 自身的 `can0` 在本 profile 中完全不参与。** 随之作废的推论：「三节点同总线、盒子位速率不匹配会用 error flag 破坏整条总线」不成立 —— CAN 段上只有盒子通道与电机两个节点。
   - **传输后端因此是 USB-CDC，不是 SocketCAN。** 适配器要接的是 `mech_control_core` 的 USB-CDC backend（RSP-001 标记为 candidate-only）。该 backend 的 `nominal_bitrate_verified` 只能为 false，而 `TransportCapabilities::is_valid()` 按 ADR-012 允许「未验证」状态，故无需改动核心类型。
   - ⚠️ **盒子 CAN 通道的位速率仍未闭合（OQ-08 / 厂商问题 A1），且已成为本 profile 的首要嫌疑。** 盒子标称 FDCAN 5 Mbps，电机为经典 CAN 1 Mbps；三份厂商代码均无位速率设置接口（已确认的负结果）。
   - **终端电阻**：项目负责人裁定本台架不做终端处理（短线、1 Mbps、受控环境）。**这是一项被明示接受的风险**，不是已验证结论；若出现间歇性错误，终端是第一排查项。
   - **顺序**：先被动监听（只收不发，`cangen` 禁用）；确认帧与语义后才允许发送命令，且首条命令为极小力矩。**注意本 profile 下"被动监听"不等于 `candump`**：盒子是协议网关，通道需先经 `MODE_FDCAN_PASS (0x12)` 且 `send_flag=0` 初始化才会转发；该初始化只写盒子，不向 CAN 总线发帧。
   - **2026-09-02 晚首次上电结果（未通过）**：USB 侧全部正常，盒子固件 4.8.8 应答我们自建的 `MODE_SET_NUM` 帧（证明帧格式/CRC 正确），7 个通道透传初始化均被接受，但**7 个通道均零 CAN 流量**。待排查项按嫌疑排序：电机接在 ②-⑧ 的 XT30(2+2) 还是 ⑫ 的独立 CAN 口；盒子通道位速率；电机是否真的在周期回报（截图中"当前模式"为空闲模式）。
   - **安全依据**：电机空载空转，且 `失控时间 = 1000 ms` / `失控刹车电流 = 0`，命令中断后自由滑行而非刹车。`direction_sign` 判断错误在空载电机上仅表现为反向旋转，无危害 —— 台架正是解决该参数最安全的场所。
   - **本 profile 的结果不构成最终部署 profile 的验收证据**，两者的带宽、仲裁与故障隔离必须分别实测。


## Alternatives considered / 替代方案

### A. 无条件接受所有设备共用一个通道

硬件成本最低，但把未知位速率、ID 和故障影响面当作已证事实。拒绝。

### B. 从第一天强制双总线或指定某个 backend

双总线隔离更强，指定 backend 也更简单，但当前第二接口、HighTorque 板卡身份及最终拓扑均未确认，无法把未验证硬件写成事实。双总线保留为条件失败后的首选整改，backend 由能力证据选择。

### C. 条件式单通道 profile + 可替换 backend + 双总线架构（提案）

允许用证据决定部署，同时保持逻辑总线和多 runtime 的扩展能力。

## Consequences / 后果

### Positive / 正面

- 不因当前接口数量把协议和 controller 固定到 `can0`、SocketCAN 或某个 USB 设备名。
- 单总线是否可用由配置、计算和测量决定，而不是由规划估算或设备外观决定。
- HighTorque 示例可以被提炼为 transport 参考，而不会侵入电机 codec、session 或 ros2_control 边界。
- 当负载、故障影响面或 1 kHz 目标超限时，有明确的分总线整改路径。

### Negative / 负面与代价

- ADR 在设备证据完成前保持 Proposed，真实 deployment 不能激活。
- 需要逐台配置读取、被动抓包、backend 能力/断连/队列测试、负载/仲裁测量和可能的第二接口采购。
- 平均占用阈值不能单独证明 deadline，需要保存更细的时间和错误统计。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，并确认 ADR-006 仍标为 Proposed 且 planning 反向链接有效。
- FND-006 配置/schema 测试拒绝重复 ID、重叠 filter、未知 profile、backend 能力不匹配、物理通道多 writer 和超过静态预算的 deployment。
- G0/G1 逐台记录协议、位速率、节点/ID 和输出 profile；仅做被动/低风险验证，未取得后续授权不得发送运动命令。
- RSP-001/002 用同一 `RawCanFrame` 契约分别验证 SocketCAN 与注入式 HighTorque CDC；CDC 测试覆盖 header/CRC、批量帧、标准/扩展、Classic/FD capability、断连、短读写和队列满，且不打开真实串口。
- G4/集成验收按 backend 记录线速/帧计数、command-to-wire、反馈 age、TX/RX queue/drop/overrun、可用的 error-passive/bus-off 或等价错误证据以及 nominal/stress 时段；全部满足本 ADR 阈值后才可接受并激活该 profile。

## Review triggers / 重审触发

- 第二物理 CAN 通道到位、HighTorque 板卡/固件被准确确认或最终 Jetson 拓扑改变；
- 任一设备位速率、ID、协议或输出 profile 与当前候选不一致；
- 实机被证明采用 AK3.0 并拟启用 `0x2A`/1 kHz 电机 I/O，或加入第三台以上电机/STM32；
- 平均/峰值负载、仲裁 deadline、queue/drop/error 或故障隔离不达标；
- CAN FD、transport backend 或其他物理链路进入/退出部署范围。
- **单电机取证台架（Decision 第 7 条）出现下列任一情况**：通用盒子的 CAN 位速率被确认与 1 Mbps 不符；台架上出现无法归因于位速率的持续总线错误（此时重审已被接受的「不做终端」风险）；台架从空载改为带载；或有意在台架上加入第二台设备 —— 加入第二台设备即离开本 profile，回到本 ADR 主体的约束。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 3、6、16 节。
- [MVP 执行与验收计划](../planning/03_mvp_delivery_plan.md)，第 6、7、8 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，第 2、3.5、5 节。
- [CubeMars 资料审查与总线预算](../planning/06_cubemars_material_review.md)，第 6 节。
- [Foundation 搭建计划](../planning/07_framework_bootstrap_plan.md)，第 6、8.1、9.5 节。
- [FND-004 ADR index](README.md)。
