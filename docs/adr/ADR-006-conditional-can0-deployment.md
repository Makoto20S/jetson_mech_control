# ADR-006：条件式单物理通道部署与双总线架构边界

- **Decision ID:** ADR-006
- **Status:** Proposed
- **Date:** 2026-08-07
- **Owner:** 项目负责人（批准）+ Integration/HIL owner（证据）
- **Scope:** 当前两台定制 AKE60-8 候选设备、两台 HI12、可选 transport backend 和未来最多两条物理 CAN 的 deployment profile

## Status rationale / 状态依据

架构应使用逻辑总线、可替换 transport backend，并保留两条物理总线的能力，这一方向已经明确；但“当前四台设备可安全共用一个物理 CAN 通道”仍缺少逐台位速率、节点/ID、输出 profile、transport 能力、仲裁和实测负载/错误证据。因此本 ADR 保持 `Proposed`，任何真实单通道激活都被阻止。

转为 `Accepted` 至少需要：逐台 G0 身份/配置；两台 HI12 协议、节点和位速率；完整 RX/TX ID/format/filter 表；共同位速率与终端证据；所选 backend/通信板的准确型号、固件、通道、bitrate 所有权和错误/队列能力；按实际 profile 计算并实测的平均/峰值占用、仲裁响应、queue/drop/error；以及 G1 被动总线和评审记录。个人 Memory 或旧 handoff 不能替代这些证据。

## Context / 上下文

最近一次已记录的 Jetson 快照只观察到 `can0`，现场意图是两台电机与两台 HI12 共总线；该硬件状态本轮未重验，第二物理通道也未确认。`company/hightorque_fdcan` 又提供了 USB-CDC raw CAN transport 的参考，但准确通信板、固件、通道数、nominal bitrate 配置和错误/时间戳能力尚未闭合，不能把示例存在等同于 transport 已可部署。

按 1 Mbit/s 经典 CAN 保守估算，当前第一目标 L02 servo-extended 的两电机 500 Hz 加两台 HI12 100 Hz 约 41.6%；第二目标 L02 motion-control/MIT-standard 若同按 500 Hz 候选频率估算约 36.6%。这些只是 profile-specific 静态预算，不证明交付设备具有共同位速率、无 ID 冲突、可接受仲裁延迟或稳定错误行为。`0x2A` 和 1 kHz 电机 I/O 属于另需实机证明的 AK3.0 补充 profile，不属于当前 L02 默认预算。

## Decision / 决策

若本提案转为 Accepted，将采用以下边界；在此之前实现只能保守验证配置并拒绝真实激活：

1. core、codec、controller 只引用 logical bus；deployment 把逻辑总线显式映射到一个具有稳定身份的物理通道。该通道可由 SocketCAN、经验证的 HighTorque USB-CDC backend、`vcan` 或 fake 提供，协议常量不硬编码 `can0` 或 `/dev/ttyACM*`。
2. 当前单物理通道是一个条件式 deployment profile，不是架构事实，也不预先选定 transport backend。四台设备只有在共同位速率、唯一 ID/节点、明确帧格式/filter、终端、backend capability 和负载/故障验收全部通过后才可激活。
3. nominal 目标为每总线平均占用不高于 50%、任意 1 秒峰值不高于 60%，RX dropped/overrun 与未解释 bus error/bus-off 为零；占用率之外还必须验证 command-to-wire、仲裁最坏响应和 queue 行为。
4. L02 servo-extended 与 motion-control/MIT-standard 必须分别预算和验收；ACTIVE 期间不探测、混发或热切换。`0x2A`、高于 L02 资料上限的反馈率、1 kHz 电机 I/O、更多电机或 STM32 只能在对应协议/固件另有证据时作为独立 profile 重算并实测。
5. backend 必须显式报告其 Classic/FD、标准/扩展帧、最大 DLC、bitrate 配置、filter、时间戳、错误状态和队列能力；不支持或未知的能力必须失败关闭，不得伪造成统一 transport 保证。
6. 任一条件不通过时，动作是增加经验证的第二个隔离物理通道、改用能力充分的 backend、分配独立物理总线或降低/调整 profile；不得假装已有 `can1` 或可用 USB 通道，也不得盲改未知设备配置来迎合单总线。

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

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 3、6、16 节。
- [MVP 执行与验收计划](../planning/03_mvp_delivery_plan.md)，第 6、7、8 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，第 2、3.5、5 节。
- [CubeMars 资料审查与总线预算](../planning/06_cubemars_material_review.md)，第 6 节。
- [Foundation 搭建计划](../planning/07_framework_bootstrap_plan.md)，第 6、8.1、9.5 节。
- [FND-004 ADR index](README.md)。
