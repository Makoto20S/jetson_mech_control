# ADR-006：条件式单 `can0` 部署与双总线架构边界

- **Decision ID:** ADR-006
- **Status:** Proposed
- **Date:** 2026-08-07
- **Owner:** 项目负责人（批准）+ Integration/HIL owner（证据）
- **Scope:** 当前两台定制 AKE60-8 候选设备、两台 HI12 和未来最多两条物理 CAN 的 deployment profile

## Status rationale / 状态依据

架构应使用逻辑总线并保留两条物理总线的能力，这一方向已经明确；但“当前四台设备可安全共用 `can0`”仍缺少逐台位速率、节点/ID、输出 profile、仲裁和实测负载/错误证据。因此本 ADR 保持 `Proposed`，任何真实单总线激活都被阻止。

转为 `Accepted` 至少需要：逐台 G0 身份/配置；两台 HI12 协议、节点和位速率；完整 RX/TX ID/format/filter 表；共同位速率与终端证据；按实际 profile 计算并实测的平均/峰值占用、仲裁响应、queue/drop/error；以及 G1 被动总线和评审记录。个人 Memory 或旧 handoff 不能替代这些证据。

## Context / 上下文

当前现场计划只有 `can0`，希望两台电机与两台 HI12 共总线；第二接口尚未确认。规划按 1 Mbit/s 经典 CAN 保守估算，两电机基础反馈 500 Hz 加两台 HI12 100 Hz 约 41.6%，两电机再启用 `0x2A` 时约 53.6%。这些是候选预算，不证明交付设备具有共同位速率、无 ID 冲突、可接受仲裁延迟或稳定错误行为。

## Decision / 决策

若本提案转为 Accepted，将采用以下边界；在此之前实现只能保守验证配置并拒绝真实激活：

1. core、codec、controller 只引用 logical bus；deployment 把逻辑总线显式映射到 `can0`、未来 `can1`、`vcan` 或 fake，协议常量不硬编码 Linux 接口名。
2. 当前单 `can0` 是一个条件式 deployment profile，不是架构事实。四台设备只有在共同位速率、唯一 ID/节点、明确帧格式/filter、终端和负载/故障验收全部通过后才可激活。
3. nominal 目标为每总线平均占用不高于 50%、任意 1 秒峰值不高于 60%，RX dropped/overrun 与未解释 bus error/bus-off 为零；占用率之外还必须验证 command-to-wire、仲裁最坏响应和 queue 行为。
4. `0x2A`、更高反馈率、1 kHz 电机 I/O、更多电机或 STM32 必须作为独立 profile 重算并实测；不得用更深软件队列掩盖物理带宽不足。
5. 任一条件不通过时，动作是增加经验证的第二个隔离 SocketCAN 接口、分配独立物理总线或降低/调整 profile；不得假装已有 `can1`，也不得盲改未知设备配置来迎合单总线。

## Alternatives considered / 替代方案

### A. 无条件接受所有设备共用 `can0`

硬件成本最低，但把未知位速率、ID 和故障影响面当作已证事实。拒绝。

### B. 从第一天强制双总线

隔离更强，但当前第二接口及最终拓扑未确认，无法把不存在的硬件写成事实。保留为条件失败后的首选整改。

### C. 条件式单总线 profile + 双总线架构（提案）

允许用证据决定部署，同时保持逻辑总线和多 runtime 的扩展能力。

## Consequences / 后果

### Positive / 正面

- 不因当前接口数量把协议和 controller 固定到 `can0`。
- 单总线是否可用由配置、计算和测量决定，而不是由规划估算或设备外观决定。
- 当负载、故障影响面或 1 kHz 目标超限时，有明确的分总线整改路径。

### Negative / 负面与代价

- ADR 在设备证据完成前保持 Proposed，真实 deployment 不能激活。
- 需要逐台配置读取、被动抓包、负载/仲裁测量和可能的第二接口采购。
- 平均占用阈值不能单独证明 deadline，需要保存更细的时间和错误统计。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，并确认 ADR-006 仍标为 Proposed 且 planning 反向链接有效。
- FND-006 配置/schema 测试拒绝重复 ID、重叠 filter、未知 profile、物理接口多 writer 和超过静态预算的 deployment。
- G0/G1 逐台记录协议、位速率、节点/ID 和输出 profile；仅做被动/低风险验证，未取得后续授权不得发送运动命令。
- G4/集成验收记录 `canbusload`、command-to-wire、反馈 age、TX/RX queue/drop/overrun、error-passive/bus-off 和 nominal/stress 时段；全部满足本 ADR 阈值后才可接受并激活该 profile。

## Review triggers / 重审触发

- 第二 SocketCAN 接口到位或最终 Jetson 拓扑改变；
- 任一设备位速率、ID、协议或输出 profile 与当前候选不一致；
- 启用 `0x2A`、1 kHz 电机 I/O、第三台以上电机或 STM32；
- 平均/峰值负载、仲裁 deadline、queue/drop/error 或故障隔离不达标；
- CAN FD 或其他物理链路进入部署范围。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 3、6、16 节。
- [MVP 执行与验收计划](../planning/03_mvp_delivery_plan.md)，第 6、7、8 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，第 2、3.5、5 节。
- [CubeMars 资料审查与总线预算](../planning/06_cubemars_material_review.md)，第 6 节。
- [Foundation 搭建计划](../planning/07_framework_bootstrap_plan.md)，第 6、8.1、9.5 节。
- [FND-004 ADR index](README.md)。
