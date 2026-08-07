# ADR-004：配置期固定协议代际和 active command profile

- **Decision ID:** ADR-004
- **Status:** Accepted
- **Date:** 2026-08-07
- **Owner:** 项目负责人（Foundation core reviewer；后续 device adapter owner 提交设备证据）
- **Scope:** 所有 vendor device session；当前重点为 CubeMars AK V3/legacy MIT 与 HI12 J1939/CANopen

## Status rationale / 状态依据

“未知协议拒绝配置、ACTIVE 期间不自动猜测或混发”是可独立于实机型号成立的安全和接口约束。接受该规则不表示当前 CubeMars 或 HI12 的交付固件已经识别，也不允许创建猜测型真实 profile。

## Context / 上下文

现有资料同时出现 AK V3 servo extended、AK V3 force-control extended 和 legacy MIT standard-frame；它们的帧格式、功能 ID、payload 和能力声明不同。HI12 的 J1939 与 CANopen 也取决于交付固件。按第一帧外观猜设备、在 ACTIVE 期间自动切换 codec，或为“兼容”而混发命令，会使 resource claim、缩放、neutral、watchdog 和故障归因失去确定语义。

## Decision / 决策

1. 每个设备实例在 `on_configure` 时绑定不可变的 `protocol_profile`、codec/session 版本、允许的帧格式、反馈集合、active command profile 和 capability。
2. 真实 profile 的关键字段（准确型号/固件兼容范围、ID/位速率、命令与反馈语义、缩放、neutral/watchdog）缺失或冲突时，configure 必须失败；不得选择“最接近型号”的默认值。
3. ACTIVE 期间禁止通过收到的帧自动改变协议代际、codec、command profile 或 ros2_control claim，禁止混发互不兼容的 servo/force/legacy 命令族。
4. CubeMars adapter 至少把 `AK V3 servo extended`、`AK V3 force-control extended` 和 `legacy MIT standard-frame` 作为不同 codec/profile；前两者只是在实机身份确认后的当前候选，legacy 仅用于明确匹配的旧代际。
5. HI12 只有在交付固件身份、节点、位速率和帧证据确认后才选择 J1939 或 CANopen session。需要改变 profile 时，先停用 controller/hardware，重新 configure 并走相应设备闸门。

## Alternatives considered / 替代方案

### A. 根据首帧自动识别协议

部署方便，但扩展/标准帧外观、部分 ID 或错误帧不足以证明固件与命令语义。拒绝。

### B. ACTIVE 期间同时发送多个候选命令族

试图兼容未知固件，却可能触发不同控制路径、错误缩放或 Flash/配置行为。明确禁止。

### C. 配置期固定并失败关闭（选定）

需要更多身份和配置证据，但 claim、codec、neutral 和测试向量可审查、可复现。

## Consequences / 后果

### Positive / 正面

- golden frame、negative frame、capability 和 lifecycle 测试可以按 profile 独立组织。
- controller 与 `SystemInterface` 不需要知道供应商品牌或猜测协议。
- 未知固件保持 INACTIVE，避免把旧 demo 或相似型号的常量当作真实设备事实。

### Negative / 负面与代价

- 初次 bring-up 前必须逐台读取身份、版本和配置，不能即插即用。
- 同一供应商不同代际需要维护明确的 profile/codec 集合和迁移说明。
- 在线 mode switch 若未来确实需要，必须设计显式 controller/hardware mode-switch 事务，不能沿用自动探测。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，并确认 ADR-004 的 planning 反向链接有效。
- 后续 adapter 测试必须为每个 profile 提供 golden、negative、DLC/ID/字节边界测试；其他 profile 的帧不得被静默接受为本 profile。
- 配置测试必须拒绝未知/冲突固件、缺少关键字段、profile 与 frame format/capability 不一致以及 ACTIVE 期间变更 profile。
- lifecycle/claim 测试必须证明 profile 改变只能发生在停用并重新 configure 后；真实 profile 激活仍受 G0/G1 与相关设备证据约束。

## Review triggers / 重审触发

- 供应商发布经过认证且具有无歧义身份握手的在线模式切换协议；
- 确认一个固件必须在同一 ACTIVE 生命周期内切换命令族，并能定义原子 claim/neutral/rollback；
- 新设备协议要求运行时协商，且协商结果可以在任何命令发送前完成并固定；
- golden/抓包证据证明当前 profile 分类错误或两个代际实际共享完全相同语义。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 7.1、9、13 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，第 3.2、4、5 节。
- [CubeMars 供应商资料审查](../planning/06_cubemars_material_review.md)。
- [Foundation 搭建计划](../planning/07_framework_bootstrap_plan.md)，第 8.1、9.2、9.5 节。
- [FND-004 ADR index](README.md)。
