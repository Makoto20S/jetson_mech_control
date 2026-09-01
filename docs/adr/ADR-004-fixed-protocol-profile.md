# ADR-004：配置期固定协议代际和 active command profile

- **Decision ID:** ADR-004
- **Status:** Accepted
- **Date:** 2026-08-07
- **Last clarified:** 2026-08-19
- **Owner:** 项目负责人（Foundation core reviewer；后续 device adapter owner 提交设备证据）
- **Scope:** 所有 vendor device session；当前重点为 L07（AK3.0 V3.2.0）的 CubeMars 力控/伺服 profiles 与 HI12 J1939/CANopen。~~原为 L02 V1.0.18~~，2026-09-01 按 [ADR-013](ADR-013-ak30-protocol-baseline.md) 切换

## Status rationale / 状态依据

“未知协议拒绝配置、ACTIVE 期间不自动猜测或混发”是可独立于实机型号成立的安全和接口约束。接受该规则不表示当前 CubeMars 或 HI12 的交付固件已经识别，也不允许创建猜测型真实 profile。

> **2026-09-01 修订（[ADR-013](ADR-013-ak30-protocol-baseline.md)）。** 本 ADR 的规则本身不变，但 CubeMars 侧的具体 profile 清单和一条论据已被取代。协议基线由 L02（AK2.0 驱动器手册）切换为 L07（AK3.0 产品手册），因为本项目驱动板 `AK54-4810-1C-A2` 只出现在 L07。受影响的是下文 Context 第 4 条与 Alternatives 中引用 L02 警告的那一条；原文保留在版本历史中，此处按新基线改写并标注。

## Context / 上下文

当前适用的 L07（AK3.0 V3.2.0，2026-09-01 起，见 [ADR-013](ADR-013-ak30-protocol-baseline.md)）同时定义伺服扩展帧（控制模式 `0–6,15,16`）与力控扩展帧（控制模式 ID `8`，三子模式由载荷区分），**两者同为 29 位扩展帧**，功能 ID、payload 布局和能力声明各不相同——力控载荷按 `KP KD 位置 速度 力矩` 打包，与伺服的定长标量载荷毫无共通之处。上一代 L02（AK2.0）的伺服 29 位/运控 11 位组合已不适用于本项目硬件。HI12 的 J1939 与 CANopen 也取决于交付固件。按第一帧外观猜设备、在 ACTIVE 期间自动切换 codec，或为“兼容”而混发命令，会使 resource claim、缩放、neutral、watchdog 和故障归因失去确定语义。

## Decision / 决策

1. 每个设备实例在 `on_configure` 时绑定不可变的 `protocol_profile`、codec/session 版本、允许的帧格式、反馈集合、active command profile 和 capability。
2. 真实 profile 的关键字段（准确型号/固件兼容范围、ID/位速率、命令与反馈语义、缩放、neutral/watchdog）缺失或冲突时，configure 必须失败；不得选择“最接近型号”的默认值。
3. ACTIVE 期间禁止通过收到的帧自动改变协议代际、codec、command profile 或 ros2_control claim，禁止混发互不兼容的 servo/MIT/force 命令族。
4. **（2026-09-01 按 [ADR-013](ADR-013-ak30-protocol-baseline.md) 改写）** CubeMars adapter 把 AK3.0（L07）的**力控**（控制模式 ID `8`）与**伺服扩展帧**（控制模式 `{0,1,2,3,4,5,6,15,16}`）作为不同 codec/profile。**两者都是 29 位扩展帧**——AK2.0 那种 11 位标准帧的运控 profile 在本项目硬件上不存在。第一阶段先实现力控，第二阶段实现伺服；该顺序推翻了 2026-08-19 基于 L02 的相反排序。力控内部的三个子模式（位置/速度/扭矩）共用控制模式 ID `8`，由载荷内容区分，必须由同一 codec 以显式配置选择，不得按收到的数据反推。任何 profile 都不得共享会混淆帧格式、ID 或位域的打包函数。
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
- 在线 mode switch 若未来确实需要，必须设计显式 controller/hardware mode-switch 事务，不能沿用自动探测；默认只允许停用、neutral/断能后重新 configure。
  - **2026-09-01 论据更正（[ADR-013](ADR-013-ak30-protocol-baseline.md)）。** 此处原本引用 L02 注意事项第 4 条「多种可选控制方式在驱动板运行时不可切换……使用错误的协议控制可能会使驱动板烧毁」作为依据。**该警告在适用的 L07 中不存在**：L07 注意事项只有短路、发热、部件检查、电压电流温度四条；L07 的上位机也删除了 L02 的「模式切换」页，§3.3 在同一台电机上依次演示伺服与力控，§3.4 固件升级为单一下拉列表。因此本条规则**不再有厂商安全警告支撑，改为本项目的主动选择**：运行期在位置伺服与力控之间混发会使控制权归属、缩放、neutral 和故障归因失去确定语义，这与固件能力无关。**后来者不得以「固件其实支持切换」为由删除本规则。** 需要说明的是，警告消失并不证明切换安全，也可能只是文档简化，因此保守侧不变。

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
- ~~用户 2026-08-19 对 L02 V1.0.18 CAN 协议和参数适用性的确认~~ —— **已于 2026-09-01 被项目负责人更正取代**：适用的是 L07（AK3.0 V3.2.0），客观依据见 [ADR-013](ADR-013-ak30-protocol-baseline.md)。该条保留以记录基线变更的历史。
- [ADR-013 AK3.0 协议基线](ADR-013-ak30-protocol-baseline.md)。
