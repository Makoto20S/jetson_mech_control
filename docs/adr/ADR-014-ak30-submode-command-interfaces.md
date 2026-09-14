# ADR-014：CompositeSystem 单命令接口形状扩展与 AK3.0 子模式命令接口

- **Decision ID:** ADR-014
- **Status:** Proposed
- **Date:** 2026-09-07
- **Owner:** 项目负责人
- **Scope:** `CompositeSystem` 的命令接口形状（`CanonicalCommand` 与每关节命令接口导出集合）、AK3.0 力控 Velocity/Torque 子模式经 `RuntimePort` 的命令映射语义、`sub_mode` 部署参数，以及状态接口形状维持不变的声明

## Status rationale / 状态依据

本 ADR 在任何实现之前提交，符合 `adapter_contract_v1.md` 第 7 条「契约变更先出 ADR」。`CanonicalCommand` 的字段集与 `CompositeSystem::validate_info` 接受的命令接口集合是冻结的 canonical 契约（`mech_hardware_ros2_control/composite_system.hpp`，Foundation RC 的一部分），扩展它们是契约变更。ADR-012 是先实现后追认的反例，本次不重复。

2026-09-06 的阶段 3 切片（PR #11）刻意把本变更挡在范围外，并在 `ak30_runtime_params.hpp` 写明：「sub-mode 不是参数：本切片仅 Position，因为那就是 CompositeSystem 导出的接口形状；Torque/Velocity 命令接口是 canonical 契约变更，需先出 ADR」。本 ADR 兑现该承诺。

**转 `Accepted` 的条件**：项目负责人批准后方可合并实现；在此之前实现可以在此分支上进行，但不得进入 `main`。批准范围仅为接口形状与命令映射语义，**不解除任何设备启用闸门**——真机上的 Velocity/Torque 命令仍需逐次授权并遵守 ADR-006 Decision 7 的台架边界。

## Context / 上下文

AK3.0 力控（L07 §4.2）的三个子模式共用控制模式 ID `8` 与同一 8 字节载荷布局（KP:12 KD:12 POS:16 VEL:12 TRQ:12），由载荷内容区分：Position（Kp+Kd+position）、Velocity（Kd+velocity）、Torque（仅 torque）。协议层 `mech_protocol_cubemars` 已按子模式完整实现并经台架三子模式闭环验证（2026-09-05 渐进路线四步全过）：

- `mapping_is_sufficient()` 按子模式门控证据（Torque：direction_sign+Kt；Velocity 另需 pole_pairs+gear_ratio；Position 另需 zero_offset+position_source_known）；
- `to_device_command()` 按子模式消费 canonical 字段并构造 wire 帧；`evidenced_state_fields()` 按子模式声明解码可证据化的状态字段，其余诚实置零。

下游 `CanonicalDeviceCommand`（`mech_control_core/adapter_template.hpp`）也早已具有 position/velocity/effort 三字段。**唯一缺失的一环**是中间的 `RuntimePort` 命令结构 `CanonicalCommand` 只有 `position`，`CompositeSystem::validate_info` 只接受 `<command_interface name="position"/>`。结果是：力控 Torque/Velocity 子模式只能通过探针使用，无法经 ros2_control 路径部署——探针自身没有 `command_stage()` 的分级看门狗消费（那是 PR #11 交给 runtime 的生产语义）。

协议层测试注释（`test_ak30_mapping.cpp`，Torque 子模式忽略 position/velocity 的用例）已预期本形状："the later ros2_control slice claims only the effort interface for a torque-mode device"。

## Decision / 决策

1. **`CanonicalCommand` 扩展为 `{position, velocity, effort}` 三字段（默认 0.0）。** `RuntimePort` 的方法签名不变——端口本来就传递 `const CanonicalCommand*` 数组，这只是载荷扩容；既有实现（`LoopbackRuntime`、`FoundationHarness`、探针）语义兼容。

2. **`CompositeSystem` 每个关节导出恰好一个命令接口，名称 ∈ {`position`, `velocity`, `effort`}**，由 URDF 声明，顺序敏感的精确形状校验保持（state 接口维持恰为 `[position, velocity, effort]` 不变）。命令接口名决定写入 `CanonicalCommand` 的哪个成员（成员指针映射），未导出的成员按构造保持 0.0。这保持 CompositeSystem 厂商中立：它不知道「子模式」，只知道通用接口名集合。

3. **AK3.0 部署中，命令接口名必须与 `sub_mode` 匹配**（Position→`position`、Velocity→`velocity`、Torque→`effort`），fail-closed：`sub_mode` 成为 `Ak30RuntimeParams` 的显式参数（`position|velocity|torque`，默认 `position`，非法值拒绝），bringup 层的结构校验（`test_deployment_files.cpp`）钉住「URDF 命令接口名 == `expected_command_interface_name(sub_mode)`」。**不采用每关节同时导出三个命令接口的超集形状**：AK3.0 力控帧的 effort 字段在所有子模式下都作为前馈 `t_ff` 随帧发送（`to_device_command` 无条件拷贝），position 模式下导出 effort 命令接口会给跨模式误写留一条静默通道，违反 ADR-004 配置期固定 profile 的主动选择。

4. **`Ak30ForceControlRuntime` 按子模式映射命令并强制安全语义：**
   - Position：`command.position = pending.position`（现状）；
   - Velocity：`command.velocity = pending.velocity`，且 **effort 强制 0**——前馈叠加是探针已证的危险路径（「Kd 路径对力矩与速度双重上界」的台架结论依赖 effort=0），该纪律由 runtime 强制而非依赖调用方自觉；
   - Torque：`command.effort = pending.effort`（`to_device_command` 忽略其余字段，协议层不动）。
   `write()` 的 finiteness 校验按子模式校验被消费的字段。

5. **看门狗语义不变**（ADR-012）：Following 只提交新鲜写入，Holding 冻结不重发，Expired 在 ≤3 周期预算内显式失败；缺失命令在任何子模式下都**不解析为 0.0**——position 上是移动到标定零位，velocity 上是命令停车，effort 上是零力矩，三者都是被合成的命令而非「无命令」。

6. **状态接口形状维持 `[position, velocity, effort]` 对三个子模式都不变。** B4 诚实门控留在 session/codec 内（`evidenced_state_fields()`），unevidenced 字段诚实置零；ADR-009「raw 值不得伪装为 effort」语义不动。`joint_state_broadcaster` 的被动消费（broadcaster-only 是当前唯一已授权的真机形态）不受影响。

7. **本 ADR 不解锁任何真机激活。** Velocity/Torque 的真机运行仍需逐次授权、遵守 ADR-006 Decision 7 台架边界，且首次上电沿用渐进路线（Velocity 先带 Kd、Torque 先小幅值——两者在空载台架上的上界已有 2026-09-05 实测背书）。

## Alternatives considered / 替代方案

### A. 每关节固定导出 position/velocity/effort 三个命令接口的超集

实现最小、CompositeSystem 校验最简单。但 AK3.0 wire 帧的 effort 字段是所有子模式下的前馈 `t_ff`：Position 模式下 claim 一个 effort 命令接口就等于给「静默前馈注入」开了通道，且接口形状与子模式证据（`mapping_is_sufficient`）脱钩。与 ADR-004 的主动不混发规则冲突。拒绝。

### B. 为 Torque/Velocity 另建独立的 SystemInterface 插件

避免触碰 CompositeSystem，但复制生命周期/claim/switch/看门狗浮出的全部语义，且「复合系统」的存在意义就是多设备共栈。拒绝。

### C. 单命令接口形状 + 子模式决定接口名（选定）

与协议层既有设计预期一致（`test_ak30_mapping.cpp` 注释）、把「接口形状必须匹配子模式证据」变成可校验的部署约束、CompositeSystem 保持厂商中立。代价是接口名→成员指针的映射与 per-variant 部署文件。

## Consequences / 后果

### Positive / 正面

- 力控三子模式全部可经 ros2_control 路径部署，享受 `command_stage()` 分级看门狗、fault-latch、NaN 拒绝等生产语义（此前只有 Position 有）。
- 「接口名 == 子模式」的 fail-closed 校验把 ADR-004 固定 profile 原则延伸到了部署文件层。
- Velocity 模式 effort=0 在 runtime 强制，把台架安全纪律变成代码不变量。
- `mech_protocol_cubemars`、`mech_control_core`、`mech_controllers` 零改动。

### Negative / 负面与代价

- `CompositeSystem` 从「position-only」放宽为「集合成员」：校验从单一名字比对变为集合成员检查，必须由测试钉住「恰好一个命令接口」不被未来改成「任意子集」。
- Torque/Velocity 尚无 ros2 控制器（`DemoController` 是 position-only）：部署变体只能带 `joint_state_broadcaster`，命令接口的 claim/write 由测试与未来控制器切片覆盖。
- 部署文件从 1 套变 3 套（position/torque/velocity URDF 变体），结构校验测试同步增长。
- URDF 声明与 `sub_mode` 参数是两个来源，不一致时靠 bringup 结构校验兜底（离线 CI 检查，非运行期）。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，且 ADR-014 在 `docs/adr/README.md` 与规划文档 02/07 的状态行同步为 `Proposed`。
- `python3 tools/ci/context_check.py` 通过（六包与依赖集不变）。
- 完整本地验证集：`build_workspace.sh`（预期 >236 tests 0 failures）、`run_sanitizers.sh`（`-fsanitize` 经 CMake cache 核实）、`git diff --check`；新 xacro 过 `xmllint --noout`。
- CompositeSystem 层：effort/velocity 形状 on_init 通过、双命令接口/未知名拒绝、每字段 NaN 拒绝 + fault latch、LoopbackRuntime 镜像语义测试。
- Runtime/集成层：三子模式金帧（从 L07 范围独立反算，不照抄手册示例行）、Velocity 帧编码 effort=0、看门狗三阶段 parity。
- 部署层：三变体参数 round-trip + 接口名匹配校验。
- 本 ADR 未主张任何实机验证；真机运行仍由逐次授权与 G0–G3 管辖。

## Review triggers / 重审触发

- 厂商确认 B4（`0x29` 位置字段编码器来源）或 B14（单圈/多圈语义）的结论要求改变状态接口形状或 per-子模式状态导出策略；
- 出现需要同时 claim 多个命令接口的设备或控制器（届时需重新评估超集形状并修订本 ADR）；
- 真机 Velocity/Torque 运行发现 runtime 强制的安全语义（effort=0、按字段 NaN 拒绝）与设备行为冲突；
- 伺服 profile（第二 profile）实现时发现单命令接口形状不适用于其命令族；
- ADR-004 固定 profile 规则或 ADR-012 看门狗语义发生修订。

## Sources / 来源

- 资产 **L07**：`ak-series-prodcut-manual-v3-2-0-for-ak-3-0-robotic-actuator-cn.pdf` §4.2（力控协议、AKE60-8 参数表：位置 ±12.56 rad、速度 ±40 rad/s、力矩 ±15 N·m、Kp 0–500、Kd 0–5）、§4.4.1（示例行，需反算验证）。
- [ADR-004 固定协议 profile](ADR-004-fixed-protocol-profile.md)、[ADR-009 effort 语义闸门](ADR-009-effort-semantic-gate.md)、[ADR-012 命令看门狗与能力诚实](ADR-012-command-watchdog-and-capability-honesty.md)、[ADR-013 AK3.0 协议基线](ADR-013-ak30-protocol-baseline.md)。
- [ADR-003 复合 SystemInterface](ADR-003-composite-system-interface.md)（本 ADR 修订其接口形状边界）。
- [AdapterContract v1](../development/adapter_contract_v1.md) 第 7 条与接口语义条款。
- [AK3.0 力控适配器设计](../development/ak30_force_control_adapter_design.md)、[2026-09-07 Torque/Velocity 切片计划](../development/plans/2026-09-07-ak30-torque-velocity-slice.md)。
- 台架证据（2026-09-03/04/05 渐进验证路线）：`docs/planning/06_cubemars_material_review.md` §6、`docs/planning/05_decisions_and_open_questions.md` §7/§8。
