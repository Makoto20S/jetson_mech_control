# ADR-009：标准 `effort` 接口必须通过物理语义闸门

- **Decision ID:** ADR-009
- **Status:** Accepted
- **Date:** 2026-08-07
- **Owner:** 项目负责人（Foundation interface owner；后续 device adapter owner 提交映射证据）
- **Scope:** canonical/ros2_control `effort [N*m]` 状态与命令，以及最小恒定命令 demo

## Status rationale / 状态依据

接受的是一个保守接口规则：没有匹配设备、固件和机械侧映射证据时，不得把电流、归一化值或未校准字段命名为标准 `effort`。这不表示两台定制 AKE60-8 已通过闸门；它们的标准 `effort` 映射仍未批准。

## Context / 上下文

AK V3.2 资料给出标准 AKE60-8 的候选 `Kt = 0.7382 N*m/A` 和输出端 `T = Kt * Iq`，但当前设备是定制双编码器版本，逐台驱动板、固件、配置、减速/机械映射和物理误差尚未确认。servo profile 可能接受 Iq，force-control profile 可能提供 torque 字段；二者都不能仅因字段名相似就自动成为关节侧 N·m。

项目还需要一个最小恒定命令 demo 来验证 controller → hardware → protocol → transport → state/diagnostics 链路。链路贯通与物理输出力矩准确是两个不同验收目标。

## Decision / 决策

1. 标准 `position [rad]`、`velocity [rad/s]`、`effort [N*m]` 只在设备型号/固件、字段语义、方向、减速比/机械侧映射、缩放/范围和 neutral/limit 已有可追溯证据时导出。
2. 真实设备缺少上述任一关键证据时，configure 必须拒绝标准 `effort`，或只导出诚实命名的专用 `motor_current`/raw/vendor interface；不得用相似名称或候选 Kt 伪装 N·m。
3. 每个真实 device profile 单独记录证据版本和适用范围。标准 AKE60-8 参数仅是当前定制实机的候选，不自动继承。
4. Foundation 最小 demo 使用 reference/simulation profile，目标可配置、有界、从零按 slew 进入并可回零，且受 finite、mode、freshness、limit 和 TTL 检查。示例数值不进入默认配置或固定验收。
5. demo 通过只证明命令/状态链路和防护契约贯通；外部计量、物理输出误差、台架安全和真实设备性能在 G0–G3/HIL 中独立验收。

## Alternatives considered / 替代方案

### A. 把 Iq 或厂商 torque 字段直接命名为 `effort`

接口看起来统一，但可能隐藏电机侧/输出侧、减速比、定制参数或归一化语义差异。拒绝。

### B. 完全禁止标准 effort，直到全部实机测试结束

最保守，却会阻止 reference profile 和已具完整证据的未来设备复用标准控制器。拒绝为通用规则。

### C. 证据闸门 + 专用 raw/current fallback（选定）

允许已证明语义的设备使用标准接口，未知设备以失败关闭或诚实专用接口继续离线开发。

## Consequences / 后果

### Positive / 正面

- controller 和上层算法看到的 `effort` 始终具有明确 N·m 语义，不被供应商字段污染。
- Foundation 可用 reference profile 验证纵向链路，同时不会产生“真实力矩已验证”的错误结论。
- 每个新 adapter 都有一致的物理量证据与配置失败规则。

### Negative / 负面与代价

- 当前定制电机在证据完成前不能导出标准 effort，可能需要临时专用 current/raw controller。
- 需要逐台配置导出、供应商确认，必要时还需外部力/力矩计量和不确定度记录。
- profile、接口文档和测试必须区分命令字段正确、机械映射正确和物理精度合格。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，并确认 ADR-009 的 planning 反向链接有效。
- FND-006 schema/capability 测试必须拒绝无单位、无来源、无机械映射或范围冲突的标准 effort profile，并允许显式 current/raw profile。
- Foundation reference demo 测试覆盖 finite、limit、slew、mode、freshness、TTL、回零和命令 generation；不得以此声明真实硬件 N·m 精度。
- 任何真实设备映射在启用前必须附匹配固件/参数证据、转换公式与单位测试；物理精度声明还必须通过 G3 台架、校准与受控测量。

## Review triggers / 重审触发

- 实机配置或供应商资料证明候选 Kt、减速/机械侧定义或 torque 字段与当前理解不同；
- 新设备提供直接校准关节力矩、弹性元件估计或其他不同 effort 来源；
- ros2_control 标准接口语义发生变化，或项目需要显式 current/torque 两级标准化接口；
- 台架测量表明协议映射虽正确但物理误差无法满足声明范围。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 7.2、7.3、9.2 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，第 3.4、4、5 节。
- [CubeMars 供应商资料审查](../planning/06_cubemars_material_review.md)。
- [Foundation 搭建计划](../planning/07_framework_bootstrap_plan.md)，第 3、8.1、9.3、13 节。
- [FND-004 ADR index](README.md)。
