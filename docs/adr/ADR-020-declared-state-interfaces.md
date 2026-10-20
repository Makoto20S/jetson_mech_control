# ADR-020：按能力声明导出状态接口

- **Decision ID:** ADR-020
- **Status:** Proposed
- **Date:** 2026-09-29
- **Owner:** Project owner
- **Scope:** Generic CompositeSystem state interface declarations and servo composition

## Status rationale / 状态依据

SERVO-001 当前实现分支的配套提案。用户要求验收仓库实际伺服实现，已授权补齐集成；
本记录不宣称该接口提案已正式 Accepted，也不授权真实设备动作。
历史力控部署仍使用原来的三项状态，本提案仅扩展通用层可表达的能力集合。

## Context / 上下文

现有 CompositeSystem 强制导出 position/velocity/effort，并校验三者有限。
新伺服 session 只有显式映射后的 position；原始 ERPM 和 Iq 不具有已确认的
关节 rad/s 与 N*m 语义。为了满足固定形状而导出零值会掩盖能力缺失。
独立测试程序绕过 session/BusRuntime 也无法验收生产实现。

## Decision / 决策

1. 通用硬件层接受 position/velocity/effort 的非空、无重复声明子集；拒绝未知名。
   只导出声明字段，只对已导出字段检查 NaN/Inf。内部未导出字段允许保持未知值。
2. 配置包含 position 命令时必须同时声明 position 状态，保留弱档位置接管的依据。
   命令接口组合和 generation 契约不变，厂商语义不进入通用层。
3. 新伺服组合固定 position 状态，声明 position 与 command_generation 命令接口；
   generation 的 claim 可选，声明仍按 ADR-017 必须存在。
   标准位置控制器/JTC 仍可使用弱档；需要速度/力矩状态的控制器不得伪造能力满足配置。
4. 现有 force plugin 的配置、三状态导出和字段含义保持不变。
   本提案不追溯改写 ADR-014/016 的历史批准范围。
5. Servo RuntimePort 复用真实 Ak30ServoPositionSession 和共享 BusRuntime；
   原始转速、电流、状态码、主机序号/时间通过诊断快照保留，不冒充 SI 状态。
   主机撤销、停用和故障只取消发送，不能声称驱动内部轨迹已停止。

## Alternatives considered / 替代方案

- 填零凑齐三状态：拒绝，会将未知物理量呈现为可用值。
- 复制独立 servo SystemInterface 全套 claim/lifecycle：拒绝，会复制通用控制边界。
- 在临时脚本实现收发保护：拒绝作为生产验收路径，绕过被测实现。

## Consequences / 后果

### Positive / 正面

能力声明与真实导出一致；保留标准位置控制器兼容，并让测试经过生产 session/bus/plugin。

### Negative / 负面与代价

配置须准确声明状态集合，控制器需要匹配可用状态。未知物理单位、坐标和停止方式
仍需有界实验验证；接口形状通过不能作为设备安全或精度认证。

## Validation / 验证

验收要求：位置单状态的导出/有限值/弱档接管测试；未知与重复状态拒绝；未声明
NaN 不误伤、声明 NaN 仍拒绝；原三状态回归；真实伺服插件注入 fake serial 的
双设备报文/生命周期/claim 测试，及标准位置控制器集成测试。
实际测试结果以本分支最终验证报告为准，本提案不预先宣称通过。

## Review triggers / 重审触发

需要新增状态名、按运行态动态改变接口集合、支持无位置反馈的位置控制，或控制器
无法使用已声明的状态集合时重审。厂商原始量转 SI 必须另有映射与计量依据。

## Sources / 来源

- [ADR-003](ADR-003-composite-system-interface.md)
- [ADR-014](ADR-014-ak30-submode-command-interfaces.md)
- [ADR-016](ADR-016-feedback-quality-fail-closed.md)
- [ADR-017](ADR-017-command-freshness-generation-interface.md)
- [伺服适配设计](../development/ak30_servo_position_design.md)
- [AdapterContract](../development/adapter_contract_v1.md)
