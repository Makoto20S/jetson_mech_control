# ADR-003：Foundation/MVP 使用复合 `ros2_control` `SystemInterface`

- **Decision ID:** ADR-003
- **Status:** Accepted
- **Date:** 2026-08-07
- **Owner:** 项目负责人（`ros2_control` integration owner）
- **Scope:** Foundation v0.1 与当前两电机/两 HI12 MVP；不预先决定未来完全独立的硬件组件

## Status rationale / 状态依据

在当前设备共享总线、共享状态/诊断和统一命令租约的前提下，复合组件是已选的 Foundation/MVP 生命周期边界。接受该软件组件形态不等于接受任何真实设备 profile 或允许激活硬件。

## Context / 上下文

ros2_control 的 hardware component 负责生命周期、资源 claim、`read/update/write` 和 controller switch。若每个电机或传感器各自打开同一 CAN，硬件组件之间会重复拥有总线、过滤和故障状态；若把所有设备拆成多个组件，又需要额外协调跨组件的 neutral、状态新鲜度和切换事务。

## Decision / 决策

1. Foundation/MVP 使用一个配置驱动的复合 `SystemInterface`，在同一控制进程中拥有所需的 `BusRuntime` 实例，并导出电机关节命令/状态与 HI12 sensor state interfaces。
2. `on_init` 只解析 schema 和声明接口；`on_configure` 创建经过冲突/能力/负载验证的 bus、codec/session；`on_activate` 要求总线健康、关键状态新鲜且命令为 neutral/无效。
3. `read()` 只复制 core 已发布的完整最新快照，不等待 CAN；`write()` 只对 canonical command 做 finite、limit、slew、mode、freshness 和 deadline 检查并提交，不直接编解码厂商帧。
4. `on_deactivate` 停止命令续租并执行定义好的 neutral 策略；`on_error` 锁存原因，要求显式 cleanup/configure/activate 恢复，不无限自动重启。
5. 若未来需要拆成多个 hardware/sensor component，必须先证明独立总线所有权、故障隔离、恢复和 controller claim 语义，并新增/修订 ADR；不能为了品牌差异拆分。

## Alternatives considered / 替代方案

### A. 每台设备一个 `SystemInterface`/`SensorInterface`

资源粒度看似清晰，但会重复打开总线，难以保证跨设备调度和统一 fault/neutral。只有在未来物理隔离和独立恢复证据成立后才可重审。

### B. 独立 CAN 网关节点 + 多个 ROS 组件

会把闭环依赖 DDS 和跨进程恢复，违背 ADR-001/002 的确定性与单 writer 边界。拒绝作为控制路径。

### C. 一个配置驱动的复合组件（选定）

统一资源 claim、生命周期和总线所有权，代价是组件内局部故障需要细粒度诊断和明确降级策略。

## Consequences / 后果

### Positive / 正面

- 单一组件可以原子地验证配置、capability、状态新鲜度、neutral 和总线负载。
- Controller Manager 的 STRICT switch 与 canonical generation 在一个硬件边界内可审查。
- 真实协议仍留在 codec/session，未来新增设备通常只增加 adapter/config/test。

### Negative / 负面与代价

- 一个 component 的生命周期错误可能使多个设备一起进入 INACTIVE/FAULT_LATCHED。
- 接口导出和诊断快照需要清楚区分电机、传感器、raw/vendor 字段，避免大而模糊的组件 API。
- 若未来确实需要独立恢复，拆分会涉及 controller 配置、claim 和部署迁移。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，并确认 ADR-003 的规划反向链接有效。
- FND-012 测试验证 configure/activate/deactivate/cleanup 的错误路径、接口导出、第二 writer 拒绝和非阻塞 `read/write`。
- FND-014 连续 lifecycle 与 STRICT switch 测试证明 generation、slew/limit 和 TTL 不因切换越界；失败时进入 neutral/INACTIVE/FAULT_LATCHED。
- 任何真实 adapter 必须通过既有 `SystemInterface` 接口；若需要修改 canonical 语义，先触发 ADR review。

## Review triggers / 重审触发

- 证明某一物理总线需要独立进程/独立安全域和独立恢复，且单 component 无法提供可接受隔离；
- ros2_control 生命周期或 resource-claim 语义无法表达复合组件的实际资源；
- 多组件拆分能在不增加 DDS 闭环依赖的前提下改善可验证性，并有重复 writer 防护证据；
- FND-012/014 的生命周期、切换或故障测试持续暴露无法通过组件边界解释的问题。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 9、10、13 节。
- [Foundation 依赖方向与 Issue 顺序](../planning/07_framework_bootstrap_plan.md)，第 4、8.1、9.2、9.4 节。
- [ros2_control Humble controller manager 文档](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)。
- [FND-004 ADR index](README.md)。
