# ADR-002：每条物理 CAN 的单一 `BusRuntime` 写者

- **Decision ID:** ADR-002
- **Status:** Accepted
- **Date:** 2026-08-07
- **Owner:** 项目负责人（Foundation core/transport owner）
- **Scope:** 同一控制进程内的每条物理 CAN；只读观察者不取得写权限

## Status rationale / 状态依据

单写者是为了解决总线命令调度、租约、错误归因和故障过期的一致性问题；它不要求当前已经启用任何物理 CAN，也不预设设备 ID 或位速率。

## Context / 上下文

SocketCAN 允许多个监听者接收匹配帧，但多个可写组件会分别维护队列、超时和重试，无法形成一个可审查的命令事实源。多个 ros2_control plugin、独立设备线程或 DDS 网关还可能在同一物理接口上互相覆盖命令。周期命令也不应在拥塞后补发已经过期的历史值。

## Decision / 决策

1. 每条物理 CAN 只能有一个获得写权限的 `BusRuntime`。它拥有 transport open/close、精确过滤、错误帧订阅、RX 接收、帧路由入口、唯一 TX 调度、总线状态和统计。
2. 每个可写设备使用最新有效命令槽（含 generation、mode、deadline、limits result）；新周期值覆盖旧值，禁止无界历史重放。配置/诊断请求使用有界低频队列。
3. 配置阶段对物理接口、逻辑总线映射和可写路由做冲突检查；第二个 writer 必须拒绝激活。逻辑总线别名不能绕过物理总线的单写者约束。
4. `candump`、监控或录制工具可以并行只读观察，但不得发送控制帧。不同物理接口可以各有一个独立 `BusRuntime`，不把多总线假装成同一队列。
5. BusRuntime 不负责协议缩放、控制算法、DDS、文件 I/O 或厂商生命周期；这些职责分别由 core、codec/session、controller 和非 RT 工具承担。

## Alternatives considered / 替代方案

### A. 每台设备独立 socket 和写线程

局部实现简单，但会产生多个 TX owner、重复过滤和无法统一的 neutral/TTL 行为。拒绝。

### B. 独立 ROS/DDS CAN 网关作为命令 owner

可集中总线访问，却把序列化、跨进程延迟和网关重启语义放入闭环。只读网关可用于诊断，不作为实时命令路径。

### C. 每条物理总线一个 BusRuntime（选定）

集中实现调度和故障计数，代价是一个 runtime 成为该物理总线的故障域；需要用多总线部署和明确恢复策略控制影响面。

## Consequences / 后果

### Positive / 正面

- 命令租约、TTL、neutral、队列覆盖和错误统计只有一个权威 owner。
- 可以在 fake、vcan 和未来 SocketCAN 后端之间复用同一上层行为契约。
- 多设备路由和总线负载预算在 configure/activation 阶段集中检查。

### Negative / 负面与代价

- 一个 BusRuntime 的故障可能影响该物理总线上的多个设备，需要 fault-latch 和独立断能策略。
- 单 runtime 的 RX/TX 调度比“每台设备一个线程”更复杂，必须测量相位、仲裁和 queue 行为。
- 只读工具与控制 owner 的权限边界需要部署配置和审查，不能只靠约定。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，并确认本 ADR 在 planning 入口有反向链接。
- FND-008 配置测试必须拒绝两个 writer 对同一物理接口或同一可写命令路由的声明。
- FND-009 fake runtime 测试必须覆盖最新命令覆盖、TTL 到期、queue-full、drop/error 计数和 bus fault 状态转移，且无 sleep。
- FND-010 vcan 测试必须证明一个写 owner 可接收过滤帧和错误帧，多个只读监听者不会消费或篡改控制队列。
- 后续 G1/G4 记录 `canbusload`、仲裁延迟、TX/RX drop 和 bus 状态；通过条件由 ADR-006 与 Foundation 验收共同约束。

## Review triggers / 重审触发

- 某个协议库只能由独立事件循环写入同一物理总线，无法嵌入或受单 writer 管理；
- 实测 command-to-wire deadline、错误隔离或恢复时间在单 runtime 下不达标；
- 需要把两个安全独立的写 owner 放在同一接口，且有明确硬件仲裁/隔离证据；
- 多总线部署暴露出逻辑别名、过滤 fan-out 或权限模型无法静态检查。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 3、4.1、5、13 节。
- [Foundation 核心契约](../planning/07_framework_bootstrap_plan.md)，第 9.1、9.4、11.2 节。
- [Linux SocketCAN 文档](https://www.kernel.org/doc/html/latest/networking/can.html)。
- [FND-004 ADR index](README.md)。
