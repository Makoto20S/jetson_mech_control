# ADR-001：纯 C++ 核心与薄 `ros2_control` 适配层

- **Decision ID:** ADR-001
- **Status:** Accepted
- **Date:** 2026-08-07
- **Owner:** 项目负责人（Foundation core/`ros2_control` owner）
- **Scope:** Foundation v0.1、MVP 以及保持 canonical 接口不变的后续设备适配

## Status rationale / 状态依据

该边界已经在 Foundation 规划和架构比较中被选定，且不依赖任何一台真实设备的固件、CAN ID 或物理标定。接受的是软件职责边界，不是硬件兼容性结论。

## Context / 上下文

控制器需要同时服务 fake、vcan 和未来多品牌设备。若把 SocketCAN、厂商位域、协议状态机和阻塞式 I/O 直接写进一个或多个 hardware plugin，协议就会依赖 ROS 生命周期，多个组件还可能争夺同一条总线。这样的实现难以脱离 ROS 做确定性单测，也会让供应商差异渗入控制器。

候选方案包括：把协议全部放进 `SystemInterface`；建立独立 CAN 网关并通过 DDS 发送闭环命令；或把传输、协议、时间和状态核心保持为不依赖 ROS 的 C++ 库，再用薄适配层接入标准生命周期。

## Decision / 决策

采用“纯 C++ 核心 + 薄 `ros2_control` 适配层”：

1. `mech_control_core` 拥有 transport 抽象、时间/新鲜度、配置/capability、路由、canonical state/command、command lease、错误统计和 `BusRuntime` 边界；公共头不得依赖 ROS headers。
2. protocol codec 和 device session 作为独立 C++ 组件，codec 只做纯编解码，session 负责设备状态、能力和命令语义；二者不打开 socket、不发布 ROS 消息。
3. `ros2_control` 适配层只负责配置映射、生命周期、资源 claim、标准状态/命令接口和非阻塞 `read()`/`write()`，具体复合组件边界由 ADR-003 规定。
4. C++ controller 只依赖 canonical 接口、质量/新鲜度和租约契约，不构造 CAN 帧、不判断供应商品牌。
5. Python、记录、诊断和 UI 不进入确定性设备 I/O 路径；跨边界数据必须经过有限、可过期的接口。

## Alternatives considered / 替代方案

### A. 协议直接写在 hardware plugin

起步代码较少，但会把总线所有权、厂商字段和 ROS 生命周期耦合，难以复用和脱 ROS 测试。拒绝作为默认架构。

### B. 独立 CAN 网关 + DDS 闭环

可隔离进程，但把序列化、DDS 调度和跨进程失效处理放入电机闭环，且不能天然保证单写者时序。只允许作为只读诊断/录制旁路，不作为命令路径。

### C. 纯 C++ 核心 + 薄适配层（选定）

增加接口和映射工作，但能统一总线所有权、测试边界和供应商扩展。

## Consequences / 后果

### Positive / 正面

- codec、路由、状态机和租约可在无 ROS、无硬件环境中单测、模糊测试和确定性重放。
- 更换 ROS 发行版或增加设备品牌时，控制器和核心 canonical 语义保持稳定。
- 适配层成为明确的生命周期/接口边界，能在 `read/write` 中禁止阻塞、日志和动态协议分支。

### Negative / 负面与代价

- 需要维护核心类型与 ROS interface 的映射，初期代码量高于直接写 plugin。
- 若边界不受约束，可能出现重复状态类型或在适配层偷偷加入厂商逻辑。
- 核心自身不提供功能安全；急停、断能、台架和设备 watchdog 仍由后续闸门单独验证。

## Validation / 验证

以下是可执行的验收条件，当前 FND-004 只验证文档结构，运行时验证留给依赖任务：

- `python3 tools/ci/check_adrs.py` 通过，并确认本 ADR 与 planning 入口链接有效。
- FND-005 之后，核心公共头的依赖检查不得发现 ROS headers；核心目标可在无 ROS 消息的单元测试中构建。
- FND-010～FND-014 的 fake/vcan 纵向测试证明 controller → canonical command → transport → state 的链路，且 controller/适配层不含厂商 CAN 位域。
- 适配层测试证明 `read()`/`write()` 不等待 CAN 帧、不执行文件/DDS/字符串格式化；失败时按生命周期契约返回可解释状态。

## Review triggers / 重审触发

- 核心必须暴露 ROS 类型、阻塞调用或供应商品牌分支才能满足一个已确认需求；
- fake/vcan 无法在脱离 ROS 的情况下重放并验证关键状态/租约语义；
- 新 ROS 发行版改变 lifecycle/resource-claim 契约，且适配层无法隔离影响；
- 实测证明单进程边界造成不可接受的故障隔离或 deadline 违约。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 1、2、4、9、11 节。
- [Foundation 搭建计划](../planning/07_framework_bootstrap_plan.md)，第 3、4、8.1、9 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，第 3.1、3.7 节。
- [FND-004 ADR index](README.md)。
