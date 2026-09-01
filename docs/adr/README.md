# Architecture Decision Records / 架构决策记录

> FND-004 ADR 基线
>
> 本目录记录 Foundation v0.1 在运行时代码实现前需要遵守的架构边界和语义约束。ADR 是共享项目事实源；个人 `memory/` 和 handoff 不替代本目录。

## Decision set / 决策集合

| ADR | Status | File | Decision boundary | Required evidence or review |
|---|---|---|---|---|
| [ADR-001](ADR-001-core-boundary.md) | Accepted | [Core boundary](ADR-001-core-boundary.md) | 纯 C++ 核心与薄 `ros2_control` 适配层 | Core 可脱离 ROS 构建/测试；适配层不承载厂商位域或阻塞 I/O |
| [ADR-002](ADR-002-bus-runtime-ownership.md) | Accepted | [BusRuntime ownership](ADR-002-bus-runtime-ownership.md) | 每条物理 CAN 的单一写者和调度所有权 | 重复 writer 配置被拒绝；fake/vcan 统计和租约行为可重复 |
| [ADR-003](ADR-003-composite-system-interface.md) | Accepted | [Composite SystemInterface](ADR-003-composite-system-interface.md) | Foundation/MVP 的复合硬件组件生命周期 | lifecycle、claim、STRICT switch 和非阻塞 `read/write` 测试 |
| [ADR-004](ADR-004-fixed-protocol-profile.md) | Accepted | [Fixed protocol profile](ADR-004-fixed-protocol-profile.md) | 配置期固定协议代际和 active command profile | golden/negative frame、capability 和 profile 冲突测试 |
| [ADR-005](ADR-005-monotonic-time-freshness.md) | Accepted | [Monotonic time and freshness](ADR-005-monotonic-time-freshness.md) | 源时间、到达时间、单调时钟和 TTL/freshness | virtual-clock 边界、重复读取不刷新时间戳、stale/TTL 测试 |
| [ADR-006](ADR-006-conditional-can0-deployment.md) | Proposed | [Conditional single-channel deployment](ADR-006-conditional-can0-deployment.md) | 当前单物理通道、多 transport backend 的证据闸门与双总线扩展边界 | 逐台设备配置、backend 能力、ID/位速率、负载、仲裁和错误证据；G0/G1/G4 评审 |
| [ADR-009](ADR-009-effort-semantic-gate.md) | Accepted | [Effort semantic gate](ADR-009-effort-semantic-gate.md) | `effort [N*m]` 的物理语义闸门与最小 demo 边界 | 匹配固件/参数、机械映射、校准和受控台架证据；G0–G3 评审后才可启用设备映射 |
| [ADR-012](ADR-012-command-watchdog-and-capability-honesty.md) | Accepted | [Command watchdog and capability honesty](ADR-012-command-watchdog-and-capability-honesty.md) | 命令看门狗分级语义、能力三态上报与远程帧表达 | 项目负责人复核追认记录；真实抖动/位速率证据；vcan 与硬件验证 |
| [ADR-013](ADR-013-ak30-protocol-baseline.md) | Accepted | [AK3.0 protocol baseline](ADR-013-ak30-protocol-baseline.md) | 协议基线由 L02（AK2.0）切换为 L07（AK3.0）、`ProtocolProfile` 重定义、力控优先、`effort` 解锁 | 驱动板 `AK54-4810-1C-A2` 对应关系；项目负责人复核 Kt 对定制版的适用性；带宽重算结论 |

## Reading and status rules / 阅读与状态规则

- `Accepted` 表示本 ADR 的架构/语义约束已经作为 Foundation 实施边界采用；它不表示真实设备、CAN 总线或物理性能已经验证。
- `Proposed` 表示方向和安全边界已写清，但仍缺少本文件列出的决定性证据或批准。实现可以据此保守拒绝未知配置，不能据此激活真实设备。
- 本轮 FND-004 冻结的是 ADR-001～ADR-006 与 ADR-009。ADR-012 是 Foundation RC 评审后的追认记录，已于 2026-08-31 复核转 Accepted；其批准范围仅限接口语义，不解除任何设备启用闸门。ADR-013 于 2026-09-01 提交并在同日复核转 Accepted，它在任何实现进入 `main` 之前完成，与 ADR-012 的追认路径相反；其批准范围为协议基线与接口语义，同样不解除任何设备启用闸门。规划中提到的 ADR-007、ADR-008、ADR-010 和 ADR-011 仍是候选后续决策，尚未形成独立规范文件。
- 供应商资料、配置导出、抓包、测量或测试与 ADR 冲突时，先停止受影响路径并按各 ADR 的“重审触发”更新记录；不得静默改写协议常量或标准接口语义。

## FND-004 verification / FND-004 验证

在仓库根目录运行：

```bash
python3 tools/ci/check_adrs.py
python3 tools/ci/context_check.py
```

检查器验证全部九个 ADR 文件、状态枚举、必需章节、内部链接和规划入口的反向链接。它是文档结构检查，不替代后续 core、vcan、ARM64 或硬件验收。

## Deferred candidates / 延后候选

以下主题保留在规划层，待出现独立范围和证据后再创建 ADR：

- ADR-007：目标原生运行与开发/CI 容器化的完整部署策略；
- ADR-008：Python 低频目标、序号和 TTL 的跨进程接口；
- ADR-010：实验大数据、rosbag、模型和外部资产保留策略；
- ADR-011：ROS 发行版迁移和单发行版支持策略。
