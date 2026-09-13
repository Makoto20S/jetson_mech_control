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
| [ADR-014](ADR-014-ak30-submode-command-interfaces.md) | Accepted | [Sub-mode command interfaces](ADR-014-ak30-submode-command-interfaces.md) | `CanonicalCommand` 扩展三字段、CompositeSystem 单命令接口 ∈ {position, velocity, effort}、AK3.0 子模式命令映射与 Velocity 模式 effort=0 强制 | 2026-09-13 项目负责人在决策实质被复述后批准接口形状与命令映射语义；Decision 3 的 fail-closed 已由 `Ak30System::on_init` 在任何设备 I/O 之前补齐运行期校验，不再只靠离线结构测试；实机运行仍逐次授权并遵守 ADR-006 Decision 7 |
| [ADR-015](ADR-015-command-transmit-authorization.md) | Accepted | [Command transmit authorization](ADR-015-command-transmit-authorization.md) | resource claim 即逐 joint 发送授权；`RuntimePort::write` 入参改为 `CommandDispatch`（命令 + 授权位）并新增 `cancel_pending(index)`；stop/deactivate/cleanup/error 立即撤销 pending 命令；硬件循环本身不构成命令刷新 | 2026-09-13 项目负责人批准接口变更；离线证据为全套测试全绿 + 无 sanitizer 报告（已核实 sanitizer 启用），并已在真实 `controller_manager` + fake transport 下验证无 claim/停用后电机命令帧计数为 0；实机运行仍受 ADR-006 Decision 7 约束 |
| [ADR-016](ADR-016-feedback-quality-fail-closed.md) | Accepted | [Feedback quality fail-closed](ADR-016-feedback-quality-fail-closed.md) | `Unknown`/`Stale`/`Invalid` 不得写入数值零；`Stale`/`Invalid` 锁存故障；`Unknown` 时关节不可被 claim（`RuntimePort` 新增 `has_valid_sample()`）；反馈有效期不得小于设备回报周期，motor1 定为 60 ms；质量/序号/到达时间/故障码必须保持可查 | 项目负责人批准策略与 `RuntimePort` 变更后方可实现；回报速率 50 Hz 已三源确认（配置导出、上位机截图、台架实测），据此认定默认 6 ms 有效期比回报周期短 3.3 倍；**回报抖动分布仍缺**，待首次只读台架运行补齐后复核余量倍数；实机运行仍受 ADR-006 Decision 7 约束 |

## Reading and status rules / 阅读与状态规则

- `Accepted` 表示本 ADR 的架构/语义约束已经作为 Foundation 实施边界采用；它不表示真实设备、CAN 总线或物理性能已经验证。
- `Proposed` 表示方向和安全边界已写清，但仍缺少本文件列出的决定性证据或批准。实现可以据此保守拒绝未知配置，不能据此激活真实设备。
- 本轮 FND-004 冻结的是 ADR-001～ADR-006 与 ADR-009。ADR-012 是 Foundation RC 评审后的追认记录，已于 2026-08-31 复核转 Accepted；其批准范围仅限接口语义，不解除任何设备启用闸门。ADR-013 于 2026-09-01 提交并在同日复核转 Accepted，它在任何实现进入 `main` 之前完成，与 ADR-012 的追认路径相反；其批准范围为协议基线与接口语义，同样不解除任何设备启用闸门。ADR-014 于 2026-09-07 以 `Proposed` 提交（CompositeSystem 接口形状与 AK3.0 子模式命令接口），走「批准 → 合并」路径，于 2026-09-13 经项目负责人批准转 Accepted；批准时其决策实质被逐条复述，范围仅为接口形状与命令映射语义。ADR-015 于 2026-09-13 在实现之前以 `Proposed` 提交（命令发送授权与 pending 撤销），同日经项目负责人批准转 Accepted；其离线证据包含真实 `controller_manager` + fake transport 的电机帧计数，但不含任何实机行为。ADR-016 于 2026-09-13 以 `Proposed` 提交（反馈质量失效关闭），同日补齐设备侧证据（回报速率 50 Hz 三源一致）后经项目负责人批准转 Accepted；其回报**抖动**分布仍缺，首次只读台架运行须补齐并据此复核有效期余量。规划中提到的 ADR-007、ADR-008、ADR-010 和 ADR-011 仍是候选后续决策，尚未形成独立规范文件。
- 供应商资料、配置导出、抓包、测量或测试与 ADR 冲突时，先停止受影响路径并按各 ADR 的“重审触发”更新记录；不得静默改写协议常量或标准接口语义。

## FND-004 verification / FND-004 验证

在仓库根目录运行：

```bash
python3 tools/ci/check_adrs.py
python3 tools/ci/context_check.py
```

检查器验证全部十个 ADR 文件、状态枚举、必需章节、内部链接和规划入口的反向链接。它是文档结构检查，不替代后续 core、vcan、ARM64 或硬件验收。

## Deferred candidates / 延后候选

以下主题保留在规划层，待出现独立范围和证据后再创建 ADR：

- ADR-007：目标原生运行与开发/CI 容器化的完整部署策略；
- ADR-008：Python 低频目标、序号和 TTL 的跨进程接口；
- ADR-010：实验大数据、rosbag、模型和外部资产保留策略；
- ADR-011：ROS 发行版迁移和单发行版支持策略。
