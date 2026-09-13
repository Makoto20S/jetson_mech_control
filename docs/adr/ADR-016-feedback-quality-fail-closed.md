# ADR-016：反馈质量诚实上报与失效关闭策略

- **Decision ID:** ADR-016
- **Status:** Proposed
- **Date:** 2026-09-13
- **Owner:** 项目负责人（`ros2_control` integration owner）
- **Scope:** `RuntimePort::read()` 与 `CanonicalState` 之间的反馈质量语义，以及反馈有效期与设备回报速率的关系；不改变 [ADR-014](ADR-014-ak30-submode-command-interfaces.md) 的状态接口形状（仍是每关节 position/velocity/effort 三个 state interface），不改变 [ADR-015](ADR-015-command-transmit-authorization.md) 的发送授权语义，不解除 [ADR-006](ADR-006-conditional-can0-deployment.md) 的任何设备启用闸门

## Status rationale / 状态依据

本 ADR 改变 canonical 状态语义（`read()` 在什么条件下算失败）并在 `RuntimePort` 上新增一个查询方法，属于 `adapter_contract_v1.md`「新增适配器检查表」第 7 条所说的 canonical 契约变更，因此在实现之前以 `Proposed` 提交。

本 ADR 还包含一项需要**设备侧证据**才能定值的配置约束（第 4 条），该证据目前不掌握。在取得 motor1 实际配置的反馈速率之前，第 4 条只能以「拒绝明显不自洽的配置」的形式实现，不能给出最终数值。

## Context / 上下文

### 当前行为

`Ak30ForceControlRuntime::publish_states()` 对 `SampleQuality` 只分两类：`Valid`/`Degraded` 发布真实数值，其余（`Unknown`/`Stale`/`Invalid`）写入数值 `0.0`，且 `read()` 仍然返回成功。

问题在于 `0.0` 是一个合法的物理量。位置接口上「位置 = 0.0」与「我们不知道位置」在导出的 `double` 里完全无法区分。任何按「读当前位置 → 以该位置为目标 hold」构造命令的控制器（这正是 2026-09-05 台架确立的位置模式首次上电动作），在反馈缺失时会得到一个指向零位的 hold 目标。[ADR-012](ADR-012-command-watchdog-and-capability-honesty.md) Decision 第 3 条已经禁止在命令侧把失效解析成 `0.0`；状态侧存在同一个洞，而且更隐蔽，因为它不经过任何命令路径就能污染命令。

证据本身并没有丢失：`StatusSnapshot` 已经携带 `quality`、`sequence`、`host_rx_time`、`raw_fault_code`。它们是在 ROS 边界上被主动丢弃的。

### 一个必须先解决的前置事实：有效期与回报速率不匹配

`Ak30RuntimeConfig::feedback_ttl_nanoseconds` 的默认值是 **6 ms**，而 `Ak30ForceControlSession` 判定 `Stale` 的条件是 `age > feedback_ttl`。

motor1 的回报速率是 **50 Hz（周期 20 ms）**，由三个独立来源一致确认：

- **设备配置导出**（`company/motor1/first.AppParams.AppParams`）：`<send_can_status_rate_hz>50</send_can_status_rate_hz>`，且 `<send_can_status>1</send_can_status>`、`<can_mode>0</can_mode>`（周期回报）。这是**配置读回值**，不是测量推算。
- **上位机截图**（`company/motor1/can.jpeg`）：「应用设置 → CAN 反馈速率 = 50Hz」。该面板的其余每一项（CAN 模式周期回报、总线速率 1Mbps、CAN ID 104、串口波特率 921600、失控时间 1000ms）都与同一份导出逐条相符，因此截图与导出取自同一设备状态。
- **台架实测**（2026-09-03）：3 秒抓取解析出 150 条 `0x2968` 反馈，平均 50.0 Hz，与标称值相符。

两者相除：在真实电机上，快照在每 20 ms 中约有 14 ms 处于 `Stale`，即大约 **70% 的控制周期**。有效期比回报周期本身短 3.3 倍，是一个在定义上就无法被满足的配置。

今天这一点不可见，正是因为 `Stale` 既不报错也不影响 `read()` 的返回值——它只是安静地发布零。**如果在不动有效期的前提下直接改成失效关闭，第一次上电会在几个毫秒内锁存故障。** 换句话说，当前的「填零」行为在功能上掩盖了一个配置错误；本 ADR 如果只改质量策略而不同时处理有效期，会把一个静默缺陷变成一个必然的启动失败。

回报速率在设备侧可配（L07 §3.1.1.1，范围 1–2000 Hz，上位机「应用设置」面板有「写入参数」）。所以这不是矛盾，是两个参数从未被放在一起校验过；解法可以落在任一侧，见 Decision 第 4 条与替代方案 F。

### 启动瞬态与「从未采样」

`StatusSnapshot::has_sample()` 已经区分了两种不同的「没有好数据」：

- `Unknown`：**从未**收到过反馈。在 500 Hz 控制、50 Hz 回报的部署里，激活后的头十来个周期必然处于这个状态，这是正常瞬态，不是故障。
- `Stale`：**曾经**收到过，之后超过有效期。这意味着设备不再说话，是真实故障。

把这两者用同一条策略处理，要么在启动时误报故障，要么在设备掉线时漏报。

## Decision / 决策

1. **不发明数值。** `Unknown`、`Stale`、`Invalid` 三种质量下，`publish_states()` 一律不写入新的数值，任何情况下都不得把 `0.0` 作为测量值呈现。
2. **`Stale` 与 `Invalid` 失效关闭。** `read()` 返回 `false`，由 `CompositeSystem` 锁存故障并要求显式 cleanup/configure/activate 恢复。设备曾经在说话而现在超时，是故障，不是可以继续运行的降级。
3. **`Unknown` 不是故障，但该关节不可被 claim。** `RuntimePort` 新增 `[[nodiscard]] virtual bool has_valid_sample() const noexcept = 0;`，`CompositeSystem::prepare_command_mode_switch()` 与 `perform_command_mode_switch()` 在其为 `false` 时拒绝 `start_interfaces`。这样「位置未知」期间没有任何控制器能取得该关节的命令接口，配合 ADR-015「无 claim 即无发送授权」，构成完整的失效关闭链：不知道位置 → 拿不到 claim → 发不出命令。启动瞬态因此既不报故障，也不可能被误用。
4. **反馈有效期必须与设备回报周期一致。** 部署配置新增设备侧回报周期（motor1 = 20 ms，取自配置读回值），`configure()` 拒绝 `feedback_ttl_nanoseconds` 小于该周期的配置——一个短于回报周期的有效期在定义上永远无法被满足。**motor1 的有效期定为 60 ms（3 × 回报周期），即容忍连续丢两帧后才判定陈旧。** 该倍数的依据是：判定陈旧的目的是「知道设备不再说话」，而「让电机停下来」由设备自身的失控保护承担——motor1 的 `timeout_msec = 1000`、`timeout_brake_current = 0`，即命令中断 1 秒后自由滑行停止。我们的有效期比那条硬保护短一个数量级还多，因此把倍数从 2 放宽到 3 以避免单帧丢失造成误停，代价可以接受。倍数本身仍应在取得回报抖动分布后复核。
5. **质量证据不得在边界上消失。** `quality`、`sequence`、`host_rx_time`、`raw_fault_code` 必须可通过 runtime 的访问器或结构化日志被测试和诊断读取。禁止用「状态接口是 double，放不下」作为丢弃它们的理由——本 ADR 不扩展状态接口形状，但证据必须在 ROS 边界之外仍然可查。
6. **本 ADR 不改变的东西：** ADR-014 的状态接口形状（仍是三个 `double`）、ADR-015 的发送授权与撤销语义、ADR-012 的命令看门狗分级。

## Alternatives considered / 替代方案

### A. 维持现状：未知/陈旧填 `0.0` 且 `read()` 成功

拒绝：这正是本 ADR 要关闭的缺陷。它使「不知道位置」在导出接口上等同于「位置是零」，而零位是位置接口上一个有实际后果的目标值。

### B. 未知/陈旧写入 NaN

拒绝：`CompositeSystem::read()` 已有 `isfinite` 校验，NaN 会直接锁存故障，效果等同于第 2 条但语义更差——它把「没有数据」和「解码出了损坏的数据」压成同一个信号，而后者是需要独立诊断的协议故障。

### C. `Unknown` 也失效关闭

拒绝：激活后到第一帧反馈之间必然存在若干控制周期，在 50 Hz 回报下约十个。这会使正常启动永远无法完成。

### D. 新增一个 `feedback_quality` 状态接口，把质量交给控制器判断

拒绝（暂时）：这会改变 ADR-014 冻结的状态接口形状，且把安全责任下放给每一个控制器实现者——任何一个忘记检查的控制器都会重新打开这个洞。第 3 条的 claim 闸门把同一个保护放在唯一的、共享的那一层。若将来出现确实需要按质量做连续降级（而非二值可用/不可用）的控制器，再重审此项。

### E. 只提高有效期，不改质量策略

拒绝：那只是把「几乎总是 Stale」调成「偶尔 Stale」，而 Stale 时依然发布零。缺陷的性质没有改变，只是触发变稀有——这比现在更危险，因为更难在测试中被观察到。

### F. 提高设备侧回报速率，而不是放宽有效期

**不拒绝，但不属于本 ADR，也不是本轮该做的事。** 回报速率在上位机「应用设置」面板可写（L07 §3.1.1.1，1–2000 Hz），把它提到数百 Hz 确实能同时缩短检测延迟并提高数据新鲜度。不在本 ADR 内决定的原因有三：

1. 它是一次**写入设备**的持久化配置变更，需要单独授权，且改动后此前所有以 50 Hz 为前提的台架记录都要标注速率已变。
2. 它改变总线负载，必须按 ADR-006 的带宽模型重算——当前部署目标是两电机加两台 HI12 共用一条 1 Mbps 总线，回报速率提高十倍不是本地决定。
3. 它还会改变 USB-CDC 侧每周期需要排空的帧数，`Ak30ForceControlRuntime` 的 `kReceiveBudget`（当前 8）是按「50 Hz 回报、2 ms 周期，稳态每周期不到一帧」推导的，需要同步复核。

**但有一个与之相关、值得单独记录的事实：** 500 Hz 的控制循环配 50 Hz 的反馈，意味着控制器手上的位置数据最旧可达 20 ms。按 2026-09-03 实测的 21 rad/s 换算，20 ms 相当于 0.42 rad ≈ **24°** 的位置不确定性。对当前台架的 hold 与小步进（速度极低）这不构成问题，但**在任何需要速度或轨迹跟随的场景下，50 Hz 反馈本身是否够用是一个独立于本 ADR 的控制问题**，不能因为有效期改对了就认为已经解决。

## Consequences / 后果

### Positive / 正面

- 「不知道位置」在整条链路上都无法被误读成一个数值，且该保护只有一处实现，不依赖每个控制器的自觉。
- 设备掉线成为一个明确的、锁存的故障，而不是一串安静的零。
- 第 4 条把一个已经存在于代码里、但从未被任何检查发现的配置错误（6 ms 有效期 vs 20 ms 回报周期）变成 configure 期的显式拒绝。

### Negative / 负面与代价

- **启动流程改变：** 控制器在第一帧有效反馈到达之前无法被激活。台架和 launch 的启动顺序需要相应调整，spawner 可能需要重试或等待。这是一次可见的行为变化，不是纯内部重构。
- `RuntimePort` 再次新增纯虚方法，所有实现者必须同步——这是继 ADR-015 之后对同一接口的第二次破坏性变更。
- 提高反馈有效期以匹配 20 ms 回报周期，意味着检测设备掉线的延迟也随之变长（数量级从毫秒到数十毫秒）。这是速率决定的物理下限，不是可以通过实现优化掉的；若需要更快的掉线检测，必须提高设备侧的回报速率，而那会增加总线负载，需要按 ADR-006 的带宽模型重算。
- `Stale` 触发锁存故障后需要显式的 cleanup/configure/activate 才能恢复，单条丢帧若超过有效期就会停机。余量倍数定得过紧会造成误停；这正是第 4 条把数值留待实测的原因。

## Validation / 验证

本 ADR 提交时以下验证尚未运行，将在实现阶段补齐并逐项报告：

- `FakeTransport` 注入无反馈、有效反馈、陈旧反馈、降级反馈、故障反馈五种情形，断言 `read()` 返回值、故障锁存、导出状态是否被改写，以及 `quality`/`sequence`/`age` 证据可读。
- 现有测试 `StaleFeedbackYieldsZeroStatesWithoutFault` 必须改写为期望安全行为并先看到 RED，不得作为「通过」证据保留。
- claim 闸门：无有效样本时 `prepare_command_mode_switch`/`perform_command_mode_switch` 拒绝 `start`；注入一帧有效反馈后允许；在真实 `controller_manager` 中验证控制器激活确实被阻塞到反馈到达之后。
- `configure()` 拒绝 `feedback_ttl_nanoseconds` 小于配置回报周期的部署参数组合。
- 完整 `tools/ci/build_workspace.sh` 与 `tools/ci/run_sanitizers.sh` 全绿，并按既有纪律核实 sanitizer 确实启用。

**已取得的证据（2026-09-13）：** motor1 的回报速率为配置值 50 Hz，三源一致（配置导出 `send_can_status_rate_hz=50`、上位机截图「CAN 反馈速率 50Hz」、台架实测 150 帧/3 秒）。Decision 第 4 条的 20 ms 周期与 60 ms 有效期据此定值。

**仍然缺少的证据：** 回报周期的**抖动分布**。现有的三秒抓包只给出平均值符合标称，不足以说明单帧间隔的最坏情况；60 ms（容忍连续丢两帧）是基于设备自身 1000 ms 失控保护留出的保守余量，不是抖动实测的结论。首次只读台架运行（逐帧记录序号与到达时间）应当补齐这一项，届时复核该倍数。

离线验证通过不构成实机验收；设备运行仍受 ADR-006 Decision 第 7 条逐次授权约束。

## Review triggers / 重审触发

- 首次只读台架运行取得反馈周期的抖动分布后，复核 Decision 第 4 条的 3 倍余量；
- 出现确实需要按反馈质量做连续降级而非二值判断的控制器，需重审替代方案 D；
- 设备侧回报速率变更（替代方案 F），需按 ADR-006 的带宽模型重算总线负载、复核 `kReceiveBudget`、并标注此前所有以 50 Hz 为前提的台架记录；
- HI12 或第二台电机接入后，若不同设备的回报速率差异使单一有效期无法同时满足，需要把有效期下沉到逐设备配置；
- 第 3 条的 claim 闸门在实际台架启动流程中被证明会造成无法自动恢复的死锁（例如 spawner 不重试），需要重新设计启动握手。

## Sources / 来源

- [ADR-012：命令看门狗与能力诚实上报](ADR-012-command-watchdog-and-capability-honesty.md)，Decision 第 3 条（失效不得解析为 `0.0`）与能力三态上报。
- [ADR-015：命令发送授权与 pending 命令撤销](ADR-015-command-transmit-authorization.md)，claim 即发送授权——第 3 条的闸门建立在其上。
- [ADR-014：AK3.0 子模式命令接口](ADR-014-ak30-submode-command-interfaces.md)，状态接口形状（本 ADR 不改变它）。
- [ADR-005：单调时间与新鲜度](ADR-005-monotonic-time-freshness.md)，源时间/到达时间分离与 TTL 语义。
- [ADR-003：复合 `ros2_control` `SystemInterface`](ADR-003-composite-system-interface.md)，Decision 第 2 条要求激活时关键状态新鲜。
- [ADR-006：条件式单通道部署](ADR-006-conditional-can0-deployment.md)，回报速率变更需重算的带宽模型。
- [适配器契约 v1](../development/adapter_contract_v1.md)，「新增适配器检查表」第 7 条。
- [CubeMars 资料评审](../planning/06_cubemars_material_review.md)，AK3.0 反馈报文与回报速率的可配置范围。
