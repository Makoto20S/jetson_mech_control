# ADR-017：命令新鲜度的硬件可观测代号接口

- **Decision ID:** ADR-017
- **Status:** Proposed
- **Date:** 2026-09-14
- **Owner:** 项目负责人（`ros2_control` integration owner）
- **Scope:** 每关节命令接口集合新增一个 `command_generation` 命令接口，使硬件层能够区分「控制器刷新了目标」与「`controller_manager` 转了一圈」；据此修订 [ADR-014](ADR-014-ak30-submode-command-interfaces.md) Decision 第 2 条（每关节恰好一个命令接口）。不改变状态接口形状，不改变 [ADR-015](ADR-015-command-transmit-authorization.md) 的 claim 即授权语义，不改变 [ADR-016](ADR-016-feedback-quality-fail-closed.md) 的反馈闸门，不解除 [ADR-006](ADR-006-conditional-can0-deployment.md) 的任何设备启用闸门

## Status rationale / 状态依据

本 ADR 改变 canonical 命令契约的形状（每关节导出的命令接口数量与语义），属于 [适配器契约 v1](../development/adapter_contract_v1.md)「新增适配器检查表」第 7 条所说的 canonical 契约变更，因此在实现之前以 `Proposed` 提交。

它同时是一周之内对 `RuntimePort` 相邻契约的**第三次**破坏性变更（ADR-015 改 `write()` 签名并新增 `cancel_pending()`；ADR-016 新增 `has_valid_sample()`）。这一点单独记录在「后果」中，因为它影响的不只是本仓库。

## Context / 上下文

### 当前缺陷（已量化，不是推断）

控制器保持 claim 但停止写入目标时，`CompositeSystem::write()` 每个周期仍然派发一条 authorized 命令，`Ak30ForceControlRuntime::write()` 每次都置 `fresh_write_`。**实测：20 个静默周期产生 20 个电机命令帧。**

该缺陷已经以 `DISABLED_SilentControllerDoesNotRefreshItsCommand` 与 `DISABLED_ReactivationDoesNotReplayTheOldTarget` 两条测试提交在仓库里，它们是本 ADR 的出口判据。

### 根因：命令接口是一块没有笔迹的白板

一个 `ros2_control` 命令接口在导出后就是一个裸 `double*`。硬件层可以读到**上面写着什么**，但无法观测**是否有人写过**：`set_value()` 不留下任何痕迹。

ADR-015 之所以能把「控制器崩溃/被停用」这条路关掉，正是因为那一半**是可观测的**——`controller_manager` 会明确宣告 claim 与 release，硬件层收得到这两个事件。新鲜度这一半没有对应的事件，因此不可能用同样的手法关闭。

### 同源的第二个症状

停用后重新激活会重放停用前的目标，因为撤销授权时**刻意保留**了最后一个命令值。把它置零会更糟：在位置接口上，一个被替换进去的 `0.0` 是一条「运动到标定零位」的命令，这正是 [ADR-012](ADR-012-command-watchdog-and-capability-honesty.md) Decision 第 3 条禁止的事。两个症状同根，应当由同一个机制一并关闭。

### 为什么「给控制器加有效期」关不掉它

两条 DISABLED 测试驱动的是 `WriterController`——一个写若干次就闭嘴的假控制器，**不是** `DemoController`。给 `DemoController` 加 TTL 不会让它们转绿，这是刻意的：判据是**硬件不能被任意控制器糊弄**，而不是「我们自己写的那个控制器行为良好」。安全保护若依赖每个控制器实现者的自觉，就等于没有保护——这与 ADR-016 拒绝替代方案 D 的理由相同。

### 已取得的前置证据（2026-09-14）

方案要求同一关节导出两个命令接口并被同一控制器同时 claim。该前提已用一次性探针在**真实 `ControllerManager`**（`update_rate` 500）下验证：

- STRICT `switch_controller` 返回 **OK**，控制器最终持有 **2** 个命令接口，两个接口写入的值都到达硬件侧；
- `prepare_command_mode_switch()` 与 `perform_command_mode_switch()` **各只被调用一次**，`start_interfaces` 在**同一次调用里一次带齐两个名字**（`joint_1/position`、`joint_1/command_generation`），二者指向同一关节。

**这条证据的边界必须同时记录**：探针使用的是一个最小的假 `SystemInterface`，**不是** `CompositeSystem`，因此它证明的是上游框架允许这种形状，**不**证明本仓库的实现会通过；它只覆盖单关节、单控制器，未覆盖两个控制器争抢同一关节，也未覆盖与 ADR-016 claim 闸门的时序交互。探针跑完即删，不作为回归测试保留——它测的是框架而不是本项目的代码。

已知会挡住实现的两处（读代码得出）：`CompositeSystem::validate_info()` 要求 `command_interfaces.size() == 1U`；`known_command_interface()` 只承认 `<joint>/<该关节唯一的命令接口名>`，会使 `validate_switch()` 拒绝代号接口名。两处正是本 ADR 修订 ADR-014 Decision 第 2 条所指向的地方。

## Decision / 决策

1. **每关节新增一个名为 `command_generation` 的命令接口。** 它与运动命令接口（`position`/`velocity`/`effort` 三者之一）并列导出，属于同一关节，由同一个控制器一并 claim。

2. **控制器每写入一次真实目标，就必须改变该关节的 `command_generation` 值。** 「真实目标」指控制器实际决定了一个新的命令值，而非仅仅被 `update()` 调用了一次。

3. **硬件层只在代号**发生变化**时才认为收到了新命令。** 判据是与基线**不相等**（`!=`），不是「大于」。使用不等而非有序比较，是为了让控制器重启后从任意值开始都能被识别为变化，而不要求全局单调。

4. **代号未变化即视为没有新命令，硬件层不得发送。** 这是本 ADR 的全部作用：把「manager 转了一圈」与「控制器说了话」分开。

5. **撤销授权时，把基线重置为命令缓冲区的当前值。** 这样重新激活之后，除非控制器写入一个新的代号，旧目标不会被重放——从而用同一个机制关闭第二个症状。首次取得 claim 时同样以缓冲区当时的值作为基线，使激活之前遗留的任何数值都不会被当作新命令。

6. **代号是计数器语义，不是数值语义。** 一个合法地保持稳定设定点的控制器会反复写入同一个目标值，因此**对运动命令值本身做变化检测是不成立的**——它无法区分「稳住不动」与「已经死了」。代号是一个独立于目标值的、专门用来表达「我又说了一次」的量。

7. **不使用时间戳。** 时间戳会引入第二个由控制器控制的时钟域，而本项目已经因为 [ADR-005](ADR-005-monotonic-time-freshness.md) 和 T4 的实测（看门狗读 `rclcpp::Time`，冻结 sim time 下陈旧目标永不失效）确立了不得依赖可被部署冻结的时钟。

8. **非有限的代号值一律拒绝，且不得被当作变化。** 与 T4 对非有限目标的处理一致：丢弃而不是钳位，更不得把它解释成刷新。

9. **修订 ADR-014 Decision 第 2 条。** 「每关节恰好一个命令接口」改为「每关节恰好一个**运动**命令接口（`position`/`velocity`/`effort` 三者之一，与 `sub_mode` fail-closed 匹配）**加上恰好一个 `command_generation` 接口**」。ADR-014 关于子模式对应关系、Velocity 强制 `effort = 0`、状态接口形状不变的其余决策一律不变。

10. **明确记录本机制观测不到的东西。** 硬件层能观测的是代号的**变化**，不是写入动作本身。一个反复写入同一个代号值的控制器，与一个停止写入的控制器在硬件层完全等价。这是设计边界而非缺陷：改变代号是控制器的契约义务，本 ADR 的威胁模型是「控制器安静地停止工作」，不是「控制器蓄意伪装」。

## Alternatives considered / 替代方案

### A. 维持现状，继续把新鲜度缺口开着

拒绝。缺口的后果按接口类型分级：position 接口上「保持最后一个目标」约等于 hold，有人在场时可接受；但 **velocity 接口上等于电机一直转，effort 接口上等于一直施力**。T8/T9 正是引入这两种控制器的地方，所以缺口最迟必须在它们之前关闭。

### B. 给控制器加目标有效期（TTL）

拒绝。见上文「为什么『给控制器加有效期』关不掉它」：判据是硬件不被任意控制器糊弄，而两条出口测试驱动的是 `WriterController`。这条替代方案连它自己的验收测试都过不了。

### C. 用时间戳接口代替计数器

拒绝。见 Decision 第 7 条：引入第二个可被部署冻结的时钟域，本项目已经为此付出过一次代价。

### D. 不新增接口，由硬件层检测运动命令值的变化

拒绝。见 Decision 第 6 条：合法地保持稳定设定点的控制器会被误判为已死，而 hold 恰恰是本项目当前唯一验证过的动作。

### E. 用 metadata 或非命令接口承载代号

拒绝。`ros2_control` 的 metadata 不经过 claim，也没有每控制周期的写入路径；任何控制器都能改或不改它，且它不在 `perform_command_mode_switch` 的授权链上。把安全信号放在一条不受授权约束的旁路上，等于绕过 ADR-015 刚刚建立的边界。

### F. 由硬件层统计 `write()` 被调用的次数

拒绝。`controller_manager` 每个控制周期都会调用 `write()`，这正是本缺陷本身——被调用的次数恰恰是那个不携带任何信息的量。

### G. 由 `CompositeSystem` 在 ROS 层缓存上一周期的命令值做对比

拒绝。这是 D 的换位实现，缺陷相同：它比较的仍然是数值而非「是否被写过」。

### H. 等待 `ros2_control` 上游提供原生的「命令是否被写入」语义

**不拒绝，但不能等。** 若上游将来提供该语义，本 ADR 的机制应当让位于它（见「重审触发」）。但当前 Humble 没有该语义，而缺口的截止点是 T8/T9。

## Consequences / 后果

### Positive / 正面

- 「沉默的控制器」与「重新激活重放旧目标」两个同源症状由同一个机制一并关闭。
- 保护落在唯一的、共享的硬件层，不依赖任何控制器实现者的自觉——与 ADR-016 claim 闸门的思路一致。
- 电机命令帧的发送频率不再等于控制周期，而等于控制器**真实的刷新频率**。稳态下这会降低总线负载。

### Negative / 负面与代价

- **这是 `RuntimePort` 相邻契约一周内的第三次破坏性变更。** 本仓库内的三个实现者会同步更新；**任何本仓库之外的实现者都会直接编译失败**。
- **任何不 claim 并改变 `command_generation` 的控制器，从此再也命令不了电机。** 这是刻意的失效关闭，但代价很实在：**`joint_trajectory_controller` 等标准 `ros2_control` 控制器不会写这个接口，因此无法直接驱动本硬件**，必须包一层或改用本仓库的控制器。这是本 ADR 最大的一项代价，接受它等于接受本项目的硬件不再是一个通用 `ros2_control` 目标。
- 发送频率随控制器刷新率变化，意味着任何按「每控制周期一帧」推导的发送侧带宽估算需要按 ADR-006 的模型复核。（接收侧的 `kReceiveBudget` 由设备回报速率决定，不受本 ADR 影响。）
- 改动面：`CompositeSystem` 与 `RuntimePort`、仓库内三个 `RuntimePort` 实现者、`WriterController` 与 `DemoController`、三个 motor1 xacro、以及部署结构测试。
- ADR-014 Decision 第 2 条被修订，此前所有引用「每关节恰好一个命令接口」的表述都需要同步。

## Validation / 验证

本 ADR 提交时以下验证尚未运行，将在实现阶段补齐并逐项报告：

- **出口判据**：`DISABLED_SilentControllerDoesNotRefreshItsCommand` 与 `DISABLED_ReactivationDoesNotReplayTheOldTarget` 两条测试启用并转绿，且**不得削弱其断言**。它们驱动 `WriterController`，即一个不受我们控制的控制器形状。
- 在真实 `controller_manager` 下验证 `CompositeSystem` 导出双命令接口后仍能被同一控制器 claim——上文探针只覆盖了假 `SystemInterface`，这一项必须在本项目的实现上重做。
- 代号未变化时不产生电机命令帧；代号变化时产生且仅产生一帧。断言对象是 `FakeTransport` 的真实 AK3.0 帧计数，不是某个 C++ 方法是否被调用。
- 撤销后重新激活，在控制器写入新代号之前不得产生任何命令帧。
- 非有限代号值被拒绝且不被计为变化。
- 两个控制器先后 claim 同一关节时基线的行为（探针未覆盖）。
- `configure()`/`on_init()` 对新接口形状的 fail-closed 校验：xacro 声明与导出形状不一致时拒绝，且拒绝发生在取得串口之前（与 T3 建立的顺序一致）。
- 完整 `tools/ci/build_workspace.sh` 与 `tools/ci/run_sanitizers.sh` 全绿，并按既有纪律核实 sanitizer 确实启用（读 CMake cache，不是只看脚本名）。

离线验证通过不构成实机验收；设备运行仍受 ADR-006 Decision 第 7 条逐次授权约束。

## Review triggers / 重审触发

- `ros2_control` 上游出现原生的「命令是否被写入」语义（替代方案 H），本机制应当让位；
- 需要接入不受本仓库控制的第三方控制器（如 `joint_trajectory_controller`）驱动本硬件时，重审「不写代号即不能命令」这一代价；
- 两个控制器争抢同一关节的实测行为与探针结论不符；
- 多关节或多设备部署下，逐关节一个代号接口的数量增长成为问题；
- 若将来证明存在合法的控制器形状，其刷新频率低到使发送频率不足以满足设备失控保护（motor1 `timeout_msec = 1000`），需要重新引入一条与代号无关的保底发送路径。

## Sources / 来源

- [ADR-014：AK3.0 子模式命令接口](ADR-014-ak30-submode-command-interfaces.md)，Decision 第 2 条由本 ADR 修订。
- [ADR-015：命令发送授权与 pending 命令撤销](ADR-015-command-transmit-authorization.md)，claim 即发送授权；本 ADR 补上它无法覆盖的那一半。
- [ADR-016：反馈质量诚实上报与失效关闭策略](ADR-016-feedback-quality-fail-closed.md)，把保护放在唯一共享层而非下放给控制器的先例（其替代方案 D）。
- [ADR-012：命令看门狗与能力诚实上报](ADR-012-command-watchdog-and-capability-honesty.md)，Decision 第 3 条（失效不得解析为 `0.0`），撤销时不得置零的依据。
- [ADR-005：单调时间与新鲜度](ADR-005-monotonic-time-freshness.md)，不得依赖可被部署冻结的时钟。
- [ADR-003：复合 `ros2_control` `SystemInterface`](ADR-003-composite-system-interface.md)，命令接口形状与 `CompositeSystem` 的职责边界。
- [ADR-006：条件式单通道部署](ADR-006-conditional-can0-deployment.md)，发送侧带宽需要复核的模型，以及 Decision 第 7 条的逐次授权闸门。
- [适配器契约 v1](../development/adapter_contract_v1.md)，「新增适配器检查表」第 7 条。
