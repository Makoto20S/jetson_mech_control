# ADR-017：命令新鲜度的可选代号接口与两档保护

- **Decision ID:** ADR-017
- **Status:** Accepted
- **Date:** 2026-09-14
- **Owner:** 项目负责人（`ros2_control` integration owner）
- **Scope:** 每关节额外导出一个 `command_generation` 命令接口，**是否 claim 它由控制器自行决定**，硬件据此在两档保护之间选择；据此修订 [ADR-014](ADR-014-ak30-submode-command-interfaces.md) Decision 第 2 条（每关节恰好一个命令接口）。不改变状态接口形状，不改变 [ADR-015](ADR-015-command-transmit-authorization.md) 的 claim 即授权语义，不改变 [ADR-016](ADR-016-feedback-quality-fail-closed.md) 的反馈闸门，不解除 [ADR-006](ADR-006-conditional-can0-deployment.md) 的任何设备启用闸门

## Status rationale / 状态依据

本 ADR 改变 canonical 命令契约的形状（每关节导出的命令接口数量与语义），属于 [适配器契约 v1](../development/adapter_contract_v1.md)「新增适配器检查表」第 7 条所说的 canonical 契约变更，因此在实现之前以 `Proposed` 提交。

它同时是一周之内对 `RuntimePort` 相邻契约的**第三次**破坏性变更（ADR-015 改 `write()` 签名并新增 `cancel_pending()`；ADR-016 新增 `has_valid_sample()`）。这一点单独记录在「后果」中，因为它影响的不只是本仓库。

**本 ADR 的第一版曾把代号接口写成强制的**，即不 claim 该接口的控制器一律无法命令电机。该版本于 2026-09-14 被项目负责人否决，理由是它废掉了本项目的一条立项需求：标准 `ros2_control` 控制器必须能自由切换、直接驱动本硬件。否决是正确的，且 [`02_architecture_and_interfaces.md`](../planning/02_architecture_and_interfaces.md) 早已写明正解是**两档**而非强制（见下文 Context）。强制版本作为替代方案 I 保留，以免重提。

**2026-09-14 转 Accepted。** 项目负责人在两档设计的实质被逐条复述后批准，复述内容包括：代号接口始终导出而是否 claim 由控制器决定、保护等级由 claim 内容观测得出、弱档缺口有意保留（第三方控制器挂在 velocity 接口上静默时电机会保持最后速度继续转），以及两条 DISABLED 出口测试的判据必须由「硬件不能被任意控制器糊弄」弱化为「不能被一个 claim 了代号接口的控制器糊弄」。批准范围为命令接口形状与两档语义，**不解除任何设备启用闸门**。日期保留为提交日（与 ADR-012 的惯例一致：提交日不变，转换记在状态依据里）。

## Context / 上下文

### 当前缺陷（已量化，不是推断）

控制器保持 claim 但停止写入目标时，`CompositeSystem::write()` 每个周期仍然派发一条 authorized 命令，`Ak30ForceControlRuntime::write()` 每次都置 `fresh_write_`。**实测：20 个静默周期产生 20 个电机命令帧。**

该缺陷已经以 `DISABLED_SilentControllerDoesNotRefreshItsCommand` 与 `DISABLED_ReactivationDoesNotReplayTheOldTarget` 两条测试提交在仓库里。

### 根因：命令接口是一块没有笔迹的白板

一个 `ros2_control` 命令接口在导出后就是一个裸 `double*`。硬件层可以读到**上面写着什么**，但无法观测**是否有人写过**：`set_value()` 不留下任何痕迹。

ADR-015 之所以能把「控制器崩溃/被停用」这条路关掉，正是因为那一半**是可观测的**——`controller_manager` 会明确宣告 claim 与 release，硬件层收得到这两个事件。新鲜度这一半没有对应的事件。

### 约束：自由切换标准控制器是立项需求，不是可选项

[`02_architecture_and_interfaces.md`](../planning/02_architecture_and_interfaces.md) 已经记录了本问题与其解法的形状：

> 标准 ros2_control double command interface 本身没有序号。项目自研控制器需要共享一个定长命令元数据契约（generation、valid、producer sequence）；对不能提供该契约的第三方控制器，只能使用较弱的 manager-loop heartbeat 模式并在风险清单中标明。

即**两档**：强契约给能提供它的控制器，弱模式给不能提供的，缺口写进风险清单。任何把强契约做成硬性前置的方案都与该记录冲突，也与项目「供应商解耦、控制器可自由切换」的目标冲突。

### 同源的第二个症状

停用后重新激活会重放停用前的目标，因为撤销授权时**刻意保留**了最后一个命令值。把它置零会更糟：在位置接口上，一个被替换进去的 `0.0` 是一条「运动到标定零位」的命令，这正是 [ADR-012](ADR-012-command-watchdog-and-capability-honesty.md) Decision 第 3 条禁止的事。

### 已取得的前置证据（2026-09-14）

方案要求同一关节导出两个命令接口并能被同一控制器一并 claim。该前提已用一次性探针在**真实 `ControllerManager`**（`update_rate` 500）下验证：

- STRICT `switch_controller` 返回 **OK**，控制器最终持有 **2** 个命令接口，两个接口写入的值都到达硬件侧；
- `prepare_command_mode_switch()` 与 `perform_command_mode_switch()` **各只被调用一次**，`start_interfaces` 在**同一次调用里一次带齐两个名字**（`joint_1/position`、`joint_1/command_generation`），二者指向同一关节。

**这条证据的边界必须同时记录**：探针使用的是一个最小的假 `SystemInterface`，**不是** `CompositeSystem`，因此它证明的是上游框架允许这种形状，**不**证明本仓库的实现会通过；它只覆盖单关节、单控制器，未覆盖两个控制器争抢同一关节，也未覆盖与 ADR-016 claim 闸门的时序交互。**它同样没有覆盖本 ADR 依赖的另一半：一个只 claim 运动接口、不 claim 代号接口的控制器能否正常激活**——这一项必须在实现阶段验证。探针跑完即删，不作为回归测试保留：它测的是框架而不是本项目的代码。

已知会挡住实现的两处（读代码得出）：`CompositeSystem::validate_info()` 要求 `command_interfaces.size() == 1U`；`known_command_interface()` 只承认 `<joint>/<该关节唯一的命令接口名>`，会使 `validate_switch()` 拒绝代号接口名。

## Decision / 决策

1. **每关节始终导出一个名为 `command_generation` 的命令接口。** 它与运动命令接口（`position`/`velocity`/`effort` 三者之一）并列，属于同一关节。**始终导出、是否 claim 由控制器决定**——把「导出」和「使用」分开，是本 ADR 与其被否决的第一版之间唯一但决定性的差别。

2. **保护等级由 claim 的内容决定，而不是由部署配置决定。** 硬件在 `perform_command_mode_switch()` 时已经拿到完整的 `start_interfaces` 列表（探针实测：两个名字在同一次调用里一起到达），因此它能够观测到该关节进入的是哪一档，无需任何额外配置项，也无需信任任何声明。

3. **强档（控制器 claim 了代号接口）：**
   1. 控制器每写入一次真实目标，必须改变该关节的代号值。
   2. 硬件只在代号**发生变化**时才认为收到新命令。判据是与基线**不相等**（`!=`），不是「大于」——使用不等而非有序比较，是为了让控制器重启后从任意值开始都能被识别为变化，而不要求全局单调。
   3. 代号未变化即视为没有新命令，硬件**不得发送**。
   4. 撤销授权时把基线重置为命令缓冲区的当前值，使重新激活之后在控制器写入新代号之前不会重放旧目标。首次取得 claim 时同样以缓冲区当时的值作为基线。
   5. 非有限的代号值一律拒绝，且不得被当作变化。

4. **弱档（控制器只 claim 了运动接口）：维持当前行为，并把缺口明确登记为已接受的风险。** 该关节按 manager 循环节奏继续派发命令，硬件无法区分「控制器刷新了」与「manager 转了一圈」。**这一档的缺口是本 ADR 有意不关闭的**，因为关闭它的唯一已知手段就是强制代号接口，而那与立项需求冲突。

5. **只 claim 代号接口而不 claim 运动接口的 switch 必须被拒绝。** 代号接口没有独立意义，它只修饰同关节的运动命令。

6. **代号是计数器语义，不是数值语义。** 一个合法地保持稳定设定点的控制器会反复写入同一个目标值，因此**对运动命令值本身做变化检测是不成立的**——它无法区分「稳住不动」与「已经死了」。代号是一个独立于目标值的、专门用来表达「我又说了一次」的量。

7. **不使用时间戳。** 时间戳会引入第二个由控制器控制的时钟域，而本项目已经因为 [ADR-005](ADR-005-monotonic-time-freshness.md) 和 T4 的实测（看门狗读 `rclcpp::Time`，冻结 sim time 下陈旧目标永不失效）确立了不得依赖可被部署冻结的时钟。

8. **修订 ADR-014 Decision 第 2 条。** 「每关节恰好一个命令接口」改为「每关节恰好一个**运动**命令接口（`position`/`velocity`/`effort` 三者之一，与 `sub_mode` fail-closed 匹配），**外加一个始终导出、可选 claim 的 `command_generation` 接口**」。ADR-014 关于子模式对应关系、Velocity 强制 `effort = 0`、状态接口形状不变的其余决策一律不变。

9. **本项目自己的控制器一律走强档。** `DemoController` 以及 T8/T9 的速度与力矩控制器必须 claim 并维护代号接口。弱档存在的目的是接纳第三方控制器，不是给自研控制器留后门。

10. **明确记录本机制观测不到的东西。** 强档下硬件能观测的是代号的**变化**，不是写入动作本身；一个反复写入同一个代号值的控制器与一个停止写入的控制器在硬件层完全等价。这是设计边界而非缺陷：本 ADR 的威胁模型是「控制器安静地停止工作」，不是「控制器蓄意伪装」。

## Alternatives considered / 替代方案

### A. 维持现状，继续把新鲜度缺口开着

拒绝。缺口的后果按接口类型分级：position 接口上「保持最后一个目标」约等于 hold，有人在场时可接受；但 **velocity 接口上等于电机一直转，effort 接口上等于一直施力**。T8/T9 正是引入这两种控制器的地方。

### B. 给控制器加目标有效期（TTL）

拒绝。两条出口测试驱动的是 `WriterController`——一个不受我们控制的控制器形状。给自研控制器加 TTL 不会让它们转绿。

### C. 用时间戳接口代替计数器

拒绝。见 Decision 第 7 条。

### D. 不新增接口，由硬件层检测运动命令值的变化

拒绝。见 Decision 第 6 条：合法地保持稳定设定点的控制器会被误判为已死，而 hold 恰恰是本项目当前唯一验证过的动作。

### E. 用 metadata 或非命令接口承载代号

拒绝。`ros2_control` 的 metadata 不经过 claim，也没有每控制周期的写入路径；任何控制器都能改或不改它，且它不在 `perform_command_mode_switch` 的授权链上。

### F. 由硬件层统计 `write()` 被调用的次数

拒绝。`controller_manager` 每个控制周期都会调用 `write()`，这正是那个不携带任何信息的量。

### G. 哨兵值：硬件消费命令后往缓冲区写入一个哨兵，下周期被覆盖即证明有人写过

**不拒绝，但本轮不做，记为后续升级项。** 它的吸引力在于**对控制器完全透明**：任何每周期写命令的控制器都自动受保护，包括第三方，从而可以把弱档整体替换掉。未在本轮采用的原因是它有一个未验证的风险——部分控制器会**读取自己的命令接口**（例如激活时以其当前值作为初值），读到哨兵会取得一个无意义的数。在确认常见控制器读取命令接口的时机之前，不能用它替换弱档。若该风险被证明可控，应当重审本 ADR 的第 4 条。

### H. 等待 `ros2_control` 上游提供原生的「命令是否被写入」语义

**不拒绝，但不能等。** 若上游将来提供该语义，本 ADR 的机制应当让位于它。当前 Humble 没有该语义，而缺口的截止点是 T8/T9。

### I. 强制代号接口：不 claim 代号即不能命令电机（本 ADR 第一版，已否决）

**已于 2026-09-14 被项目负责人否决。** 它确实能对所有控制器关闭缺口，但代价是 `joint_trajectory_controller` 等标准 `ros2_control` 控制器无法直接驱动本硬件，即本项目不再是一个通用 `ros2_control` 目标。「控制器可自由切换」是本项目的立项目标之一，且 `02_architecture_and_interfaces.md` 早已把正解记为两档。记录在此以免重提。

## Consequences / 后果

### Positive / 正面

- 对 claim 了代号接口的控制器，「沉默的控制器」与「重新激活重放旧目标」两个同源症状由同一个机制一并关闭。
- 保护落在硬件层，对该档控制器而言不依赖控制器实现者的自觉——只依赖一件可被硬件观测的事：代号变了没有。
- **标准 `ros2_control` 控制器不受影响**，未被 claim 的命令接口在 `ros2_control` 中是完全正常的情形。
- 强档下电机命令帧的发送频率不再等于控制周期，而等于控制器**真实的刷新频率**，稳态下降低总线负载。

### Negative / 负面与代价

- **弱档的缺口是有意留下的。** 一个第三方控制器挂在 velocity 接口上、保持 active 但不再写目标时，**电机会保持最后一个速度继续转**。这是接纳任意控制器的代价，必须进入风险清单，并在任何使用第三方控制器的台架运行前复述。
- **两条 DISABLED 出口测试的判据必须改写。** 它们当前驱动的 `WriterController` 只 claim 运动接口，即落在弱档，因此**在本 ADR 下它们不会转绿**。原判据「硬件不能被任意控制器糊弄」在两档设计下**不再成立且不可能成立**——它与「任意控制器都能驱动本硬件」是同一件事的两面。新判据必须写成：硬件不能被**一个 claim 了代号接口的**控制器糊弄。这是一次真实的判据弱化，不得以「测试转绿了」掩盖。
- 这是 `RuntimePort` 相邻契约一周内的第三次破坏性变更。本仓库内的三个实现者会同步更新；**任何本仓库之外的实现者都会直接编译失败**。
- 强档下发送频率随控制器刷新率变化，任何按「每控制周期一帧」推导的发送侧带宽估算需要按 ADR-006 的模型复核。（接收侧的 `kReceiveBudget` 由设备回报速率决定，不受本 ADR 影响。）
- 硬件需要同时维护两条命令路径，测试面翻倍：每个行为都要在强档与弱档各验一次。
- 改动面：`CompositeSystem` 与 `RuntimePort`、仓库内三个 `RuntimePort` 实现者、`WriterController` 与 `DemoController`、三个 motor1 xacro、以及部署结构测试。
- ADR-014 Decision 第 2 条被修订，此前所有引用「每关节恰好一个命令接口」的表述都需要同步。

## Validation / 验证

本 ADR 提交时以下验证尚未运行，将在实现阶段补齐并逐项报告：

- **强档出口判据**：`DISABLED_SilentControllerDoesNotRefreshItsCommand` 与 `DISABLED_ReactivationDoesNotReplayTheOldTarget` 改写为由一个 claim 了代号接口的控制器驱动，然后启用并转绿。改写本身必须留下记录，说明判据的弱化范围（见「后果」）。
- **弱档行为被钉死而不是被遗忘**：一条测试断言只 claim 运动接口的控制器仍然能够正常激活并发送命令，另一条断言它在静默时**仍会**持续发帧——把已接受的风险变成一条会在行为改变时变红的测试，而不是一句文档。
- 在真实 `controller_manager` 下验证 `CompositeSystem` 导出双命令接口后：同时 claim 两者可以激活；只 claim 运动接口也可以激活；只 claim 代号接口被拒绝。上文探针只覆盖了第一种，且用的是假 `SystemInterface`。
- 强档下代号未变化时不产生电机命令帧；代号变化时产生且仅产生一帧。断言对象是 `FakeTransport` 的真实 AK3.0 帧计数，不是某个 C++ 方法是否被调用。
- 强档下撤销后重新激活，在控制器写入新代号之前不得产生任何命令帧。
- 非有限代号值被拒绝且不被计为变化。
- 两个控制器先后 claim 同一关节时基线的行为（探针未覆盖）。
- `configure()`/`on_init()` 对新接口形状的 fail-closed 校验：xacro 声明与导出形状不一致时拒绝，且拒绝发生在取得串口之前（与 T3 建立的顺序一致）。
- 完整 `tools/ci/build_workspace.sh` 与 `tools/ci/run_sanitizers.sh` 全绿，并按既有纪律核实 sanitizer 确实启用（读 CMake cache，不是只看脚本名）。

离线验证通过不构成实机验收；设备运行仍受 ADR-006 Decision 第 7 条逐次授权约束。

## Review triggers / 重审触发

- 替代方案 G（哨兵值）的读取时机风险被证明可控，应当重审 Decision 第 4 条——它可以把弱档整体替换掉，从而对所有控制器关闭缺口；
- `ros2_control` 上游出现原生的「命令是否被写入」语义（替代方案 H），本机制应当让位；
- 实际部署中出现第三方控制器驱动 velocity 或 effort 接口，需要在该次运行前复述弱档缺口的物理后果，并考虑是否临时禁止该组合；
- 两个控制器争抢同一关节的实测行为与探针结论不符；
- 多关节或多设备部署下，逐关节一个代号接口的数量增长成为问题；
- 若将来证明存在合法的控制器形状，其刷新频率低到使强档的发送频率不足以满足设备失控保护（motor1 `timeout_msec = 1000`），需要重新引入一条与代号无关的保底发送路径。

## Sources / 来源

- [架构与接口](../planning/02_architecture_and_interfaces.md)，命令元数据契约与第三方控制器降级模式的原始记录——本 ADR 的两档设计出自该处。
- [ADR-014：AK3.0 子模式命令接口](ADR-014-ak30-submode-command-interfaces.md)，Decision 第 2 条由本 ADR 修订。
- [ADR-015：命令发送授权与 pending 命令撤销](ADR-015-command-transmit-authorization.md)，claim 即发送授权；本 ADR 补上它无法覆盖的那一半。
- [ADR-016：反馈质量诚实上报与失效关闭策略](ADR-016-feedback-quality-fail-closed.md)，把保护放在唯一共享层的先例，以及其替代方案 D 的权衡。
- [ADR-012：命令看门狗与能力诚实上报](ADR-012-command-watchdog-and-capability-honesty.md)，Decision 第 3 条（失效不得解析为 `0.0`），撤销时不得置零的依据。
- [ADR-005：单调时间与新鲜度](ADR-005-monotonic-time-freshness.md)，不得依赖可被部署冻结的时钟。
- [ADR-003：复合 `ros2_control` `SystemInterface`](ADR-003-composite-system-interface.md)，命令接口形状与 `CompositeSystem` 的职责边界。
- [ADR-006：条件式单通道部署](ADR-006-conditional-can0-deployment.md)，发送侧带宽需要复核的模型，以及 Decision 第 7 条的逐次授权闸门。
- [适配器契约 v1](../development/adapter_contract_v1.md)，「新增适配器检查表」第 7 条。
