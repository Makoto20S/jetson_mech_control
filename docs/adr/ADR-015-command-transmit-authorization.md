# ADR-015：命令发送授权与 pending 命令撤销

- **Decision ID:** ADR-015
- **Status:** Accepted
- **Date:** 2026-09-13
- **Owner:** 项目负责人（`ros2_control` integration owner）
- **Scope:** `RuntimePort` 与 `CompositeSystem` 之间的命令发送边界；不改变 `CanonicalCommand`/`CanonicalState` 字段布局、[ADR-012](ADR-012-command-watchdog-and-capability-honesty.md) 的分级看门狗语义或 [ADR-014](ADR-014-ak30-submode-command-interfaces.md) 的命令接口形状，也不解除 [ADR-006](ADR-006-conditional-can0-deployment.md) 的任何设备启用闸门

## Status rationale / 状态依据

本 ADR 在 `RuntimePort` 上新增一个纯虚函数并改变 `write()` 的入参类型，属于 `adapter_contract_v1.md`「新增适配器检查表」第 7 条所说的 canonical 契约变更，因此按该条要求**在实现之前**以 `Proposed` 提交（走 ADR-013/ADR-014 的「先提交、后实现」路径，而不是 ADR-012 的事后追认路径）。项目负责人于 2026-09-13 批准接口变更与决策条款，状态转为 `Accepted`。

批准范围仅限本文件列出的命令发送授权语义与 `RuntimePort` 接口形状。它**不**解除任何设备启用闸门：真实设备运行仍受 ADR-006 Decision 第 7 条逐次授权约束，且本 ADR 的验证全部在离线层级完成（见 Validation）。

## Context / 上下文

2026-09-12 的进度审计在仓库外用 `FakeTransport`/`FakeSerial` 离线复现了两个行为：在没有任何 resource claim 的情况下执行 `read → write → read` 会生成电机命令帧；在释放 claim 之后的 22 ms 内仍然发出 11 帧，且硬件组件没有进入 fault。这两条复现的是当前 `feat/ak30-bench-composition` 分支 `c7f359e` 的实际代码路径，不是历史推测。

根因有两条，都在本 ADR 覆盖的边界上：

1. `CompositeSystem::write()` 把整个 `commands_` 向量交给 `runtime_->write()`，其源码注释本身就写明「including unclaimed interfaces」。`claimed_` 只参与 `perform_command_mode_switch()` 的记账和回滚，从不参与发送判定。未 claim 的 joint 因此把默认构造的 `CanonicalCommand{0, 0, 0}` 当作一个合法目标下发——在位置接口上这正是 ADR-012 明令禁止的「命令移动到零位」。
2. `Ak30ForceControlRuntime::write()` 在每次被调用时置 `fresh_write_ = true`，其注释写明「every write (even an unchanged one) counts as a refresh」。于是 `controller_manager` 只要还在按周期调用硬件的 `write()`，旧目标就被无限续租，即使 controller 早已停止、被释放或故障。

第二条之所以不能就地修掉，是因为 ros2_control 的 command interface 是一个裸 `double*`：控制器直接写内存，硬件层无法观察 `set_value()` 被调用了几次。「manager 调用了 `write()`」在信息论上推不出「controller 刷新了目标」。因此发送资格必须绑定到硬件层**可以**观察到的东西——已激活的 command claim 与显式的撤销事件——而不是绑定到硬件循环本身。

[ADR-003](ADR-003-composite-system-interface.md) Decision 第 3 条已要求 `write()` 做 freshness 与 deadline 检查，第 4 条已要求 `on_deactivate` 「停止命令续租」。当前实现不符合这两条。本 ADR 不是新方向，而是把 ADR-003 的既有意图转成一个可测试、可证伪的显式契约。

## Decision / 决策

1. **claim 即发送授权，且逐 joint 独立。** 一个 joint 只有在 `perform_command_mode_switch()` 的 `start_interfaces` 中成功获得其命令接口之后，才具备把命令交给 `RuntimePort` 的资格。初始状态、`stop_interfaces`、`on_deactivate`、`on_cleanup`、`on_error` 一律撤销该资格。
2. **未授权的 joint 不下发命令。** `RuntimePort::write()` 的入参由 `const CanonicalCommand*` 改为 `const CommandDispatch*`，其中 `CommandDispatch = { CanonicalCommand command; bool authorized; }`。命令与其授权位成对传递，因为二者只有成对才有意义：单独一个 `CanonicalCommand` 无法区分「控制器命令了零」与「没人 claim 这个 joint，所以这是默认构造的占位值」。实现方必须把 `authorized == false` 解释为「该 joint 完全没有命令」，既不是零命令，也不构成保持既有命令存活的理由。当一个周期内没有任何 joint 获得授权时，`CompositeSystem` 不调用 `write()`，使「`controller_manager` 转了一圈」不以任何形式抵达设备层。
3. **`RuntimePort` 新增 `virtual void cancel_pending(std::size_t index) noexcept = 0;`。** 释放 claim、`on_deactivate`、`on_cleanup`、`on_error` 必须调用它，清除实现方为该资源持有的 pending 命令、新鲜标志和尚未发出的租约。撤销是立即生效的状态转换，不得依赖硬 TTL 自然过期来达成。逐资源而非全局：释放一个 joint 不得取消另一个 joint 上仍然有效的命令。被拒绝的 `perform_command_mode_switch()` 不得产生任何撤销——strict switch 是全有或全无的。
4. **停止控制器不得用「停止调用硬件 `write()`」来模拟。** 任何声称覆盖停止/释放/故障语义的测试，必须让 `controller_manager`（或等价的 fake manager 循环）继续按周期调用 `read()`/`write()`，并断言电机帧计数保持不变。仅停止硬件写的旧测试不构成该语义的证据。
5. **不得使用单一全局布尔表达授权。** 多 joint 部署中，一个 joint 获得授权不得使另一个无关 joint 具备发送资格。
6. **本 ADR 不改变的东西：** `CanonicalCommand`/`CanonicalState` 的三字段布局、ADR-014 的「每关节恰好一个命令接口」形状、ADR-012 的 Following/Holding/Expired 分级与 `<=3` 控制周期预算、以及状态接口的导出形状。

## Alternatives considered / 替代方案

### A. 用 NaN sentinel 表示「未授权」

在未授权 joint 的命令字段写入 NaN，由 runtime 识别后跳过。拒绝：与 `write()` 现有的 finite 校验直接冲突（该校验会把 NaN 判为 fault 并锁存），并且要求协议层证明 NaN 在任何路径上都不会抵达编码器。用一个哨兵值同时表达「故障」和「未授权」两种含义，是把可判定的状态退化成需要约定的数值。

### A2. 用并列的 `const bool* authorized` 数组作为第二个入参

拒绝：两个等长数组必须由调用方保持一致，一旦某条路径只更新其中一个，类型系统不会提示。`CommandDispatch` 把配对关系放进类型里，使「命令没有携带其授权」在结构上无法表达。

### B. 把授权判定完全放进 runtime 内部

拒绝：`RuntimePort` 按设计不知道 ros2_control 的 resource claim；claim 信息只存在于 `CompositeSystem`。把判定下沉会迫使每个适配器各自重新实现一遍 claim 语义，正是 `adapter_contract_v1.md` 第 1 条要避免的重复。

### C. 靠硬 TTL 自然过期代替显式撤销

拒绝：这正是已被复现的缺陷行为。释放 claim 之后仍存在最长一个硬 TTL 的发送窗口（实测 22 ms/11 帧），而「控制器已经停止」与「命令还剩几毫秒有效期」是两件必须可区分的事。

### D. `CompositeSystem` 持逐 joint 授权 + `RuntimePort::cancel_pending()`（选定）

授权判定留在唯一知道 claim 的那一层，撤销作为一个显式的、可在测试中观察的边界事件向下传递。代价是接口新增一个纯虚函数。

## Consequences / 后果

### Positive / 正面

- 「无 claim / 已释放 / 已停用 / 已故障时电机 TX 计数为 0」成为一个可以在离线 fake transport 上直接断言的命题，不再依赖「轴没有动」这类非证据。
- 控制器停止语义与硬件循环解耦：`controller_manager` 继续以 500 Hz 调用硬件不再改变结论。
- 多 joint 部署的授权边界与 ADR-002 的单写者模型对齐，不会因为一个 joint 的 claim 泄漏给整条总线。

### Negative / 负面与代价

- `RuntimePort` 新增纯虚函数，现有全部实现者（`LoopbackRuntime`、各测试 fake、`Ak30ForceControlRuntime`）必须同步实现，属于一次性的破坏性接口变更。
- `CompositeSystem::write()` 从「整块下发」变为「逐 joint 分派」，调用路径比现在复杂。
- 授权状态是 `CompositeSystem` 新增的持久状态，必须在全部生命周期跃迁中维护一致；遗漏任何一条撤销路径都会重新打开本 ADR 试图关闭的窗口，因此生命周期测试的覆盖要求随之提高。

## Validation / 验证

以下为 2026-09-13 在 x86_64 开发工作站上实际运行的结果。命令与原始日志保存在项目本地 `/tmp/mech-control-plan-20260913/`（不入 Git）。

已运行并通过：

- 失败先行（RED → GREEN）：`CompositeSystem.UnclaimedJointIsNeverHandedToTheRuntime` —— 单关节、零 claim、10 个 manager 周期。修复前测得 `write_calls() == 10`（每周期一次未授权下发），修复后为 `0`，且不 fault。
- `CompositeSystem.ReleasingClaimRevokesAuthorizationAndCancelsPending` —— claim 后一个周期产生恰一次下发；释放 claim 后记录到一次 `cancel_pending(0)`，随后 10 个周期下发计数保持不变且不 fault。
- `CompositeSystem.AuthorizationDoesNotLeakBetweenJoints` —— 两关节只 claim 其一，5 个周期后被 claim 的 joint 下发 5 次、未 claim 的 0 次，且未授权 joint 的命令值从未抵达 runtime。
- `CompositeSystem.DeactivateRevokesAllAuthorizationAndRequiresReclaim` —— `on_deactivate` 逐 joint 撤销并各记录一次撤销；重新 `on_activate` 但不重新 claim 时 10 个周期无下发；重新 claim 后恢复。
- `CompositeSystem.ErrorRevokesAuthorizationAndCancelsPending` —— `on_error` 撤销、锁存，后续 `write()` 返回 ERROR 而非下发。
- `CompositeSystem.RejectedSwitchLeavesAuthorizationAndPendingIntact` —— 被拒绝的 strict switch 不产生任何撤销，授权与 pending 命令保持原样。
- `Ak30RuntimeTest` 四项：未授权 dispatch 不产生 TX；`cancel_pending()` 后跨软/硬 TTL 的 10 个周期 TX 恒为 0；撤销后重新 claim 可恢复发送；越界 index 无副作用。
- start/stop 同周期、重复 claim、未知接口、双 start、未 claim 就 stop 的既有回滚语义与带 `/` 的关节名回归测试全部保持通过，无回退。
- `MECH_OUTPUT_ROOT=<tmp> MECH_SKIP_ROSDEP=1 bash tools/ci/build_workspace.sh`：**264 tests，0 errors / 0 failures / 0 skipped**（本变更前基线为 254 项全绿，新增 10 项）。
- `bash tools/ci/run_sanitizers.sh <tmp>`：264 tests 全绿，无 AddressSanitizer/UndefinedBehaviorSanitizer 报告。已按既有纪律核实 sanitizer 确实启用（CMake cache 中 `CMAKE_CXX_FLAGS` 含 `-fsanitize=address,undefined -fno-omit-frame-pointer`），而不是仅凭「grep 不到报告」。
- `python3 tools/ci/context_check.py`、`python3 tools/ci/check_adrs.py`、`git diff --check`：全部 PASS。

**未运行**：

- 真实 `controller_manager` + fake hardware 的集成测试（加载本组件、由一个只在前 N 个周期写命令的 controller 驱动、manager 继续循环）。原因：本轮的授权边界修复先在组件与 runtime 层级完成，该集成层作为独立后续任务执行。因此本 ADR 的 E1 级证据目前只到「单元测试 + fake runtime」层级，**不得**被表述为 `controller_manager` 生命周期已验收。
- ARM64（Jetson）重跑、vcan 回环、任何真实设备运行或运动。离线通过不构成实机验收。

## Review triggers / 重审触发

- ros2_control 未来提供可被硬件层观察的命令刷新计数或 generation 机制，使「controller 是否刷新」不再需要用 claim 代理；
- 引入 ADR-002 规划的多设备 `BusRuntime` 调度后，发送授权的归属层从 `CompositeSystem` 迁移；
- 出现一类设备要求「未 claim 时仍必须周期发送保持帧」，与 Decision 第 2 条直接冲突；
- 逐 joint 授权在实际多 joint 部署中被证明无法表达某种跨 joint 的原子切换需求。

## Sources / 来源

- [ADR-003：复合 `ros2_control` `SystemInterface`](ADR-003-composite-system-interface.md)，Decision 第 3、4 条。
- [ADR-012：命令看门狗与能力诚实上报](ADR-012-command-watchdog-and-capability-honesty.md)，分级看门狗与「位置命令不得解析为 0.0」。
- [ADR-014：AK3.0 子模式命令接口](ADR-014-ak30-submode-command-interfaces.md)，命令接口形状。
- [适配器契约 v1](../development/adapter_contract_v1.md)，「新增适配器检查表」第 1、7 条。
- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 9、10、13 节。
- 2026-09-12 进度审计的离线复现结果（未 claim 发帧、释放 claim 后继续发帧），报告与诊断源码保存在项目本地 `tmp/`，按用户约定不进入 Git，故此处不作链接。
