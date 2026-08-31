# ADR-012：命令看门狗分级语义、transport 能力三态上报与远程帧表达

- **Decision ID:** ADR-012
- **Status:** Accepted
- **Date:** 2026-08-27
- **Accepted:** 2026-08-31（项目负责人复核后批准）
- **Owner:** 项目负责人（Foundation core/controller owner）
- **Scope:** `TargetLimiter`/controller 命令看门狗、`TransportCapabilities` 能力上报、`RawCanFrame` 远程帧字段

## Status rationale / 状态依据

本 ADR 记录的三项契约变更是在 Foundation RC（`v0.1.0-foundation-rc1`，commit `a056492`）代码评审中发现缺陷后实施的。按 [适配器契约](../development/adapter_contract_v1.md) 第 40 行第 7 条，canonical 契约变更本应先出 ADR 再实现；本轮顺序颠倒，因此本文件是**追认记录**。实现随 PR #4 合并为 `f3012ee`，ADR 于 2026-08-31 经项目负责人复核后转 `Accepted`。

**流程违规的事实不因批准而消失，保留在此备查。** 批准的是这三项契约本身，不是"先实现后补 ADR"这个做法；后续 canonical 契约变更仍须先出 ADR。

**批准范围仅限接口语义。** ADR-012 不授权、也从不曾授权任何真实设备启用：CAN 使能、电机命令和物理通道激活仍然只受 [ADR-006](ADR-006-conditional-can0-deployment.md) 与 G0–G3 证据闸门约束，与本 ADR 的状态无关。同样地，本 ADR 中标注为"未验证"的 SocketCAN netlink 位速率读取、`SO_RCVBUF` 回读、RTR 线上往返和错误帧解码，状态转为 `Accepted` 后**仍然未验证**——批准的是契约形状，不是这些代码路径在真实硬件上的正确性。

## Context / 上下文

RC 评审复现了三类问题：

1. **位置命令的失效动作不安全。** `TargetLimiter` 在 TTL 过期和输入非法两种情况下都把位置命令解算为 `0.0`。对位置接口而言 `0.0` 不是中性值，而是「以限速运动到零位」这一主动动作，恰好发生在上游失联的时刻。
2. **能力上报含虚构成分。** `TransportCapabilities::is_valid()` 要求 `nominal_bitrate_hz` 非零，但 SocketCAN 从不读取接口实际位速率，vcan 根本没有位速率，7 路 USB-CDC 盒子的位速率由固件固定且厂商未公开（见 [待确认项 OQ-08](../planning/05_decisions_and_open_questions.md)）。该规则的实际效果是**强迫操作者编造一个数字**，与 [ADR-006](ADR-006-conditional-can0-deployment.md) 「不支持或未知的能力必须失败关闭，不得伪造」直接冲突。`queue_capacity` 同样只是声明值，从不设置 `SO_RCVBUF`。
3. **远程帧被误解码为数据帧。** `CAN_RTR_FLAG` 在 ID 掩码中被丢弃，远程帧被当作携带 `can_dlc` 字节有效载荷的普通数据帧交付，而那些字节是接收缓冲区里的残留内容。

## Decision / 决策

1. **命令看门狗采用三段语义。** 设软 TTL `ttl_nanoseconds` 与硬 TTL `hard_ttl_nanoseconds`，且 `hard_ttl > ttl > 0` 在 `configure()` 时强制校验：
   - `t < ttl`：跟随已提交目标；
   - `ttl <= t < hard_ttl`：**冻结在最后一个有效命令**并报告降级，不产生任何新运动；
   - `t >= hard_ttl`：`update()` 返回 `ERROR`，进入显式失败。
2. **看门狗总预算受既有指标约束。** 软 TTL 与 hold 阶段之和必须落在 [MVP 交付计划](../planning/03_mvp_delivery_plan.md) 规定的 `<=3` 个控制周期（500 Hz 下 `<=6 ms`）内。默认值取 `ttl=4 ms`、`hard_ttl=6 ms`。
3. **位置类命令在任何阶段都不得被静默替换为 `0.0`。** 输入非法时返回保持值而非零；保持值由调用方第一个有限输入播种，因此 `on_activate()` 之后立即进入保持，含义是「保持由状态接口播种的当前命令」，而不是跳到零位。
4. **能力上报采用三态。** 每个可测量的量配一个 `*_verified` 标志：
   - `verified == true`：本进程从真实通道读回（netlink、ioctl、getsockopt 等）；
   - `verified == false` 且值非零：操作者声明，未经校验；
   - `verified == false` 且值为零：**确实未知，且明确不作声明**——这是合法状态。
   `is_valid()` 因此不再要求非零位速率，只拒绝自相矛盾的「已验证却为零」。
5. **`RawCanFrame` 显式表达远程帧。** 新增 `remote_request`，仅限 Classic CAN，不与 BRS 共存，`create()` 对远程帧清零 payload（远程帧在总线上不携带数据），`payload_size` 保留请求的 DLC。`TransportCapabilities::supports_remote_frames` 声明后端能否表达它；不能表达的后端必须拒绝而非静默降级为数据帧。

## Alternatives considered / 替代方案

### A. 保留「TTL 到期回零」

实现最简单，且对速度/力矩接口确实安全。但对位置接口语义相反，且 RC 评审中该行为发生在无任何故障上报的情况下。拒绝。

### B. TTL 到期立即返回 `ERROR`，不设 hold 阶段

语义最干净，与 [ADR-005](ADR-005-monotonic-time-freshness.md) 第 4 条字面一致。代价是瞬时抖动会频繁停用控制器。在获得真实设备抖动分布证据前，保留一个有界 hold 阶段更稳妥；该阶段有硬上限，不会退化为无限保持。

### C. 三段看门狗（选定）

多一个配置参数和一个状态枚举，但把「上游短暂抖动」与「上游真正失联」区分开，且总预算仍受既有 `<=3` 周期指标约束。

### D. 位速率保持必填，由操作者声明

保持 `is_valid()` 不变，改为在文档中提醒该值未经校验。拒绝：文档提醒不会阻止 `capabilities()` 的消费者把它当作事实，而 ADR-006 要求的是结构性的失败关闭。

## Consequences / 后果

### Positive / 正面

- 位置命令在上游失联时不再产生运动，且失败是显式的而非静默的。
- 能力消费者可以区分「实测值」「声明值」「未知」，OQ-08 这类未解问题不再被一个编造的数字掩盖。
- 远程帧不再伪装成数据帧，避免上层把缓冲区残留当作有效载荷解码。

### Negative / 负面与代价

- 多一个 `hard_ttl_nanoseconds` 配置项，配置错误（`hard_ttl <= ttl`）会在 `configure()` 阶段失败。
- hold 阶段期间下游仍会收到命令，消费者必须查询看门狗阶段才能区分「跟随中」与「已冻结」；本 ADR 尚未把该阶段映射到 `SampleQuality::Degraded`/`StatusSnapshot`。
- `verified` 标志新增两个字段，所有 `TransportCapabilities` 构造点必须显式赋值。
- 远程帧字段扩大了 `RawCanFrame`，而 [Foundation 核心契约](../planning/07_framework_bootstrap_plan.md) 第 9.1 节原先未列举 RTR。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 与 `python3 tools/ci/context_check.py` 通过。
- 五包 clean build/test 通过；ASan/UBSan 无报告。
- 看门狗：集成测试逐周期断言「跟随 → 冻结（不朝零移动）→ 第 3 周期失败」，直接编码 `<=3` 控制周期预算；`configure()` 拒绝 `hard_ttl <= ttl`。
- 能力：未声明位速率被判为未知而非非法；「已验证却为零」被拒绝；构造期 `verified` 均为 `false`，仅 `open()` 实测后置真。
- 远程帧：帧层往返与 RTR+FD 拒绝有单元测试覆盖。
- **未验证：** SocketCAN 的 netlink 位速率读取、`SO_RCVBUF` 回读、RTR 线上往返和软件 `frame_type` 过滤只有 `MECH_RUN_VCAN_TESTS` 门控测试覆盖，本轮没有真实或虚拟 CAN 接口被创建或启用。错误帧解码路径即使有 vcan 也无法端到端测试，因为 CAN_RAW 拒绝发送带 `CAN_ERR_FLAG` 的帧。

## Review triggers / 重审触发

- 获得真实电机/上游的抖动与延迟分布证据，可据此重新选择 `ttl`/`hard_ttl` 默认值或取消 hold 阶段。当前默认值 4 ms / 6 ms 只由 `03_mvp_delivery_plan.md` 的 `<=3` 控制周期预算推得，没有任何实测抖动数据支持；
- 决定把看门狗阶段映射到 `SampleQuality`/`StatusSnapshot` 并统一到 core 的 `CommandSlot`，届时 controller 层的独立实现应当撤销；
- 厂商给出 7 路盒子的位速率书面答复（OQ-08），或 netlink 读取在目标硬件上被实测验证；
- L02/HI12 profile 需要使用远程帧，届时 [ADR-004](ADR-004-fixed-protocol-profile.md) 必须同步说明。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 8、12 节故障表。
- [MVP 交付计划](../planning/03_mvp_delivery_plan.md)，命令 watchdog 指标行。
- [Foundation 核心契约](../planning/07_framework_bootstrap_plan.md)，第 9.1、9.3 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，OQ-08。
- [适配器契约 v1](../development/adapter_contract_v1.md)，第 7 条。
- [ADR-005](ADR-005-monotonic-time-freshness.md)、[ADR-006](ADR-006-conditional-can0-deployment.md)、[ADR-004](ADR-004-fixed-protocol-profile.md)。
- [FND-004 ADR index](README.md)。
