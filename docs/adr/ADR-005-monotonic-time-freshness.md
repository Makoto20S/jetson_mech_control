# ADR-005：用单调时钟管理 freshness/TTL，并保留源时间

- **Decision ID:** ADR-005
- **Status:** Accepted
- **Date:** 2026-08-07
- **Owner:** 项目负责人（Foundation core/time owner）
- **Scope:** canonical state/command、device session、BusRuntime、`SystemInterface` 和 controller 的时间语义

## Status rationale / 状态依据

单调时钟用于本机 elapsed-time 判定、源时间不得被重复读取覆盖，是独立于设备品牌的基础语义。接受该约束不表示任何设备已经提供可靠采样时钟或完成跨时钟同步。

## Context / 上下文

设备采样时刻、内核接收时刻、主机单调到达时刻、controller 周期时刻和 ROS/墙钟各有不同语义。用 `now()` 覆盖旧快照会把 100 Hz 传感器伪装成 500 Hz/1 kHz 新状态；用墙钟计算 TTL 又会受 NTP 或人工改时影响。部分设备没有源时间，部分设备只有计数器，不能补造不存在的采样精度。

## Decision / 决策

1. freshness、timeout、command deadline/TTL 和控制周期 elapsed time 统一使用主机单调时钟域；墙钟/ROS time 不参与安全关键过期判定。
2. canonical state 按信号组保存 `value`、`valid`、`quality`、`source_time`（若有）、`kernel_rx_time`（若有）、`host_rx_mono`、`age`、`sequence/generation` 和源时间有效性/时钟域。
3. 重复 `read()` 旧快照时保持源时间和接收时间不变，只按当前单调时钟递增 `age`；不得把时间戳刷新为本周期 `now()`。
4. canonical command 保存 producer generation/sequence、提交单调时间、deadline、mode 和 limits result；controller 停止刷新后，旧命令必须在 TTL 到期后失效。
5. 若设备提供可验证的时钟，使用显式 offset/drift/uncertainty 映射；若不提供，只记录主机到达时间并将 `sample_time_valid=false`，不得伪造设备采样时刻。

## Alternatives considered / 替代方案

### A. 所有时间统一使用 ROS `now()` 或墙钟

接口简单，但时钟跳变会破坏 timeout，且无法区分采样与发布时间。拒绝。

### B. 每个控制周期刷新传感器时间戳

表面上得到高频数据，实际上掩盖 stale 状态并破坏延迟分析。明确禁止。

### C. 多时间字段 + 单调过期（选定）

数据结构更丰富，但可以诚实表达来源、不确定性和不同速率。

## Consequences / 后果

### Positive / 正面

- virtual clock 可以无 sleep 地测试 stale、deadline、TTL 和时钟边界。
- 500 Hz/1 kHz controller 可以安全重复读取 100/200 Hz 快照而不伪造新样本。
- 后续设备时钟、内核 timestamp 和 ROS stamp 可以在明确质量/不确定度下共存。

### Negative / 负面与代价

- canonical snapshot 和诊断需要保存多个时间字段与时钟域元数据。
- 跨设备对齐不能仅靠到达时间，缺乏公共序号/设备时钟时只能标记不相干或不确定。
- monotonic time 不提供绝对 UTC；记录系统必须另外保存受控映射。

## Validation / 验证

- `python3 tools/ci/check_adrs.py` 通过，并确认 ADR-005 的 planning 反向链接有效。
- FND-005/FND-007 virtual-clock 单元测试覆盖零/负/溢出边界、时钟推进、重复读取、source-time invalid 和 TTL 精确边界，测试不得依赖 wall-clock sleep。
- FND-009 测试证明 controller 停止刷新后命令按 deadline 失效；drop/duplicate/reorder 不会刷新旧样本时间。
- FND-012/014 的 `read()` 和 controller switch 测试证明 age 递增、实际 `dt` 被使用，ROS stamp 不覆盖 source/host monotonic evidence。

## Review triggers / 重审触发

- 目标平台提供经验证的 PTP/硬件时间戳，需要把硬件时钟提升为明确的主要时间域；
- ROS 发行版或 controller_manager 改变 update time/duration 契约；
- 设备提供公共采样序号/同步时钟，可提高 coherent sample 语义；
- 实测发现当前字段不足以定位 deadline、仲裁或跨传感器对齐问题。

## Sources / 来源

- [架构与接口设计](../planning/02_architecture_and_interfaces.md)，第 5、8、10、12 节。
- [Foundation 核心契约](../planning/07_framework_bootstrap_plan.md)，第 9.3、9.4、11 节。
- [已确认决策与待确认项](../planning/05_decisions_and_open_questions.md)，第 3.5 节。
- [FND-004 ADR index](README.md)。
