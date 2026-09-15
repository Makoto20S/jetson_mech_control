# ADR-018：上游目标有效期与硬件命令租约分离

- **Decision ID:** ADR-018
- **Status:** Accepted
- **Date:** 2026-09-15
- **Owner:** Project owner
- **Scope:** PositionCommandController target policy and deployment configuration; hardware lease unchanged

## Status rationale / 状态依据

On 2026-09-15, the project owner authorized separating upstream target lifetime
from the unchanged 4/6 ms hardware lease and delegated implementation choices.
Within that scope, the primary implementation selected the provisional 100/106 ms
target policy recorded here. This decision establishes neither a measured
real-time bound nor authorization for device operation.

## Context / 上下文

The architecture specifies 20-50 Hz upstream targets and a 500 Hz control loop.
A 6 ms target lifetime cannot bridge even a nominal 20 ms producer interval.
ADR-012 applied the three-control-cycle watchdog budget to both boundaries;
the MVP requirement concerns loss of controller-to-hardware refresh.
The owner approved resolving these boundaries and preserving the hardware
lease on 2026-09-15. This ADR records that authorized policy before implementation.

## Decision / 决策

1. PositionCommandController uses monotonic callback arrival for upstream target aging.
   Its explicit target soft/hard lifetime is 100/106 ms for the initial 20-50 Hz
   deployment. The 100 ms value is an engineering policy of two slowest nominal
   periods, not a measured worst-case guarantee. The 6 ms hold tail is aligned
   with the independent hardware expiry budget.
2. Preserve the hardware command soft/hard lease at 4/6 ms and its existing
   configuration ceiling. While a target is Following, the controller generates
   fresh commands at the control rate. At target soft expiry it holds its last
   finite command and stops advancing command_generation; at hard expiry it
   returns ERROR. The strong-tier hardware independently expires within its
   lease from the last observed refresh. Scheduling delays are not hidden.
3. Use distinct target_ttl_nanoseconds and target_hard_ttl_nanoseconds ROS
   parameter names. Reject explicitly configured legacy names instead of
   silently ignoring them. Keep hardware command parameter names unchanged.
4. Preserve no-zero substitution, arrival aging, inactive-target rejection,
   activation epoch protection and fresh-target requirement after reactivation.
   Serialize non-RT producers when publishing target/generation metadata;
   the RT update path must not acquire their blocking mutex.
5. Standard ros2_control controllers remain directly usable through ADR-017's
   weak tier. This target policy does not detect their upstream silence or
   strengthen that tier's accepted guarantees.

## Alternatives considered / 替代方案

- Retain 4/6 ms target lifetime: incompatible with intended producer cadence.
- Increase hardware lease to target cadence: violates the three-cycle budget.
- Use test-only 100/150 ms in production: hardware expires after generation
  stops near soft expiry, so the additional 50 ms does not govern the chain.

## Consequences / 后果

### Positive / 正面

The intended 20-50 Hz producer can refresh a target without widening the
controller-to-hardware lease, and distinct parameter names make the two safety
boundaries reviewable in deployment files.

### Negative / 负面与代价

Upstream loss may permit interpolation for up to the 100 ms target lifetime;
this is distinct from detecting a stalled controller within its 6 ms hardware
lease. Neither implies a physical stop deadline. Existing deployments using
the old controller parameter names must migrate explicitly. No device operation
is authorized by this decision; G0-G3 and per-run authorization still apply.

## Validation / 验证

Require deterministic 20 Hz refresh, exact 100/106 ms boundaries, delayed
consumption, producer concurrency and lifecycle regressions; process acceptance
must sustain 20 Hz and prove silence after upstream loss. Deployment tests must
pin the independent hardware ceiling. Record actual ARM64 no-device upstream
timing observations separately; finite nominal/stress samples characterize
only that host and run and do not establish a hard real-time bound.

## Review triggers / 重审触发

Revisit the target lifetime when the actual application producer/workload is
available, observed arrival or consumption delays approach the policy budget,
or a deployment requires a shorter upstream-loss response. Do not substitute
motor feedback rate measurements for target producer timing evidence.

## Sources / 来源

- [ADR-012](ADR-012-command-watchdog-and-capability-honesty.md)
- [ADR-017](ADR-017-command-freshness-generation-interface.md)
- [Architecture](../planning/02_architecture_and_interfaces.md), upstream target cadence
- [MVP plan](../planning/03_mvp_delivery_plan.md), command watchdog
