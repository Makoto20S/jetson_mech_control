# ADR-019：硬件位置包络与比例项边界

- **Decision ID:** ADR-019
- **Status:** Proposed
- **Date:** 2026-09-17
- **Owner:** Project owner
- **Scope:** Ak30ForceControlRuntime Position sub-mode command envelope; motor1 deployment parameters

## Status rationale / 状态依据

The project owner approved this design on 2026-09-17, before implementation,
as part of the T10 design document. It is therefore submitted `Proposed` on the
approve-then-implement path used by ADR-015 through ADR-017. It moves to
`Accepted` only after motor1 bench evidence taken with a standard
`ros2_control` controller and owner acceptance. The historical Kp4 trial later
observed a position-error latch (planning index), but acceptance is still pending.
The Position auxiliary-command extension has no real-device validation.
This decision authorizes no device operation;
[ADR-006](ADR-006-conditional-can0-deployment.md) Decision 7 per-run
authorization and the G0-G3 gates still apply.

## Context / 上下文

Standard `ros2_control` controllers driving this hardware directly is a
founding requirement, and [ADR-017](ADR-017-command-freshness-generation-interface.md)
keeps that door open with its weak tier: a controller that claims only the
motion interface makes every authorized manager cycle look like a refresh, so
the hardware can never observe that it went silent. ADR-017 Decision 4 accepted
that gap deliberately and required it to be registered as a risk; the entry in
`03_mvp_delivery_plan.md` §12 was never made. §6's "命令 watchdog" acceptance
row is not wrong about a stalled manager — the hard TTL expires in both tiers —
but it is silent about the case the weak tier actually leaves open: a
controller that stays active while it no longer produces new targets.

Nothing at the hardware layer bounds a Position target today.
`single_joint_command_controller` bounds it controller-side, which a
third-party controller bypasses entirely; the only hardware-side check is the
wire codec's encodable `+/-12.56 rad` range.
[Architecture](../planning/02_architecture_and_interfaces.md) §10 item 4
planned a hardware-side final bound on every command precisely so that a
mis-initialized controller could not escape the boundary, and no slice
delivered it.

The physics make the missing bound concrete. In Position sub-mode the proportional
torque term is `Kp * (command - measured)`, with motor1 shipping `Kp = 1 N*m/rad`
and `Kd = 1`. On the T9 bench (2026-09-17, unloaded shaft) static friction was
about `0.2 N*m`, and `0.2 N*m` accelerated the shaft at roughly `28 rad/s^2`
after breakaway. The distance between the commanded and the measured position
is therefore the force lever, and until now any controller could set it to any
encodable value.

## Decision / 决策

1. **Three deployment parameters, canonical rad:** `position_min_rad`,
   `position_max_rad`, `position_max_error_rad`. They are **required in
   Position sub-mode**. In Velocity and Torque they are optional, but the set
   is all-or-nothing: once any one of the three appears, all three must be
   present and valid. Missing, non-finite, `min >= max` or `error <= 0`
   rejects initialization before any device I/O, in `on_init`.
   The envelope check itself runs only in Position sub-mode.
2. **Absolute bounds are checked unconditionally** for every Position command
   in `submit_stored()`. The bounds are inclusive: a violation is
   `target < min` or `target > max`.
3. **The error bound `|target - measured| > position_max_error_rad` is
   evaluated only when `publish_states()` judged this cycle's feedback sample
   usable.** The target is never compared against an absent or stale number.
   [ADR-016](ADR-016-feedback-quality-fail-closed.md) already fails `read()`
   closed on a sample that aged out, so the only sample-less path left is the
   never-sampled startup transient, in which the joint cannot be claimed and
   no command can exist.
4. **Nothing is clamped.** A violating command is dropped; no frame carrying
   that value ever leaves.
5. **A violation latches.** The runtime sets its position-envelope latch,
   emits `FeedbackTelemetryReason::PositionEnvelope`, and `read()` fails every
   cycle until `configure()`/`start()` runs. It also clears
   `has_valid_sample()`, so under ADR-016 the joint becomes unclaimable until
   the lifecycle restarts. Claim cancellation does not clear it. This is the
   same fail-closed rule the `torque_max_abs_erpm` overspeed latch follows.
6. **The gate lives at the shared hardware layer, so it binds every
   controller** — in-house or upstream, strong tier or weak tier. ADR-014's
   interface shape, ADR-015's claim authorization, ADR-016's feedback gate and
   ADR-017's two tiers are unchanged.
7. **motor1 ships `-12.0 / 6.0 / 0.5`** in `config/motor1.urdf.xacro`. The
   absolute bounds match the existing controller YAML and add nothing beyond
   it; the error bound is new, and it is a real limit on the shipped
   deployment's reach — see Consequences/Negative.

## Physical meaning / 物理含义

The position-error envelope bounds only the nominal proportional term:
`|Kp * (position_target - position_feedback)| <= Kp * position_max_error_rad`
at the accepted feedback sample. At Kp=1, 0.5 rad gives a 0.5 N*m proportional
term budget. It is not a total torque limit, even when desired velocity and
feedforward are zero: the Kd damping term remains.

With the 2026-09-21 Position extension (ADR-014), the drive receives
`Kp*(p_des-p) + Kd*(v_des-v) + torque_ff`. Separate
`position_max_abs_velocity_rad_s` and `position_max_abs_feedforward_nm` bound
only the requested auxiliary values. They default to zero, and enabling an
auxiliary interface requires its explicit positive bound. Invalid auxiliary
values reject/latch the entire pending tuple; position envelope and feedback
freshness still apply. No software total-torque clamp or actual-speed bound
is claimed.

Bounds are evaluated on host-accepted feedback and cannot bound later device
behavior while it holds a previous command. No host cancellation/deactivation
is proof of immediate torque disable or physical rest. The extension has only
offline validation in this PR; real experiments await explicit owner approval.

## Alternatives considered / 替代方案

- **Clamp instead of reject.** Rejected. Clamping turns an invalid command
  into a valid-looking one, which is the reason `ak30_force_wire.hpp`'s encoder
  already refuses to clamp where the vendor reference does. A clamped command
  also makes a controller that is aiming outside the envelope look like it is
  working.
- **Controller-side bounds only.** Rejected. That is what
  `single_joint_command_controller` already does, and a third-party controller
  bypasses it. A bound that only self-written controllers honour bounds
  nothing, and accepting arbitrary controllers is the founding requirement.
- **A per-cycle step limit (slew) instead of an error bound.** Rejected. At
  500 Hz a per-cycle limit is meaningless within one cycle, and it is
  undefined for the first command after activation — which is exactly the
  dangerous one. Comparing against the measured position bounds both.

## Consequences / 后果

### Positive / 正面

- Every controller passes the same gate at the only layer all of them share.
  An upstream controller with a wrong goal cannot ask for more than
  `position_max_error_rad * Kp`, and cannot travel outside the absolute bounds.
- The weak-tier gap ADR-017 accepted keeps its likelihood but acquires a
  bounded physical consequence. It is registered as `03_mvp_delivery_plan.md`
  §12 R23.
- A violation is visible rather than absorbed: a telemetry reason plus a
  latched `read()` failure, not a silently altered command.

### Negative / 负面与代价

- A legitimate large step — a controller activated far from its target, for
  instance — latches the whole component, not just that one command. Choosing
  the bound trades reach against the proportional-term budget; there is no value that
  avoids both failure modes.
- **The shipped deployment's reach is materially reduced, and the numbers say
  by roughly how much.** At `Kp = 1` the shaft lags far behind the command: on
  the 2026-09-05 progressive bench route a `+10 deg` target moved the shaft
  `0.2 deg` while the command itself completed, and the move settled at
  `9.8 deg` of error (`0.171 rad`); the `+30 deg` run settled at `15.7 deg`
  (`0.274 rad`). Those figures are recorded in `docs/planning/README.md` §3,
  阶段 2 ("位置步进 +2°/+5° 与 +30° 全程（Kp=1 摩擦稳态误差）").
  `PositionCommandController` meanwhile ramps its command open-loop at
  `max_slew_per_second: 2.0 rad/s` regardless of where the shaft is. Neither
  settled error would trip `position_max_error_rad = 0.5` by itself; the
  transient is what is expected to. While the ramp is in flight the command
  runs away from a shaft that has barely moved, so command-minus-measured error
  peaks well above the error the move finally settles at. A single target much
  farther than `0.5 rad` (`29 deg`) from the current position will therefore
  exceed the bound part way through the ramp and latch the hardware before the
  shaft reaches the target, after which STRICT deactivation will be refused
  until the controller manager restarts (the T9 finding below). This is a
  prediction from the ramp shape and the `Kp = 1` bench numbers, not an
  observed latch — no bench run of the envelope exists. The `2.0 rad/s` slew is
  likewise not reachable under this envelope at `Kp = 1` at all: with `Kd = 1`
  and zero commanded velocity, sustaining `2 rad/s` would need more than
  `2 rad` of error. The absolute bounds are unchanged; this is the error bound
  alone, and it is a new, deliberate limit on the shipped deployment rather
  than a restatement of what the controller YAML already did. The
  owner-approved `0.5` is kept because it is the proportional-term budget (`0.5 N*m`),
  not a travel budget — bench procedures must still plan each move as if it
  were one, treating `29 deg` as the ceiling on a single target displacement,
  and the T10 trajectory's `0.1745 rad` step is inside it.
- Once the latch fails `read()` closed, ros2_control moves the hardware into
  its error state and **refuses STRICT deactivation** (observed on the T9 bench
  and recorded in `mech_bringup`'s README). Bench tooling must take its
  post-latch rest evidence from a fresh passive observation, not from a
  deactivation acknowledgement.
- `03_mvp_delivery_plan.md` §6's "命令 watchdog" criterion still holds in both
  tiers for a stalled manager, whose hard TTL expires either way. What it does
  not cover is a weak-tier controller that stays active while no longer
  producing new targets; the row is annotated to say so and that case is R23.
- This bounds the consequence, not the cause. A silent-but-active weak-tier
  controller still holds its last command; the envelope limits how hard, not
  how long.
- Three more deployment numbers must stay consistent between the xacro and the
  controller YAML, and they are per joint — nothing here generalizes to a
  second motor by itself.

## Validation / 验证

Offline coverage delivered with this decision: parameter rules
(required/finite/ordering/positive, other sub-modes accept-but-validate the set
as all-or-nothing); inclusive
bounds at both ends; the error bound in both signs; no error-bound evaluation
without a usable sample; the latch surviving claim cancellation and a
subsequent in-range command; the telemetry reason; a weak-tier controller
driven through a real `ControllerManager` receiving zero frames for the
violating command and having its STRICT deactivation refused afterwards; and a
cross-guard that the xacro bounds agree with the controller YAML.

Offline passing is not bench acceptance. The status moves to `Accepted` only
after a motor1 run in which a standard controller drives the hardware under
this envelope.

## Review triggers / 重审触发

- Any change to motor1's `Kp`: the ceiling is `position_max_error_rad * Kp`,
  so the same rad bound then means a different N*m.
- Any load, spring or gravity offset on the shaft: the friction and
  acceleration numbers this bound was sized against were taken unloaded.
- A second joint or a second motor: bounds are per joint and must not be
  inherited.
- Any change to ADR-017's tier structure, including replacing the weak tier
  with its alternative G.
- Bench evidence contradicting the `0.5 rad` ceiling in either direction: it
  latching during legitimate motion, or `0.5 N*m` proving too much for the
  fixture.

## Sources / 来源

- [ADR-016](ADR-016-feedback-quality-fail-closed.md)，反馈质量闸门与锁存先例。
- [ADR-017](ADR-017-command-freshness-generation-interface.md)，Decision 第 4 条的弱档缺口。
- [ADR-014](ADR-014-ak30-submode-command-interfaces.md)，子模式命令映射。
- [ADR-006](ADR-006-conditional-can0-deployment.md)，Decision 第 7 条逐次授权。
- [Architecture](../planning/02_architecture_and_interfaces.md)，§10 第 4 条硬件层最终限幅。
- [MVP plan](../planning/03_mvp_delivery_plan.md)，§6 命令 watchdog 与 §12 R23。
- [T10 design](../development/t10_jtc_interchangeability_design.md)，本决策的批准设计。
- [mech_bringup README](../../ros2_ws/src/mech_bringup/README.md)，T9 锁存后停用被拒的记录。
