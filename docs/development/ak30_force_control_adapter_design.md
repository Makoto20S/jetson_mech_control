# AK3.0 Force-Control Adapter Design

- **Status:** Implemented — `mech_protocol_cubemars`, with provisional Position mapping enabled for controlled bench validation. Production activation remains gated by ADR-006 and G0–G3.
- **Date:** 2026-09-01
- **Scope:** `mech_protocol_cubemars` — the AK3.0 force-control codec and device session, offline only
- **Governs:** the first vendor protocol adapter under `AdapterContract v1`
- **Supersedes:** an earlier draft written against L02 (AK2.0). That baseline was wrong for this hardware; see [ADR-013](../adr/ADR-013-ak30-protocol-baseline.md).

## 1. Scope

### In scope

- A new `mech_protocol_cubemars` package holding the AK3.0 force-control codec and device session.
- Force control is control mode ID `8`. Its three sub-modes — position (`Kp`+`Kd`+position), velocity (`Kd`+velocity) and torque — share that one ID and are distinguished by payload content.
- The `0x29` periodic feedback decoder.
- Offline tests only: golden, negative, boundary, watchdog, lifecycle, and simulator integration.

### Explicitly out of scope

| Excluded | Reason |
|---|---|
| The AK3.0 servo profile (modes `0–6`, `15`, `16`) | Second implementation profile; separate codec, separate PR |
| Any AK2.0 / L02 profile | Wrong hardware generation; the 11-bit standard MIT profile does not exist here |
| Enabling `0x2A` or single-turn mode | Both are written to Flash by mode `16`; `0x2A` also breaks the bus budget at 500 Hz |
| Any change to `mech_hardware_ros2_control` | Interface export is a later slice |
| Any real device activation | Gated by ADR-006 and G0–G3 |

## 2. Governing constraints

| Source | Constraint |
|---|---|
| `adapter_contract_v1.md` | Codec is pure, no I/O or session state; session borrows an injected transport; one explicit `ProtocolProfile`, no runtime auto-detection |
| `ADR-013` | L07 is the baseline; force control is the first profile; profile fixed at configure with **no ACTIVE-period mixing — by our choice, not a firmware limit**; golden vectors must be back-solved |
| `ADR-009` (as amended) | The Kt blocker is lifted: `Kt = 0.7382 N·m/A`, `T = Kt × Iq` at the **output shaft**. Direction, zero chain, encoder source and physical accuracy stay gated |
| `ADR-012` | Staged watchdog: follow → freeze last valid → explicit ERROR past a hard TTL; a position command never resolves to `0.0`; the whole watchdog fits `<=3` control cycles. **This package classifies the watchdog stage via `command_stage()` and never synthesizes or re-sends a command; acting on `Holding`/`Expired` — freezing, erroring, or otherwise — is the caller's responsibility and is not implemented here** |
| `02:81` / `02:82` | Codec owns byte order, bit fields, scaling, range, error codes. Session owns firmware capability, sample aggregation, freshness, command mode, device state machine |
| `02:160` | `on_configure` must validate **firmware range**, frame format, codec, feedback set and command set, and bind the profile |
| `06 §5` | 500 Hz for two motors plus two HI12 is 41.6%; enabling `0x2A` reaches 53.6% and is excluded |

## 3. Layering

Same three-part split as the superseded draft — that part of the design survived the baseline change intact.

```
RawCanFrame
   ├─ ak30_force_wire      bytes ↔ device-native units (rad, rad/s, N·m, A)
   ├─ ak30_mapping         device-native ↔ canonical SI, holds tri-state parameters
   ├─ ak30_force_codec     implements DeviceCodec by composing the two above
   └─ ak30_force_session   implements DeviceSession
```

Golden vectors are asserted against the wire layer so test numbers trace to the manual with no interpretation in between. Everything unevidenced lives in the mapping layer.

Mapping sits inside the codec, constructor-injected, because `DeviceCodec::decode` outputs `CanonicalDeviceState` directly; putting it in the session would have the codec emit device-native numbers into a struct whose field names contradict their units.

## 4. Wire layer

### 4.1 Frame identifier

`control_mode << 8 | drive_id`, extended (29-bit). Force control is control mode `8`; drive ID occupies bits `[7:0]` and therefore cannot exceed 255. For motor1 (`drive_id = 104 = 0x68`) the command ID is `0x0868`.

### 4.2 Command payload — DLC 8

| DATA | 0 | 1[7:4] | 1[3:0] | 2 | 3 | 4 | 5 | 6[7:4] | 6[3:0] | 7 |
|---|---|---|---|---|---|---|---|---|---|---|
| field | KP hi 8 | KP lo 4 | KD hi 4 | KD lo 8 | pos hi 8 | pos lo 8 | vel hi 8 | vel lo 4 | trq hi 4 | trq lo 8 |

Widths: `KP` 12, `KD` 12, position 16, velocity 12, torque 12 — 64 bits exactly.

**This order is `KP KD POS VEL TRQ`.** The AK2.0 MIT frame and the widely-copied Arduino demos use `POS VEL KP KD TRQ` on an 11-bit standard frame. A packing helper must never be shared between them.

### 4.3 Normalization — AKE60-8 (L07 p.37)

| quantity | range | bits |
|---|---|---:|
| position | ±12.56 rad | 16 |
| velocity | ±40.0 rad/s | 12 |
| torque | ±15.0 N·m | 12 |
| `KP` | 0–500 | 12 |
| `KD` | 0–5 | 12 |

Signed quantities map linearly onto the unsigned field with midpoint `(2^bits − 1) / 2`; `KP`/`KD` map from zero. Position, `KP` and `KD` ranges are merged cells spanning all models in the source table, confirmed by reading the page as an image rather than trusting `pdftotext`.

**Two defects in L07 that the implementation deliberately does not reproduce.**

1. §4.4's printed `float_to_uint()` scales by `(1 << bits) / span`. The manual's
   own example table was generated with `((1 << bits) - 1) / span`, and the
   printed formula reproduces none of the manual's examples: position `0 rad`
   becomes `80 00` instead of `7F FF`, velocity `0` becomes `0x800` instead of
   `0x7FF`, and velocity `-6 rad/s` becomes `64 98` instead of `64 87`. At
   `p_des = P_MAX` it yields `p_int = 65536`, which overflows the 16-bit field
   to `0x0000` and decodes as `-12.56 rad` — **maximum position commands
   minimum position**. `EncodingMaxPositionDoesNotWrapToMinimum` pins this.
2. §4.4's worked examples are stated to use **AK10-9** constants (`±12.56 rad`,
   `±28.0 rad/s`, `±54.0 N·m`), not AKE60-8's. Decoding an example row with
   AKE60-8's ranges gives a wrong answer, which is why the test suite carries
   `ak10_9_ranges()` purely for manual cross-checking.

Quantization therefore truncates with divisor `((1 << bits) - 1)`, and encoding
**rejects** out-of-range input rather than clamping as the vendor code does:
clamping converts an invalid command into a valid-looking one, which is the
degradation ADR-012 forbids.

### 4.4 Feedback decoder — function ID `0x29`, DLC 8

Position `int16 × 0.1°`, velocity `int16 × 10` ERPM, Iq `int16 × 0.01 A`, driver-board temperature `int8`, `DATA[7]` status byte.

`输出端转速 = ERPM ÷ pole_pairs ÷ gear_ratio` (L07 §4.3.1) — the manual gives this conversion explicitly.

**`DATA[7]` carries two disjoint meanings.** Fault codes are `0`–`7` (`0` none, `1` motor over-temperature, `2` over-current, `3` over-voltage, `4` under-voltage, `5` encoder, `6` MOSFET over-temperature, `7` stall). **`0x77` is the disable-succeeded acknowledgement**, returned once after control mode `15`. Decoding `0x77` as a fault would misreport a successful disable, which is the safety path. Any other value is unknown and must be surfaced as such rather than silently mapped.

## 5. Mapping layer

| Parameter | Initial state for motor1 | Basis |
|---|---|---|
| `pole_pairs` = 14 | **verified** | Screenshot and XML export agree |
| `gear_ratio` = 8 | **verified (2026-09-02, bench)** | Three agreeing sources: the host tool's dedicated `减速器参数设置 → Ratio: 8`; L07's own model-naming convention (AK80-**9** is 9:1, AK60-**39** is 39:1, AKH70-**48** is 48:1, so AKE60-**8** is 8:1); and the displayed ratio. `si_gear_ratio = 0` is a different, unset VESC-lineage SI-display field, not a counter-source |
| `zero_offset` = 330.07° | **bench-validated (2026-09-05)** | Owner-approved screenshot value (export still records `336.28°`). The hold-current bench test closed the conversion loop: canonical -263.758° + 330.07° = 66.3° ≈ raw feedback 66.4°, and position stayed constant to the digit across 150 samples while effort sat at the noise floor — the mapping is self-consistent under position replay. Does not validate the mapping for commanded travel across a wrap boundary (B14 open) |
| `position_source_shaft` = output | **provisional** | Bench mapping treats `0x29` as output-shaft position; L07 does not state the position source explicitly. Bench evidence (§10) supports a stable absolute reading that follows external rotation, but single-turn vs multi-turn folding is unproven |
| `direction_sign` = +1.0 | **verified (2026-09-03, bench)** | Measured chain: positive command (+0.8/+1.0 rad/s, Kd path) → clockwise rotation (owner-observed, two runs) → feedback position increases → Iq predominantly positive. Canonical positive = feedback-increase direction. Vendor answer B9 now serves only to cross-check the `foc_encoder_inverted`/`m_invert_direction` layer chain |
| `firmware_id` = `AKE60_8_DE_V3.4` | unverified | Operator-asserted; no firmware query exists on this protocol |
| `torque_constant` = 0.7382 N·m/A | **verified (bench closed-loop)** | L07 p.37 for AKE60-8, owner-guaranteed for this variant (ADR-013 §4). Bench: +0.1 N·m command → effort echo 0.081–0.111 N·m ≈ `Kt × Iq` |

### Hard rules

1. **Fail closed at configure.** `configure()` returns `InvalidConfiguration` unless every mapping parameter consumed by the configured sub-mode is verified. The same mapping converts both directions, so suppressing only decode would still let the session emit a command computed from an unverified mapping — worse, because it moves the motor. `ADR-009` Decision 2 prescribes exactly this: configure rejects, it does not downgrade.
   - **Consequence, accepted:** Torque and Velocity use confirmed mappings. Position now uses the owner-approved `330.07°`/output-shaft mapping provisionally; a low-gain bench test must validate or revise it before production use.
2. **`SampleQuality` is not the guard.** Nothing outside `status.hpp` reads it as of `9317d76`; marking a sample `Degraded` would enforce nothing. Quality still reports genuine per-sample conditions such as staleness.
3. Effort may now be populated, because Kt is verified and the manual states T is output-shaft. This is the one thing the baseline change unblocked outright.

## 6. Configuration and validation

The sub-mode is **explicit configuration**, not inferred from the payload. Force control's three sub-modes share control mode ID `8`, so unlike the servo profile the ID cannot identify them — which is precisely why inferring from data would be a silent-failure path.

`configure()` returns `InvalidConfiguration` when:

- `command_id` bits `[28:8]` are not `8`;
- `frame_format` is not `Extended` or `frame_type` is not `Classic` — note the superseded enum required `Standard` here, which would have rejected every valid configuration;
- `feedback_id` is not `0x29 << 8 | drive_id`;
- the declared firmware identifier is outside the configured accepted range (`02:160`);
- capabilities lack extended-ID support, or the configuration requests BRS or remote frames;
- any mapping parameter the configured sub-mode consumes is unverified (§5 rule 1);
- `KP`/`KD` are outside 0–500 / 0–5.

## 7. Session

Force control has **no host-side activation handshake** and, being impedance control, none of the servo position mode's "runs to target at maximum speed" hazard. The first-command plausibility check designed for the superseded servo-first draft is therefore **not carried over**; it guarded a hazard this profile does not have.

The session classifies the command's watchdog stage — following, holding, or expired — through `command_stage()`, and guarantees it will never synthesize or re-send a command on the caller's behalf; a position command never resolves to `0.0`. Acting on a `Holding` or `Expired` stage — freezing the last valid command, raising an explicit ERROR, or anything else `ADR-012` prescribes — is the caller's responsibility and is not implemented in this package.

A fault code in `1`–`7` latches a fault. `StatusSnapshot::raw_fault_code` preserves the raw byte; the decoded meaning never overwrites it.

## 8. Testing

| Group | Cases |
|---|---|
| Golden | Every sub-mode encoder and the `0x29` decoder. **Each vector back-solved against the documented ranges, never copied** — L07 §4.4.1's velocity example has `DATA[5]=0x98` where `0x9B` is correct, an ~11% error no offline test would otherwise catch. The manual's torque, position, `KP` and `KD` examples do verify exactly and are usable as-is once checked |
| Identifier | Mode/drive composition; drive ID > 255 rejected; control mode ≠ 8 rejected |
| Negative | Wrong DLC; standard frame presented as extended; BRS set; remote frame; wrong feedback ID |
| Command validation | NaN/Inf; position beyond ±12.56 rad; velocity beyond ±40 rad/s; torque beyond ±15 N·m; `KP`/`KD` out of range |
| Status byte | Fault codes 1–7 latch; **`0x77` decodes as disable-acknowledged, not a fault**; unknown values surface as unknown |
| Mapping gate | `configure()` rejects every permutation with an unverified parameter the sub-mode consumes, accepts the fully verified one |
| Session | Command before any feedback; watchdog follow/freeze/error; duplicate, out-of-order and missing feedback; fault recovery |
| Cross-profile | An AK2.0-style 11-bit standard MIT frame and a servo-profile extended frame are both rejected |
| Integration | Against `mech_simulation`'s fake transport, no real CAN or serial access |

## 9. Registration and CI

`tools/ci/context_check.py` validates the exact package set and each package's internal dependency edges, so adding a package is not free.

1. Add `mech_protocol_cubemars` to `EXPECTED_PACKAGES` with its exact internal dependency set, or CI fails.
2. Provide `CMakeLists.txt`, `package.xml` with the matching name, and `test/test_package_marker.cpp` — all three are required by the checker.
3. L07 is already registered in `manifests/assets.yaml` with a verified hash, satisfying the adapter contract's evidence-source requirement.

## 10. Still open, and what each blocks

> **Bench status 2026-09-05:** the progressive bench-validation route (pure torque → Kd-damped velocity → zero-displacement hold → position steps → +30° full travel) completed on the real motor with closed-loop evidence for all three sub-modes, all under the ADR-006 Decision-7 forensic-bench profile. That evidence resolved several items below by measurement; each is updated in place. It does **not** constitute production approval for the final deployment profile.

- **`0x29` encoder source** — provisionally mapped as output-shaft for the bench. Bench evidence supports a stable absolute reading that follows hand rotation in real time, and the hold test closed the canonical↔raw conversion loop. Still open: which encoder the value comes from (vendor question B4), and single-turn vs multi-turn folding (B14, see below).
- ~~**Direction and zero chain** — blocks `direction_sign`, hence all sub-modes including torque. Vendor question B9.~~ **Resolved by measurement (2026-09-03):** `direction_sign = +1.0` verified on the bench (positive command → clockwise → feedback increases → Iq positive); vendor answer B9 now serves only to cross-check the `foc_encoder_inverted`/`m_invert_direction` layer chain.
- ~~**Gear ratio conflict** (`si_gear_ratio = 0` vs displayed 8) — vendor question B8.~~ **Closed 2026-09-02.** There was no conflict: `si_gear_ratio` is a different, unset VESC-lineage SI-display field. See the §5 table. B8 is withdrawn.
- ~~**Force-control feedback frame** — vendor question B13.~~ **Closed by bench measurement (2026-09-03):** `0x29` periodic feedback at 50 Hz works independently of the force-control command family (semantics match L07 field-for-field, status byte 0). The vendor answer retains only cross-check value.
- **Single-turn / `0x2A` Flash state** — position semantics may depend on a persisted setting invisible on the wire. Vendor question B14. **Bench evidence so far:** position is stable and hand-following, and the hold/delta runs never crossed a wrap boundary, so folding semantics are unproven; after high-speed runs the reading parked at 3200.0° (3200.0 mod 360 = 320.0, and int16×0.1° wraps at ±3276.7°), so single-turn 0–360° folding vs an in-range multi-turn reading cannot be distinguished yet. **Next evidence: a continuous low-speed Kd-damped run logging the raw position sequence sample-by-sample** (owner decision; any run needs fresh per-test authorization). The `多圈模式` / `单圈模式` buttons in the host tool's trajectory-planning panel remain a host-side command-shaping choice, not the device's persisted feedback mode.
- ~~**Which shaft the command velocity refers to** — vendor question B15.~~ **Strong bench evidence (2026-09-03), pending vendor confirmation:** position deltas during the Kd tests imply output-shaft speeds of 0.608/0.784 rad/s for 0.8/1.0 rad/s commands (a motor-side field would produce only ~134 ERPM vs the measured 660–1150). The implementation's output-side assumption is bench-consistent; close formally with the vendor answer.
- **`ADR-012`'s staged freeze/error actions** — this package implements only the classification half: `command_stage()` reports `Following`/`Holding`/`Expired` and the session never synthesizes or re-sends a command. It does not freeze the last valid command or raise an explicit ERROR past the hard TTL; there is no caller yet to drive that behavior. This blocks nothing offline. **The ros2_control hardware-plugin slice must close it by polling `command_stage()` and acting on `Holding` and `Expired` itself; shipping that plugin without consuming `command_stage()` would leave the watchdog decorative.** That slice is the next planned work after PR #9.
