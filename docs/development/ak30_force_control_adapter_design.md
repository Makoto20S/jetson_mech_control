# AK3.0 Force-Control Adapter Design

- **Status:** Draft for review
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
| `ADR-012` | Staged watchdog: follow → freeze last valid → explicit ERROR past a hard TTL; a position command never resolves to `0.0`; the whole watchdog fits `<=3` control cycles |
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

### 4.4 Feedback decoder — function ID `0x29`, DLC 8

Position `int16 × 0.1°`, velocity `int16 × 10` ERPM, Iq `int16 × 0.01 A`, driver-board temperature `int8`, `DATA[7]` status byte.

`输出端转速 = ERPM ÷ pole_pairs ÷ gear_ratio` (L07 §4.3.1) — the manual gives this conversion explicitly.

**`DATA[7]` carries two disjoint meanings.** Fault codes are `0`–`7` (`0` none, `1` motor over-temperature, `2` over-current, `3` over-voltage, `4` under-voltage, `5` encoder, `6` MOSFET over-temperature, `7` stall). **`0x77` is the disable-succeeded acknowledgement**, returned once after control mode `15`. Decoding `0x77` as a fault would misreport a successful disable, which is the safety path. Any other value is unknown and must be surfaced as such rather than silently mapped.

## 5. Mapping layer

| Parameter | Initial state for motor1 | Basis |
|---|---|---|
| `pole_pairs` = 14 | **verified** | Screenshot and XML export agree |
| `gear_ratio` = 8 | unverified | Export `si_gear_ratio = 0` contradicts the displayed 8 |
| `zero_offset` | unverified | Screenshot `330.07°` vs export `336.28°` |
| `position_source_shaft` | **unknown** | L07 writes 输出端 explicitly for torque and speed but not for position |
| `direction_sign` | unverified | `foc_encoder_inverted` and `m_invert_direction` act at different layers |
| `firmware_id` = `AKE60_8_DE_V3.4` | unverified | Operator-asserted; no firmware query exists on this protocol |
| `torque_constant` = 0.7382 N·m/A | **verified** | L07 p.37 for AKE60-8, owner-guaranteed for this variant (ADR-013 §4) |

### Hard rules

1. **Fail closed at configure.** `configure()` returns `InvalidConfiguration` unless every mapping parameter consumed by the configured sub-mode is verified. The same mapping converts both directions, so suppressing only decode would still let the session emit a command computed from an unverified mapping — worse, because it moves the motor. `ADR-009` Decision 2 prescribes exactly this: configure rejects, it does not downgrade.
   - **Consequence, accepted:** with motor1's evidence only `pole_pairs` and `torque_constant` are verified, so the position and velocity sub-modes will refuse to configure until vendor answers land. **The torque sub-mode consumes only `torque_constant` and `direction_sign`** — it is therefore the closest to usable, and the first thing to unblock if `direction_sign` is resolved.
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

Watchdog follows `ADR-012` unchanged: follow → freeze last valid → explicit ERROR past the hard TTL, and a position command never resolves to `0.0`.

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

- **`0x29` encoder source** — blocks `position_source_shaft`, hence the position sub-mode. Vendor question B4.
- **Direction and zero chain** — blocks `direction_sign`, hence all sub-modes including torque. Vendor question B9. **This is the shortest path to a usable adapter.**
- **Gear ratio conflict** (`si_gear_ratio = 0` vs displayed 8) — vendor question B8.
- **Force-control feedback frame** — L07 §4.3.1 is titled "servo mode feedback" and §4.2 defines only the command; the manual never states what feedback looks like in force-control mode. `0x29` as a command-family-independent status frame is the reasonable inference. Vendor question B13.
- **Single-turn / `0x2A` Flash state** — position semantics depend on a persisted setting invisible on the wire. Vendor question B14.
