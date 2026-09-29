# mech_protocol_cubemars

AK3.0 force-control codec and device session for CubeMars AKE60-8 actuators,
plus an offline servo wire codec and position session, per
`docs/development/ak30_force_control_adapter_design.md` and ADR-013.

## Scope

Force control is control mode ID `8`, a 29-bit extended Classic CAN frame.
Its three sub-modes (position, velocity, torque) share that one ID and are
distinguished by payload content, so the sub-mode is explicit configuration
and is never inferred from received data.

The servo wire codec implements only L07 §4.1.7 p34–35 mode 6: extended
Classic CAN ID `(6 << 8) | drive_id`, DLC 8, signed big-endian int32 raw
device degrees ×10000, then two signed big-endian int16 fields for positive
electrical speed ERPM /10 and positive raw acceleration /10. The manual labels
acceleration ERPM/s²; its physical interpretation on the custom DE_V3.4 device
is not verified. Values truncate toward zero; invalid, non-finite,
out-of-range, or subquantum positive limits are rejected without clamping.
The position range is the manual's ±36000° intersected with int32
representability after scaling. Since 36000 × 10000 = 360,000,000, the manual
bound is tighter. The positive speed and acceleration wire maximum is
32767 × 10 = 327670 raw units. These are numeric encoding limits, not
manufacturer acceptance or approved motion limits for a particular motor.

L07 §4.3.1 p41–42 passive `0x29` feedback is decoded only for a configured
8-bit drive ID and an extended Classic Rx data frame of DLC 8. Position is
raw device degrees, speed electrical ERPM, current Iq amperes, and board
temperature signed °C. Status 0 is normal, 1–7 known fault, `0x77` a disable
acknowledgment, and other values unknown. A disable acknowledgment is never
a usable position sample. L07 §4.1.8 p35–36 mode 15 DLC 0 is exposed only as
an explicit torque disable request; the acknowledgment gives no controlled
stopping guarantee.

This wire layer does not infer current control mode, encoder source, shaft,
joint origin, direction, torque, or wrap. It supplies no transport, verified
device mapping, approved target envelope, or hardware activation.
Mode 4, modes 0–3, mode 5 zero setting, mode 16 Flash write, `0x2A`,
single-turn configuration, any AK2.0/L02 profile, ros2_control interface
export, and real device access remain outside this slice. Servo inputs must
not inherit motor1's force-mode mapping defaults.

## Offline position session

`Ak30ServoPositionSession` prepares one mode-6 frame from a canonical position
command after a fresh, accepted `0x29` feedback sample. It owns no transport,
does no I/O, and never sends a target on activation or deactivation. Its
configuration requires separately declared affine target and feedback
transforms, an explicit canonical position envelope and target error limit,
positive raw electrical speed and acceleration fields, command TTLs bounded
by the 6 ms contract, and a positive feedback TTL. The evidence flags indicate a caller-supplied offline
fixture; they do not establish real 104/105 calibration. The snapshot exposes
position only in canonical units, while speed and current retain their raw
wire unit names. It does not assert a canonical velocity or effort.

A single bus owner must first verify Classic CAN, extended-frame, and 8-byte
payload transport capabilities. It must route feedback by bus and CAN ID
before checking frame shape, so a matching malformed Classic/FD frame reaches
the session and revokes any prepared authorization. It then runs `receive`
with an observer that passes every routed frame to the matching session,
cancels routes whose sample or claim is no longer eligible, prepares and
submits new leases, then calls `transmit`. It must
cancel the route on **any** rejected preparation or admission so an older
pending target cannot survive a failed replacement. The test-only two-device
FakeTransport harness demonstrates this sequence. Before every transmit
attempt it also calls `pending_target_still_authorized(frame, generation,
deadline, now)` for each pending lease and cancels a route if feedback age or
position has made its prepared target unsafe. The check is read-only and
does not consume a generation. Matching unusable feedback invalidates the
prepared lease even if a normal frame follows in the same receive batch; a
new prepare is required. A matching malformed,
stale, unknown-status, or disable-ack frame revokes position authorization;
a known fault latches until deliberate deactivate/reconfigure. Host arrival
time is not a device timestamp and cannot detect old gateway-buffered frames
retimestamped by a transport. This offline primitive makes no physical stop
or production `DeviceSession`/ROS plugin/deployment claim.

## Two defects in the vendor manual that this package deliberately does not copy

1. L07's printed `float_to_uint()` scales by `(1 << bits) / span`. The manual's
   own worked-example table was generated with `((1 << bits) - 1) / span`, and
   the printed formula reproduces none of the manual's examples. This package
   uses `((1 << bits) - 1)`. With the printed formula, commanding `P_MAX`
   (+12.56 rad) produces `p_int = 65536`, which overflows 16 bits to `0x0000`
   and decodes as **-12.56 rad** — maximum position commands minimum position.
   `EncodingMaxPositionDoesNotWrapToMinimum` pins this.
2. L07 §4.4.1's `速度设置为 6rad/s` row reads `... 98 67 FF`; the correct byte
   is `0x9B`. The `-6 rad/s` row in the same table is correct, which is what
   proves the packing right and this one cell wrong.

The §4.4 example code is explicitly `参数以 AK10-9 为例` — velocity ±28 rad/s,
torque ±54 N·m. Tests citing manual examples use `ak10_9_ranges()`; AKE60-8 is
±40 / ±15.

## Evidence gate

The existing force-control `configure()` fails closed unless every mapping
parameter the configured sub-mode consumes is verified. Motor1 now has verified
`pole_pairs`,
`gear_ratio`, `torque_constant` and `direction_sign`, plus an owner-approved
provisional position mapping using `zero_offset = 330.07°` and an output-shaft
interpretation. All three sub-modes therefore configure for controlled bench
validation; Position semantics remain provisional until the low-gain sequence
test settles B4/B14.

The optional `mech_bringup/ak30_torque_probe` is a bench bring-up tool, not
the production ros2_control integration. Its current canonical-state speed
check is intentionally ineffective in Torque mode because that mode does not
evidence `velocity`; raw-ERPM abort handling is a follow-up probe enhancement,
not part of this protocol package's contract.
