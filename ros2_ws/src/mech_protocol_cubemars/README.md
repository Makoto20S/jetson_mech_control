# mech_protocol_cubemars

AK3.0 force-control codec and device session for CubeMars AKE60-8 actuators,
per `docs/development/ak30_force_control_adapter_design.md` and ADR-013.

## Scope

Force control is control mode ID `8`, a 29-bit extended Classic CAN frame.
Its three sub-modes (position, velocity, torque) share that one ID and are
distinguished by payload content, so the sub-mode is explicit configuration
and is never inferred from received data.

Not implemented here: the AK3.0 servo profile, any AK2.0/L02 profile, `0x2A`,
single-turn mode, ros2_control interface export, and any real device access.

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

`configure()` fails closed unless every mapping parameter the configured
sub-mode consumes is verified. With motor1's present evidence `pole_pairs`,
`gear_ratio` and `torque_constant` are verified, so **every sub-mode still
refuses to configure**; `direction_sign` (vendor question B9) is the single
parameter that unblocks both the torque and velocity sub-modes. The position
sub-mode additionally needs the zero offset and the encoder's shaft (B4).
