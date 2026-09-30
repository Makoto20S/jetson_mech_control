# mech_hardware_ros2_control

Thin composite `SystemInterface` boundary. The Foundation scaffold has no
vendor protocol fields and no socket ownership.

Humble continues to call `read()` and `write()` while hardware is INACTIVE.
After configure or normal deactivate, these calls succeed without invoking the
stopped runtime: state values remain the last stored values, not fresh samples,
and no command is submitted. Claims are refused until activation and usable
feedback. Existing faults still return ERROR; inactive cycles never clear them.
INACTIVE performs no RX or freshness monitoring; an OK return does not establish
fresh feedback or physical rest.
Cleanup/configure is the recovery path. Deactivation revokes pending commands
immediately and does not synthesize a zero or a braking command.

## State declarations

Each joint declares a nonempty subset of the standard `position`, `velocity`,
and `effort` state interfaces. Unknown or repeated names are rejected. A joint
with a position command must declare position state so a weak controller claim
can seed its hold from accepted feedback. State interfaces are exported in
position, velocity, effort order for the fields declared by that joint.

A runtime may keep unsupported canonical fields unavailable (NaN). Only
declared fields are published and checked for finite values on each active
read. A nonfinite declared value latches the existing hardware fault. Existing
three-state declarations retain the same behavior.

## Position command bundles

A joint may declare one of `position`, `velocity`, `effort`, `position+velocity`,
`position+effort`, or `position+velocity+effort`, plus `command_generation`.
Motion names retain standard units/meaning; gains are not exported here.
Configured motion members must be claimed/released as a whole. A compatible
controller owns and updates the tuple together; claiming generation enables
ADR-017 freshness for the entire tuple. The hardware callback sees aggregate
names, not controller identities, so it cannot prove single-owner identity for
several controllers started in the same transaction. Split ownership is unsupported.

New position-bundle claims clear auxiliary buffers. Weak claims also seed the
position from accepted feedback; strong claims wait for a changed generation.
Revocation cancels the pending tuple and clears auxiliary values. Position-only
controllers still use a position-only declaration; they must not partially claim
a larger bundle. See [design](../../../docs/development/position_feedforward_design.md).
