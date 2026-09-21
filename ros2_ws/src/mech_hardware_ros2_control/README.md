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
