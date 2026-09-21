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
