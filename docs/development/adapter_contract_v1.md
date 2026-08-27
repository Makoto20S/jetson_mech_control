# AdapterContract v1

`AdapterContract v1` is frozen at Foundation RC. A new device adapter supplies
codec/session behavior above `RawCanFrame` and below canonical state/command
semantics; it does not modify controllers or own a transport.

## Required boundary

- Pure C++ codec: validated bytes in/out, no ROS, I/O, sleep, allocation on the
  active path, device probing, or process exit.
- Session: fixed configure-time `ProtocolProfile`, explicit state machine,
  command validation, freshness, limits, watchdog/TTL, faults, and canonical
  snapshots. A position command must never resolve a timeout or an invalid
  input to `0.0`, which on a position interface is a commanded move to the zero
  position; follow the staged watchdog in
  [ADR-012](../adr/ADR-012-command-watchdog-and-capability-honesty.md).
- Transport: injected `Transport&`; the adapter never opens SocketCAN or serial.
- Capabilities: reject unavailable Classic/FD/BRS, ID format, timestamp, error,
  filter, bitrate, or payload features rather than synthesizing them. Quantities
  that could be measured or merely asserted carry an explicit `*_verified` flag
  (`nominal_bitrate_verified`, `queue_capacity_verified`): verified means the
  backend read the value back from the real channel in this process, unverified
  with a value means an operator asserted it, and unverified with zero means
  genuinely unknown and deliberately not claimed. An adapter must not treat an
  unverified value as fact. See
  [ADR-012](../adr/ADR-012-command-watchdog-and-capability-honesty.md).
- Remote frames: `RawCanFrame::remote_request` carries CAN 2.0 RTR (Classic
  only, never with BRS, payload zero-filled, `payload_size` is the requested
  DLC). A backend that cannot represent RTR reports
  `supports_remote_frames = false` and rejects such frames; it never silently
  degrades one into a data frame.
- Lifecycle: configure/activate/deactivate/cleanup are repeatable; ACTIVE never
  probes or changes profile.
- Semantics: position/velocity/effort are exported only when the physical
  mapping is evidenced. Current/raw values never masquerade as effort.

## Mandatory tests

- Golden encode/decode vectors tied to a registered evidence source.
- Invalid ID, DLC, flag, CRC/field, NaN/Inf, limit, profile, and capability cases.
- Duplicate/out-of-order/missing feedback, stale state, TTL, disconnect,
  queue-full, and fault recovery.
- Simulator integration without real CAN/serial access.
- Existing controllers and `mech_control_core` compile unchanged.

## New adapter checklist

1. Add a dedicated `mech_protocol_<device>` package; do not add brand branches
   to core, controllers, or the composite hardware plugin.
2. Register authoritative manuals/assets and their hashes before golden vectors.
3. Add one explicit `ProtocolProfile`; no runtime auto-detection.
4. Implement the interfaces in `adapter_template.hpp` and inject transport/time.
5. Map only evidenced canonical semantics and preserve raw fault/source times.
6. Pass offline tests before requesting G0-G3 or HIL authorization.
7. Submit any canonical contract change as an ADR before implementation.
