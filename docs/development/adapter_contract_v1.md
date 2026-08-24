# AdapterContract v1

`AdapterContract v1` is frozen at Foundation RC. A new device adapter supplies
codec/session behavior above `RawCanFrame` and below canonical state/command
semantics; it does not modify controllers or own a transport.

## Required boundary

- Pure C++ codec: validated bytes in/out, no ROS, I/O, sleep, allocation on the
  active path, device probing, or process exit.
- Session: fixed configure-time `ProtocolProfile`, explicit state machine,
  command validation, freshness, limits, watchdog/TTL, faults, and canonical
  snapshots.
- Transport: injected `Transport&`; the adapter never opens SocketCAN or serial.
- Capabilities: reject unavailable Classic/FD/BRS, ID format, timestamp, error,
  filter, bitrate, or payload features rather than synthesizing them.
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
