# mech_control_core

Vendor-independent C++17 core package boundary. The FND-005 public types define
validated raw CAN frames, host monotonic time, optional raw source timestamps,
and status/sample metadata without introducing ROS headers into this API.

`RawCanFrame` uses byte-length payloads: Classic CAN is limited to 8 bytes and
CAN FD to 64 bytes. Standard and extended identifiers are explicit, and invalid
values are rejected by the factory functions. Source timestamps retain their
raw device/transport ticks; only host arrival time participates in freshness
until a clock mapping is proven.

FND-006 adds typed schema/configuration and transport capabilities with
deterministic rejection of missing fields, duplicate routes, unknown profiles,
and incompatible frame/capability combinations. FND-008 and FND-009 provide
filter routing, snapshot age calculation, command leases, bounded latest-value
slots, and single-writer `BusRuntime` ownership without ROS or vendor fields.
