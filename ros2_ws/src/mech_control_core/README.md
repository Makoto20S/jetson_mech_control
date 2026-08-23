# mech_control_core

Vendor-independent C++17 core package boundary. The FND-005 public types define
validated raw CAN frames, host monotonic time, optional raw source timestamps,
and status/sample metadata without introducing ROS headers into this API.

`RawCanFrame` uses byte-length payloads: Classic CAN is limited to 8 bytes and
CAN FD to 64 bytes. Standard and extended identifiers are explicit, and invalid
values are rejected by the factory functions. Source timestamps retain their
raw device/transport ticks; only host arrival time participates in freshness
until a clock mapping is proven.
