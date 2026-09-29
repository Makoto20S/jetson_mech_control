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

For shared-bus sessions, `BusRuntime::receive(now, observer)` drains at most
64 frames, invokes the non-owning `RxObserver` once for each routed frame in
order, and faults if the observer rejects a frame or a snapshot cannot be
stored. The observer is a `noexcept` function pointer plus context and must
not re-enter the runtime. The caller checks its session's feedback freshness
between `receive()` and `transmit(now)`; splitting the phases alone does not
make old feedback fresh. `poll(now)` remains the legacy receive-then-transmit
convenience path and continues counting snapshot overflow without faulting.

`cancel(route_id)` clears a bound route's pending host lease immediately while
retaining its generation watermark in the current epoch. Unknown or unbound
routes return `InvalidCommand`; a stopped/faulted runtime returns
`NotRunning`. Cancellation does not retract a transmitted frame or stop a
physical device. Stop, restart, and fault/recovery clear leases, route bindings,
generation watermarks, snapshots, and the fairness cursor. Transport buffers
and pre-epoch source timestamps remain session-level responsibilities. All
runtime calls require serialization; the observer cannot call runtime methods.
