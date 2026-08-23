# RSP-001: Transport backend evaluation

> Status: completed offline evaluation on 2026-08-24
>
> Scope: SocketCAN and the HighTorque USB-CDC raw-CAN path only. No CAN
> interface, serial device, motor, HI12, or communication board was opened.

## Decision

Keep both backends below the same `RawCanFrame`/`Transport`/`BusRuntime`
contract, but do not treat them as interchangeable implementations:

1. **SocketCAN:** accept a minimal direct Linux RAW socket adapter as the
   `FND-010` implementation target. `ros2_socketcan` remains an API and
   license reference; its ROS node/DDS path is not part of the motor command
   loop.
2. **HighTorque USB-CDC / USB2CAN:** keep as the primary deployment candidate,
   but reference-only until `RSP-002` injected-serial tests and supplier board
   evidence close the missing capabilities. Reimplement the framing behind
   the project contract; do not copy or instantiate the vendor `canport` class.
3. **No backend is accepted for real activation by this record.** ADR-006
   remains `Proposed`; physical bitrate, device IDs, firmware, load, and error
   evidence still require the project gates.

## Evidence matrix

| Capability | Direct Linux RAW SocketCAN | HighTorque USB-CDC reference | RSP-001 result |
|---|---|---|---|
| License/provenance | Linux UAPI; no third-party code required | Local example has no own LICENSE or copyright header; embedded `wjwwood/serial` is MIT | Use direct SocketCAN; read-only protocol reference for CDC |
| API and ownership | `socket(PF_CAN, SOCK_RAW, CAN_RAW)`, `bind`, `setsockopt`, `recvmsg`/`sendmsg`; can be owned by one `BusRuntime` | `canport` constructor opens a serial port, starts its own receive thread, and owns the device | Wrap both behind injected `Transport`; no vendor ownership leak |
| Non-blocking behavior | `O_NONBLOCK`/pollable file descriptor; `CAN_RAW_FILTER` and `CAN_RAW_ERR_FILTER` are kernel options | Example uses serial reads and a background thread; constructor/version failures call `exit(1)` | SocketCAN is the FND-010 target; CDC requires a new controlled adapter |
| Frame capabilities | Standard/extended CAN, CAN FD via `CAN_RAW_FD_FRAMES`, error frames via `CAN_RAW_ERR_FILTER` | `MODE_FDCAN_PASS (0x12)` carries ID, flags, length and payload; example accepts Classic/FD/BRS fields | Canonical capability fields remain explicit; no synthesized CDC guarantees |
| Filters and fan-out | Kernel filters are available; multiple read-only sockets can observe matching frames | No equivalent kernel filter API; filtering is a project-side router concern | `FrameRouter` owns logical routing above the backend |
| Time | `SO_TIMESTAMPING`/receive ancillary data is available when the interface/driver supports it; hardware timestamp is optional | No source timestamp field was found in the CDC frame | Report unavailable timestamps instead of using USB read time as sample time |
| Error state | Linux error frames include arbitration, ACK, passive, bus-off and restart flags | Error command IDs `0x0F`/`0x11` are defined in the source but not parsed by the example | SocketCAN can expose errors; CDC error capability is unavailable until RSP-002/supplier evidence |
| Bitrate ownership | Configured on the Linux CAN netdevice, outside the raw socket | No bitrate-set command found; the board nominal rate is firmware-fixed and undocumented | Require explicit deployment capability; do not infer 1 Mbps or 5 Mbps |
| ARM64 | Linux UAPI and C++17 adapter are portable; target build is a later FND-010/ARM64 check | Protocol fields are portable, but the supplier example was not copied or built | No ARM64 claim for the vendor sample; no hardware side effect |

## Source checks performed

- Linux headers inspected: `/usr/include/linux/can.h`,
  `/usr/include/linux/can/raw.h`, and `/usr/include/linux/can/error.h`.
  They define `CAN_RAW_FILTER`, `CAN_RAW_ERR_FILTER`,
  `CAN_RAW_FD_FRAMES`, `CAN_ERR_BUSOFF`, `CAN_ERR_CRTL_RX_PASSIVE`, and the
  standard/extended CAN flags.
- HighTorque source inspected: `company/hightorque_fdcan(2)/hightorque_fdcan`.
  `src/canport.cpp` confirms serial enumeration by VID/PID `0xCAF1:0xFFFF`,
  board version requirement `>=4.8.8`, constructor-owned receive thread,
  `exit(1)` failure paths, CDC CRC8/CRC16 checks, and `MODE_FDCAN_PASS`.
- Existing planning/evidence records identify the seven-channel box as seven
  independent `/dev/ttyACM*` CDC channels. That is the project's USB2CAN
  hardware path; it is not evidence of a Linux `can0` netdevice.

## Follow-up boundaries

- `FND-010` may implement SocketCAN without waiting for a vendor license.
- `RSP-002` must use fake/injected serial and golden/negative vectors for CDC
  framing, short reads/writes, queue-full, disconnect, capability failures,
  and standard/extended plus Classic/FD/BRS flags.
- A real USB2CAN/HighTorque deployment still requires exact board/firmware,
  nominal bitrate, error semantics, stable channel identity, and G0-G3/ADR-006
  evidence.
