#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mech_control_core/router.hpp"
#include "mech_control_core/transport.hpp"

namespace mech::mech_control_core {

struct SocketCanOptions final {
  std::string interface_name;
  std::uint16_t logical_bus{0U};
  std::size_t receive_queue_capacity{64U};
  std::uint32_t nominal_bitrate_hz{0U};
  bool enable_can_fd{false};
  bool enable_error_frames{false};
  std::vector<FrameFilter> filters;
};

// Direct Linux RAW SocketCAN adapter. Opening is explicit and never happens in
// the constructor, so configuration and tests cannot touch a device by
// accident.
//
// Error-frame encoding
// ---------------------
// SocketCAN error frames (CAN_ERR_FLAG set on the wire) carry information that
// does not fit the data-frame meaning of RawCanFrame's `id`/`payload` fields:
// a class-of-error bitmask (CAN_ERR_* from linux/can/error.h) plus up to 8
// controller/protocol/transceiver detail bytes. Rather than extend frame.hpp
// (frozen), an error frame reuses the existing fields with a distinct
// encoding, gated entirely on `error_frame == true`:
//   - `id.value`   holds the raw CAN_ERR_* class bitmask (the `can_id` word
//                  masked with CAN_ERR_MASK, i.e. with CAN_ERR_FLAG/
//                  CAN_EFF_FLAG/CAN_RTR_FLAG cleared). `id.format` is always
//                  CanFrameFormat::Extended -- chosen only because
//                  CAN_ERR_MASK is 29 bits wide and would not fit CanId's
//                  11-bit Standard range, not because the error frame was
//                  actually an extended-format frame. `id.format` carries no
//                  data-frame meaning here.
//   - `payload[0..7]` holds the 8 SocketCAN error detail bytes verbatim
//                  (arbitration-lost bit, controller state, protocol
//                  violation type/location, transceiver status, and the tx/rx
//                  error counters), in the same byte order the kernel uses
//                  (see linux/can/error.h: data[0]..data[7]).
//   - `payload_size` is always 8 (CAN_ERR_DLC) for an error frame.
//   - `type` is always CanFrameType::Classic; `remote_request` is always
//     false (error frames cannot be RTR).
// This encoding is internal to this backend; consumers must branch on
// `error_frame` before interpreting `id`/`payload` at all.
//
// Timestamp domain
// -----------------
// try_receive() reports SourceTimestamp{SourceClockDomain::Transport, ticks}.
// DECISION (recorded here because it is a judgement call, not silence): this
// backend attempts SO_TIMESTAMPING with SOF_TIMESTAMPING_RX_SOFTWARE |
// SOF_TIMESTAMPING_SOFTWARE first, and falls back to SO_TIMESTAMPNS if the
// kernel rejects it (see open() in the .cpp). It does NOT claim the
// SO_TIMESTAMPING path yields CLOCK_MONOTONIC. Per
// Documentation/networking/timestamping.rst, the general software timestamp
// reported as ts[0] of `struct scm_timestamping` (which is exactly what
// SOF_TIMESTAMPING_SOFTWARE selects) is produced by ktime_get_real(), i.e.
// CLOCK_REALTIME -- the same wall-clock domain as SO_TIMESTAMPNS. This could
// not be independently confirmed against a real CAN adapter or kernel trace
// in this change (no hardware access; vcan-only testing), so the switch to
// SO_TIMESTAMPING is made for forward-compatibility with any future kernel
// path that does offer a monotonic option, but it is NOT relied upon to fix
// the wall-clock problem today.
//
// The practical consequence: regardless of which of the two setsockopt calls
// actually succeeded (open() does not record which one at runtime, because
// neither is trusted to be monotonic -- see above), every ticks value
// delivered by this backend today MUST be treated as CLOCK_REALTIME
// (wall-clock) -- it steps and jumps under NTP/PTP discipline, is not
// guaranteed monotonic between samples, and is NOT safe to difference
// against MonotonicTime. SourceClockDomain has no value that distinguishes
// "wall-clock" from "monotonic" (only Transport/Device), so this cannot be
// expressed in the type system without touching the frozen time.hpp; it is
// recorded here in prose instead.
class SocketCanTransport final : public Transport {
 public:
  explicit SocketCanTransport(SocketCanOptions options);
  ~SocketCanTransport() override;

  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;

  [[nodiscard]] static TransportCapabilities capabilities_for(
      const SocketCanOptions& options) noexcept;
  [[nodiscard]] TransportKind kind() const noexcept override;
  [[nodiscard]] const TransportCapabilities& capabilities() const noexcept override;
  [[nodiscard]] bool is_open() const noexcept override;
  bool open() noexcept override;
  void close() noexcept override;
  [[nodiscard]] TransportResult try_receive(RawCanFrame& frame) noexcept override;
  [[nodiscard]] TransportResult try_send(const RawCanFrame& frame) noexcept override;
  [[nodiscard]] TransportStats stats() const noexcept override;

 private:
  SocketCanOptions options_;
  TransportCapabilities capabilities_;
  int socket_fd_{-1};
  TransportStats stats_{};
  // True as soon as any configured filter carries a frame_type. SocketCAN's
  // kernel filters cannot express frame_type (Classic vs FD), so when this
  // is set, try_receive() re-applies the frame_type half of every such
  // filter in software after the kernel has already applied the id/mask
  // half. This keeps `supports_filters == true` honest instead of silently
  // failing open() for a filter set the hardware only partially supports.
  bool has_software_frame_type_filters_{false};
};

}  // namespace mech::mech_control_core
