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
};

}  // namespace mech::mech_control_core
