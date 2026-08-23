#pragma once

#include <cstdint>

#include "mech_control_core/config.hpp"
#include "mech_control_core/frame.hpp"

namespace mech::mech_control_core {

enum class TransportResult : std::uint8_t {
  Ok,
  WouldBlock,
  Disconnected,
  QueueFull,
  Invalid,
  Fault,
};

struct TransportStats final {
  std::uint64_t rx_frames{0U};
  std::uint64_t tx_frames{0U};
  std::uint64_t rx_dropped{0U};
  std::uint64_t tx_dropped{0U};
  std::uint64_t queue_full{0U};
  std::uint64_t errors{0U};
};

class Transport {
 public:
  virtual ~Transport() = default;

  [[nodiscard]] virtual TransportKind kind() const noexcept = 0;
  [[nodiscard]] virtual const TransportCapabilities& capabilities()
      const noexcept = 0;
  [[nodiscard]] virtual bool is_open() const noexcept = 0;
  virtual bool open() noexcept = 0;
  virtual void close() noexcept = 0;
  [[nodiscard]] virtual TransportResult try_receive(
      RawCanFrame& frame) noexcept = 0;
  [[nodiscard]] virtual TransportResult try_send(
      const RawCanFrame& frame) noexcept = 0;
  [[nodiscard]] virtual TransportStats stats() const noexcept = 0;
};

}  // namespace mech::mech_control_core
