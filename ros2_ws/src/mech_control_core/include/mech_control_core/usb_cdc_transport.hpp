#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "mech_control_core/transport.hpp"

namespace mech::mech_control_core {

class CdcSerialPort {
 public:
  virtual ~CdcSerialPort() = default;
  [[nodiscard]] virtual bool is_open() const noexcept = 0;
  virtual bool open() noexcept = 0;
  virtual void close() noexcept = 0;
  [[nodiscard]] virtual TransportResult read_some(
      std::uint8_t* data, std::size_t capacity, std::size_t& size) noexcept = 0;
  [[nodiscard]] virtual TransportResult write_all(
      const std::uint8_t* data, std::size_t size) noexcept = 0;
};

struct CdcProtocolVersion final {
  std::uint8_t major{0U};
  std::uint8_t minor{0U};
  std::uint8_t patch{0U};
};

struct UsbCdcOptions final {
  std::uint16_t logical_bus{0U};
  std::uint32_t nominal_bitrate_hz{0U};
  std::size_t receive_queue_capacity{64U};
  std::size_t serial_read_capacity{1024U};
  std::optional<CdcProtocolVersion> verified_board_version;
};

struct CdcFrameBatch final {
  std::array<RawCanFrame, 64U> frames{};
  std::size_t size{0U};
};

class UsbCdcCodec final {
 public:
  static constexpr std::uint8_t kHeader = 0xF7U;
  static constexpr std::uint8_t kPassCommand = 0x12U;
  static constexpr std::size_t kMaxPayload = 512U;
  static constexpr CdcProtocolVersion kMinimumBoardVersion{4U, 8U, 8U};

  [[nodiscard]] static bool supports_version(CdcProtocolVersion version) noexcept;

  [[nodiscard]] static bool encode(const RawCanFrame& frame,
                                   std::array<std::uint8_t, 528U>& output,
                                   std::size_t& size) noexcept;
  [[nodiscard]] static bool decode(const std::uint8_t* data, std::size_t size,
                                   std::uint16_t logical_bus,
                                   MonotonicTime host_arrival,
                                   CdcFrameBatch& output) noexcept;
};

// Controlled USB-CDC backend. The serial port is injected, making all framing
// tests independent of /dev/ttyACM* and keeping device ownership outside core.
class UsbCdcTransport final : public Transport {
 public:
  UsbCdcTransport(CdcSerialPort& serial, UsbCdcOptions options);

  [[nodiscard]] TransportKind kind() const noexcept override;
  [[nodiscard]] const TransportCapabilities& capabilities() const noexcept override;
  [[nodiscard]] bool is_open() const noexcept override;
  bool open() noexcept override;
  void close() noexcept override;
  [[nodiscard]] TransportResult try_receive(RawCanFrame& frame) noexcept override;
  [[nodiscard]] TransportResult try_send(const RawCanFrame& frame) noexcept override;
  [[nodiscard]] TransportStats stats() const noexcept override;

 private:
  [[nodiscard]] TransportResult fill_rx() noexcept;
  CdcSerialPort& serial_;
  UsbCdcOptions options_;
  TransportCapabilities capabilities_;
  std::array<std::uint8_t, 1024U> input_{};
  std::array<std::uint8_t, 4096U> pending_{};
  std::size_t pending_size_{0U};
  std::deque<RawCanFrame> rx_;
  TransportStats stats_{};
};

}  // namespace mech::mech_control_core
