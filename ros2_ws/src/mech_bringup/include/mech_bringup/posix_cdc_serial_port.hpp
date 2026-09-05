#pragma once

#include <cstdint>
#include <string>

#include "mech_control_core/usb_cdc_transport.hpp"

namespace mech::mech_bringup {

// CdcSerialPort over a real Linux terminal device (/dev/ttyACM*). This is the
// one piece the Foundation deliberately left as an injected interface: the
// production ros2_control slice will own its construction policy. The probe
// keeps it deliberately small and single-threaded, matching the transport's
// single-driver-thread contract.
class PosixCdcSerialPort final : public mech_control_core::CdcSerialPort {
 public:
  explicit PosixCdcSerialPort(std::string device_path) noexcept;

  ~PosixCdcSerialPort() override;

  PosixCdcSerialPort(const PosixCdcSerialPort&) = delete;
  PosixCdcSerialPort& operator=(const PosixCdcSerialPort&) = delete;

  [[nodiscard]] bool is_open() const noexcept override;
  bool open() noexcept override;
  void close() noexcept override;
  [[nodiscard]] mech_control_core::TransportResult read_some(
      std::uint8_t* data, std::size_t capacity,
      std::size_t& size) noexcept override;
  [[nodiscard]] mech_control_core::TransportResult write_all(
      const std::uint8_t* data, std::size_t size) noexcept override;

  // The vendor pass-through init frame: MODE_FDCAN_PASS (0x12) with a
  // six-byte zero payload, i.e. send_flag=0. The vendor's own fdcan_init()
  // sends exactly these bytes (its config member is never copied into the
  // payload, so cfg=0x00 is what actually ships, and 0x00 vs 0x07 measured
  // identical on the bench). Must be called after open() and before the first
  // try_send/try_receive; the transport itself does not send it.
  [[nodiscard]] bool send_pass_through_init() noexcept;

 private:
  std::string device_path_;
  int fd_{-1};
};

}  // namespace mech::mech_bringup
