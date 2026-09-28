#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "mech_control_core/usb_cdc_transport.hpp"

namespace mech::mech_control_core {

// Linux terminal-device implementation for USB CDC-ACM adapters such as
// /dev/ttyACM0. The descriptor is non-blocking and owned by this object.
class PosixCdcSerialPort final : public CdcSerialPort {
 public:
  explicit PosixCdcSerialPort(std::string device_path) noexcept;
  ~PosixCdcSerialPort() override;

  PosixCdcSerialPort(const PosixCdcSerialPort&) = delete;
  PosixCdcSerialPort& operator=(const PosixCdcSerialPort&) = delete;

  [[nodiscard]] bool is_open() const noexcept override;
  bool open() noexcept override;
  void close() noexcept override;
  [[nodiscard]] TransportResult read_some(
      std::uint8_t* data, std::size_t capacity,
      std::size_t& size) noexcept override;
  [[nodiscard]] TransportResult write_all(
      const std::uint8_t* data, std::size_t size) noexcept override;

  [[nodiscard]] bool send_pass_through_init() noexcept;

 private:
  std::string device_path_;
  int fd_{-1};
};

}  // namespace mech::mech_control_core
