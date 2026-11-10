#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "mech_control_core/usb_cdc_transport.hpp"

namespace mech::mech_ctrboard_bridge {

using mech_control_core::CdcSerialPort;
using mech_control_core::TransportResult;

class CdcWriteTraceSink {
 public:
  virtual ~CdcWriteTraceSink() = default;
  [[nodiscard]] virtual std::int64_t begin_write() noexcept = 0;
  virtual void record_write(const std::uint8_t* data, std::size_t length,
                            std::int64_t result, std::size_t requested,
                            int error, std::int64_t begin) noexcept = 0;
};

// Linux terminal-device implementation for USB CDC-ACM adapters such as
// /dev/ttyACM0. The descriptor is non-blocking, exclusively locked, and owned
// by this object. The optional sink observes actual kernel writes without
// introducing a dependency from core to a deployment-specific trace type.
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

  void set_write_trace_sink(CdcWriteTraceSink* sink) noexcept {
    if (fd_ < 0) trace_sink_ = sink;
  }

  void set_read_timing_enabled(bool enabled) noexcept {
    if (fd_ < 0) read_timing_enabled_ = enabled;
  }
  [[nodiscard]] std::int64_t last_read_end_ns() const noexcept {
    return last_read_end_ns_;
  }
  [[nodiscard]] std::uint64_t read_clock_errors() const noexcept {
    return read_clock_errors_;
  }

 private:
  bool read_timing_enabled_{false};
  std::int64_t last_read_end_ns_{0};
  std::uint64_t read_clock_errors_{0};
  std::string device_path_;
  int fd_{-1};
  CdcWriteTraceSink* trace_sink_{nullptr};
};

}  // namespace mech::mech_ctrboard_bridge
