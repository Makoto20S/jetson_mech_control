#pragma once

#include <algorithm>
#include "mech_bringup/command_trace.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <ostream>
#include <vector>
#include "mech_control_core/usb_cdc_transport.hpp"

namespace mech::mech_bringup {
// Optional single-owner diagnostic tap. Storage is allocated before activation;
// I/O calls only copy bounded bytes. dump() belongs after runtime shutdown.
// TX success means host serial acceptance, not acknowledgement by the motor.
class SerialTrace final : public mech_control_core::CdcSerialPort {
 public:
  explicit SerialTrace(std::shared_ptr<mech_control_core::CdcSerialPort> port,
                       std::size_t capacity = 32768U, CommandTrace* chain = nullptr,
                       const std::string& snapshot_path = {})
      : port_(std::move(port)), records_(std::max(std::size_t{1}, capacity), snapshot_path, 2),
        chain_(chain) {}
  static constexpr std::size_t record_bytes = 1064;
  void seal_snapshot() noexcept { records_.seal(); }
  // Startup only: mapped buffers must exist before constructing a real port.
  bool attach_port(std::shared_ptr<mech_control_core::CdcSerialPort> port) noexcept {
    if (port_ || !port) return false;
    port_ = std::move(port); return true;
  }
  bool is_open() const noexcept override { return port_->is_open(); }
  bool open() noexcept override { return port_->open(); }
  void close() noexcept override { port_->close(); }
  mech_control_core::TransportResult read_some(
      std::uint8_t* data, std::size_t capacity, std::size_t& size) noexcept override {
    const auto begin = now();
    const auto result = port_->read_some(data, capacity, size);
    if (result != mech_control_core::TransportResult::WouldBlock &&
        (size != 0U || result != mech_control_core::TransportResult::Ok))
      capture(false, begin, result, data, size,
          result == mech_control_core::TransportResult::Ok ? std::min(size, capacity) : 0U);
    return result;
  }
  mech_control_core::TransportResult write_all(
      const std::uint8_t* data, std::size_t size) noexcept override {
    const auto begin = now();
    if (chain_) {
      ++chain_->io;
      chain_->bytes("serial_request", data, size, 0, size, 0, begin);
    }
    const auto result = port_->write_all(data, size);
    if (chain_) chain_->bytes("serial_result", nullptr, 0,
        static_cast<int>(result), size, 0, begin);
    capture(true, begin, result, data, size, size);
    return result;
  }
  void dump(std::ostream& out) const {
    out << "{\"schema_version\":1,\"capacity\":" << records_.size()
        << ",\"total\":" << total_ << ",\"overwritten\":"
        << (total_ > records_.size() ? total_ - records_.size() : 0U) << "}\n";
    const auto count = std::min(total_, static_cast<std::uint64_t>(records_.size()));
    constexpr char digits[] = "0123456789abcdef";
    for (std::uint64_t i = total_ - count; i < total_; ++i) {
      const auto& r = records_[i % records_.size()];
      out << "{\"sequence\":" << i << ",\"direction\":\"" << (r.tx ? "tx" : "rx")
          << "\",\"begin_ns\":" << r.begin << ",\"end_ns\":" << r.end
          << ",\"result\":" << static_cast<unsigned>(r.result)
          << ",\"size\":" << r.size << ",\"captured\":" << r.captured << ",\"hex\":\"";
      for (std::size_t j = 0; j < r.captured; ++j)
        out << digits[r.bytes[j] >> 4U] << digits[r.bytes[j] & 15U];
      out << "\"}\n";
    }
  }
 private:
  struct Record {
    // Snapshot offsets: begin0, end8, size16, captured24, result32,
    // tx33, bytes34..1057, padding1058..1063. Stride1064.
    std::int64_t begin{}, end{};
    std::size_t size{}, captured{};
    mech_control_core::TransportResult result{};
    bool tx{};
    std::array<std::uint8_t, 1024U> bytes{};
  };
  static_assert(sizeof(Record) == record_bytes);
  static_assert(offsetof(Record, result) == 32);
  static_assert(offsetof(Record, bytes) == 34);
  static std::int64_t now() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  void capture(bool tx, std::int64_t begin, mech_control_core::TransportResult result,
               const std::uint8_t* data, std::size_t size, std::size_t readable) noexcept {
    auto& r = records_[total_ % records_.size()];
    r.tx = tx; r.begin = begin; r.end = now(); r.result = result; r.size = size;
    r.captured = data ? std::min(readable, r.bytes.size()) : 0U;
    if (r.captured) std::copy_n(data, r.captured, r.bytes.begin());
    ++total_;
    records_.publish(total_, true);
  }
  std::shared_ptr<mech_control_core::CdcSerialPort> port_;
  TraceRecordStorage<Record> records_;
  std::uint64_t total_{};
  CommandTrace* chain_{nullptr};
};
}  // namespace mech::mech_bringup
