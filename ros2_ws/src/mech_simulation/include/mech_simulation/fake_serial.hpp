#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <algorithm>
#include <optional>
#include <vector>

#include "mech_control_core/usb_cdc_transport.hpp"

namespace mech::mech_simulation {

class FakeSerial final : public mech_control_core::CdcSerialPort {
 public:
  explicit FakeSerial(std::size_t capacity = 4096U) : capacity_(capacity) {}

  [[nodiscard]] bool is_open() const noexcept override { return open_; }
  bool open() noexcept override {
    open_ = true;
    return true;
  }
  void close() noexcept override { open_ = false; }

  [[nodiscard]] mech_control_core::TransportResult read_some(
      std::uint8_t* data, std::size_t capacity,
      std::size_t& size) noexcept override {
    size = 0U;
    if (!open_) return mech_control_core::TransportResult::Disconnected;
    if (forced_read_result_.has_value()) {
      const auto result = *forced_read_result_;
      // A forced result is consumed once, so a test can script a single
      // anomalous read (e.g. Ok-with-zero-size) followed by normal behaviour.
      forced_read_result_.reset();
      if (result == mech_control_core::TransportResult::Ok) {
        const auto drained =
            std::min({forced_read_size_, capacity, incoming_.size()});
        for (std::size_t index = 0U; index < drained; ++index) {
          data[index] = incoming_.front();
          incoming_.pop_front();
        }
        // In "lie about size" mode, report the requested size even if it
        // exceeds what was actually available/drained. This models a
        // misbehaving driver and lets a test deterministically exercise the
        // transport's defence against a read_some() that over-reports.
        size = force_read_size_exceeds_capacity_ ? forced_read_size_ : drained;
      }
      return result;
    }
    if (incoming_.empty()) return mech_control_core::TransportResult::WouldBlock;
    const auto count = std::min(capacity, incoming_.size());
    for (std::size_t index = 0U; index < count; ++index) {
      data[index] = incoming_.front();
      incoming_.pop_front();
    }
    size = count;
    return mech_control_core::TransportResult::Ok;
  }

  [[nodiscard]] mech_control_core::TransportResult write_all(
      const std::uint8_t* data, std::size_t size) noexcept override {
    if (!open_) return mech_control_core::TransportResult::Disconnected;
    if (forced_write_result_.has_value()) {
      const auto result = *forced_write_result_;
      forced_write_result_.reset();
      if (result == mech_control_core::TransportResult::Ok) {
        outgoing_.insert(outgoing_.end(), data, data + size);
      }
      return result;
    }
    if (outgoing_.size() + size > capacity_) {
      return mech_control_core::TransportResult::QueueFull;
    }
    outgoing_.insert(outgoing_.end(), data, data + size);
    return mech_control_core::TransportResult::Ok;
  }

  [[nodiscard]] bool inject_rx(const std::vector<std::uint8_t>& bytes) noexcept {
    if (incoming_.size() + bytes.size() > capacity_) return false;
    incoming_.insert(incoming_.end(), bytes.begin(), bytes.end());
    return true;
  }

  [[nodiscard]] std::vector<std::uint8_t> take_tx() {
    return {outgoing_.begin(), outgoing_.end()};
  }

  void clear_tx() noexcept { outgoing_.clear(); }

  // Forces the next read_some() call to return `result`. If `result` is Ok,
  // up to `size` bytes are drained from the injected queue (still bounded by
  // the caller's capacity and by what is actually pending) -- this lets a
  // test script "Ok but size == 0", i.e. a benign no-data read.
  //
  // If `report_size_verbatim` is true, the *reported* size is `size` exactly
  // (not clamped to what was actually drained), simulating a driver that
  // over-reports how many bytes it wrote. This is the hook regression tests
  // use to deterministically drive the transport's pending-buffer overflow
  // guard without depending on framing-loop arithmetic.
  void force_next_read(mech_control_core::TransportResult result,
                       std::size_t size = 0U,
                       bool report_size_verbatim = false) noexcept {
    forced_read_result_ = result;
    forced_read_size_ = size;
    force_read_size_exceeds_capacity_ = report_size_verbatim;
  }

  // Forces the next write_all() call to return `result` without touching the
  // outgoing buffer (unless `result` is Ok).
  void force_next_write(mech_control_core::TransportResult result) noexcept {
    forced_write_result_ = result;
  }

 private:
  std::size_t capacity_;
  bool open_{false};
  std::deque<std::uint8_t> incoming_;
  std::deque<std::uint8_t> outgoing_;
  std::optional<mech_control_core::TransportResult> forced_read_result_;
  std::size_t forced_read_size_{0U};
  bool force_read_size_exceeds_capacity_{false};
  std::optional<mech_control_core::TransportResult> forced_write_result_;
};

}  // namespace mech::mech_simulation
