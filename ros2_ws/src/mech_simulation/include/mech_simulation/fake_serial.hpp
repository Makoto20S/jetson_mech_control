#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <algorithm>
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

 private:
  std::size_t capacity_;
  bool open_{false};
  std::deque<std::uint8_t> incoming_;
  std::deque<std::uint8_t> outgoing_;
};

}  // namespace mech::mech_simulation
