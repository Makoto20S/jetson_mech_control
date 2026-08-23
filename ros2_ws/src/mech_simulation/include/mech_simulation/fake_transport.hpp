#pragma once

#include <cstddef>
#include <deque>

#include "mech_control_core/transport.hpp"

namespace mech::mech_simulation {

class FakeTransport final : public mech_control_core::Transport {
 public:
  explicit FakeTransport(
      std::size_t capacity = 16U,
      mech_control_core::TransportCapabilities capabilities =
          default_capabilities())
      : capacity_(capacity), capabilities_(capabilities) {}

  [[nodiscard]] static mech_control_core::TransportCapabilities
  default_capabilities() noexcept {
    return mech_control_core::TransportCapabilities{
        true, true, true, true, true, true, true, false, true, true, 1000000U,
        64U, 16U};
  }

  [[nodiscard]] mech_control_core::TransportKind kind() const noexcept override {
    return mech_control_core::TransportKind::Fake;
  }

  [[nodiscard]] const mech_control_core::TransportCapabilities& capabilities()
      const noexcept override {
    return capabilities_;
  }

  [[nodiscard]] bool is_open() const noexcept override { return open_; }

  bool open() noexcept override {
    open_ = true;
    return true;
  }

  void close() noexcept override { open_ = false; }

  [[nodiscard]] mech_control_core::TransportResult try_receive(
      mech_control_core::RawCanFrame& frame) noexcept override {
    if (!open_) {
      return mech_control_core::TransportResult::Disconnected;
    }
    if (rx_.empty()) {
      return mech_control_core::TransportResult::WouldBlock;
    }
    frame = rx_.front();
    rx_.pop_front();
    ++stats_.rx_frames;
    return mech_control_core::TransportResult::Ok;
  }

  [[nodiscard]] mech_control_core::TransportResult try_send(
      const mech_control_core::RawCanFrame& frame) noexcept override {
    if (!open_) {
      return mech_control_core::TransportResult::Disconnected;
    }
    if (frame.direction != mech_control_core::FrameDirection::Tx ||
        !frame.is_valid()) {
      ++stats_.errors;
      return mech_control_core::TransportResult::Invalid;
    }
    if (tx_.size() >= capacity_) {
      ++stats_.tx_dropped;
      ++stats_.queue_full;
      return mech_control_core::TransportResult::QueueFull;
    }
    tx_.push_back(frame);
    ++stats_.tx_frames;
    return mech_control_core::TransportResult::Ok;
  }

  [[nodiscard]] mech_control_core::TransportStats stats() const noexcept override {
    return stats_;
  }

  [[nodiscard]] mech_control_core::TransportResult inject_receive(
      const mech_control_core::RawCanFrame& frame) noexcept {
    if (!open_) {
      return mech_control_core::TransportResult::Disconnected;
    }
    if (rx_.size() >= capacity_) {
      ++stats_.rx_dropped;
      ++stats_.queue_full;
      return mech_control_core::TransportResult::QueueFull;
    }
    rx_.push_back(frame);
    return mech_control_core::TransportResult::Ok;
  }

  [[nodiscard]] bool take_transmit(mech_control_core::RawCanFrame& frame) noexcept {
    if (tx_.empty()) {
      return false;
    }
    frame = tx_.front();
    tx_.pop_front();
    return true;
  }

  [[nodiscard]] std::size_t pending_receive() const noexcept { return rx_.size(); }
  [[nodiscard]] std::size_t pending_transmit() const noexcept { return tx_.size(); }

 private:
  std::size_t capacity_;
  mech_control_core::TransportCapabilities capabilities_;
  bool open_{false};
  std::deque<mech_control_core::RawCanFrame> rx_;
  std::deque<mech_control_core::RawCanFrame> tx_;
  mech_control_core::TransportStats stats_;
};

}  // namespace mech::mech_simulation
