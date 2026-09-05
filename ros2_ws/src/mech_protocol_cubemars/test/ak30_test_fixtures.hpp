#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "mech_control_core/transport.hpp"
#include "mech_protocol_cubemars/ak30_force_session.hpp"

namespace mech::mech_protocol_cubemars::testing {

// motor1: drive id 104 decimal = 0x68, so command 0x0868 and feedback 0x2968.
inline constexpr std::uint16_t kDriveId = 104U;
inline constexpr std::uint32_t kCommandId = 0x0868U;
inline constexpr std::uint32_t kFeedbackId = 0x2968U;
inline constexpr std::uint16_t kLogicalBus = 0U;

// A transport double with an injectable send result. A double that reproduces
// only the success path is how the RC shipped a defect that permanently
// faulted the bus on a transient WouldBlock: no fake ever produced it.
class RecordingTransport final : public mech_control_core::Transport {
 public:
  explicit RecordingTransport(
      mech_control_core::TransportCapabilities capabilities) noexcept
      : capabilities_(capabilities) {}

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
      mech_control_core::RawCanFrame&) noexcept override {
    return mech_control_core::TransportResult::WouldBlock;
  }
  [[nodiscard]] mech_control_core::TransportResult try_send(
      const mech_control_core::RawCanFrame& frame) noexcept override {
    if (next_send_result_ != mech_control_core::TransportResult::Ok) {
      return next_send_result_;
    }
    sent_.push_back(frame);
    return mech_control_core::TransportResult::Ok;
  }
  [[nodiscard]] mech_control_core::TransportStats stats() const noexcept override {
    return mech_control_core::TransportStats{};
  }

  void inject_send_result(mech_control_core::TransportResult result) noexcept {
    next_send_result_ = result;
  }
  [[nodiscard]] const std::vector<mech_control_core::RawCanFrame>& sent()
      const noexcept {
    return sent_;
  }

 private:
  mech_control_core::TransportCapabilities capabilities_;
  std::vector<mech_control_core::RawCanFrame> sent_;
  mech_control_core::TransportResult next_send_result_{
      mech_control_core::TransportResult::Ok};
  bool open_{false};
};

[[nodiscard]] inline mech_control_core::TransportCapabilities
classic_extended_capabilities() {
  mech_control_core::TransportCapabilities capabilities{};
  capabilities.supports_classic_can = true;
  capabilities.supports_extended_frames = true;
  capabilities.supports_non_blocking_io = true;
  capabilities.max_payload_bytes = 8U;
  capabilities.queue_capacity = 16U;
  // Bitrate stays unverified with value zero: this is a fake channel and has
  // no bitrate to read back. ADR-012 makes that a legal state.
  return capabilities;
}

[[nodiscard]] inline Ak30Mapping fully_verified_mapping() {
  Ak30Mapping mapping{};
  mapping.pole_pairs = {14.0, true};
  mapping.gear_ratio = {8.0, true};
  mapping.zero_offset_rad = {5.760604931781636, true};
  mapping.direction_sign = {1.0, true};
  mapping.torque_constant_nm_per_a = {0.7382, true};
  mapping.position_source_known = true;
  mapping.position_is_output_shaft = true;
  return mapping;
}

[[nodiscard]] inline mech_control_core::DeviceConfig valid_device_config() {
  mech_control_core::DeviceConfig config{};
  config.device_id = 1U;
  config.name = "motor1";
  config.logical_bus = kLogicalBus;
  config.profile = mech_control_core::ProtocolProfile::Ak30ForceControlExtended;
  config.frame_type = mech_control_core::CanFrameType::Classic;
  config.frame_format = mech_control_core::CanFrameFormat::Extended;
  config.command_id = mech_control_core::CanId::create(
      kCommandId, mech_control_core::CanFrameFormat::Extended);
  config.feedback_id = mech_control_core::CanId::create(
      kFeedbackId, mech_control_core::CanFrameFormat::Extended);
  config.command_payload_bytes = 8U;
  config.feedback_payload_bytes = 8U;
  config.writable = true;
  return config;
}

[[nodiscard]] inline Ak30SessionConfig valid_session_config() {
  Ak30SessionConfig config{};
  config.drive_id = kDriveId;
  config.sub_mode = ForceControlSubMode::Torque;
  config.mapping = fully_verified_mapping();
  config.gains = ForceControlGains{};
  // Operator-asserted encoded firmware version; motor1 displays AKE60_8_DE_V3.4.
  config.firmware_id = 0x0304U;
  config.firmware_id_min = 0x0300U;
  config.firmware_id_max = 0x03FFU;
  config.command_ttl_nanoseconds = 4000000;
  config.command_hard_ttl_nanoseconds = 6000000;
  config.feedback_ttl_nanoseconds = 6000000;
  return config;
}

[[nodiscard]] inline mech_control_core::MonotonicTime at(
    std::int64_t nanoseconds) {
  return mech_control_core::MonotonicTime::from_nanoseconds(nanoseconds).value();
}

// position 900 -> 90.0 deg, speed 1000 -> 10000 ERPM, Iq 200 -> 2.0 A, 40 C.
[[nodiscard]] inline std::array<std::uint8_t, 64U> feedback_payload(
    std::uint8_t status) {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03U;
  payload[1] = 0x84U;
  payload[2] = 0x03U;
  payload[3] = 0xE8U;
  payload[4] = 0x00U;
  payload[5] = 0xC8U;
  payload[6] = 0x28U;
  payload[7] = status;
  return payload;
}

[[nodiscard]] inline mech_control_core::RawCanFrame feedback_frame(
    std::uint8_t status, mech_control_core::MonotonicTime arrival) {
  return mech_control_core::RawCanFrame::create(
             kLogicalBus,
             mech_control_core::CanId::create(
                 kFeedbackId, mech_control_core::CanFrameFormat::Extended)
                 .value(),
             mech_control_core::CanFrameType::Classic,
             mech_control_core::FrameDirection::Rx, 8U,
             feedback_payload(status), arrival)
      .value();
}

}  // namespace mech::mech_protocol_cubemars::testing
