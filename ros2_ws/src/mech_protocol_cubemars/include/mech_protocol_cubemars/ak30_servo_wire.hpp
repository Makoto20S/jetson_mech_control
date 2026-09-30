#pragma once

#include <cstdint>

#include "mech_control_core/frame.hpp"

namespace mech::mech_protocol_cubemars {

// L07 section 4.1.7, p34-35. Inputs are raw device units, with no shaft or
// canonical joint mapping implied. The manual labels acceleration ERPM/s²;
// its physical interpretation is not established for the custom device.
struct ServoPositionSpeedCommand final {
  double target_deg{0.0};
  double speed_erpm{0.0};
  double acceleration_raw{0.0};
};

// Encodes only mode 6. Returns false without changing output on invalid input.
// Address is an integer so values outside the documented 8-bit drive-ID field
// can be rejected rather than silently truncated.
[[nodiscard]] bool encode_servo_position_speed(
    int drive_id, const ServoPositionSpeedCommand& command,
    std::uint16_t logical_bus, mech_control_core::MonotonicTime now,
    mech_control_core::RawCanFrame& output) noexcept;

// L07 section 4.1.8 p35-36: mode 15 DLC0 requests torque disable. A 0x77
// feedback status acknowledges it; neither is a controlled stopping guarantee.
// This is a separate explicit operation and is never called automatically.
[[nodiscard]] bool encode_servo_disable(
    int drive_id, std::uint16_t logical_bus, mech_control_core::MonotonicTime now,
    mech_control_core::RawCanFrame& output) noexcept;

enum class ServoFeedbackStatus : std::uint8_t {
  Normal,
  KnownFault,
  DisableAcknowledged,
  Unknown,
};

// L07 section 4.3.1 p41-42. No encoder source, shaft, origin, control mode,
// torque, or wrap interpretation is implied by these raw fields.
struct ServoFeedback final {
  double position_deg{0.0};
  double electrical_speed_erpm{0.0};
  double current_iq_a{0.0};
  int board_temperature_c{0};
  std::uint8_t raw_status{0U};
  ServoFeedbackStatus status{ServoFeedbackStatus::Unknown};

  [[nodiscard]] bool usable_position_sample() const noexcept {
    return status == ServoFeedbackStatus::Normal;
  }
};

// Accepts only this configured drive's extended Classic 0x29 Rx data DLC8.
// Invalid frames leave output untouched. Disable ack is classified separately.
[[nodiscard]] bool decode_servo_feedback(
    int configured_id, const mech_control_core::RawCanFrame& frame,
    ServoFeedback& output) noexcept;

}  // namespace mech::mech_protocol_cubemars
