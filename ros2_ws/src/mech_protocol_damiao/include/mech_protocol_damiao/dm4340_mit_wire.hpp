#pragma once

#include <array>
#include <cstdint>

#include "mech_control_core/frame.hpp"

namespace mech::mech_protocol_damiao {

struct MitLimits final {
  double position_rad;
  double velocity_rad_s;
  double torque_nm;
};

struct MitCommand final {
  double position_rad;
  double velocity_rad_s;
  double kp;
  double kd;
  double torque_nm;
};

struct MitFeedback final {
  double position_rad;
  double velocity_rad_s;
  double torque_nm;
  std::uint8_t raw_fault;  // Vendor ERR nibble: 0=disabled, 1=enabled, 8..=faults.
  std::uint8_t raw_motor_id;
  std::uint8_t raw_byte_6;
  std::uint8_t raw_byte_7;
};

[[nodiscard]] constexpr bool feedback_status_ok(std::uint8_t status) noexcept {
  return status == 0U || status == 1U;
}

[[nodiscard]] constexpr bool feedback_motor_enabled(std::uint8_t status) noexcept {
  return status == 1U;
}

// These are SDK reference ranges, not verified limits of the attached motor.
inline constexpr MitLimits kSdkDm4340Limits{12.5, 10.0, 28.0};
inline constexpr MitLimits kSdkDm4310Limits{12.5, 30.0, 10.0};

[[nodiscard]] bool valid_limits(const MitLimits& limits) noexcept;

// Invalid or out-of-range commands fail; the wire layer never silently clamps.
[[nodiscard]] bool encode_mit(const MitCommand& command,
                              const MitLimits& limits,
                              std::array<std::uint8_t, 8>& payload) noexcept;

// The caller supplies the configured feedback CAN ID and motor ID explicitly.
[[nodiscard]] bool decode_mit_feedback(
    const mech_control_core::RawCanFrame& frame, std::uint16_t logical_bus,
    std::uint32_t feedback_can_id, std::uint8_t motor_id,
    const MitLimits& limits, MitFeedback& feedback) noexcept;

}  // namespace mech::mech_protocol_damiao
