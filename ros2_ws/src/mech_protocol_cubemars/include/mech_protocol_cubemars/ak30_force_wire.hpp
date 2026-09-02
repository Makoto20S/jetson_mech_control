#pragma once

#include <array>
#include <cstdint>

namespace mech::mech_protocol_cubemars {

// Force control is control mode ID 8 (L07 section 4.2). Its three sub-modes
// share this one ID and are distinguished by payload content.
inline constexpr std::uint32_t kForceControlModeId = 8U;
// Periodic feedback function ID (L07 section 4.3.1).
inline constexpr std::uint32_t kFeedbackFunctionId = 0x29U;
inline constexpr std::size_t kForceControlPayloadBytes = 8U;

using ForceControlPayload = std::array<std::uint8_t, kForceControlPayloadBytes>;

// Per-model normalization limits from the force-control parameter table,
// L07 p.37. All signed quantities are symmetric about zero in that table.
struct ForceControlRanges final {
  double position_max_rad{12.56};
  double velocity_max_rad_s{40.0};
  double torque_max_nm{15.0};
  double kp_max{500.0};
  double kd_max{5.0};

  [[nodiscard]] constexpr bool is_valid() const noexcept {
    return position_max_rad > 0.0 && velocity_max_rad_s > 0.0 &&
           torque_max_nm > 0.0 && kp_max > 0.0 && kd_max > 0.0;
  }
};

// The project's motor. Position, Kp and Kd are merged cells spanning every
// model in the source table.
[[nodiscard]] constexpr ForceControlRanges ake60_8_ranges() noexcept {
  return ForceControlRanges{12.56, 40.0, 15.0, 500.0, 5.0};
}

// Not a motor we own. L07 section 4.4's worked examples are stated to use
// AK10-9 constants, so tests that cite the manual need these to decode its
// example rows correctly. Never use these for a real device.
[[nodiscard]] constexpr ForceControlRanges ak10_9_ranges() noexcept {
  return ForceControlRanges{12.56, 28.0, 54.0, 500.0, 5.0};
}

// A force-control command in device-native units. All five fields ride every
// frame; a sub-mode is expressed by which of them are non-zero, never by a
// different frame or identifier.
struct ForceControlCommand final {
  double kp{0.0};
  double kd{0.0};
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  double torque_nm{0.0};
};

[[nodiscard]] constexpr std::uint32_t force_control_can_id(
    std::uint8_t drive_id) noexcept {
  return (kForceControlModeId << 8U) | static_cast<std::uint32_t>(drive_id);
}

[[nodiscard]] constexpr std::uint32_t feedback_can_id(
    std::uint8_t drive_id) noexcept {
  return (kFeedbackFunctionId << 8U) | static_cast<std::uint32_t>(drive_id);
}

// Maps a value onto an unsigned field of `bits` width by truncation.
//
// The divisor is (1 << bits) - 1, NOT (1 << bits). L07 prints a
// float_to_uint() that uses (1 << bits); that formula reproduces none of the
// manual's own worked examples, and at position maximum it yields 65536, which
// overflows the 16-bit field to zero and decodes as minimum position. See the
// package README.
[[nodiscard]] std::uint32_t quantize(double value, double minimum,
                                     double maximum, unsigned bits) noexcept;

[[nodiscard]] double dequantize(std::uint32_t raw, double minimum,
                                double maximum, unsigned bits) noexcept;

// Packs a command into the 8-byte payload. Returns false, leaving `payload`
// untouched, when `ranges` is degenerate or any field is non-finite or outside
// its range. Deliberately does not clamp: the vendor reference clamps, which
// turns an invalid command into a valid-looking one.
[[nodiscard]] bool encode_force_control(const ForceControlCommand& command,
                                        const ForceControlRanges& ranges,
                                        ForceControlPayload& payload) noexcept;

}  // namespace mech::mech_protocol_cubemars
