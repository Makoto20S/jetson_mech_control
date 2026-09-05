#include "mech_protocol_cubemars/ak30_force_wire.hpp"

#include <cmath>
#include <cstdint>

namespace mech::mech_protocol_cubemars {
namespace {

[[nodiscard]] bool within(double value, double low, double high) noexcept {
  return std::isfinite(value) && value >= low && value <= high;
}

}  // namespace

std::uint32_t quantize(double value, double minimum, double maximum,
                       unsigned bits) noexcept {
  if (bits == 0U || bits > 31U) {
    return 0U;
  }
  const double span = maximum - minimum;
  if (!std::isfinite(span) || !(span > 0.0) || !std::isfinite(value)) {
    return 0U;
  }
  const std::uint32_t max_raw = (1U << bits) - 1U;
  if (value < minimum) {
    value = minimum;
  } else if (value > maximum) {
    value = maximum;
  }
  const double scaled = (value - minimum) * (static_cast<double>(max_raw) / span);
  if (scaled <= 0.0) {
    return 0U;
  }
  if (scaled >= static_cast<double>(max_raw)) {
    return max_raw;
  }
  // Truncation, not rounding: it is what reproduces the manual's example table.
  return static_cast<std::uint32_t>(scaled);
}

double dequantize(std::uint32_t raw, double minimum, double maximum,
                  unsigned bits) noexcept {
  if (bits == 0U || bits > 31U) {
    return minimum;
  }
  const std::uint32_t max_raw = (1U << bits) - 1U;
  if (raw > max_raw) {
    raw = max_raw;
  }
  return minimum + (static_cast<double>(raw) / static_cast<double>(max_raw)) *
                       (maximum - minimum);
}

bool encode_force_control(const ForceControlCommand& command,
                          const ForceControlRanges& ranges,
                          ForceControlPayload& payload) noexcept {
  if (!ranges.is_valid()) {
    return false;
  }
  if (!within(command.position_rad, -ranges.position_max_rad,
              ranges.position_max_rad) ||
      !within(command.velocity_rad_s, -ranges.velocity_max_rad_s,
              ranges.velocity_max_rad_s) ||
      !within(command.torque_nm, -ranges.torque_max_nm, ranges.torque_max_nm) ||
      !within(command.kp, 0.0, ranges.kp_max) ||
      !within(command.kd, 0.0, ranges.kd_max)) {
    return false;
  }

  const std::uint32_t kp = quantize(command.kp, 0.0, ranges.kp_max, 12U);
  const std::uint32_t kd = quantize(command.kd, 0.0, ranges.kd_max, 12U);
  const std::uint32_t position =
      quantize(command.position_rad, -ranges.position_max_rad,
               ranges.position_max_rad, 16U);
  const std::uint32_t velocity =
      quantize(command.velocity_rad_s, -ranges.velocity_max_rad_s,
               ranges.velocity_max_rad_s, 12U);
  const std::uint32_t torque = quantize(
      command.torque_nm, -ranges.torque_max_nm, ranges.torque_max_nm, 12U);

  // Field order is KP KD POS VEL TRQ. AK2.0 MIT and the widely copied Arduino
  // demos use POS VEL KP KD TRQ on an 11-bit standard frame; a packing helper
  // must never be shared between the two.
  payload[0] = static_cast<std::uint8_t>(kp >> 4U);
  payload[1] = static_cast<std::uint8_t>(((kp & 0x0FU) << 4U) | (kd >> 8U));
  payload[2] = static_cast<std::uint8_t>(kd & 0xFFU);
  payload[3] = static_cast<std::uint8_t>(position >> 8U);
  payload[4] = static_cast<std::uint8_t>(position & 0xFFU);
  payload[5] = static_cast<std::uint8_t>(velocity >> 4U);
  payload[6] =
      static_cast<std::uint8_t>(((velocity & 0x0FU) << 4U) | (torque >> 8U));
  payload[7] = static_cast<std::uint8_t>(torque & 0xFFU);
  return true;
}

namespace {

[[nodiscard]] std::int16_t big_endian_int16(std::uint8_t high,
                                            std::uint8_t low) noexcept {
  const auto raw = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(high) << 8U) | static_cast<std::uint16_t>(low));
  return static_cast<std::int16_t>(raw);
}

}  // namespace

void decode_feedback(const ForceControlPayload& payload,
                     ForceControlFeedback& output) noexcept {
  output.position_deg =
      static_cast<double>(big_endian_int16(payload[0], payload[1])) * 0.1;
  output.electrical_speed_erpm =
      static_cast<double>(big_endian_int16(payload[2], payload[3])) * 10.0;
  output.current_iq_a =
      static_cast<double>(big_endian_int16(payload[4], payload[5])) * 0.01;
  output.board_temperature_c =
      static_cast<double>(static_cast<std::int8_t>(payload[6]));
  output.raw_status = payload[7];
}

}  // namespace mech::mech_protocol_cubemars
