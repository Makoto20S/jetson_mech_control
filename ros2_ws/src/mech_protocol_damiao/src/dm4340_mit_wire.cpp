#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

#include <cmath>
#include <cstdint>

namespace mech::mech_protocol_damiao {
namespace {

bool in_range(double value, double minimum, double maximum) noexcept {
  return std::isfinite(value) && value >= minimum && value <= maximum;
}

std::uint16_t quantize(double value, double minimum, double maximum,
                       std::uint32_t maximum_code) noexcept {
  return static_cast<std::uint16_t>(
      ((value - minimum) / (maximum - minimum)) * maximum_code);
}

double expand(std::uint16_t code, double minimum, double maximum,
              std::uint32_t maximum_code) noexcept {
  return static_cast<double>(code) * (maximum - minimum) / maximum_code +
         minimum;
}

}  // namespace

bool valid_limits(const MitLimits& limits) noexcept {
  return std::isfinite(limits.position_rad) && limits.position_rad > 0.0 &&
         std::isfinite(limits.velocity_rad_s) && limits.velocity_rad_s > 0.0 &&
         std::isfinite(limits.torque_nm) && limits.torque_nm > 0.0;
}

bool encode_mit(const MitCommand& command, const MitLimits& limits,
                std::array<std::uint8_t, 8>& payload) noexcept {
  if (!valid_limits(limits) ||
      !in_range(command.position_rad, -limits.position_rad,
                limits.position_rad) ||
      !in_range(command.velocity_rad_s, -limits.velocity_rad_s,
                limits.velocity_rad_s) ||
      !in_range(command.kp, 0.0, 500.0) ||
      !in_range(command.kd, 0.0, 5.0) ||
      !in_range(command.torque_nm, -limits.torque_nm, limits.torque_nm)) {
    return false;
  }
  const auto p = quantize(command.position_rad, -limits.position_rad,
                          limits.position_rad, 0xFFFFU);
  const auto v = quantize(command.velocity_rad_s, -limits.velocity_rad_s,
                          limits.velocity_rad_s, 0xFFFU);
  const auto kp = quantize(command.kp, 0.0, 500.0, 0xFFFU);
  const auto kd = quantize(command.kd, 0.0, 5.0, 0xFFFU);
  const auto t = quantize(command.torque_nm, -limits.torque_nm,
                          limits.torque_nm, 0xFFFU);
  payload = {static_cast<std::uint8_t>(p >> 8),
             static_cast<std::uint8_t>(p),
             static_cast<std::uint8_t>(v >> 4),
             static_cast<std::uint8_t>(((v & 0xFU) << 4) | (kp >> 8)),
             static_cast<std::uint8_t>(kp),
             static_cast<std::uint8_t>(kd >> 4),
             static_cast<std::uint8_t>(((kd & 0xFU) << 4) | (t >> 8)),
             static_cast<std::uint8_t>(t)};
  return true;
}

bool decode_mit_feedback(const mech_control_core::RawCanFrame& frame,
                         std::uint16_t logical_bus,
                         std::uint32_t feedback_can_id,
                         std::uint8_t motor_id, const MitLimits& limits,
                         MitFeedback& feedback) noexcept {
  using namespace mech_control_core;
  if (!valid_limits(limits) || motor_id > 0xFU ||
      feedback_can_id > kMaxStandardCanId ||
      frame.logical_bus != logical_bus ||
      frame.id.format != CanFrameFormat::Standard ||
      frame.id.value != feedback_can_id ||
      frame.type != CanFrameType::Classic ||
      frame.direction != FrameDirection::Rx || frame.payload_size != 8U ||
      frame.error_frame || frame.remote_request || frame.bitrate_switch) {
    return false;
  }
  const auto& data = frame.payload;
  const auto reported_id = static_cast<std::uint8_t>(data[0] & 0xFU);
  if (reported_id != motor_id) {
    return false;
  }
  const auto p = static_cast<std::uint16_t>((data[1] << 8) | data[2]);
  const auto v = static_cast<std::uint16_t>((data[3] << 4) | (data[4] >> 4));
  const auto t = static_cast<std::uint16_t>(((data[4] & 0xFU) << 8) | data[5]);
  feedback = MitFeedback{expand(p, -limits.position_rad, limits.position_rad,
                                0xFFFFU),
                         expand(v, -limits.velocity_rad_s,
                                limits.velocity_rad_s, 0xFFFU),
                         expand(t, -limits.torque_nm, limits.torque_nm,
                                0xFFFU),
                         static_cast<std::uint8_t>(data[0] >> 4),
                         reported_id, data[6], data[7]};
  return true;
}

}  // namespace mech::mech_protocol_damiao
