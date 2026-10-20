#include "mech_protocol_cubemars/ak30_servo_wire.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mech::mech_protocol_cubemars {
namespace {
using mech_control_core::CanFrameFormat;
using mech_control_core::CanFrameType;
using mech_control_core::CanId;
using mech_control_core::FrameDirection;
using mech_control_core::RawCanFrame;

[[nodiscard]] bool valid_drive_id(int drive_id) noexcept {
  return drive_id >= 0 && drive_id <= 255;
}

[[nodiscard]] bool make_frame(int drive_id, std::uint32_t mode,
                              std::uint8_t length,
                              const std::array<std::uint8_t,
                                  mech_control_core::kMaxCanPayloadBytes>& payload,
                              std::uint16_t logical_bus,
                              mech_control_core::MonotonicTime now,
                              RawCanFrame& output) noexcept {
  if (!valid_drive_id(drive_id)) return false;
  const auto id = CanId::create((mode << 8U) |
      static_cast<std::uint32_t>(drive_id), CanFrameFormat::Extended);
  if (!id) return false;
  const auto frame = RawCanFrame::create(logical_bus, *id,
      CanFrameType::Classic, FrameDirection::Tx, length, payload, now);
  if (!frame) return false;
  output = *frame;
  return true;
}

void write_be16(std::uint16_t value,
                std::array<std::uint8_t,
                    mech_control_core::kMaxCanPayloadBytes>& payload,
                std::size_t offset) noexcept {
  payload[offset] = static_cast<std::uint8_t>(value >> 8U);
  payload[offset + 1U] = static_cast<std::uint8_t>(value);
}

[[nodiscard]] int read_be16(std::uint8_t high, std::uint8_t low) noexcept {
  const int raw = (static_cast<int>(high) << 8) | static_cast<int>(low);
  return raw >= 0x8000 ? raw - 0x10000 : raw;
}
}  // namespace

bool encode_servo_position_speed(int drive_id,
                                 const ServoPositionSpeedCommand& command,
                                 std::uint16_t logical_bus,
                                 mech_control_core::MonotonicTime now,
                                 RawCanFrame& output) noexcept {
  // L07 p34-35 documents +/-36000 degrees; scaling this maximum by 10000
  // gives 360,000,000, safely inside int32. The representability check stays
  // explicit because it is a separate wire constraint.
  if (!valid_drive_id(drive_id) || !std::isfinite(command.target_deg) ||
      command.target_deg < -36000.0 || command.target_deg > 36000.0 ||
      !std::isfinite(command.speed_erpm) ||
      !std::isfinite(command.acceleration_raw) ||
      command.speed_erpm <= 0.0 || command.speed_erpm > 327670.0 ||
      command.acceleration_raw <= 0.0 ||
      command.acceleration_raw > 327670.0) return false;

  const double scaled_position = std::trunc(command.target_deg * 10000.0);
  const double scaled_speed = std::trunc(command.speed_erpm / 10.0);
  const double scaled_acceleration = std::trunc(command.acceleration_raw / 10.0);
  if (scaled_position < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
      scaled_position > static_cast<double>(std::numeric_limits<std::int32_t>::max()) ||
      scaled_speed < 1.0 || scaled_speed > 32767.0 ||
      scaled_acceleration < 1.0 || scaled_acceleration > 32767.0) return false;

  const auto position = static_cast<std::uint32_t>(
      static_cast<std::int32_t>(scaled_position));
  const auto speed = static_cast<std::uint16_t>(scaled_speed);
  const auto acceleration = static_cast<std::uint16_t>(scaled_acceleration);
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  payload[0] = static_cast<std::uint8_t>(position >> 24U);
  payload[1] = static_cast<std::uint8_t>(position >> 16U);
  payload[2] = static_cast<std::uint8_t>(position >> 8U);
  payload[3] = static_cast<std::uint8_t>(position);
  write_be16(speed, payload, 4U);
  write_be16(acceleration, payload, 6U);
  return make_frame(drive_id, 6U, 8U, payload, logical_bus, now, output);
}

bool encode_servo_disable(int drive_id, std::uint16_t logical_bus,
                          mech_control_core::MonotonicTime now,
                          RawCanFrame& output) noexcept {
  const std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  return make_frame(drive_id, 15U, 0U, payload, logical_bus, now, output);
}

bool decode_servo_feedback(int configured_id, const RawCanFrame& frame,
                           ServoFeedback& output) noexcept {
  if (!valid_drive_id(configured_id) || !frame.is_valid() ||
      frame.id.format != CanFrameFormat::Extended ||
      frame.id.value != ((0x29U << 8U) | static_cast<std::uint32_t>(configured_id)) ||
      frame.type != CanFrameType::Classic ||
      frame.direction != FrameDirection::Rx || frame.error_frame ||
      frame.remote_request || frame.bitrate_switch || frame.payload_size != 8U)
    return false;

  ServoFeedback decoded{};
  decoded.position_deg = static_cast<double>(
      read_be16(frame.payload[0], frame.payload[1])) * 0.1;
  decoded.electrical_speed_erpm = static_cast<double>(
      read_be16(frame.payload[2], frame.payload[3])) * 10.0;
  decoded.current_iq_a = static_cast<double>(
      read_be16(frame.payload[4], frame.payload[5])) * 0.01;
  decoded.board_temperature_c = frame.payload[6] < 0x80U
      ? static_cast<int>(frame.payload[6])
      : static_cast<int>(frame.payload[6]) - 256;
  decoded.raw_status = frame.payload[7];
  if (decoded.raw_status == 0U) decoded.status = ServoFeedbackStatus::Normal;
  else if (decoded.raw_status <= 7U) decoded.status = ServoFeedbackStatus::KnownFault;
  else if (decoded.raw_status == 0x77U)
    decoded.status = ServoFeedbackStatus::DisableAcknowledged;
  else decoded.status = ServoFeedbackStatus::Unknown;
  output = decoded;
  return true;
}

}  // namespace mech::mech_protocol_cubemars
