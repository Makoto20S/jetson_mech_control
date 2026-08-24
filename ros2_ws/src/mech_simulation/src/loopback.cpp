#include "mech_simulation/loopback.hpp"

#include <algorithm>

namespace mech::mech_simulation {
namespace {

constexpr std::uint8_t kCommandMarker = 0xC1U;
constexpr std::uint8_t kFeedbackMarker = 0xC2U;

void put_i32(std::uint8_t* data, std::int32_t value) noexcept {
  const auto unsigned_value = static_cast<std::uint32_t>(value);
  for (int index = 0; index < 4; ++index) {
    data[index] = static_cast<std::uint8_t>(unsigned_value >> (index * 8));
  }
}

std::int32_t get_i32(const std::uint8_t* data) noexcept {
  const auto value = static_cast<std::uint32_t>(data[0]) |
                     (static_cast<std::uint32_t>(data[1]) << 8U) |
                     (static_cast<std::uint32_t>(data[2]) << 16U) |
                     (static_cast<std::uint32_t>(data[3]) << 24U);
  return static_cast<std::int32_t>(value);
}

std::optional<mech_control_core::RawCanFrame> make_frame(
    std::uint16_t logical_bus, std::uint32_t id_value,
    mech_control_core::FrameDirection direction,
    const std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes>& payload,
    mech_control_core::MonotonicTime now) noexcept {
  const auto id = mech_control_core::CanId::create(
      id_value, mech_control_core::CanFrameFormat::Standard);
  return mech_control_core::RawCanFrame::create(
      logical_bus, *id, mech_control_core::CanFrameType::Classic, direction,
      8U, payload, now);
}

}  // namespace

std::optional<mech_control_core::RawCanFrame> LoopbackCodec::encode_command(
    std::uint16_t logical_bus, std::uint16_t device_id,
    std::int32_t target_milli, std::uint8_t sequence,
    mech_control_core::MonotonicTime now) noexcept {
  if (logical_bus == 0U || device_id == 0U || device_id > 0x6FFU) {
    return std::nullopt;
  }
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  payload[0] = kCommandMarker;
  put_i32(payload.data() + 1U, target_milli);
  payload[5] = sequence;
  return make_frame(logical_bus, 0x100U + device_id,
                    mech_control_core::FrameDirection::Tx, payload, now);
}

std::optional<LoopbackCommand> LoopbackCodec::decode_command(
    const mech_control_core::RawCanFrame& frame) noexcept {
  if (frame.direction != mech_control_core::FrameDirection::Tx ||
      frame.id.format != mech_control_core::CanFrameFormat::Standard ||
      frame.type != mech_control_core::CanFrameType::Classic ||
      frame.payload_size != 8U || frame.payload[0] != kCommandMarker ||
      frame.id.value < 0x101U || frame.id.value > 0x7FFU) {
    return std::nullopt;
  }
  return LoopbackCommand{static_cast<std::uint16_t>(frame.id.value - 0x100U),
                         get_i32(frame.payload.data() + 1U),
                         frame.payload[5]};
}

std::optional<mech_control_core::RawCanFrame> LoopbackCodec::encode_feedback(
    std::uint16_t logical_bus, std::uint16_t device_id,
    std::int32_t position_milli, std::uint8_t fault_code,
    std::uint8_t sequence, mech_control_core::MonotonicTime now) noexcept {
  if (logical_bus == 0U || device_id == 0U || device_id > 0x67FU) {
    return std::nullopt;
  }
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  payload[0] = kFeedbackMarker;
  put_i32(payload.data() + 1U, position_milli);
  payload[5] = fault_code;
  payload[6] = sequence;
  return make_frame(logical_bus, 0x180U + device_id,
                    mech_control_core::FrameDirection::Rx, payload, now);
}

std::optional<LoopbackFeedback> LoopbackCodec::decode_feedback(
    const mech_control_core::RawCanFrame& frame) noexcept {
  if (frame.direction != mech_control_core::FrameDirection::Rx ||
      frame.id.format != mech_control_core::CanFrameFormat::Standard ||
      frame.type != mech_control_core::CanFrameType::Classic ||
      frame.payload_size != 8U || frame.payload[0] != kFeedbackMarker ||
      frame.id.value < 0x181U || frame.id.value > 0x7FFU) {
    return std::nullopt;
  }
  return LoopbackFeedback{static_cast<std::uint16_t>(frame.id.value - 0x180U),
                          get_i32(frame.payload.data() + 1U), frame.payload[5],
                          frame.payload[6]};
}

SimulatedDevice::SimulatedDevice(std::uint16_t logical_bus,
                                 std::uint16_t device_id,
                                 std::int32_t max_step_milli)
    : logical_bus_(logical_bus),
      device_id_(device_id),
      max_step_milli_(std::max<std::int32_t>(1, max_step_milli)) {}

bool SimulatedDevice::accept_command(
    const mech_control_core::RawCanFrame& frame) noexcept {
  const auto command = LoopbackCodec::decode_command(frame);
  if (!command.has_value() || command->device_id != device_id_ ||
      frame.logical_bus != logical_bus_) {
    return false;
  }
  target_milli_ = command->target_milli;
  sequence_ = command->sequence;
  return true;
}

std::optional<mech_control_core::RawCanFrame> SimulatedDevice::step(
    mech_control_core::MonotonicTime now) noexcept {
  if (position_milli_ < target_milli_) {
    position_milli_ = std::min(position_milli_ + max_step_milli_, target_milli_);
  } else if (position_milli_ > target_milli_) {
    position_milli_ = std::max(position_milli_ - max_step_milli_, target_milli_);
  }
  return LoopbackCodec::encode_feedback(logical_bus_, device_id_, position_milli_,
                                         fault_code_, sequence_, now);
}

}  // namespace mech::mech_simulation
