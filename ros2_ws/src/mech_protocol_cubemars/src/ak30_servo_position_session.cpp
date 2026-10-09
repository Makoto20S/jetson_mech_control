#include "mech_protocol_cubemars/ak30_servo_position_session.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace mech::mech_protocol_cubemars {
namespace {
using mech_control_core::AdapterResult;
using mech_control_core::MonotonicTime;

constexpr std::int64_t kHostHardTtlNs = 6000000;

bool valid_affine(const ServoPositionAffine& affine) noexcept {
  return affine.evidence_declared && std::isfinite(affine.scale) &&
      affine.scale != 0.0 && std::isfinite(affine.offset);
}

std::int32_t encoded_position(const mech_control_core::RawCanFrame& frame) noexcept {
  const std::uint32_t bits = (static_cast<std::uint32_t>(frame.payload[0]) << 24U) |
      (static_cast<std::uint32_t>(frame.payload[1]) << 16U) |
      (static_cast<std::uint32_t>(frame.payload[2]) << 8U) |
      static_cast<std::uint32_t>(frame.payload[3]);
  const std::int64_t signed_bits = bits >= 0x80000000U
      ? static_cast<std::int64_t>(bits) - 0x100000000LL
      : static_cast<std::int64_t>(bits);
  return static_cast<std::int32_t>(signed_bits);
}
}  // namespace

AdapterResult Ak30ServoPositionSession::configure(
    const ServoPositionSessionConfig& config) noexcept {
  if (active_) return AdapterResult::InvalidConfiguration;
  if (config.profile != mech_control_core::ProtocolProfile::Ak30ServoExtended ||
      config.drive_id > 255U || !valid_affine(config.target_rad_to_deg) ||
      !valid_affine(config.feedback_deg_to_rad) ||
      !std::isfinite(config.position_min_rad) ||
      !std::isfinite(config.position_max_rad) ||
      config.position_min_rad >= config.position_max_rad ||
      !std::isfinite(config.max_target_error_rad) ||
      config.max_target_error_rad <= 0.0 ||
      config.command_ttl_ns <= 0 || config.command_hard_ttl_ns <= 0 ||
      config.command_ttl_ns >= config.command_hard_ttl_ns ||
      config.command_hard_ttl_ns > kHostHardTtlNs ||
      config.feedback_ttl_ns <= 0) return AdapterResult::InvalidConfiguration;
  mech_control_core::RawCanFrame probe{};
  if (!encode_servo_position_speed(config.drive_id,
      {0.0, config.speed_erpm, config.acceleration_raw},
      config.logical_bus, MonotonicTime{}, probe))
    return AdapterResult::InvalidConfiguration;
  config_ = config;
  configured_ = true;
  fault_latched_ = false;
  last_arrival_.reset();
  sample_ = {};
  last_generation_ = 0U;
  last_prepared_.reset();
  last_prepared_deadline_.reset();
  return AdapterResult::Ok;
}

AdapterResult Ak30ServoPositionSession::activate(MonotonicTime epoch_start) noexcept {
  if (!configured_ || active_) return AdapterResult::InvalidConfiguration;
  active_ = true;
  fault_latched_ = false;
  epoch_start_ = epoch_start;
  last_arrival_.reset();
  sample_ = {};
  last_generation_ = 0U;
  last_prepared_.reset();
  last_prepared_deadline_.reset();
  return AdapterResult::Ok;
}

void Ak30ServoPositionSession::deactivate() noexcept {
  active_ = false;
  fault_latched_ = false;
  last_arrival_.reset();
  sample_ = {};
  last_generation_ = 0U;
  last_prepared_.reset();
  last_prepared_deadline_.reset();
}

AdapterResult Ak30ServoPositionSession::accept_feedback(
    const mech_control_core::RawCanFrame& frame, MonotonicTime now) noexcept {
  if (!active_) return AdapterResult::InvalidConfiguration;
  // Wrong routes must not change this session's last good sample.
  if (frame.logical_bus != config_.logical_bus ||
      frame.id.value != (0x2900U | config_.drive_id))
    return AdapterResult::InvalidCommand;
  const auto age = mech_control_core::elapsed_since(frame.host_arrival, now);
  const bool ordered_arrival = epoch_start_ <= frame.host_arrival && age &&
      (!last_arrival_ || *last_arrival_ < frame.host_arrival);
  // A matching observed frame, even if malformed or non-normal, advances the
  // arrival watermark. An older healthy frame cannot supersede its status.
  if (ordered_arrival) last_arrival_ = frame.host_arrival;
  ServoFeedback decoded{};
  if (!decode_servo_feedback(config_.drive_id, frame, decoded)) {
    invalidate_prepared();
    sample_.availability = ServoPositionAvailability::Invalid;
    return AdapterResult::InvalidCommand;
  }
  if (decoded.status == ServoFeedbackStatus::KnownFault) {
    invalidate_prepared();
    fault_latched_ = true;
    sample_.availability = ServoPositionAvailability::Fault;
    sample_.raw_status = decoded.raw_status;
    return AdapterResult::Fault;
  }
  if (fault_latched_) return AdapterResult::Fault;
  if (!ordered_arrival || age->nanoseconds() >= config_.feedback_ttl_ns) {
    invalidate_prepared();
    sample_.availability = ServoPositionAvailability::Stale;
    return AdapterResult::Stale;
  }
  if (decoded.status != ServoFeedbackStatus::Normal) {
    invalidate_prepared();
    sample_.availability = decoded.status == ServoFeedbackStatus::DisableAcknowledged
        ? ServoPositionAvailability::DisableAcknowledged
        : ServoPositionAvailability::Unknown;
    sample_.raw_status = decoded.raw_status;
    return AdapterResult::InvalidCommand;
  }
  const double position = decoded.position_deg * config_.feedback_deg_to_rad.scale +
      config_.feedback_deg_to_rad.offset;
  if (!std::isfinite(position) || position < config_.position_min_rad ||
      position > config_.position_max_rad) {
    invalidate_prepared();
    sample_.availability = ServoPositionAvailability::Invalid;
    return AdapterResult::InvalidCommand;
  }
  sample_.feedback_position_deg = decoded.position_deg;
  sample_.temperature_c = decoded.board_temperature_c;
  sample_.position_rad = position;
  sample_.electrical_speed_erpm = decoded.electrical_speed_erpm;
  sample_.current_iq_a = decoded.current_iq_a;
  sample_.availability = ServoPositionAvailability::Fresh;
  sample_.raw_status = 0U;
  ++sample_.sequence;
  sample_.host_rx_time = frame.host_arrival;
  return AdapterResult::Ok;
}

ServoPositionSnapshot Ak30ServoPositionSession::snapshot(MonotonicTime now) const noexcept {
  auto result = sample_;
  if (!active_) {
    result.availability = ServoPositionAvailability::Unknown;
  } else if (fault_latched_) {
    result.availability = ServoPositionAvailability::Fault;
  } else if (result.availability == ServoPositionAvailability::Fresh) {
    const auto age = mech_control_core::elapsed_since(*result.host_rx_time, now);
    if (!age || age->nanoseconds() >= config_.feedback_ttl_ns)
      result.availability = ServoPositionAvailability::Stale;
  }
  if (result.availability != ServoPositionAvailability::Fresh) {
    result.feedback_position_deg = std::numeric_limits<double>::quiet_NaN();
    result.temperature_c = std::numeric_limits<double>::quiet_NaN();
    result.position_rad = std::numeric_limits<double>::quiet_NaN();
    result.electrical_speed_erpm = std::numeric_limits<double>::quiet_NaN();
    result.current_iq_a = std::numeric_limits<double>::quiet_NaN();
  }
  return result;
}

AdapterResult Ak30ServoPositionSession::prepare_position(
    const mech_control_core::CanonicalDeviceCommand& command,
    MonotonicTime now, mech_control_core::RawCanFrame& output) noexcept {
  if (!active_) return AdapterResult::InvalidConfiguration;
  if (fault_latched_) return AdapterResult::Fault;
  const auto state = snapshot(now);
  if (state.availability != ServoPositionAvailability::Fresh)
    return AdapterResult::Stale;
  const auto ttl = mech_control_core::elapsed_since(now, command.deadline);
  if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
      !std::isfinite(command.effort) || command.velocity != 0.0 ||
      command.effort != 0.0 || command.generation == 0U ||
      command.generation <= last_generation_ || !ttl ||
      ttl->nanoseconds() == 0 || ttl->nanoseconds() > config_.command_ttl_ns ||
      ttl->nanoseconds() > config_.command_hard_ttl_ns)
    return AdapterResult::InvalidCommand;
  if (command.position < config_.position_min_rad ||
      command.position > config_.position_max_rad ||
      std::abs(command.position - state.position_rad) > config_.max_target_error_rad)
    return AdapterResult::InvalidCommand;
  const double degrees = command.position * config_.target_rad_to_deg.scale +
      config_.target_rad_to_deg.offset;
  if (!std::isfinite(degrees)) return AdapterResult::InvalidCommand;
  mech_control_core::RawCanFrame candidate{};
  if (!encode_servo_position_speed(config_.drive_id,
      {degrees, config_.speed_erpm, config_.acceleration_raw},
      config_.logical_bus, now, candidate)) return AdapterResult::InvalidCommand;
  const double quantized_degrees = static_cast<double>(encoded_position(candidate)) / 10000.0;
  const double canonical = (quantized_degrees - config_.target_rad_to_deg.offset) /
      config_.target_rad_to_deg.scale;
  if (!std::isfinite(canonical) || canonical < config_.position_min_rad ||
      canonical > config_.position_max_rad ||
      std::abs(canonical - state.position_rad) > config_.max_target_error_rad)
    return AdapterResult::InvalidCommand;
  output = candidate;
  last_generation_ = command.generation;
  last_prepared_ = candidate;
  last_prepared_deadline_ = command.deadline;
  return AdapterResult::Ok;
}

bool Ak30ServoPositionSession::pending_target_still_authorized(
    const mech_control_core::RawCanFrame& frame, std::uint64_t generation,
    MonotonicTime deadline, MonotonicTime now) const noexcept {
  if (!active_ || fault_latched_ || !last_prepared_ ||
      !last_prepared_deadline_ || generation != last_generation_ ||
      !(deadline == *last_prepared_deadline_) || deadline <= now) return false;
  const auto& prepared = *last_prepared_;
  if (!frame.is_valid() || frame.logical_bus != prepared.logical_bus ||
      frame.id.value != prepared.id.value ||
      frame.id.format != prepared.id.format || frame.type != prepared.type ||
      frame.direction != prepared.direction ||
      frame.payload_size != prepared.payload_size ||
      !(frame.host_arrival == prepared.host_arrival) ||
      !(epoch_start_ <= prepared.host_arrival) || now < prepared.host_arrival ||
      frame.source_timestamp.has_value() || prepared.source_timestamp.has_value() ||
      frame.error_frame || frame.remote_request || frame.bitrate_switch)
    return false;
  for (std::size_t i = 0U; i < prepared.payload.size(); ++i) {
    if (frame.payload[i] != prepared.payload[i]) return false;
  }
  const auto state = snapshot(now);
  if (state.availability != ServoPositionAvailability::Fresh) return false;
  const double quantized_degrees = static_cast<double>(encoded_position(frame)) / 10000.0;
  const double canonical = (quantized_degrees - config_.target_rad_to_deg.offset) /
      config_.target_rad_to_deg.scale;
  return std::isfinite(canonical) && canonical >= config_.position_min_rad &&
      canonical <= config_.position_max_rad &&
      std::abs(canonical - state.position_rad) <= config_.max_target_error_rad;
}

}  // namespace mech::mech_protocol_cubemars
