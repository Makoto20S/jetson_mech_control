#include "mech_protocol_damiao/dm4340_mit_session.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace mech::mech_protocol_damiao {
namespace {

using mech_control_core::AdapterResult;
using mech_control_core::MonotonicTime;

bool valid_config(const Dm4340MitSessionConfig& c) noexcept {
  return c.profile ==
             mech_control_core::ProtocolProfile::Dm4340MitStandard &&
      c.logical_bus != 0U && c.command_can_id != 0U &&
      c.command_can_id <= mech_control_core::kMaxStandardCanId &&
      c.feedback_can_id <= mech_control_core::kMaxStandardCanId &&
      c.command_can_id != c.feedback_can_id && c.motor_id != 0U &&
      c.motor_id <= 0xFU && c.firmware_evidence_declared &&
      c.firmware_id != 0U && c.firmware_min != 0U &&
      c.firmware_min <= c.firmware_id &&
      c.firmware_id <= c.firmware_max &&
      c.limits_evidence_declared && valid_limits(c.limits) &&
      c.joint.evidence_declared && std::isfinite(c.joint.scale) &&
      c.joint.scale != 0.0 && std::isfinite(c.joint.offset_rad) &&
      std::isfinite(c.position_min_rad) &&
      std::isfinite(c.position_max_rad) &&
      c.position_min_rad < c.position_max_rad &&
      std::isfinite(c.max_target_error_rad) &&
      c.max_target_error_rad > 0.0 &&
      std::isfinite(c.kp) && c.kp >= 0.0 && c.kp <= 500.0 &&
      std::isfinite(c.kd) && c.kd >= 0.0 && c.kd <= 5.0 &&
      std::isfinite(c.velocity_feedforward_nm) &&
      c.velocity_feedforward_nm >= 0.0 &&
      std::isfinite(c.max_command_velocity_rad_s) &&
      c.max_command_velocity_rad_s >= 0.0 &&
      c.max_command_velocity_rad_s <= c.limits.velocity_rad_s &&
      std::isfinite(c.max_command_effort_nm) &&
      c.max_command_effort_nm >= 0.0 &&
      c.max_command_effort_nm * std::abs(c.joint.scale) <= c.limits.torque_nm &&
      std::isfinite(c.max_feedback_velocity_rad_s) &&
      c.max_feedback_velocity_rad_s >= 0.0 &&
      c.max_feedback_velocity_rad_s <= c.limits.velocity_rad_s &&
      std::isfinite(c.max_feedback_torque_nm) &&
      c.max_feedback_torque_nm >= 0.0 &&
      c.max_feedback_torque_nm <= c.limits.torque_nm &&
      c.velocity_feedforward_nm <= c.max_feedback_torque_nm &&
      c.max_command_effort_nm * std::abs(c.joint.scale) <=
          c.max_feedback_torque_nm &&
      (c.velocity_feedforward_nm == 0.0 ||
       (c.max_command_velocity_rad_s > 0.0 && c.kp == 0.0 && c.kd > 0.0)) &&
      c.kd * c.max_command_velocity_rad_s +
          c.velocity_feedforward_nm <= c.max_feedback_torque_nm &&
      (c.max_command_velocity_rad_s == 0.0 ||
       (c.max_feedback_velocity_rad_s > 0.0 &&
        c.max_feedback_torque_nm > 0.0)) &&
      c.command_ttl_ns > 0 &&
      c.feedback_ttl_ns > 0;
}

void invalidate_values(Dm4340MitSnapshot& sample) noexcept {
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  sample.joint_position_rad = invalid;
  sample.raw_motor_position_rad = invalid;
  sample.raw_motor_velocity_rad_s = invalid;
  sample.raw_motor_torque_nm = invalid;
}

}  // namespace

AdapterResult Dm4340MitSession::configure(
    const Dm4340MitSessionConfig& config) noexcept {
  if (active_ || !valid_config(config)) {
    return AdapterResult::InvalidConfiguration;
  }
  config_ = config;
  configured_ = true;
  fault_latched_ = false;
  last_arrival_.reset();
  sample_ = {};
  last_generation_ = 0U;
  clear_prepared();
  return AdapterResult::Ok;
}

AdapterResult Dm4340MitSession::activate(MonotonicTime epoch_start) noexcept {
  if (!configured_ || active_) {
    return AdapterResult::InvalidConfiguration;
  }
  active_ = true;
  fault_latched_ = false;
  epoch_start_ = epoch_start;
  last_arrival_.reset();
  sample_ = {};
  last_generation_ = 0U;
  clear_prepared();
  return AdapterResult::Ok;
}

void Dm4340MitSession::deactivate() noexcept {
  active_ = false;
  fault_latched_ = false;
  last_arrival_.reset();
  sample_ = {};
  last_generation_ = 0U;
  clear_prepared();
}

AdapterResult Dm4340MitSession::accept_feedback(
    const mech_control_core::RawCanFrame& frame, MonotonicTime now) noexcept {
  if (!active_) {
    return AdapterResult::InvalidConfiguration;
  }
  if (frame.logical_bus != config_.logical_bus ||
      frame.id.format != mech_control_core::CanFrameFormat::Standard ||
      frame.id.value != config_.feedback_can_id) {
    return AdapterResult::InvalidCommand;
  }
  const auto age = mech_control_core::elapsed_since(frame.host_arrival, now);
  const bool ordered = epoch_start_ <= frame.host_arrival && age &&
      (!last_arrival_ || *last_arrival_ <= frame.host_arrival);
  if (ordered) {
    last_arrival_ = frame.host_arrival;
  }
  MitFeedback decoded{};
  if (!decode_mit_feedback(frame, config_.logical_bus,
                           config_.feedback_can_id, config_.motor_id,
                           config_.limits, decoded)) {
    clear_prepared();
    sample_.availability = DmFeedbackAvailability::Invalid;
    invalidate_values(sample_);
    return AdapterResult::InvalidCommand;
  }
  if (!feedback_status_ok(decoded.raw_fault)) {
    clear_prepared();
    fault_latched_ = true;
    sample_.availability = DmFeedbackAvailability::Fault;
    sample_.raw_fault = decoded.raw_fault;
    invalidate_values(sample_);
    return AdapterResult::Fault;
  }
  if (fault_latched_) {
    return AdapterResult::Fault;
  }
  if ((config_.max_feedback_velocity_rad_s > 0.0 &&
       std::abs(decoded.velocity_rad_s) > config_.max_feedback_velocity_rad_s) ||
      (config_.max_feedback_torque_nm > 0.0 &&
       std::abs(decoded.torque_nm) > config_.max_feedback_torque_nm)) {
    clear_prepared();
    fault_latched_ = true;
    sample_.availability = DmFeedbackAvailability::Fault;
    invalidate_values(sample_);
    return AdapterResult::Fault;
  }
  if (!ordered || age->nanoseconds() >= config_.feedback_ttl_ns) {
    clear_prepared();
    sample_.availability = DmFeedbackAvailability::Stale;
    invalidate_values(sample_);
    return AdapterResult::Stale;
  }
  const double joint_position = decoded.position_rad * config_.joint.scale +
      config_.joint.offset_rad;
  if (!std::isfinite(joint_position) ||
      joint_position < config_.position_min_rad ||
      joint_position > config_.position_max_rad) {
    clear_prepared();
    sample_.availability = DmFeedbackAvailability::Invalid;
    invalidate_values(sample_);
    return AdapterResult::InvalidCommand;
  }
  sample_.joint_position_rad = joint_position;
  sample_.raw_motor_position_rad = decoded.position_rad;
  sample_.raw_motor_velocity_rad_s = decoded.velocity_rad_s;
  sample_.raw_motor_torque_nm = decoded.torque_nm;
  sample_.availability = DmFeedbackAvailability::Fresh;
  sample_.raw_fault = decoded.raw_fault;
  sample_.host_rx_time = frame.host_arrival;
  ++sample_.sequence;
  return AdapterResult::Ok;
}

Dm4340MitSnapshot Dm4340MitSession::snapshot(MonotonicTime now) const noexcept {
  auto result = sample_;
  if (!active_) {
    result.availability = DmFeedbackAvailability::Unknown;
  } else if (fault_latched_) {
    result.availability = DmFeedbackAvailability::Fault;
  } else if (result.availability == DmFeedbackAvailability::Fresh) {
    const auto age = mech_control_core::elapsed_since(*result.host_rx_time, now);
    if (!age || age->nanoseconds() >= config_.feedback_ttl_ns) {
      result.availability = DmFeedbackAvailability::Stale;
    }
  }
  if (result.availability != DmFeedbackAvailability::Fresh) {
    invalidate_values(result);
  }
  return result;
}

AdapterResult Dm4340MitSession::prepare_position(
    const mech_control_core::CanonicalDeviceCommand& command,
    MonotonicTime now, mech_control_core::RawCanFrame& output) noexcept {
  if (!active_) {
    clear_prepared();
    return AdapterResult::InvalidConfiguration;
  }
  if (fault_latched_) {
    clear_prepared();
    return AdapterResult::Fault;
  }
  const auto state = snapshot(now);
  if (state.availability != DmFeedbackAvailability::Fresh) {
    clear_prepared();
    return AdapterResult::Stale;
  }
  const auto ttl = mech_control_core::elapsed_since(now, command.deadline);
  if (!std::isfinite(command.position) ||
      !std::isfinite(command.velocity) ||
      !std::isfinite(command.effort) || command.velocity != 0.0 ||
      command.effort != 0.0 || command.generation == 0U ||
      command.generation <= last_generation_ || !ttl ||
      ttl->nanoseconds() == 0 ||
      ttl->nanoseconds() > config_.command_ttl_ns ||
      command.position < config_.position_min_rad ||
      command.position > config_.position_max_rad ||
      std::abs(command.position - state.joint_position_rad) >
          config_.max_target_error_rad) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  const double motor_position =
      (command.position - config_.joint.offset_rad) / config_.joint.scale;
  std::array<std::uint8_t, 8> packed{};
  if (!encode_mit({motor_position, 0.0, config_.kp, config_.kd, 0.0},
                  config_.limits, packed)) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  for (std::size_t i = 0U; i < packed.size(); ++i) {
    payload[i] = packed[i];
  }
  const auto frame = mech_control_core::RawCanFrame::create(
      config_.logical_bus,
      *mech_control_core::CanId::create(
          config_.command_can_id, mech_control_core::CanFrameFormat::Standard),
      mech_control_core::CanFrameType::Classic,
      mech_control_core::FrameDirection::Tx, 8U, payload, now);
  if (!frame) {
    clear_prepared();
    return AdapterResult::InvalidConfiguration;
  }
  const auto p = static_cast<std::uint16_t>((packed[0] << 8) | packed[1]);
  const double quantized_motor = static_cast<double>(p) *
      (2.0 * config_.limits.position_rad) / 65535.0 -
      config_.limits.position_rad;
  const double quantized_joint = quantized_motor * config_.joint.scale +
      config_.joint.offset_rad;
  if (!std::isfinite(quantized_joint) ||
      quantized_joint < config_.position_min_rad ||
      quantized_joint > config_.position_max_rad ||
      std::abs(quantized_joint - state.joint_position_rad) >
          config_.max_target_error_rad) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  output = *frame;
  last_generation_ = command.generation;
  last_prepared_ = output;
  last_deadline_ = command.deadline;
  last_prepared_joint_position_rad_ = quantized_joint;
  last_prepared_requires_enabled_ = false;
  return AdapterResult::Ok;
}

AdapterResult Dm4340MitSession::prepare_velocity(
    const mech_control_core::CanonicalDeviceCommand& command,
    MonotonicTime now, mech_control_core::RawCanFrame& output) noexcept {
  if (!active_ || config_.max_command_velocity_rad_s <= 0.0 ||
      config_.kp != 0.0 || config_.kd <= 0.0) {
    clear_prepared();
    return AdapterResult::InvalidConfiguration;
  }
  if (fault_latched_) {
    clear_prepared();
    return AdapterResult::Fault;
  }
  const auto state = snapshot(now);
  if (state.availability != DmFeedbackAvailability::Fresh) {
    clear_prepared();
    return AdapterResult::Stale;
  }
  if (!feedback_motor_enabled(state.raw_fault)) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  const auto ttl = mech_control_core::elapsed_since(now, command.deadline);
  if (!std::isfinite(command.position) || command.position != 0.0 ||
      !std::isfinite(command.velocity) || !std::isfinite(command.effort) ||
      command.effort != 0.0 || command.generation == 0U ||
      command.generation <= last_generation_ || !ttl ||
      ttl->nanoseconds() == 0 || ttl->nanoseconds() > config_.command_ttl_ns ||
      std::abs(command.velocity) >
          config_.max_command_velocity_rad_s * std::abs(config_.joint.scale)) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  std::array<std::uint8_t, 8> packed{};
  const double motor_velocity = command.velocity / config_.joint.scale;
  const double feedforward = motor_velocity > 0.0
      ? config_.velocity_feedforward_nm
      : (motor_velocity < 0.0 ? -config_.velocity_feedforward_nm : 0.0);
  if (!encode_mit({0.0, motor_velocity, 0.0, config_.kd, feedforward},
                  config_.limits, packed)) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  for (std::size_t i = 0U; i < packed.size(); ++i) payload[i] = packed[i];
  const auto frame = mech_control_core::RawCanFrame::create(
      config_.logical_bus,
      *mech_control_core::CanId::create(
          config_.command_can_id, mech_control_core::CanFrameFormat::Standard),
      mech_control_core::CanFrameType::Classic,
      mech_control_core::FrameDirection::Tx, 8U, payload, now);
  if (!frame) {
    clear_prepared();
    return AdapterResult::InvalidConfiguration;
  }
  output = *frame;
  last_generation_ = command.generation;
  last_prepared_ = output;
  last_deadline_ = command.deadline;
  last_prepared_requires_enabled_ = true;
  return AdapterResult::Ok;
}

AdapterResult Dm4340MitSession::prepare_effort(
    const mech_control_core::CanonicalDeviceCommand& command,
    MonotonicTime now, mech_control_core::RawCanFrame& output) noexcept {
  if (!active_ || config_.max_command_effort_nm <= 0.0 ||
      config_.max_command_velocity_rad_s != 0.0 || config_.kp != 0.0 ||
      config_.kd != 0.0 || config_.velocity_feedforward_nm != 0.0) {
    clear_prepared();
    return AdapterResult::InvalidConfiguration;
  }
  if (fault_latched_) {
    clear_prepared();
    return AdapterResult::Fault;
  }
  const auto state = snapshot(now);
  if (state.availability != DmFeedbackAvailability::Fresh) {
    clear_prepared();
    return AdapterResult::Stale;
  }
  if (!feedback_motor_enabled(state.raw_fault)) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  const auto ttl = mech_control_core::elapsed_since(now, command.deadline);
  if (!std::isfinite(command.position) || command.position != 0.0 ||
      !std::isfinite(command.velocity) || command.velocity != 0.0 ||
      !std::isfinite(command.effort) ||
      std::abs(command.effort) > config_.max_command_effort_nm ||
      command.generation == 0U || command.generation <= last_generation_ ||
      !ttl || ttl->nanoseconds() == 0 ||
      ttl->nanoseconds() > config_.command_ttl_ns) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  std::array<std::uint8_t, 8> packed{};
  if (!encode_mit({0.0, 0.0, 0.0, 0.0,
                   command.effort * config_.joint.scale},
                  config_.limits, packed)) {
    clear_prepared();
    return AdapterResult::InvalidCommand;
  }
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  for (std::size_t i = 0U; i < packed.size(); ++i) payload[i] = packed[i];
  const auto frame = mech_control_core::RawCanFrame::create(
      config_.logical_bus,
      *mech_control_core::CanId::create(
          config_.command_can_id, mech_control_core::CanFrameFormat::Standard),
      mech_control_core::CanFrameType::Classic,
      mech_control_core::FrameDirection::Tx, 8U, payload, now);
  if (!frame) {
    clear_prepared();
    return AdapterResult::InvalidConfiguration;
  }
  output = *frame;
  last_generation_ = command.generation;
  last_prepared_ = output;
  last_deadline_ = command.deadline;
  last_prepared_requires_enabled_ = true;
  return AdapterResult::Ok;
}

bool Dm4340MitSession::pending_target_still_authorized(
    const mech_control_core::RawCanFrame& frame,
    std::uint64_t generation, MonotonicTime deadline,
    MonotonicTime now) const noexcept {
  if (!active_ || fault_latched_ || !last_prepared_ || !last_deadline_ ||
      generation != last_generation_ || !(deadline == *last_deadline_) ||
      deadline <= now || now < last_prepared_->host_arrival ||
      snapshot(now).availability != DmFeedbackAvailability::Fresh) {
    return false;
  }
  const auto state = snapshot(now);
  if (!last_prepared_requires_enabled_ &&
      (!std::isfinite(last_prepared_joint_position_rad_) ||
       std::abs(last_prepared_joint_position_rad_ -
                state.joint_position_rad) > config_.max_target_error_rad)) {
    return false;
  }
  if (last_prepared_requires_enabled_ && !feedback_motor_enabled(state.raw_fault))
    return false;
  const auto& expected = *last_prepared_;
  if (!frame.is_valid() || frame.logical_bus != expected.logical_bus ||
      frame.id.value != expected.id.value ||
      frame.id.format != expected.id.format || frame.type != expected.type ||
      frame.direction != expected.direction ||
      frame.payload_size != expected.payload_size ||
      !(frame.host_arrival == expected.host_arrival) ||
      frame.source_timestamp || frame.error_frame || frame.remote_request ||
      frame.bitrate_switch) {
    return false;
  }
  return frame.payload == expected.payload;
}

void Dm4340MitSession::clear_prepared() noexcept {
  last_prepared_.reset();
  last_deadline_.reset();
  last_prepared_requires_enabled_ = false;
  last_prepared_joint_position_rad_ =
      std::numeric_limits<double>::quiet_NaN();
}

}  // namespace mech::mech_protocol_damiao
