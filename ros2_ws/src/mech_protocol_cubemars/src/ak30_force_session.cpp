#include "mech_protocol_cubemars/ak30_force_session.hpp"

namespace mech::mech_protocol_cubemars {

using mech_control_core::AdapterResult;
using mech_control_core::CanFrameFormat;
using mech_control_core::CanFrameType;
using mech_control_core::ProtocolProfile;

Ak30ForceControlSession::Ak30ForceControlSession(
    mech_control_core::Transport& transport, Ak30SessionConfig config) noexcept
    : mech_control_core::DeviceSession(transport), config_(config) {}

AdapterResult Ak30ForceControlSession::configure(
    const mech_control_core::DeviceConfig& config,
    const mech_control_core::TransportCapabilities& capabilities) noexcept {
  // ACTIVE never changes profile or re-binds configuration.
  if (lifecycle_ == Lifecycle::Active) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config.profile != ProtocolProfile::Ak30ForceControlExtended) {
    return AdapterResult::InvalidConfiguration;
  }
  const auto requirements =
      mech_control_core::profile_requirements(config.profile);
  if (!requirements.has_value() ||
      requirements->frame_type != config.frame_type ||
      requirements->frame_format != config.frame_format) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.drive_id > 255U) {
    return AdapterResult::InvalidConfiguration;
  }
  const auto drive_id = static_cast<std::uint8_t>(config_.drive_id);
  if (!config.command_id.has_value() || !config.feedback_id.has_value() ||
      config.command_id->format != CanFrameFormat::Extended ||
      config.feedback_id->format != CanFrameFormat::Extended ||
      config.command_id->value != force_control_can_id(drive_id) ||
      config.feedback_id->value != feedback_can_id(drive_id)) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config.command_payload_bytes != kForceControlPayloadBytes ||
      config.feedback_payload_bytes != kForceControlPayloadBytes) {
    return AdapterResult::InvalidConfiguration;
  }
  if (!config.writable) {
    return AdapterResult::InvalidConfiguration;
  }
  if (!capabilities.is_valid() || !capabilities.supports_classic_can ||
      !capabilities.supports_extended_frames ||
      capabilities.max_payload_bytes < kForceControlPayloadBytes) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.firmware_id_min > config_.firmware_id_max ||
      config_.firmware_id < config_.firmware_id_min ||
      config_.firmware_id > config_.firmware_id_max) {
    return AdapterResult::InvalidConfiguration;
  }
  if (!mapping_is_sufficient(config_.mapping, config_.sub_mode)) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.gains.kp < 0.0 || config_.gains.kp > config_.mapping.ranges.kp_max ||
      config_.gains.kd < 0.0 || config_.gains.kd > config_.mapping.ranges.kd_max) {
    return AdapterResult::InvalidConfiguration;
  }
  if (config_.command_ttl_nanoseconds <= 0 ||
      config_.command_hard_ttl_nanoseconds <= config_.command_ttl_nanoseconds ||
      config_.command_hard_ttl_nanoseconds > kMaxHardTtlNanoseconds ||
      config_.feedback_ttl_nanoseconds <= 0) {
    return AdapterResult::InvalidConfiguration;
  }

  logical_bus_ = config.logical_bus;
  codec_.emplace(drive_id, config_.sub_mode, config_.mapping, config_.gains);
  lifecycle_ = Lifecycle::Ready;
  fault_latched_ = false;
  sequence_ = 0U;
  last_state_ = mech_control_core::CanonicalDeviceState{};
  last_feedback_time_.reset();
  last_command_time_.reset();
  return AdapterResult::Ok;
}

// Task 7 replaces these stubs.
AdapterResult Ak30ForceControlSession::activate() noexcept {
  return AdapterResult::InvalidConfiguration;
}
void Ak30ForceControlSession::deactivate() noexcept {}
AdapterResult Ak30ForceControlSession::submit(
    const mech_control_core::CanonicalDeviceCommand&,
    mech_control_core::MonotonicTime) noexcept {
  return AdapterResult::InvalidConfiguration;
}
AdapterResult Ak30ForceControlSession::process(
    const mech_control_core::RawCanFrame&,
    mech_control_core::MonotonicTime) noexcept {
  return AdapterResult::InvalidConfiguration;
}
mech_control_core::CanonicalDeviceState Ak30ForceControlSession::snapshot(
    mech_control_core::MonotonicTime) const noexcept {
  return last_state_;
}
CommandStage Ak30ForceControlSession::command_stage(
    mech_control_core::MonotonicTime) const noexcept {
  return CommandStage::Expired;
}

}  // namespace mech::mech_protocol_cubemars
