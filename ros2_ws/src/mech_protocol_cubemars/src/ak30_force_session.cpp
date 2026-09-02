#include "mech_protocol_cubemars/ak30_force_session.hpp"

#include "mech_control_core/time.hpp"

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

AdapterResult Ak30ForceControlSession::activate() noexcept {
  if (lifecycle_ == Lifecycle::Unconfigured || !codec_.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }
  fault_latched_ = false;
  last_command_time_.reset();
  lifecycle_ = Lifecycle::Active;
  return AdapterResult::Ok;
}

void Ak30ForceControlSession::deactivate() noexcept {
  if (lifecycle_ == Lifecycle::Active) {
    lifecycle_ = Lifecycle::Ready;
  }
  last_command_time_.reset();
}

AdapterResult Ak30ForceControlSession::submit(
    const mech_control_core::CanonicalDeviceCommand& command,
    mech_control_core::MonotonicTime now) noexcept {
  if (lifecycle_ != Lifecycle::Active || !codec_.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }
  if (fault_latched_) {
    return AdapterResult::Fault;
  }
  // An already-expired deadline is an invalid command, not a reason to invent
  // one. Nothing here ever substitutes a default value for a stale input.
  if (command.deadline.nanoseconds() <= now.nanoseconds()) {
    return AdapterResult::InvalidCommand;
  }

  mech_control_core::RawCanFrame frame{};
  const AdapterResult encoded = codec_->encode(command, logical_bus_, now, frame);
  if (encoded != AdapterResult::Ok) {
    return encoded;
  }

  switch (transport().try_send(frame)) {
    case mech_control_core::TransportResult::Ok:
      last_command_time_ = now;
      return AdapterResult::Ok;
    // Backpressure is retryable. Faulting the bus on a transient WouldBlock is
    // the RC defect this branch exists to avoid.
    case mech_control_core::TransportResult::WouldBlock:
    case mech_control_core::TransportResult::QueueFull:
      return AdapterResult::WouldBlock;
    case mech_control_core::TransportResult::Disconnected:
      return AdapterResult::Disconnected;
    case mech_control_core::TransportResult::Invalid:
      return AdapterResult::InvalidCommand;
    case mech_control_core::TransportResult::Fault:
      fault_latched_ = true;
      return AdapterResult::Fault;
  }
  return AdapterResult::Fault;
}

AdapterResult Ak30ForceControlSession::process(
    const mech_control_core::RawCanFrame& frame,
    mech_control_core::MonotonicTime now) noexcept {
  if (lifecycle_ != Lifecycle::Active || !codec_.has_value()) {
    return AdapterResult::InvalidConfiguration;
  }
  mech_control_core::CanonicalDeviceState decoded{};
  const AdapterResult result = codec_->decode(frame, decoded);
  if (result != AdapterResult::Ok) {
    return result;
  }

  ++sequence_;
  last_state_ = decoded;
  last_feedback_time_ = now;
  if (classify_status(static_cast<std::uint8_t>(decoded.status.raw_fault_code)) ==
      StatusMeaning::Fault) {
    fault_latched_ = true;
  }
  return AdapterResult::Ok;
}

mech_control_core::CanonicalDeviceState Ak30ForceControlSession::snapshot(
    mech_control_core::MonotonicTime now) const noexcept {
  mech_control_core::CanonicalDeviceState state = last_state_;
  if (!last_feedback_time_.has_value()) {
    // No sample yet. StatusSnapshot::create() requires Unknown quality to carry
    // no host_rx_time, which a default-constructed snapshot already satisfies.
    state.status = mech_control_core::StatusSnapshot{};
    state.status.device_state = fault_latched_
                                    ? mech_control_core::DeviceState::Fault
                                    : (lifecycle_ == Lifecycle::Active
                                           ? mech_control_core::DeviceState::Active
                                           : mech_control_core::DeviceState::Ready);
    return state;
  }

  state.status.sequence = sequence_;
  const auto age = mech_control_core::elapsed_since(last_feedback_time_.value(), now);
  const bool stale =
      !age.has_value() || age->nanoseconds() > config_.feedback_ttl_nanoseconds;
  if (stale) {
    state.status.quality = mech_control_core::SampleQuality::Stale;
  }
  state.status.device_state = fault_latched_
                                  ? mech_control_core::DeviceState::Fault
                                  : (lifecycle_ == Lifecycle::Active
                                         ? mech_control_core::DeviceState::Active
                                         : mech_control_core::DeviceState::Ready);
  return state;
}

CommandStage Ak30ForceControlSession::command_stage(
    mech_control_core::MonotonicTime now) const noexcept {
  if (!last_command_time_.has_value()) {
    return CommandStage::Expired;
  }
  const auto age = mech_control_core::elapsed_since(last_command_time_.value(), now);
  if (!age.has_value()) {
    return CommandStage::Expired;
  }
  // Strict `<` on both bounds. The tests pin the transition instants exactly:
  // submitted at t=1000 ns with ttl 4 ms, age 3999999 is the last Following
  // sample and age 4000000 is the first Holding one. Using `<=` here shifts
  // every boundary by one nanosecond and fails those assertions.
  if (age->nanoseconds() < config_.command_ttl_nanoseconds) {
    return CommandStage::Following;
  }
  if (age->nanoseconds() < config_.command_hard_ttl_nanoseconds) {
    return CommandStage::Holding;
  }
  return CommandStage::Expired;
}

}  // namespace mech::mech_protocol_cubemars
