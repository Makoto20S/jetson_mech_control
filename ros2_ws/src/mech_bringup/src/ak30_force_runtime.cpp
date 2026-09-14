#include "mech_bringup/ak30_force_runtime.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include "mech_control_core/adapter_template.hpp"
#include "mech_control_core/config.hpp"
#include "mech_control_core/frame.hpp"
#include "mech_control_core/status.hpp"
#include "mech_control_core/time.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

namespace mech::mech_bringup {
namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::DeviceConfig;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::ProtocolProfile;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::SampleQuality;
using mech::mech_control_core::TransportResult;
using mech::mech_hardware_ros2_control::CanonicalCommand;
using mech::mech_hardware_ros2_control::CanonicalState;
using mech::mech_protocol_cubemars::Ak30SessionConfig;
using mech::mech_protocol_cubemars::feedback_can_id;
using mech::mech_protocol_cubemars::force_control_can_id;
using mech::mech_protocol_cubemars::kMaxHardTtlNanoseconds;
using mech::mech_protocol_cubemars::mapping_is_sufficient;

// Receiving drains at most this many frames per cycle: the bus poller
// budget in BusRuntime is 64, and the session's feedback runs at 50 Hz, so
// one frame per 2 ms cycle is the physical steady state.
constexpr std::size_t kReceiveBudget = 8U;

[[nodiscard]] bool config_is_valid(const Ak30RuntimeConfig& config) noexcept {
  if (config.drive_id > 255U || config.logical_bus == 0U ||
      config.control_period_nanoseconds <= 0 ||
      config.command_ttl_nanoseconds <= 0 ||
      config.command_hard_ttl_nanoseconds <=
          config.command_ttl_nanoseconds ||
      config.command_hard_ttl_nanoseconds > kMaxHardTtlNanoseconds ||
      config.feedback_ttl_nanoseconds <= 0 ||
      !std::isfinite(config.gains.kp) || config.gains.kp < 0.0 ||
      !std::isfinite(config.gains.kd) || config.gains.kd < 0.0 ||
      !mapping_is_sufficient(config.mapping, config.sub_mode)) {
    return false;
  }
  return true;
}

[[nodiscard]] Ak30SessionConfig session_config_from(
    const Ak30RuntimeConfig& config) noexcept {
  Ak30SessionConfig session_config{};
  session_config.drive_id = config.drive_id;
  session_config.sub_mode = config.sub_mode;
  session_config.mapping = config.mapping;
  session_config.gains = config.gains;
  session_config.firmware_id = config.firmware_id;
  session_config.firmware_id_min = config.firmware_id_min;
  session_config.firmware_id_max = config.firmware_id_max;
  session_config.command_ttl_nanoseconds = config.command_ttl_nanoseconds;
  session_config.command_hard_ttl_nanoseconds =
      config.command_hard_ttl_nanoseconds;
  session_config.feedback_ttl_nanoseconds = config.feedback_ttl_nanoseconds;
  return session_config;
}

[[nodiscard]] DeviceConfig device_config_from(
    const Ak30RuntimeConfig& config) noexcept {
  DeviceConfig device{};
  device.device_id = config.device_id;
  device.name = "motor";
  device.logical_bus = config.logical_bus;
  device.profile = ProtocolProfile::Ak30ForceControlExtended;
  device.frame_type = CanFrameType::Classic;
  device.frame_format = CanFrameFormat::Extended;
  device.command_id = CanId::create(force_control_can_id(config.drive_id),
                                   CanFrameFormat::Extended);
  device.feedback_id = CanId::create(feedback_can_id(config.drive_id),
                                     CanFrameFormat::Extended);
  device.command_payload_bytes = 8U;
  device.feedback_payload_bytes = 8U;
  device.writable = true;
  return device;
}

}  // namespace

Ak30ForceControlRuntime::Ak30ForceControlRuntime(
    mech::mech_control_core::Transport& transport, Clock clock,
    Ak30RuntimeConfig config) noexcept
    : transport_(transport),
      clock_(std::move(clock)),
      config_(config),
      session_(transport, session_config_from(config)) {}

bool Ak30ForceControlRuntime::configure(
    std::size_t resource_count) noexcept {
  if (resource_count == 0U || resource_count > 1U || started_ ||
      !config_is_valid(config_)) {
    return false;
  }
  const DeviceConfig device = device_config_from(config_);
  const auto configured =
      session_.configure(device, transport_.capabilities());
  if (configured != AdapterResult::Ok) {
    return false;
  }
  pending_.assign(resource_count, CanonicalCommand{});
  resource_count_ = resource_count;
  have_pending_ = false;
  fresh_write_ = false;
  submitted_once_ = false;
  holding_ = false;
  expired_ = false;
  configured_ = true;
  return true;
}

bool Ak30ForceControlRuntime::start() noexcept {
  if (!configured_ || started_) {
    return false;
  }
  if (!transport_.is_open() && !transport_.open()) {
    return false;
  }
  if (session_.activate() != AdapterResult::Ok) {
    return false;
  }
  started_ = true;
  return true;
}

void Ak30ForceControlRuntime::stop() noexcept {
  if (!started_) {
    return;
  }
  session_.deactivate();
  started_ = false;
  have_pending_ = false;
  fresh_write_ = false;
  submitted_once_ = false;
  holding_ = false;
  expired_ = false;
}

bool Ak30ForceControlRuntime::write(const CanonicalCommand* commands,
                                    std::size_t count) noexcept {
  if (!started_ || commands == nullptr || count != resource_count_) {
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (!std::isfinite(commands[index].position)) {
      return false;
    }
  }
  for (std::size_t index = 0; index < count; ++index) {
    pending_[index] = commands[index];
  }
  have_pending_ = true;
  // The controller refreshed its command: the next read submits it, and
  // every write (even an unchanged one) counts as a refresh.
  fresh_write_ = true;
  return true;
}

bool Ak30ForceControlRuntime::submit_stored(MonotonicTime now) noexcept {
  const auto stage = session_.command_stage(now);
  holding_ = stage == mech::mech_protocol_cubemars::CommandStage::Holding;
  // The session reports Expired both for "no command accepted yet" and for
  // "the last accepted command aged past the hard TTL". Only the latter is
  // the ADR-012 explicit failure; the former is the freshly-activated state
  // in which the first submit is still allowed.
  expired_ = stage == mech::mech_protocol_cubemars::CommandStage::Expired &&
             submitted_once_;
  if (expired_) {
    return false;
  }
  if (!have_pending_ || !fresh_write_) {
    // Nothing was written yet, or the stored command is not a fresh
    // controller refresh: send nothing. Re-submitting a stale command would
    // hold the session's watchdog in Following forever and mask a dead
    // controller; inventing 0.0 would command a move to the zero position
    // (ADR-012 Decision 3).
    return true;
  }
  if (stage == mech::mech_protocol_cubemars::CommandStage::Holding) {
    // Frozen: the last accepted command stays the device's commanded state.
    // Submitting again would reset the session's stage clock and swallow the
    // escalation to Expired.
    return true;
  }

  // stage == Following with a fresh command, or the first submit ever (the
  // session's Expired there only means "no command yet").
  mech::mech_control_core::CanonicalDeviceCommand command{};
  command.position = pending_[0].position;
  command.deadline = *MonotonicTime::from_nanoseconds(
      now.nanoseconds() + 2 * config_.control_period_nanoseconds);
  const auto result = session_.submit(command, now);
  if (result == AdapterResult::Ok) {
    fresh_write_ = false;
    submitted_once_ = true;
    return true;
  }
  if (result == AdapterResult::WouldBlock) {
    // Retryable backpressure: keep fresh_write_ and retry next cycle. The
    // command's own deadline (two control periods) bounds the retry window -
    // once it passes, submit() rejects it as InvalidCommand and this runtime
    // fails explicitly.
    return true;
  }
  return false;
}

void Ak30ForceControlRuntime::publish_states(
    CanonicalState* states, std::size_t count,
    MonotonicTime now) const noexcept {
  if (states == nullptr || count != resource_count_) {
    return;
  }
  const auto state = session_.snapshot(now);
  for (std::size_t index = 0; index < count; ++index) {
    const bool usable =
        state.status.quality == SampleQuality::Valid ||
        state.status.quality == SampleQuality::Degraded;
    states[index].position = usable ? state.position : 0.0;
    states[index].velocity = usable ? state.velocity : 0.0;
    states[index].effort = usable ? state.effort : 0.0;
  }
}

bool Ak30ForceControlRuntime::read(CanonicalState* states,
                                   std::size_t count) noexcept {
  if (!started_ || states == nullptr || count != resource_count_) {
    return false;
  }
  const MonotonicTime now = clock_();

  if (!submit_stored(now)) {
    return false;
  }

  // Drain received frames into the session. Receiving WouldBlock (no frame
  // this cycle) is the steady state, not an error.
  for (std::size_t index = 0; index < kReceiveBudget; ++index) {
    RawCanFrame frame{};
    const auto received = transport_.try_receive(frame);
    if (received == TransportResult::WouldBlock) {
      break;
    }
    if (received != TransportResult::Ok) {
      return false;
    }
    const auto processed = session_.process(frame, now);
    if (processed != AdapterResult::Ok &&
        processed != AdapterResult::InvalidCommand) {
      // A frame that is not this device's feedback (e.g. a foreign drive on
      // a shared bus) decodes as InvalidCommand and is ignored; anything
      // else is a real failure.
      return false;
    }
  }

  if (session_.fault_latched()) {
    return false;
  }

  publish_states(states, count, now);
  return true;
}

}  // namespace mech::mech_bringup
