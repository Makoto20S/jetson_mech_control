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
using mech::mech_hardware_ros2_control::CommandDispatch;
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
  if (config.sub_mode ==
          mech::mech_protocol_cubemars::ForceControlSubMode::Torque &&
      (!std::isfinite(config.torque_max_abs_erpm) ||
       config.torque_max_abs_erpm <= 0.0)) {
    return false;
  }
  // ADR-019: checked in every sub-mode, because an envelope that cannot be
  // evaluated is not an envelope. The error bound is the ceiling on the
  // torque the device's own Kp term may develop, so zero would forbid every
  // command while a non-finite or unordered bound would forbid none.
  if (!std::isfinite(config.position_min_rad) ||
      !std::isfinite(config.position_max_rad) ||
      !std::isfinite(config.position_max_error_rad) ||
      config.position_min_rad >= config.position_max_rad ||
      config.position_max_error_rad <= 0.0) {
    return false;
  }
  if (config.drive_id > 255U || config.logical_bus == 0U ||
      config.control_period_nanoseconds <= 0 ||
      config.command_ttl_nanoseconds <= 0 ||
      config.command_hard_ttl_nanoseconds <=
          config.command_ttl_nanoseconds ||
      config.command_hard_ttl_nanoseconds > kMaxHardTtlNanoseconds ||
      config.feedback_period_nanoseconds <= 0 ||
      config.feedback_ttl_nanoseconds <= 0 ||
      // ADR-016 Decision 4: a window shorter than the device's own reporting
      // period can never be satisfied. motor1 reports at a configured 50 Hz,
      // so the 6 ms that used to be the default here was 3.3x too short.
      config.feedback_ttl_nanoseconds < config.feedback_period_nanoseconds ||
      !std::isfinite(config.gains.kp) || config.gains.kp < 0.0 ||
      !std::isfinite(config.gains.kd) || config.gains.kd < 0.0 ||
      !mapping_is_sufficient(config.mapping, config.sub_mode)) {
    return false;
  }
  return true;
}

// The CanonicalCommand member this runtime's sub-mode consumes (ADR-014):
// CompositeSystem routes the joint's single command interface into one
// member, and the sub-mode decides which member reaches the device command.
[[nodiscard]] double consumed_command_value(
    const Ak30RuntimeConfig& config,
    const mech_hardware_ros2_control::CanonicalCommand& command) noexcept {
  switch (config.sub_mode) {
    case mech::mech_protocol_cubemars::ForceControlSubMode::Velocity:
      return command.velocity;
    case mech::mech_protocol_cubemars::ForceControlSubMode::Torque:
      return command.effort;
    case mech::mech_protocol_cubemars::ForceControlSubMode::Position:
      break;
  }
  return command.position;
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
    Ak30RuntimeConfig config, FeedbackTelemetryCapture* telemetry) noexcept
    : transport_(transport),
      clock_(std::move(clock)),
      config_(config),
      session_(transport, session_config_from(config)), telemetry_(telemetry) {}

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
  pending_deadline_ = MonotonicTime{};
  have_pending_ = false;
  fresh_write_ = false;
  submitted_once_ = false;
  holding_ = false;
  expired_ = false;
  last_status_ = mech::mech_control_core::StatusSnapshot{};
  has_valid_sample_ = false;
  has_observed_status_ = false;
  raw_erpm_available_ = false;
  torque_overspeed_latched_ = false;
  position_envelope_latched_ = false;
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
  raw_erpm_available_ = false;
  torque_overspeed_latched_ = false;
  position_envelope_latched_ = false;
  started_ = true;
  return true;
}

void Ak30ForceControlRuntime::stop() noexcept {
  if (!started_) {
    return;
  }
  session_.deactivate();
  started_ = false;
  pending_deadline_ = MonotonicTime{};
  have_pending_ = false;
  fresh_write_ = false;
  submitted_once_ = false;
  holding_ = false;
  expired_ = false;
  last_status_ = mech::mech_control_core::StatusSnapshot{};
  has_valid_sample_ = false;
  has_observed_status_ = false;
  raw_erpm_available_ = false;
}

void Ak30ForceControlRuntime::capture_status(
    FeedbackTelemetryKind kind, FeedbackTelemetryReason reason,
    const mech::mech_control_core::StatusSnapshot& status,
    MonotonicTime observed_at) noexcept {
  if (telemetry_ == nullptr) return;
  FeedbackTelemetryEvent event{};
  event.kind = kind;
  event.reason = reason;
  event.host_receive_sequence = status.sequence;
  event.observed_at_nanoseconds = observed_at.nanoseconds();
  event.quality = status.quality;
  event.device_state = status.device_state;
  event.raw_fault_code = status.raw_fault_code;
  event.raw_erpm_available = raw_erpm_available_ &&
      status.sequence == raw_erpm_status_.sequence &&
      status.host_rx_time == raw_erpm_status_.host_rx_time;
  if (event.raw_erpm_available) event.raw_erpm = raw_erpm_;
  event.host_rx_available = status.host_rx_time.has_value();
  if (event.host_rx_available) {
    event.host_rx_nanoseconds = status.host_rx_time->nanoseconds();
    const auto age = mech::mech_control_core::elapsed_since(
        *status.host_rx_time, observed_at);
    if (age.has_value()) {
      event.age_available = true;
      event.age_nanoseconds = age->nanoseconds();
    }
  }
  (void)telemetry_->try_push(event);
}

void Ak30ForceControlRuntime::capture_error(
    FeedbackTelemetryReason reason, MonotonicTime observed_at) noexcept {
  last_status_ = session_.snapshot(observed_at).status;
  if (!has_observed_status_ ||
      last_status_.quality != observed_status_.quality ||
      last_status_.device_state != observed_status_.device_state ||
      last_status_.raw_fault_code != observed_status_.raw_fault_code) {
    capture_status(FeedbackTelemetryKind::StatusTransition,
                   FeedbackTelemetryReason::None, last_status_, observed_at);
    observed_status_ = last_status_;
    has_observed_status_ = true;
  }
  capture_status(FeedbackTelemetryKind::RuntimeError, reason, last_status_,
                 observed_at);
}

bool Ak30ForceControlRuntime::write(const CommandDispatch* commands,
                                    std::size_t count) noexcept {
  if (!started_ || commands == nullptr || count != resource_count_) {
    return false;
  }
  // ADR-015: an unauthorized dispatch is not a command. It must not become a
  // pending command and must not refresh an earlier one - "the
  // controller_manager called write()" is not "a controller commanded
  // something", because ros2_control command interfaces are raw double
  // pointers and the hardware layer cannot observe set_value() calls.
  bool fresh_any = false;
  for (std::size_t index = 0; index < count; ++index) {
    if (!commands[index].authorized) continue;
    // ADR-017: freshness gates whether a NEW command is accepted, but not
    // whether the value is checked. A non-finite command is a defect on the
    // cycle it appears, stale or not, and skipping the check here would make
    // the amount of validation depend on how talkative the controller is.
    if (commands[index].fresh) fresh_any = true;
    // Validate the field(s) the sub-mode consumes (ADR-014): a Torque-mode
    // device reads effort, a Velocity-mode device reads velocity, and a
    // Position-mode device reads position. The other members stay at their
    // constructed zeros and are never mapped into the device command.
    if (!std::isfinite(consumed_command_value(config_, commands[index].command))) {
      return false;
    }
  }
  // ADR-017 Decision 3.3: the claim is held and the manager cycled, but no
  // controller refreshed anything. Returning here leaves any pending command
  // alone on purpose - it is still bounded by its own deadline, and a
  // controller falling quiet is not a reason to abandon a command it already
  // gave. That is what separates this from losing authorization, which does
  // drop the pending command through cancel_pending().
  if (!fresh_any) {
    return true;
  }
  // The validity window is minted here, once, from the moment the command was
  // received - not in submit_stored(), which runs again on every retry.
  const auto issued_at = clock_().nanoseconds();
  const auto maximum = std::numeric_limits<std::int64_t>::max();
  if (config_.control_period_nanoseconds > (maximum - issued_at) / 2) {
    return false;
  }
  const auto deadline = MonotonicTime::from_nanoseconds(
      issued_at + 2 * config_.control_period_nanoseconds);
  if (!deadline.has_value()) {
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (commands[index].authorized && commands[index].fresh) {
      pending_[index] = commands[index].command;
    }
  }
  pending_deadline_ = *deadline;
  have_pending_ = true;
  // The controller refreshed its command: the next read submits it. Before
  // ADR-017 every authorized write counted as a refresh, which is exactly the
  // defect - the hardware could not tell a controller apart from the loop.
  fresh_write_ = true;
  return true;
}

void Ak30ForceControlRuntime::cancel_pending(std::size_t index) noexcept {
  if (index >= resource_count_) {
    return;
  }
  // This runtime carries a single joint (configure() rejects more), so
  // dropping its pending command clears the whole lease. The session's own
  // command stage is left alone: submit() is simply never called again until
  // a new authorized write arrives, which is what stops transmission.
  pending_[index] = CanonicalCommand{};
  pending_deadline_ = MonotonicTime{};
  have_pending_ = false;
  fresh_write_ = false;
  // ADR-015 Decision 3: revocation must end the lease as an immediate state
  // transition, and must not be left to the hard TTL to achieve. Without this
  // the session's stage keeps ageing the pre-revoke command, so one hard TTL
  // after any clean deactivation read() fails and the ResourceManager takes the
  // component into an error state - measured at exactly 8 ms for a command
  // submitted at 2 ms with a 6 ms hard TTL. That turns every ordinary
  // controller swap (T8/T9 activate one controller in place of another) into a
  // hardware fault.
  //
  // Resetting this returns the runtime to its post-start() state: no command
  // has been accepted, so the session's Expired stage means "nothing yet"
  // rather than "we let a live command go stale", which is the distinction
  // submit_stored() draws. holding_ is deliberately NOT reset - it is derived
  // from the session and describes the DEVICE, which really is still sitting on
  // the last target it was given until its own loss-of-control backstop fires.
  //
  // This opens no hole: nothing transmits until a new authorized AND fresh
  // write arrives (ADR-017), and the watchdog re-arms the moment one is
  // submitted.
  submitted_once_ = false;
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
    // controller refresh: send nothing. This is what implements ADR-012's
    // Holding freeze - the last accepted command stays the device's commanded
    // state and the session's stage clock keeps advancing on real time, so the
    // escalation to Expired actually happens. Re-submitting would reset that
    // clock and mask a dead controller; inventing 0.0 would command a move to
    // the zero position (ADR-012 Decision 3).
    return true;
  }
  if (pending_deadline_.nanoseconds() <= now.nanoseconds()) {
    // The command outlived its own window while the transport was blocked (or
    // while nobody read). Drop it and fail explicitly. This check sits ahead
    // of submitted_once_ on purpose: "nothing has gone out yet" is not a
    // reason to keep an aged target alive, and the session would reject the
    // frame anyway - failing here makes the reason legible instead of
    // arriving as a generic InvalidCommand.
    pending_[0] = CanonicalCommand{};
    pending_deadline_ = MonotonicTime{};
    have_pending_ = false;
    fresh_write_ = false;
    return false;
  }
  // ADR-019: the hardware-side envelope. Absolute bounds are checked
  // unconditionally; the error bound needs a usable feedback sample, and
  // ADR-016 already refuses to reach this point on stale feedback, so the
  // only sample-less path is the never-sampled startup transient, where no
  // claim can exist. Nothing is clamped: a violating target is dropped, the
  // runtime latches, and read() keeps failing until the lifecycle restarts.
  if (config_.sub_mode ==
      mech::mech_protocol_cubemars::ForceControlSubMode::Position) {
    const double target = pending_[0].position;
    const auto state = session_.snapshot(now);
    const bool usable = state.status.quality == SampleQuality::Valid ||
                        state.status.quality == SampleQuality::Degraded;
    const bool outside_bounds = target < config_.position_min_rad ||
                                target > config_.position_max_rad;
    const bool outside_error = usable &&
        std::abs(target - state.position) > config_.position_max_error_rad;
    if (outside_bounds || outside_error) {
      position_envelope_latched_ = true;
      has_valid_sample_ = false;
      pending_[0] = CanonicalCommand{};
      pending_deadline_ = MonotonicTime{};
      have_pending_ = false;
      fresh_write_ = false;
      capture_error(FeedbackTelemetryReason::PositionEnvelope, now);
      return false;
    }
  }
  // Reaching here means a FRESH command is in hand, so a Holding stage must
  // not block it. The stage describes the age of the session's last ACCEPTED
  // command - a fact about the past - while the freeze that stage calls for is
  // about not replaying that command, which the fresh_write_ check above
  // already enforces. Refusing here instead would silently swallow the first
  // command of a controller that resumes after a gap, or of a controller that
  // just re-claimed the joint, for the width of the Holding window. Submitting
  // resets the stage clock, which is correct: a new target really does put the
  // session back into Following.
  //
  // The session's Expired stage before the first submit only means "no command
  // yet" and was already separated from the real failure above.
  mech::mech_control_core::CanonicalDeviceCommand command{};
  // ADR-014: map only the member the sub-mode consumes into the device
  // command; the others keep CanonicalDeviceCommand's zeros, so a Torque
  // device never sees a position/velocity and a Position device never sees
  // an effort. Velocity forces effort = 0: the wire's effort field rides
  // along as feedforward t_ff in every sub-mode, and the bench-proven Kd
  // bound (torque <= Kd * velocity command) only holds without feedforward
  // - the probe discipline, now enforced by the runtime instead of trusting
  // the caller.
  switch (config_.sub_mode) {
    case mech::mech_protocol_cubemars::ForceControlSubMode::Velocity:
      command.velocity = pending_[0].velocity;
      break;
    case mech::mech_protocol_cubemars::ForceControlSubMode::Torque:
      command.effort = pending_[0].effort;
      break;
    case mech::mech_protocol_cubemars::ForceControlSubMode::Position:
      command.position = pending_[0].position;
      break;
  }
  command.deadline = pending_deadline_;
  const auto result = session_.submit(command, now);
  if (result == AdapterResult::Ok) {
    fresh_write_ = false;
    submitted_once_ = true;
    // Counted here and nowhere else: this is the one point where a device
    // command is known to have left the host. The WouldBlock branch below
    // deliberately does not count - it retries the same command.
    ++motor_command_frames_;
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

bool Ak30ForceControlRuntime::publish_states(
    CanonicalState* states, std::size_t count, MonotonicTime now) noexcept {
  if (states == nullptr || count != resource_count_) {
    return false;
  }
  const auto state = session_.snapshot(now);
  last_status_ = state.status;
  if (!has_observed_status_ ||
      state.status.quality != observed_status_.quality ||
      state.status.device_state != observed_status_.device_state ||
      state.status.raw_fault_code != observed_status_.raw_fault_code) {
    capture_status(FeedbackTelemetryKind::StatusTransition,
                   FeedbackTelemetryReason::None, state.status, now);
    observed_status_ = state.status;
    has_observed_status_ = true;
  }
  const bool usable = state.status.quality == SampleQuality::Valid ||
                      state.status.quality == SampleQuality::Degraded;
  has_valid_sample_ = usable;
  if (!usable) {
    // ADR-016 Decision 1: leave the caller's buffer alone. Writing 0.0 here
    // would publish a number that is indistinguishable from a real
    // measurement - on the position interface, from the joint actually being
    // at the zero position.
    //
    // Decision 2/3: never having sampled is the normal startup transient at
    // 50 Hz reporting against a 500 Hz loop, so it is survivable; the joint
    // stays unclaimable via has_valid_sample() and therefore uncommandable.
    // Having sampled and then aged out means the device stopped talking,
    // which is a fault.
    return !state.status.has_sample();
  }
  for (std::size_t index = 0; index < count; ++index) {
    states[index].position = state.position;
    states[index].velocity = state.velocity;
    states[index].effort = state.effort;
  }
  return true;
}

bool Ak30ForceControlRuntime::read(CanonicalState* states,
                                   std::size_t count) noexcept {
  if (!started_ || states == nullptr || count != resource_count_) {
    return false;
  }
  const bool torque_mode = config_.sub_mode ==
      mech::mech_protocol_cubemars::ForceControlSubMode::Torque;
  if (torque_overspeed_latched_) {
    has_valid_sample_ = false;
    capture_error(FeedbackTelemetryReason::TorqueOverspeed, clock_());
    return false;
  }
  if (position_envelope_latched_) {
    has_valid_sample_ = false;
    capture_error(FeedbackTelemetryReason::PositionEnvelope, clock_());
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
      capture_error(FeedbackTelemetryReason::TransportReceive, clock_());
      return false;
    }
    // Observe after dequeue: a transport can stamp a frame while this read
    // cycle is draining, so the pre-drain cycle timestamp may be in its past.
    const MonotonicTime observed_at = clock_();
    if (frame.logical_bus != config_.logical_bus) continue;
    const auto processed = session_.process(frame, observed_at);
    if (processed != AdapterResult::Ok &&
        processed != AdapterResult::InvalidCommand) {
      // A frame that is not this device's feedback (e.g. a foreign drive on
      // a shared bus) decodes as InvalidCommand and is ignored; anything
      // else is a real failure.
      capture_error(FeedbackTelemetryReason::FrameProcessing, clock_());
      return false;
    }
    if (processed == AdapterResult::Ok) {
      const auto status = session_.snapshot(observed_at).status;
      // Session acceptance proves shape, device identity and timestamp ordering.
      // Reuse the wire decoder to retain the raw field that canonical Torque
      // state deliberately cannot represent. Never derive ERPM from SI velocity.
      mech::mech_protocol_cubemars::ForceControlPayload payload{};
      for (std::size_t byte = 0; byte < payload.size(); ++byte) {
        payload[byte] = frame.payload[byte];
      }
      mech::mech_protocol_cubemars::ForceControlFeedback feedback{};
      mech::mech_protocol_cubemars::decode_feedback(payload, feedback);
      raw_erpm_ = feedback.electrical_speed_erpm;
      raw_erpm_status_ = status;
      raw_erpm_available_ = true;
      last_status_ = status;
      capture_status(FeedbackTelemetryKind::FeedbackFrame,
                     FeedbackTelemetryReason::None, status, observed_at);
      if (!has_observed_status_ || status.quality != observed_status_.quality ||
          status.device_state != observed_status_.device_state ||
          status.raw_fault_code != observed_status_.raw_fault_code) {
        capture_status(FeedbackTelemetryKind::StatusTransition,
                       FeedbackTelemetryReason::None, status, observed_at);
        observed_status_ = status;
        has_observed_status_ = true;
      }
      // Check every accepted frame, even when another frame is already queued.
      // The crossing is a lifecycle latch; claim cancellation cannot clear it.
      if (torque_mode && std::abs(raw_erpm_) > config_.torque_max_abs_erpm) {
        torque_overspeed_latched_ = true;
        has_valid_sample_ = false;
        capture_error(FeedbackTelemetryReason::TorqueOverspeed, observed_at);
      }
    }
  }

  if (torque_overspeed_latched_) return false;

  if (session_.fault_latched()) {
    last_status_ = session_.snapshot(clock_()).status;
    capture_error(FeedbackTelemetryReason::FaultLatched, clock_());
    return false;
  }

  const auto observed_at = clock_();
  if (!publish_states(states, count, observed_at)) {
    capture_error(FeedbackTelemetryReason::FeedbackUnusable, observed_at);
    return false;
  }
  // Feedback quality is the safety gate for transmission. Validate it before
  // submitting a fresh controller command so a target refreshed during lost
  // feedback cannot produce one final frame on the cycle that detects Stale.
  const MonotonicTime submission_now = clock_();
  if (torque_mode) {
    const auto status = session_.snapshot(submission_now).status;
    const bool usable = raw_erpm_available_ && status.has_sample() &&
        status.sequence == raw_erpm_status_.sequence &&
        status.host_rx_time == raw_erpm_status_.host_rx_time &&
        (status.quality == SampleQuality::Valid ||
         status.quality == SampleQuality::Degraded) &&
        !session_.fault_latched();
    if (!usable && (have_pending_ || status.has_sample())) {
      has_valid_sample_ = false;
      capture_error(FeedbackTelemetryReason::TorqueSpeedUnavailable, submission_now);
      return false;
    }
  }
  if (!submit_stored(submission_now)) {
    // The envelope already named the reason; a generic one on top of it would
    // point the reader at the transport instead of at the target.
    if (!position_envelope_latched_) {
      capture_error(FeedbackTelemetryReason::CommandSubmission, submission_now);
    }
    return false;
  }
  return true;
}

}  // namespace mech::mech_bringup
