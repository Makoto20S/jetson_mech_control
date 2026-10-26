#include "mech_bringup/dm4340_runtime.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>
#include <cmath>
#include <limits>
#include <utility>

#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

namespace mech::mech_bringup {
namespace {
using namespace mech_control_core;
using mech_protocol_damiao::DmFeedbackAvailability;

constexpr std::size_t kMaxJoints = 6U;
constexpr std::array<std::uint8_t, 8> kDisable{
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};
constexpr std::array<std::uint8_t, 8> kEnable{
    0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};

bool supported(const TransportCapabilities& caps) noexcept {
  return caps.is_valid() && caps.supports_classic_can &&
         caps.supports_standard_frames && caps.supports_non_blocking_io &&
         caps.max_payload_bytes >= 8U;
}
}  // namespace

const char* dm4340_fault_name(Dm4340FaultReason reason) noexcept {
  switch (reason) {
    case Dm4340FaultReason::None: return "none";
    case Dm4340FaultReason::StatusQuerySend: return "status_query_send";
    case Dm4340FaultReason::Receive: return "receive";
    case Dm4340FaultReason::FeedbackRejected: return "feedback_rejected";
    case Dm4340FaultReason::FeedbackExpired: return "feedback_expired";
    case Dm4340FaultReason::MotorNotEnabled: return "motor_not_enabled";
    case Dm4340FaultReason::CommandDeadline: return "command_deadline";
    case Dm4340FaultReason::CommandRejected: return "command_rejected";
    case Dm4340FaultReason::CommandPreparation: return "command_preparation";
    case Dm4340FaultReason::CommandQueue: return "command_queue";
    case Dm4340FaultReason::CommandTransmit: return "command_transmit";
    case Dm4340FaultReason::StopFailed: return "stop_failed";
  }
  return "unknown";
}

Dm4340Runtime::Dm4340Runtime(
    Transport& transport, Clock clock, Dm4340RuntimeConfig config,
    BusOwnershipRegistry& ownership)
    : transport_(transport), clock_(std::move(clock)), config_(std::move(config)),
      router_(kMaxJoints),
      bus_(config_.logical_bus, config_.physical_bus, transport_, router_,
           ownership, kMaxJoints, kMaxJoints) {}

std::uint16_t Dm4340Runtime::route(std::size_t index) const noexcept {
  return static_cast<std::uint16_t>(index + 1U);
}

bool Dm4340Runtime::configure(std::size_t count) noexcept {
  if (running_ || !clock_ || count == 0U || count > kMaxJoints ||
      count != config_.joints.size() || config_.logical_bus == 0U ||
      config_.physical_bus.empty() ||
      (!config_.motor_side_timeout_verified && !config_.supervised_effort_bench) ||
      (config_.supervised_effort_bench &&
       config_.command_mode != Dm4340CommandMode::Effort) ||
      config_.control_period_ns <= 0 ||
      config_.command_hard_ttl_ns > 6000000 ||
      config_.control_period_ns <
          (config_.command_hard_ttl_ns + 2) / 3 ||
      config_.query_period_ns <= 0 ||
      !supported(transport_.capabilities())) return false;
  if (configured_) return count == devices_.size();
  for (std::size_t index = 0; index < count; ++index) {
    const auto& joint = config_.joints[index];
    if (joint.logical_bus != config_.logical_bus ||
        joint.command_ttl_ns <= config_.control_period_ns ||
        config_.command_hard_ttl_ns <= joint.command_ttl_ns ||
        config_.query_period_ns >= joint.feedback_ttl_ns ||
        (config_.command_mode == Dm4340CommandMode::Position
             ? joint.kp <= 0.0 || joint.kd <= 0.0 ||
                   joint.max_command_velocity_rad_s != 0.0 ||
                   joint.max_command_effort_nm != 0.0 ||
                   joint.velocity_feedforward_nm != 0.0 ||
                   joint.kp * joint.max_target_error_rad / std::abs(joint.joint.scale) +
                       joint.kd * joint.max_feedback_velocity_rad_s >
                           joint.max_feedback_torque_nm
             : config_.command_mode == Dm4340CommandMode::Velocity
             ? joint.kp != 0.0 || joint.kd <= 0.0 ||
                   joint.max_command_velocity_rad_s <= 0.0 ||
                   joint.max_command_effort_nm != 0.0
             : joint.max_command_effort_nm <= 0.0 ||
                   joint.max_command_velocity_rad_s != 0.0 ||
                   joint.kp != 0.0 || joint.kd != 0.0) ||
        joint.max_feedback_velocity_rad_s <= 0.0 ||
        joint.max_feedback_torque_nm <= 0.0) return false;
    for (std::size_t prior = 0; prior < index; ++prior) {
      if (joint.command_can_id == config_.joints[prior].command_can_id ||
          joint.motor_id == config_.joints[prior].motor_id)
        return false;
    }
    mech_protocol_damiao::Dm4340MitSession probe;
    if (probe.configure(joint) != AdapterResult::Ok) return false;
  }
  try {
    devices_.resize(count);
  } catch (...) {
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (devices_[index].session.configure(config_.joints[index]) !=
        AdapterResult::Ok) return false;
    FrameFilter filter{};
    filter.format = CanFrameFormat::Standard;
    filter.value = config_.joints[index].feedback_can_id;
    filter.mask = kMaxStandardCanId;
    // Identical feedback IDs are intentional; the payload motor ID selects
    // the owning session in observe().
    if (router_.add_route({route(index), filter, 1U})) return false;
  }
  configured_ = true;
  return true;
}

bool Dm4340Runtime::start() noexcept {
  if (!configured_ || running_ || bus_.start() != RuntimeResult::Ok) return false;
  const auto now = clock_();
  for (auto& device : devices_) {
    if (device.session.activate(now) != AdapterResult::Ok) {
      for (auto& active : devices_) active.session.deactivate();
      bus_.stop();
      return false;
    }
  }
  for (auto& device : devices_) {
    device.pending.reset();
    device.lease.reset();
    device.hard_deadline.reset();
    device.last_query.reset();
    device.last_command_tx.reset();
    device.generation = 0U;
    device.ever_fresh = false;
    device.command_sent = false;
    device.claimed = false;
    device.disarmed = false;
    device.disable_pending = false;
  }
  last_query_tx_.reset();
  last_read_entry_.reset();
  last_write_entry_.reset();
  read_gap_ns_ = write_gap_ns_ = read_work_ns_ = -1;
  deadline_diagnostic_.reset();
  feedback_diagnostic_.reset();
  faulted_ = false;
  fault_reason_ = Dm4340FaultReason::None;
  rejected_status_ = 0U;
  running_ = true;
  return true;
}

void Dm4340Runtime::stop() noexcept {
  if (!running_) return;
  (void)halt_all(false);
  for (auto& device : devices_) device.session.deactivate();
  bus_.stop();
  running_ = false;
}

bool Dm4340Runtime::confirm_disabled() noexcept {
  if (!running_) return true;
  (void)halt_all(false);
  std::array<bool, kMaxJoints> confirmed{};
  // Clear queued feedback before sending a new disable/query exchange. A stale
  // disabled packet must not acknowledge this lifecycle transition.
  for (std::size_t n = 0; n < 4096U; ++n) {
    RawCanFrame frame{};
    const auto result = transport_.try_receive(frame);
    if (result == TransportResult::WouldBlock) break;
    if (result != TransportResult::Ok || n == 4095U) return false;
  }
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(300);
  // Lifecycle-only serialized disable/query exchanges: back-to-back USB-CAN
  // frames were observed to lose replies on the real dual-motor gateway.
  for (std::size_t i = 0; i < devices_.size(); ++i) {
    const auto& joint = config_.joints[i];
    auto next_send = std::chrono::steady_clock::now();
    while (!confirmed[i] && std::chrono::steady_clock::now() < deadline) {
      if (std::chrono::steady_clock::now() >= next_send) {
        if (!send_special(joint.command_can_id, kDisable)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!send_special(0x7FFU, {joint.motor_id, 0U, 0xCCU, 0U,
                                 0U, 0U, 0U, 0U})) return false;
        next_send = std::chrono::steady_clock::now() + std::chrono::milliseconds(30);
      }
      for (std::size_t n = 0; n < 128U; ++n) {
        RawCanFrame frame{};
        const auto result = transport_.try_receive(frame);
        if (result == TransportResult::WouldBlock) break;
        if (result != TransportResult::Ok) return false;
        mech_protocol_damiao::MitFeedback feedback{};
        if (mech_protocol_damiao::decode_mit_feedback(
                frame, config_.logical_bus, joint.feedback_can_id,
                joint.motor_id, joint.limits, feedback))
          confirmed[i] = feedback.raw_fault == 0U;
      }
      if (!confirmed[i]) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!confirmed[i]) break;
  }
  bool all = true;
  for (std::size_t i = 0; i < devices_.size(); ++i) all = all && confirmed[i];
  if (all) return true;
  faulted_ = true;
  fault_reason_ = Dm4340FaultReason::StopFailed;
  return false;
}

bool Dm4340Runtime::send_special(
    std::uint32_t id, const std::array<std::uint8_t, 8>& bytes) noexcept {
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  for (std::size_t i = 0; i < bytes.size(); ++i) payload[i] = bytes[i];
  const auto can_id = CanId::create(id, CanFrameFormat::Standard);
  if (!can_id) return false;
  const auto frame = RawCanFrame::create(
      config_.logical_bus, *can_id, CanFrameType::Classic,
      FrameDirection::Tx, 8U, payload, clock_());
  return frame && transport_.try_send(*frame) == TransportResult::Ok;
}

bool Dm4340Runtime::halt_device(std::size_t index) noexcept {
  auto& device = devices_[index];
  device.pending.reset();
  device.lease.reset();
  device.hard_deadline.reset();
  device.claimed = false;
  if (running_) (void)bus_.cancel(route(index));
  const auto sample = device.session.snapshot(clock_());
  const bool enabled = sample.availability == DmFeedbackAvailability::Fresh &&
      mech_protocol_damiao::feedback_motor_enabled(sample.raw_fault);
  bool stopped = true;
  if (running_ && !device.disarmed && (device.command_sent || enabled)) {
    device.disarmed = true;
    device.disable_pending = true;
    if (device.command_sent) {
      std::array<std::uint8_t, 8> neutral{};
      stopped = mech_protocol_damiao::encode_mit(
          {0.0, 0.0, 0.0, config_.joints[index].kd, 0.0},
          config_.joints[index].limits, neutral) &&
          send_special(config_.joints[index].command_can_id, neutral);
    }
  }
  if (running_ && device.disable_pending) {
    const bool disabled = send_special(
        config_.joints[index].command_can_id, kDisable);
    device.disable_pending = !disabled;
    stopped = disabled && stopped;
  }
  device.command_sent = false;
  return stopped;
}

bool Dm4340Runtime::halt_all(bool fault) noexcept {
  bool stopped = true;
  for (std::size_t index = 0; index < devices_.size(); ++index)
    stopped = halt_device(index) && stopped;
  faulted_ = faulted_ || fault || !stopped;
  if (!stopped && fault_reason_ == Dm4340FaultReason::None)
    fault_reason_ = Dm4340FaultReason::StopFailed;
  return stopped && !fault;
}

bool Dm4340Runtime::fail(Dm4340FaultReason reason) noexcept {
  if (fault_reason_ == Dm4340FaultReason::None) fault_reason_ = reason;
  return halt_all(true);
}

bool Dm4340Runtime::fail_deadline(
    std::size_t index, MonotonicTime now, bool during_write) noexcept {
  if (!deadline_diagnostic_) {
    const auto& device = devices_[index];
    const auto duration = [now](const std::optional<MonotonicTime>& from) {
      const auto elapsed = from ? elapsed_since(*from, now) : std::nullopt;
      return elapsed ? elapsed->nanoseconds() : std::int64_t{-1};
    };
    const auto overdue = elapsed_since(*device.hard_deadline, now)->nanoseconds();
    deadline_diagnostic_ = Dm4340DeadlineDiagnostic{
        index, during_write, config_.command_hard_ttl_ns + overdue, overdue,
        read_gap_ns_, write_gap_ns_, read_work_ns_,
        duration(device.last_command_tx)};
  }
  return fail(Dm4340FaultReason::CommandDeadline);
}

bool Dm4340Runtime::observe(void* context, std::uint16_t selected,
                                    const RawCanFrame& frame) noexcept {
  auto& self = *static_cast<Dm4340Runtime*>(context);
  if (selected == 0U || selected > self.devices_.size()) {
    self.fault_reason_ = Dm4340FaultReason::FeedbackRejected;
    self.faulted_ = true;
    return false;
  }
  const auto index = static_cast<std::size_t>(selected - 1U);
  if (frame.payload_size == 8U &&
      (frame.payload[0] & 0x0FU) != self.config_.joints[index].motor_id)
    return true;
  if (self.devices_[index].session.accept_feedback(frame, self.clock_()) !=
      AdapterResult::Ok) {
    if (self.fault_reason_ == Dm4340FaultReason::None) {
      self.fault_reason_ = Dm4340FaultReason::FeedbackRejected;
      self.rejected_status_ = frame.payload_size > 0U ? frame.payload[0] >> 4U : 0U;
      const auto& diagnostic = self.devices_[index].session.feedback_diagnostic();
      if (diagnostic) self.feedback_diagnostic_ = Dm4340FeedbackDiagnostic{index, *diagnostic};
    }
    self.faulted_ = true;
    return false;
  }
  return true;
}

bool Dm4340Runtime::authorize_send(
    void* context, const CommandLease& selected, MonotonicTime& now) noexcept {
  auto& self = *static_cast<Dm4340Runtime*>(context);
  now = self.clock_();
  if (selected.route_id == 0U || selected.route_id > self.devices_.size())
    return false;
  const auto index = static_cast<std::size_t>(selected.route_id - 1U);
  const auto& device = self.devices_[index];
  return device.claimed && !device.disarmed && device.pending && device.lease &&
         device.hard_deadline && now < *device.hard_deadline &&
         selected.generation == device.pending->generation &&
         device.session.pending_target_still_authorized(
             selected.frame, selected.generation, selected.deadline, now);
}

bool Dm4340Runtime::prepare_and_transmit(MonotonicTime now) noexcept {
  if (!bus_.last_receive_drained()) return true;
  bool any_pending = false;
  for (std::size_t index = 0; index < devices_.size(); ++index) {
    auto& device = devices_[index];
    if (!device.pending || device.pending->deadline <= now) continue;
    any_pending = true;
    if (device.lease) continue;
    RawCanFrame frame{};
    const auto prepared = config_.command_mode == Dm4340CommandMode::Velocity
        ? device.session.prepare_velocity(*device.pending, now, frame)
        : config_.command_mode == Dm4340CommandMode::Effort
        ? device.session.prepare_effort(*device.pending, now, frame)
        : device.session.prepare_position(*device.pending, now, frame);
    if (prepared !=
        AdapterResult::Ok) return fail(Dm4340FaultReason::CommandPreparation);
    const auto submitted = CommandLease::create(
        route(index), device.pending->generation, frame, now,
        device.pending->deadline);
    if (!submitted || bus_.submit(*submitted) != RuntimeResult::Ok)
      return fail(Dm4340FaultReason::CommandQueue);
    device.lease = *submitted;
  }
  if (!any_pending) return true;
  const auto result = bus_.transmit(clock_(), {this, &authorize_send});
  const auto tx_observed = clock_();
  if (result != RuntimeResult::Ok && result != RuntimeResult::QueueFull)
    return fail(Dm4340FaultReason::CommandTransmit);
  for (std::size_t index = 0; index < devices_.size(); ++index) {
    auto& device = devices_[index];
    if (device.pending && bus_.was_sent(route(index), device.pending->generation)) {
      device.command_sent = true;
      device.last_command_tx = tx_observed;
      device.pending.reset();
      device.lease.reset();
    }
  }
  return true;
}

bool Dm4340Runtime::read(
    mech_hardware_ros2_control::CanonicalState* states,
    std::size_t count) noexcept {
  if (!running_ || faulted_ || !states || count != devices_.size()) return false;
  auto now = clock_();
  const auto read_gap = last_read_entry_ ? elapsed_since(*last_read_entry_, now)
                                         : std::nullopt;
  read_gap_ns_ = read_gap ? read_gap->nanoseconds() : -1;
  last_read_entry_ = now;
  for (std::size_t index = 0; index < devices_.size(); ++index) {
    auto& device = devices_[index];
    const auto elapsed = device.last_query ? elapsed_since(*device.last_query, now)
                                           : std::nullopt;
    if (!device.last_query || !elapsed ||
        elapsed->nanoseconds() >= config_.query_period_ns) {
      if (config_.stagger_status_queries && last_query_tx_) {
        const auto gap = elapsed_since(*last_query_tx_, now);
        if (gap && gap->nanoseconds() < config_.query_period_ns /
                static_cast<std::int64_t>(devices_.size())) continue;
      }
      const std::array<std::uint8_t, 8> query{
          config_.joints[index].motor_id, 0U, 0xCCU, 0U, 0U, 0U, 0U, 0U};
      if (!send_special(0x7FFU, query))
        return fail(Dm4340FaultReason::StatusQuerySend);
      device.last_query = now;
      last_query_tx_ = now;
    }
  }
  if (bus_.receive(now, {this, &observe}) != RuntimeResult::Ok || faulted_)
    return fail(Dm4340FaultReason::Receive);
  now = clock_();
  const auto read_work = elapsed_since(*last_read_entry_, now);
  read_work_ns_ = read_work ? read_work->nanoseconds() : -1;
  for (std::size_t index = 0; index < devices_.size(); ++index) {
    auto& device = devices_[index];
    const auto sample = device.session.snapshot(now);
    if (sample.availability != DmFeedbackAvailability::Fresh) {
      if (device.ever_fresh || device.pending || device.command_sent)
        return fail(Dm4340FaultReason::FeedbackExpired);
      continue;
    }
    device.ever_fresh = true;
    states[index].position = sample.joint_position_rad;
    states[index].velocity = sample.raw_motor_velocity_rad_s *
        config_.joints[index].joint.scale;
    states[index].effort = sample.raw_motor_torque_nm /
        config_.joints[index].joint.scale;
    if (device.command_sent &&
        !mech_protocol_damiao::feedback_motor_enabled(sample.raw_fault))
      return fail(Dm4340FaultReason::MotorNotEnabled);
    if (device.hard_deadline && *device.hard_deadline <= now)
      return fail_deadline(index, now, false);
  }
  return prepare_and_transmit(now);
}

bool Dm4340Runtime::write(
    const mech_hardware_ros2_control::CommandDispatch* commands,
    std::size_t count) noexcept {
  if (!running_ || faulted_ || !commands || count != devices_.size())
    return false;
  const auto now = clock_();
  const auto write_gap = last_write_entry_ ? elapsed_since(*last_write_entry_, now)
                                           : std::nullopt;
  write_gap_ns_ = write_gap ? write_gap->nanoseconds() : -1;
  last_write_entry_ = now;
  for (std::size_t index = 0; index < count; ++index) {
    const auto& dispatch = commands[index];
    const auto& device = devices_[index];
    if (!dispatch.authorized) continue;
    if (device.hard_deadline && *device.hard_deadline <= now)
      return fail_deadline(index, now, true);
    if (!dispatch.fresh) continue;
    const auto& config = config_.joints[index];
    if (!valid_sample(index) || !std::isfinite(dispatch.command.velocity) ||
        !std::isfinite(dispatch.command.position) ||
        !std::isfinite(dispatch.command.effort) ||
        (config_.command_mode == Dm4340CommandMode::Position
             ? dispatch.command.velocity != 0.0 || dispatch.command.effort != 0.0
             : config_.command_mode == Dm4340CommandMode::Velocity
             ? dispatch.command.position != 0.0 || dispatch.command.effort != 0.0 ||
                   std::abs(dispatch.command.velocity) >
                       config.max_command_velocity_rad_s *
                           std::abs(config.joint.scale)
             : dispatch.command.position != 0.0 || dispatch.command.velocity != 0.0) ||
        now.nanoseconds() > std::numeric_limits<std::int64_t>::max() -
                                config_.command_hard_ttl_ns ||
        device.generation == std::numeric_limits<std::uint64_t>::max())
      return fail(Dm4340FaultReason::CommandRejected);
  }
  for (std::size_t index = 0; index < count; ++index) {
    auto& device = devices_[index];
    const auto& dispatch = commands[index];
    if (!dispatch.authorized) {
      if (device.claimed) cancel_pending(index);
      if (faulted_) return false;
      continue;
    }
    device.claimed = true;
    if (!dispatch.fresh) continue;
    (void)bus_.cancel(route(index));
    device.lease.reset();
    ++device.generation;
    device.pending = CanonicalDeviceCommand{
        dispatch.command.position, dispatch.command.velocity,
        config_.command_mode == Dm4340CommandMode::Effort
            ? std::clamp(dispatch.command.effort,
                         -config_.joints[index].max_command_effort_nm,
                         config_.joints[index].max_command_effort_nm)
            : dispatch.command.effort, device.generation,
        *MonotonicTime::from_nanoseconds(
            now.nanoseconds() + config_.joints[index].command_ttl_ns)};
    device.hard_deadline = *MonotonicTime::from_nanoseconds(
        now.nanoseconds() + config_.command_hard_ttl_ns);
  }
  return true;
}

void Dm4340Runtime::cancel_pending(std::size_t index) noexcept {
  if (!running_ || index >= devices_.size()) return;
  auto& device = devices_[index];
  if (!device.claimed && !device.pending && !device.command_sent &&
      !device.disable_pending) return;
  if (!halt_device(index)) (void)fail(Dm4340FaultReason::StopFailed);
}

std::optional<std::uint8_t> Dm4340Runtime::fresh_feedback_status(
    std::size_t index) const noexcept {
  if (!running_ || faulted_ || index >= devices_.size()) return std::nullopt;
  const auto sample = devices_[index].session.snapshot(clock_());
  if (sample.availability != DmFeedbackAvailability::Fresh) return std::nullopt;
  return sample.raw_fault;
}

bool Dm4340Runtime::valid_sample(std::size_t index) const noexcept {
  const auto& device = devices_[index];
  if (!running_ || faulted_ || device.disarmed) return false;
  const auto sample = device.session.snapshot(clock_());
  return sample.availability == DmFeedbackAvailability::Fresh &&
         mech_protocol_damiao::feedback_motor_enabled(sample.raw_fault);
}

bool Dm4340Runtime::has_valid_sample() const noexcept {
  if (!running_ || faulted_) return false;
  for (std::size_t index = 0; index < devices_.size(); ++index)
    if (!valid_sample(index)) return false;
  return true;
}

bool Dm4340Runtime::has_fresh_ready_feedback() const noexcept {
  if (!running_ || faulted_ || devices_.empty()) return false;
  for (const auto& device : devices_) {
    const auto sample = device.session.snapshot(clock_());
    if (device.claimed || device.pending || device.command_sent ||
        device.disarmed || device.disable_pending ||
        sample.availability != DmFeedbackAvailability::Fresh ||
        !mech_protocol_damiao::feedback_status_ok(sample.raw_fault)) return false;
  }
  return true;
}

bool Dm4340Runtime::adopt_or_enable_from_feedback() noexcept {
  if (!has_fresh_ready_feedback()) return false;
  for (std::size_t index = 0; index < devices_.size(); ++index) {
    auto& device = devices_[index];
    const auto sample = device.session.snapshot(clock_());
    if (sample.availability != DmFeedbackAvailability::Fresh ||
        !mech_protocol_damiao::feedback_status_ok(sample.raw_fault)) {
      (void)halt_all(true);
      return false;
    }
    if (sample.raw_fault == 0U) {
      if (!send_special(config_.joints[index].command_can_id, kEnable)) {
        (void)halt_all(true);
        return false;
      }
      // Lifecycle-only handshake, not the active control loop. The gateway
      // needs spaced management frames, including consecutive enable requests.
      if (config_.stagger_status_queries)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    device.disable_pending = true;
  }
  return true;
}

}  // namespace mech::mech_bringup
