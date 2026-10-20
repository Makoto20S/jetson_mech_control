#include "mech_bringup/ak30_servo_runtime.hpp"

#include <cmath>
#include "mech_bringup/command_trace.hpp"
#include <limits>
#include <utility>

namespace mech::mech_bringup {
namespace {
using namespace mech_control_core;
using mech_protocol_cubemars::ServoPositionAvailability;

constexpr std::size_t kMaxJoints = 6U;
constexpr std::int64_t kMaxHardTtlNs = 6000000;

bool supports_servo(const TransportCapabilities& caps) noexcept {
  return caps.is_valid() && caps.supports_classic_can &&
         caps.supports_extended_frames && caps.supports_non_blocking_io &&
         caps.max_payload_bytes >= 8U;
}
}  // namespace

Ak30ServoRuntime::Ak30ServoRuntime(Transport& transport, Clock clock,
                                   Ak30ServoRuntimeConfig config,
                                   BusOwnershipRegistry& ownership, CommandTrace* trace)
    : trace_(trace), transport_(transport),
      clock_(std::move(clock)),
      config_(std::move(config)),
      router_(kMaxJoints),
      bus_(config_.logical_bus, config_.physical_bus, transport_, router_,
           ownership, kMaxJoints, kMaxJoints) {}

void Ak30ServoRuntime::record_fault(Ak30ServoFaultReason reason,
    Ak30ServoFaultPhase phase, MonotonicTime now, std::size_t index,
    AdapterResult adapter, RuntimeResult runtime) noexcept {
  if (first_fault_) return;
  Ak30ServoFault fault{};
  fault.reason = reason;
  fault.phase = phase;
  fault.now = now;
  fault.index = index;
  fault.previous_time_ns = last_time_ ? last_time_->nanoseconds() : 0;
  fault.adapter_result = adapter;
  fault.runtime_result = runtime;
  if (index < sessions_.size()) {
    fault.drive_id = config_.joints[index].drive_id;
    fault.max_target_error_rad = config_.joints[index].max_target_error_rad;
    fault.feedback = sessions_[index].snapshot(now);
    const auto& entry = pending_[index];
    if (entry.command) {
      fault.target_position_rad = entry.command->position;
      fault.soft_deadline_ns = entry.command->deadline.nanoseconds();
      fault.hard_deadline_ns = entry.hard_deadline.nanoseconds();
    }
  }
  first_fault_ = fault;
  if (trace_) trace_->record("fault", index, 0, fault.target_position_rad,
      fault.feedback.position_rad, static_cast<int>(reason), now.nanoseconds(),
      static_cast<int>(phase));
}

const char* fault_reason_name(Ak30ServoFaultReason reason) noexcept {
  switch (reason) {
    case Ak30ServoFaultReason::ClockRegression: return "ClockRegression";
    case Ak30ServoFaultReason::FeedbackRejected: return "FeedbackRejected";
    case Ak30ServoFaultReason::FeedbackUnavailable: return "FeedbackUnavailable";
    case Ak30ServoFaultReason::HardDeadline: return "HardDeadline";
    case Ak30ServoFaultReason::UnsentSoftDeadline: return "UnsentSoftDeadline";
    case Ak30ServoFaultReason::PositionPreparation: return "PositionPreparation";
    case Ak30ServoFaultReason::LeaseSubmission: return "LeaseSubmission";
    case Ak30ServoFaultReason::TargetAuthorization: return "TargetAuthorization";
    case Ak30ServoFaultReason::InvalidDispatch: return "InvalidDispatch";
    case Ak30ServoFaultReason::ReceiveFailure: return "ReceiveFailure";
    case Ak30ServoFaultReason::TransmitFailure: return "TransmitFailure";
    case Ak30ServoFaultReason::SelectedLeaseMissing: return "SelectedLeaseMissing";
  }
  return "Unknown";
}

const char* fault_phase_name(Ak30ServoFaultPhase phase) noexcept {
  switch (phase) {
    case Ak30ServoFaultPhase::Read: return "Read";
    case Ak30ServoFaultPhase::Write: return "Write";
    case Ak30ServoFaultPhase::ReceiveObserver: return "ReceiveObserver";
    case Ak30ServoFaultPhase::SendGuard: return "SendGuard";
    case Ak30ServoFaultPhase::Watchdog: return "Watchdog";
    case Ak30ServoFaultPhase::Pending: return "Pending";
  }
  return "Unknown";
}

bool Ak30ServoRuntime::configure(std::size_t resource_count) noexcept {
  if (started_ || !clock_ || resource_count == 0U ||
      resource_count > kMaxJoints || resource_count != config_.joints.size() ||
      config_.logical_bus == 0U || config_.physical_bus.empty() ||
      config_.control_period_ns <= 0 ||
      !supports_servo(transport_.capabilities())) {
    return false;
  }
  // CompositeSystem may configure the same immutable layout again after
  // cleanup. stop() already reset bus/session epochs; rebuilding routes would
  // add duplicates to the existing router.
  if (configured_) return resource_count == sessions_.size();
  for (std::size_t index = 0; index < resource_count; ++index) {
    const auto& joint = config_.joints[index];
    if (joint.logical_bus != config_.logical_bus ||
        joint.command_hard_ttl_ns > kMaxHardTtlNs ||
        joint.command_hard_ttl_ns / config_.control_period_ns > 3 ||
        (joint.command_hard_ttl_ns / config_.control_period_ns == 3 &&
         joint.command_hard_ttl_ns % config_.control_period_ns != 0)) {
      return false;
    }
    for (std::size_t previous = 0; previous < index; ++previous) {
      if (config_.joints[previous].drive_id == joint.drive_id) return false;
    }
    mech_protocol_cubemars::Ak30ServoPositionSession probe;
    if (probe.configure(joint) != AdapterResult::Ok) return false;
  }
  sessions_.resize(resource_count);
  pending_.resize(resource_count);
  for (std::size_t index = 0; index < resource_count; ++index) {
    if (sessions_[index].configure(config_.joints[index]) != AdapterResult::Ok)
      return false;
    FrameFilter filter{};
    filter.format = CanFrameFormat::Extended;
    filter.value = 0x2900U | config_.joints[index].drive_id;
    filter.mask = kMaxExtendedCanId;
    // Shape validation belongs to the session; malformed matching frames
    // must reach it so they can revoke a previously usable sample.
    filter.frame_type.reset();
    if (router_.add_route({route(index), filter, 0U}).has_value()) return false;
  }
  configured_ = true;
  return true;
}

bool Ak30ServoRuntime::start() noexcept {
  if (!configured_ || started_ || bus_.start() != RuntimeResult::Ok) return false;
  const auto now = clock_();
  for (auto& session : sessions_) {
    if (session.activate(now) != AdapterResult::Ok) {
      for (auto& activated : sessions_) activated.deactivate();
      bus_.stop();
      return false;
    }
  }
  for (auto& entry : pending_) entry = Pending{};
  first_fault_.reset();
  started_ = true;
  if (trace_) trace_->record("runtime_start");
  has_valid_sample_ = false;
  ever_full_sample_ = false;
  group_fault_ = false;
  last_time_ = now;
  return true;
}

void Ak30ServoRuntime::stop() noexcept {
  if (trace_ && started_) trace_->record("runtime_stop");
  for (auto& entry : pending_) entry = Pending{};
  for (auto& session : sessions_) session.deactivate();
  bus_.stop();
  started_ = false;
  has_valid_sample_ = false;
  ever_full_sample_ = false;
  group_fault_ = false;
  last_time_.reset();
}

bool Ak30ServoRuntime::observe_frame(void* context, std::uint16_t routed,
                                     const RawCanFrame& frame) noexcept {
  auto& self = *static_cast<Ak30ServoRuntime*>(context);
  if (routed == 0U || routed > self.sessions_.size()) return false;
  // USB-CDC stamps host_arrival inside try_receive(), after read() captured
  // its pre-receive time. Observe the clock after the transport has stamped
  // this frame so normal causality does not appear to be a future sample.
  const auto observed_now = self.clock_();
  if (self.last_time_ && observed_now < *self.last_time_) {
    self.record_fault(Ak30ServoFaultReason::ClockRegression,
        Ak30ServoFaultPhase::ReceiveObserver, observed_now, routed - 1U);
    self.group_fault_ = true;
    return false;
  }
  self.last_time_ = observed_now;
  const auto result = self.sessions_[routed - 1U].accept_feedback(frame,
                                                                 observed_now);
  if (self.trace_) {
    self.trace_->joint = routed - 1U;
    self.trace_->frame("feedback_frame", frame, static_cast<int>(result));
    const auto sample = self.sessions_[routed - 1U].snapshot(observed_now);
    self.trace_->record("feedback", routed - 1U, sample.sequence,
        sample.position_rad, sample.feedback_position_deg,
        static_cast<int>(sample.availability),
        sample.host_rx_time ? sample.host_rx_time->nanoseconds() : 0,
        sample.raw_status);
  }
  if (result != AdapterResult::Ok && self.ever_full_sample_) {
    self.record_fault(Ak30ServoFaultReason::FeedbackRejected,
        Ak30ServoFaultPhase::ReceiveObserver, observed_now, routed - 1U, result);
    // The bus observer faults the bus and clears its command epoch. Do not
    // call BusRuntime::cancel() reentrantly from inside receive().
    self.group_fault_ = true;
    return false;
  }
  return true;
}

bool Ak30ServoRuntime::guard_send(void* context, const CommandLease& selected,
                                  MonotonicTime& attempt_now) noexcept {
  auto& self = *static_cast<Ak30ServoRuntime*>(context);
  attempt_now = self.clock_();
  if (self.last_time_ && attempt_now < *self.last_time_) {
    self.record_fault(Ak30ServoFaultReason::ClockRegression,
        Ak30ServoFaultPhase::SendGuard, attempt_now);
    self.group_fault_ = true;
    return false;
  }
  self.last_time_ = attempt_now;
  bool selected_is_current = false;
  for (std::size_t index = 0; index < self.sessions_.size(); ++index) {
    if (self.sessions_[index].snapshot(attempt_now).availability !=
        ServoPositionAvailability::Fresh) {
      self.record_fault(Ak30ServoFaultReason::FeedbackUnavailable,
          Ak30ServoFaultPhase::SendGuard, attempt_now, index);
      self.group_fault_ = true;
      return false;
    }
    const auto& entry = self.pending_[index];
    if (!entry.command) continue;
    if (entry.hard_deadline <= attempt_now) {
      self.record_fault(Ak30ServoFaultReason::HardDeadline,
          Ak30ServoFaultPhase::SendGuard, attempt_now, index);
      self.group_fault_ = true;
      return false;
    }
    if (!entry.lease) continue;  // Already in Holding; no lease can send.
    const bool selected_entry = selected.route_id == self.route(index) &&
                                selected.generation == entry.lease->generation;
    if (entry.lease->deadline <= attempt_now) {
      // A previously accepted peer may enter Holding while another route is
      // being sent. The selected unsent lease may never cross its soft TTL.
      // was_sent() is a read-only slot query; no bus mutation is reentrant.
      if (selected_entry ||
          !self.bus_.was_sent(self.route(index), entry.lease->generation)) {
        self.record_fault(Ak30ServoFaultReason::UnsentSoftDeadline,
            Ak30ServoFaultPhase::SendGuard, attempt_now, index);
        self.group_fault_ = true;
        return false;
      }
      continue;
    }
    if (!self.sessions_[index].pending_target_still_authorized(
            entry.lease->frame, entry.lease->generation,
            entry.lease->deadline, attempt_now)) {
      self.record_fault(Ak30ServoFaultReason::TargetAuthorization,
          Ak30ServoFaultPhase::SendGuard, attempt_now, index);
      self.group_fault_ = true;
      return false;
    }
    if (selected_entry) selected_is_current = true;
  }
  if (!selected_is_current) {
    self.record_fault(Ak30ServoFaultReason::SelectedLeaseMissing,
                      Ak30ServoFaultPhase::SendGuard, attempt_now);
    self.group_fault_ = true;
  }
  if (selected_is_current && self.trace_) {
    self.trace_->joint = selected.route_id - 1U;
    self.trace_->generation = selected.generation;
    self.trace_->frame("selected", selected.frame);
  }
  return selected_is_current;
}

bool Ak30ServoRuntime::fail_group() noexcept {
  group_fault_ = true;
  has_valid_sample_ = false;
  for (std::size_t index = 0; index < pending_.size(); ++index) {
    pending_[index].command.reset();
    pending_[index].lease.reset();
    (void)bus_.cancel(route(index));
  }
  return false;
}

bool Ak30ServoRuntime::check_time(MonotonicTime now) noexcept {
  if (last_time_ && now < *last_time_) {
    record_fault(Ak30ServoFaultReason::ClockRegression, phase_, now);
    return fail_group();
  }
  last_time_ = now;
  return true;
}

bool Ak30ServoRuntime::check_watchdog(MonotonicTime now) noexcept {
  for (std::size_t index = 0; index < pending_.size(); ++index) {
    auto& entry = pending_[index];
    if (!entry.command) continue;
    if (entry.hard_deadline <= now) {
      record_fault(Ak30ServoFaultReason::HardDeadline,
          Ak30ServoFaultPhase::Watchdog, now, index);
      return fail_group();
    }
    if (entry.command->deadline <= now) {
      // An unaccepted target fails at soft expiry. Only a target actually
      // sent on this route/generation may enter Holding until hard expiry.
      if (!bus_.was_sent(route(index), entry.command->generation)) {
        record_fault(Ak30ServoFaultReason::UnsentSoftDeadline,
            Ak30ServoFaultPhase::Watchdog, now, index);
        return fail_group();
      }
      if (entry.lease) {
        (void)bus_.cancel(route(index));
        entry.lease.reset();
      }
    }
  }
  return true;
}

bool Ak30ServoRuntime::check_pending(MonotonicTime now) noexcept {
  if (!check_watchdog(now)) return false;
  for (std::size_t index = 0; index < sessions_.size(); ++index) {
    if (sessions_[index].snapshot(now).availability != ServoPositionAvailability::Fresh) {
      record_fault(Ak30ServoFaultReason::FeedbackUnavailable,
          Ak30ServoFaultPhase::Pending, now, index);
      return fail_group();
    }
  }
  for (std::size_t index = 0; index < pending_.size(); ++index) {
    auto& entry = pending_[index];
    if (!entry.command) continue;
    if (entry.command->deadline <= now) continue;
    if (sessions_[index].snapshot(now).availability !=
            ServoPositionAvailability::Fresh) {
      record_fault(Ak30ServoFaultReason::FeedbackUnavailable,
          Ak30ServoFaultPhase::Pending, now, index);
      return fail_group();
    }
    if (!entry.lease) {
      RawCanFrame frame{};
      const auto prepared = sessions_[index].prepare_position(*entry.command, now, frame);
      if (prepared != AdapterResult::Ok) {
        record_fault(Ak30ServoFaultReason::PositionPreparation,
            Ak30ServoFaultPhase::Pending, now, index, prepared);
        return fail_group();
      }
      if (trace_) {
        trace_->joint = index; trace_->generation = entry.command->generation;
        trace_->frame("prepared", frame);
      }
      const auto lease = CommandLease::create(route(index),
          entry.command->generation, frame,
          *MonotonicTime::from_nanoseconds(
              entry.command->deadline.nanoseconds() -
              config_.joints[index].command_ttl_ns), entry.command->deadline);
      const auto submitted = lease ? bus_.submit(*lease) : RuntimeResult::InvalidCommand;
      if (submitted != RuntimeResult::Ok) {
        record_fault(Ak30ServoFaultReason::LeaseSubmission,
            Ak30ServoFaultPhase::Pending, now, index, AdapterResult::Ok, submitted);
        return fail_group();
      }
      entry.lease = *lease;
      if (trace_) trace_->record("submitted", index, lease->generation,
          entry.command->position, 0, 0, lease->deadline.nanoseconds(),
          static_cast<int>(submitted));
    } else if (!sessions_[index].pending_target_still_authorized(
                   entry.lease->frame, entry.lease->generation,
                   entry.lease->deadline, now)) {
      record_fault(Ak30ServoFaultReason::TargetAuthorization,
          Ak30ServoFaultPhase::Pending, now, index);
      return fail_group();
    }
  }
  return true;
}

bool Ak30ServoRuntime::read(mech_hardware_ros2_control::CanonicalState* states,
                            std::size_t count) noexcept {
  if (!started_ || group_fault_ || states == nullptr ||
      count != sessions_.size()) return false;
  phase_ = Ak30ServoFaultPhase::Read;
  const auto receive_now = clock_();
  if (!check_time(receive_now)) return false;
  const auto received = bus_.receive(receive_now, {this, &observe_frame});
  if (received != RuntimeResult::Ok || group_fault_) {
    record_fault(Ak30ServoFaultReason::ReceiveFailure, Ak30ServoFaultPhase::Read, receive_now,
                 static_cast<std::size_t>(-1), AdapterResult::Ok, received);
    return fail_group();
  }
  const auto now = clock_();
  if (!check_time(now)) return false;
  bool all_fresh = true;
  for (const auto& session : sessions_) {
    if (session.snapshot(now).availability != ServoPositionAvailability::Fresh)
      all_fresh = false;
  }
  has_valid_sample_ = all_fresh;
  if (!all_fresh) {
    if (ever_full_sample_) {
      for (std::size_t index = 0; index < sessions_.size(); ++index) {
        if (sessions_[index].snapshot(now).availability != ServoPositionAvailability::Fresh) {
          record_fault(Ak30ServoFaultReason::FeedbackUnavailable,
              Ak30ServoFaultPhase::Read, now, index);
          break;
        }
      }
      return fail_group();
    }
    return true;
  }
  ever_full_sample_ = true;
  for (std::size_t index = 0; index < count; ++index) {
    states[index].position = sessions_[index].snapshot(now).position_rad;
    states[index].velocity = std::numeric_limits<double>::quiet_NaN();
    states[index].effort = std::numeric_limits<double>::quiet_NaN();
  }
  if (!check_watchdog(now)) return false;
  // A full budget does not prove the RX queue empty. The next frame can
  // revoke a target, so never submit or send before WouldBlock was observed.
  if (!bus_.last_receive_drained()) return true;
  if (!check_pending(now)) return false;
  const auto send_now = clock_();
  if (!check_time(send_now) || !check_pending(send_now)) return false;
  const auto transmitted = bus_.transmit(send_now, {this, &guard_send});
  if (transmitted != RuntimeResult::Ok &&
      transmitted != RuntimeResult::QueueFull) {
    record_fault(Ak30ServoFaultReason::TransmitFailure, Ak30ServoFaultPhase::Read, send_now,
                 static_cast<std::size_t>(-1), AdapterResult::Ok, transmitted);
    return fail_group();
  }
  return true;
}

bool Ak30ServoRuntime::write(
    const mech_hardware_ros2_control::CommandDispatch* commands,
    std::size_t count) noexcept {
  if (!started_ || group_fault_ || commands == nullptr ||
      count != sessions_.size()) return false;
  phase_ = Ak30ServoFaultPhase::Write;
  const auto now = clock_();
  if (!check_time(now)) return false;
  if (trace_) {
    for (std::size_t index = 0; index < count; ++index) {
      const auto& d = commands[index];
      trace_->record("dispatch", index, 0, d.command.position, 0,
          (d.authorized ? 1 : 0) | (d.fresh ? 2 : 0));
    }
  }
  // A released claim ends only that route's host watchdog, even if its old
  // deadline is reached on this call. All still-claimed commands must pass
  // their existing watchdog before any fresh generation can replace them.
  for (std::size_t index = 0; index < count; ++index) {
    if (!commands[index].authorized) cancel_pending(index);
  }
  if (!check_watchdog(now)) return false;
  // Validate the entire group before changing any route. A bad member can
  // never leave another member's earlier lease available for transmission.
  for (std::size_t index = 0; index < count; ++index) {
    const auto& dispatch = commands[index];
    if (!dispatch.authorized || !dispatch.fresh) continue;
    const auto& command = dispatch.command;
    const auto ttl = config_.joints[index].command_ttl_ns;
    const auto hard_ttl = config_.joints[index].command_hard_ttl_ns;
    if (!has_valid_sample_ ||
        sessions_[index].snapshot(now).availability !=
            ServoPositionAvailability::Fresh ||
        !std::isfinite(command.position) ||
        !std::isfinite(command.velocity) ||
        !std::isfinite(command.effort) || command.velocity != 0.0 ||
        command.effort != 0.0 || ttl <= 0 ||
        now.nanoseconds() > std::numeric_limits<std::int64_t>::max() - ttl ||
        hard_ttl <= 0 ||
        now.nanoseconds() > std::numeric_limits<std::int64_t>::max() - hard_ttl ||
        pending_[index].generation == std::numeric_limits<std::uint64_t>::max()) {
      record_fault(Ak30ServoFaultReason::InvalidDispatch,
          Ak30ServoFaultPhase::Write, now, index);
      if (first_fault_) first_fault_->target_position_rad = command.position;
      return fail_group();
    }
  }
  for (std::size_t index = 0; index < count; ++index) {
    const auto& dispatch = commands[index];
    if (!dispatch.authorized || !dispatch.fresh) continue;
    auto& entry = pending_[index];
    (void)bus_.cancel(route(index));
    entry.lease.reset();
    ++entry.generation;
    entry.command = CanonicalDeviceCommand{
        dispatch.command.position, 0.0, 0.0, entry.generation,
        *MonotonicTime::from_nanoseconds(
            now.nanoseconds() + config_.joints[index].command_ttl_ns)};
    entry.hard_deadline = *MonotonicTime::from_nanoseconds(
        now.nanoseconds() + config_.joints[index].command_hard_ttl_ns);
    if (trace_) trace_->record("stored", index, entry.generation,
        entry.command->position, sessions_[index].snapshot(now).position_rad,
        0, entry.command->deadline.nanoseconds());
  }
  return true;
}

void Ak30ServoRuntime::cancel_pending(std::size_t index) noexcept {
  if (index >= pending_.size()) return;
  if (trace_ && pending_[index].command)
    trace_->record("cancel", index, pending_[index].generation);
  pending_[index].command.reset();
  pending_[index].lease.reset();
  (void)bus_.cancel(route(index));
}

std::optional<mech_protocol_cubemars::ServoPositionSnapshot>
Ak30ServoRuntime::diagnostic_snapshot(std::size_t index) const noexcept {
  if (index >= sessions_.size() || !started_) return std::nullopt;
  return sessions_[index].snapshot(clock_());
}

bool Ak30ServoRuntime::has_valid_sample() const noexcept {
  if (!started_ || group_fault_ || !has_valid_sample_) return false;
  const auto now = clock_();
  for (const auto& session : sessions_) {
    if (session.snapshot(now).availability != ServoPositionAvailability::Fresh)
      return false;
  }
  return true;
}

std::uint16_t Ak30ServoRuntime::route(std::size_t index) const noexcept {
  return static_cast<std::uint16_t>(index + 1U);
}

}  // namespace mech::mech_bringup
