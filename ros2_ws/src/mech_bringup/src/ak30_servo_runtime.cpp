#include "mech_bringup/ak30_servo_runtime.hpp"

#include <cmath>
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
                                   BusOwnershipRegistry& ownership)
    : transport_(transport),
      clock_(std::move(clock)),
      config_(std::move(config)),
      router_(kMaxJoints),
      bus_(config_.logical_bus, config_.physical_bus, transport_, router_,
           ownership, kMaxJoints, kMaxJoints) {}

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
  started_ = true;
  has_valid_sample_ = false;
  ever_full_sample_ = false;
  group_fault_ = false;
  last_time_ = now;
  return true;
}

void Ak30ServoRuntime::stop() noexcept {
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
    self.group_fault_ = true;
    return false;
  }
  self.last_time_ = observed_now;
  const auto result = self.sessions_[routed - 1U].accept_feedback(frame,
                                                                 observed_now);
  if (result != AdapterResult::Ok && self.ever_full_sample_) {
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
    self.group_fault_ = true;
    return false;
  }
  self.last_time_ = attempt_now;
  bool selected_is_current = false;
  for (std::size_t index = 0; index < self.sessions_.size(); ++index) {
    if (self.sessions_[index].snapshot(attempt_now).availability !=
        ServoPositionAvailability::Fresh) {
      self.group_fault_ = true;
      return false;
    }
    const auto& entry = self.pending_[index];
    if (!entry.command) continue;
    if (entry.hard_deadline <= attempt_now) {
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
        self.group_fault_ = true;
        return false;
      }
      continue;
    }
    if (!self.sessions_[index].pending_target_still_authorized(
            entry.lease->frame, entry.lease->generation,
            entry.lease->deadline, attempt_now)) {
      self.group_fault_ = true;
      return false;
    }
    if (selected_entry) selected_is_current = true;
  }
  if (!selected_is_current) self.group_fault_ = true;
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
  if (last_time_ && now < *last_time_) return fail_group();
  last_time_ = now;
  return true;
}

bool Ak30ServoRuntime::check_watchdog(MonotonicTime now) noexcept {
  for (std::size_t index = 0; index < pending_.size(); ++index) {
    auto& entry = pending_[index];
    if (!entry.command) continue;
    if (entry.hard_deadline <= now) return fail_group();
    if (entry.command->deadline <= now) {
      // An unaccepted target fails at soft expiry. Only a target actually
      // sent on this route/generation may enter Holding until hard expiry.
      if (!bus_.was_sent(route(index), entry.command->generation))
        return fail_group();
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
  for (const auto& session : sessions_) {
    if (session.snapshot(now).availability !=
        ServoPositionAvailability::Fresh) return fail_group();
  }
  for (std::size_t index = 0; index < pending_.size(); ++index) {
    auto& entry = pending_[index];
    if (!entry.command) continue;
    if (entry.command->deadline <= now) continue;
    if (sessions_[index].snapshot(now).availability !=
            ServoPositionAvailability::Fresh) {
      return fail_group();
    }
    if (!entry.lease) {
      RawCanFrame frame{};
      if (sessions_[index].prepare_position(*entry.command, now, frame) !=
          AdapterResult::Ok) return fail_group();
      const auto lease = CommandLease::create(route(index),
          entry.command->generation, frame,
          *MonotonicTime::from_nanoseconds(
              entry.command->deadline.nanoseconds() -
              config_.joints[index].command_ttl_ns), entry.command->deadline);
      if (!lease || bus_.submit(*lease) != RuntimeResult::Ok) return fail_group();
      entry.lease = *lease;
    } else if (!sessions_[index].pending_target_still_authorized(
                   entry.lease->frame, entry.lease->generation,
                   entry.lease->deadline, now)) {
      return fail_group();
    }
  }
  return true;
}

bool Ak30ServoRuntime::read(mech_hardware_ros2_control::CanonicalState* states,
                            std::size_t count) noexcept {
  if (!started_ || group_fault_ || states == nullptr ||
      count != sessions_.size()) return false;
  const auto receive_now = clock_();
  if (!check_time(receive_now)) return false;
  const auto received = bus_.receive(receive_now, {this, &observe_frame});
  if (received != RuntimeResult::Ok || group_fault_) return fail_group();
  const auto now = clock_();
  if (!check_time(now)) return false;
  bool all_fresh = true;
  for (const auto& session : sessions_) {
    if (session.snapshot(now).availability != ServoPositionAvailability::Fresh)
      all_fresh = false;
  }
  has_valid_sample_ = all_fresh;
  if (!all_fresh) {
    if (ever_full_sample_) return fail_group();
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
      transmitted != RuntimeResult::QueueFull) return fail_group();
  return true;
}

bool Ak30ServoRuntime::write(
    const mech_hardware_ros2_control::CommandDispatch* commands,
    std::size_t count) noexcept {
  if (!started_ || group_fault_ || commands == nullptr ||
      count != sessions_.size()) return false;
  const auto now = clock_();
  if (!check_time(now)) return false;
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
        pending_[index].generation == std::numeric_limits<std::uint64_t>::max())
      return fail_group();
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
  }
  return true;
}

void Ak30ServoRuntime::cancel_pending(std::size_t index) noexcept {
  if (index >= pending_.size()) return;
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
