#pragma once

#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mech_control_core/router.hpp"
#include "mech_control_core/transport.hpp"

namespace mech::mech_control_core {

struct FrameSnapshot final {
  RawCanFrame frame;
  std::uint64_t sequence{0U};
  MonotonicTime host_rx_time;
  MonotonicDuration age;
  bool fresh{false};
};

// Thread-affinity: SnapshotStore performs NO internal synchronization (no
// mutex, no atomics). publish() is expected to be called only from the bus
// poller thread (via BusRuntime::poll()); read()/size() may be called from
// a different thread (e.g. a ros2_control read cycle) ONLY if the caller
// provides its own external synchronization against concurrent publish().
// Without that, concurrent publish()/read() is a data race.
class SnapshotStore final {
 public:
  explicit SnapshotStore(std::size_t capacity = 32U) : capacity_(capacity) {
    entries_.reserve(capacity_);
  }

  [[nodiscard]] bool publish(std::uint16_t route_id,
                             const RawCanFrame& frame) {
    for (auto& entry : entries_) {
      if (entry.route_id == route_id) {
        entry.snapshot.frame = frame;
        entry.snapshot.host_rx_time = frame.host_arrival;
        ++entry.snapshot.sequence;
        return true;
      }
    }
    if (entries_.size() >= capacity_) {
      return false;
    }
    entries_.push_back(Entry{route_id,
                             FrameSnapshot{frame, 1U, frame.host_arrival,
                                           MonotonicDuration::from_nanoseconds(0)
                                               .value(),
                                           true}});
    return true;
  }

  [[nodiscard]] std::optional<FrameSnapshot> read(
      std::uint16_t route_id, MonotonicTime now,
      MonotonicDuration freshness_ttl) const noexcept {
    for (const auto& entry : entries_) {
      if (entry.route_id != route_id) {
        continue;
      }
      auto snapshot = entry.snapshot;
      const auto age = elapsed_since(snapshot.host_rx_time, now);
      if (!age.has_value()) {
        snapshot.age = *MonotonicDuration::from_nanoseconds(0);
        snapshot.fresh = false;
      } else {
        snapshot.age = *age;
        snapshot.fresh = *age <= freshness_ttl;
      }
      return snapshot;
    }
    return std::nullopt;
  }

  [[nodiscard]] std::size_t size() const noexcept { return entries_.size(); }

 private:
  struct Entry final {
    std::uint16_t route_id;
    FrameSnapshot snapshot;
  };

  std::size_t capacity_;
  std::vector<Entry> entries_;
};

struct CommandLease final {
  std::uint16_t route_id{0U};
  std::uint64_t generation{0U};
  RawCanFrame frame{};
  MonotonicTime submitted_at{*MonotonicTime::from_nanoseconds(0)};
  MonotonicTime deadline{*MonotonicTime::from_nanoseconds(0)};

  [[nodiscard]] static std::optional<CommandLease> create(
      std::uint16_t route_id, std::uint64_t generation,
      const RawCanFrame& frame, MonotonicTime submitted_at,
      MonotonicTime deadline) noexcept {
    if (route_id == 0U || generation == 0U ||
        frame.direction != FrameDirection::Tx || !frame.is_valid() ||
        deadline < submitted_at) {
      return std::nullopt;
    }
    return CommandLease{route_id, generation, frame, submitted_at, deadline};
  }

  [[nodiscard]] bool expired(MonotonicTime now) const noexcept {
    return deadline <= now;
  }
};

class CommandSlot final {
 public:
  [[nodiscard]] bool bind(std::uint16_t route_id) noexcept {
    if (route_id == 0U || (route_id_ != 0U && route_id_ != route_id)) {
      return false;
    }
    route_id_ = route_id;
    return true;
  }

  [[nodiscard]] std::uint16_t route_id() const noexcept { return route_id_; }

  [[nodiscard]] bool replace(const CommandLease& lease) noexcept {
    if (lease.generation <= last_generation_) {
      return false;
    }
    lease_ = lease;
    last_generation_ = lease.generation;
    sent_generation_ = 0U;
    return true;
  }

  [[nodiscard]] std::optional<CommandLease> pending(MonotonicTime now) const
      noexcept {
    if (!lease_.has_value() || lease_->expired(now) ||
        lease_->generation == sent_generation_) {
      return std::nullopt;
    }
    return lease_;
  }

  [[nodiscard]] bool has_expired(MonotonicTime now) const noexcept {
    return lease_.has_value() && lease_->expired(now) &&
           lease_->generation != sent_generation_;
  }

  void mark_sent() noexcept {
    if (lease_.has_value()) {
      sent_generation_ = lease_->generation;
    }
  }

  void clear() noexcept { lease_.reset(); }

 private:
  std::uint16_t route_id_{0U};
  std::optional<CommandLease> lease_;
  std::uint64_t last_generation_{0U};
  std::uint64_t sent_generation_{0U};
};

enum class OwnershipAcquireError : std::uint8_t {
  InvalidChannel,
  AlreadyOwned,
  CapacityExceeded,
};

class BusOwnershipRegistry final {
 public:
  explicit BusOwnershipRegistry(std::size_t capacity = 8U)
      : capacity_(capacity) {
    owners_.reserve(capacity_);
  }

  // Prefer try_acquire() for callers that need to distinguish *why*
  // acquisition failed. Kept for source compatibility with existing callers
  // that only care about the boolean outcome.
  [[nodiscard]] bool acquire(const std::string& physical_channel) {
    return !try_acquire(physical_channel).has_value();
  }

  // Returns std::nullopt on success, or the specific reason acquisition
  // failed otherwise.
  [[nodiscard]] std::optional<OwnershipAcquireError> try_acquire(
      const std::string& physical_channel) {
    if (physical_channel.empty()) {
      return OwnershipAcquireError::InvalidChannel;
    }
    if (owns(physical_channel)) {
      return OwnershipAcquireError::AlreadyOwned;
    }
    if (owners_.size() >= capacity_) {
      return OwnershipAcquireError::CapacityExceeded;
    }
    owners_.push_back(physical_channel);
    return std::nullopt;
  }

  void release(const std::string& physical_channel) noexcept {
    for (auto iterator = owners_.begin(); iterator != owners_.end(); ++iterator) {
      if (*iterator == physical_channel) {
        owners_.erase(iterator);
        return;
      }
    }
  }

  [[nodiscard]] bool owns(const std::string& physical_channel) const noexcept {
    for (const auto& owner : owners_) {
      if (owner == physical_channel) {
        return true;
      }
    }
    return false;
  }

 private:
  std::size_t capacity_;
  std::vector<std::string> owners_;
};

enum class RuntimeState : std::uint8_t {
  Stopped,
  Running,
  Fault,
};

enum class RuntimeResult : std::uint8_t {
  Ok,
  NotRunning,
  AlreadyOwned,
  ChannelCapacityExceeded,
  TransportOpenFailed,
  InvalidTransport,
  InvalidCommand,
  CommandCapacityExceeded,
  QueueFull,
  Disconnected,
  Fault,
  InvalidState,
};

struct RuntimeStats final {
  std::uint64_t rx_frames{0U};
  std::uint64_t rx_unrouted{0U};
  std::uint64_t rx_invalid{0U};
  std::uint64_t tx_frames{0U};
  std::uint64_t expired_commands{0U};
  std::uint64_t queue_full{0U};
  std::uint64_t would_block{0U};
  std::uint64_t transport_errors{0U};
  std::uint64_t bus_faults{0U};
  std::uint64_t snapshot_overflow{0U};
};

// -----------------------------------------------------------------------
// Thread-affinity contract (documentation only -- BusRuntime/SnapshotStore
// perform NO internal synchronization: no mutex, no atomics).
//
// BusRuntime is intended to be driven by exactly one "bus poller" thread
// which owns start()/stop()/recover()/poll() and must call them serially
// (never concurrently with each other, and never re-entrantly). submit()
// is expected to be called from a *different* thread -- typically a
// ros2_control read/write cycle -- but submit() and poll()/start()/stop()/
// recover() on the SAME BusRuntime instance MUST NOT be invoked
// concurrently without external synchronization added by the caller.
// There is no lock-free guarantee here: CommandSlot/SnapshotStore state is
// read and written without ordering constraints, so a caller that wants
// submit() to run on a different thread than poll() must add its own
// mutex (or equivalent) around both call sites. Similarly, snapshots() and
// stats() return const references/values that alias mutable runtime state;
// reading them from a thread other than the poller thread while poll() is
// concurrently running is a data race unless externally synchronized.
// -----------------------------------------------------------------------
class BusRuntime final {
 public:
  BusRuntime(std::uint16_t logical_bus, std::string physical_channel,
             Transport& transport, FrameRouter& router,
             BusOwnershipRegistry& ownership, std::size_t command_capacity = 32U,
             std::size_t snapshot_capacity = 32U)
      : logical_bus_(logical_bus),
        physical_channel_(std::move(physical_channel)),
        transport_(transport),
        router_(router),
        ownership_(ownership),
        snapshots_(snapshot_capacity),
        command_slots_(command_capacity) {}

  ~BusRuntime() { stop(); }

  // May only be called from the bus-poller thread. See the thread-affinity
  // contract above BusRuntime.
  [[nodiscard]] RuntimeResult start() noexcept {
    if (state_ != RuntimeState::Stopped) {
      return RuntimeResult::InvalidState;
    }
    if (!transport_.capabilities().is_valid()) {
      return RuntimeResult::InvalidTransport;
    }
    const auto acquire_error = ownership_.try_acquire(physical_channel_);
    if (acquire_error.has_value()) {
      return *acquire_error == OwnershipAcquireError::CapacityExceeded
                 ? RuntimeResult::ChannelCapacityExceeded
                 : RuntimeResult::AlreadyOwned;
    }
    if (!transport_.open()) {
      ownership_.release(physical_channel_);
      return RuntimeResult::TransportOpenFailed;
    }
    state_ = RuntimeState::Running;
    return RuntimeResult::Ok;
  }

  // May only be called from the bus-poller thread. See the thread-affinity
  // contract above BusRuntime. Idempotent: safe to call from Running,
  // Fault, or Stopped. Releases the ownership-registry entry at most once
  // per acquisition (guarded by the `state_ != Stopped` check), so calling
  // stop() twice in a row -- or calling it as part of recover() -- never
  // double-releases.
  void stop() noexcept {
    if (state_ != RuntimeState::Stopped) {
      transport_.close();
      ownership_.release(physical_channel_);
    }
    state_ = RuntimeState::Stopped;
  }

  // Deliberate recovery path out of RuntimeState::Fault. Equivalent to
  // stop() followed by start(): it releases the channel and re-opens the
  // transport rather than trying to resume with whatever state the
  // transport was left in. May only be called from the bus-poller thread.
  // Returns InvalidState if the runtime was not in Fault (recover() is not
  // a generic "(re)start" -- use start() for that).
  [[nodiscard]] RuntimeResult recover() noexcept {
    if (state_ != RuntimeState::Fault) {
      return RuntimeResult::InvalidState;
    }
    stop();
    return start();
  }

  // May be called from a different thread than poll(), but see the
  // thread-affinity contract above BusRuntime for the synchronization that
  // implies is the caller's responsibility.
  [[nodiscard]] RuntimeResult submit(const CommandLease& lease) noexcept {
    if (state_ != RuntimeState::Running) {
      return RuntimeResult::NotRunning;
    }
    if (lease.frame.logical_bus != logical_bus_ ||
        !router_.contains(lease.route_id) ||
        lease.frame.direction != FrameDirection::Tx ||
        !lease.frame.is_valid()) {
      return RuntimeResult::InvalidCommand;
    }
    CommandSlot* target = nullptr;
    for (auto& slot : command_slots_) {
      if (slot.route_id() == lease.route_id) {
        target = &slot;
        break;
      }
      if (target == nullptr && slot.route_id() == 0U) {
        target = &slot;
      }
    }
    if (target == nullptr) {
      return RuntimeResult::CommandCapacityExceeded;
    }
    if (!target->bind(lease.route_id) || !target->replace(lease)) {
      return RuntimeResult::InvalidCommand;
    }
    return RuntimeResult::Ok;
  }

  // May only be called from the bus-poller thread. See the thread-affinity
  // contract above BusRuntime.
  [[nodiscard]] RuntimeResult poll(MonotonicTime now) noexcept {
    if (state_ != RuntimeState::Running) {
      return RuntimeResult::NotRunning;
    }
    RawCanFrame frame{};
    constexpr std::size_t kReceiveBudget = 64U;
    for (std::size_t received = 0U; received < kReceiveBudget; ++received) {
      const auto result = transport_.try_receive(frame);
      if (result == TransportResult::WouldBlock) {
        break;
      }
      if (result == TransportResult::Invalid) {
        // A malformed/truncated frame from a peer is a normal bus event,
        // not a runtime failure: count it and keep receiving so one bad
        // peer cannot wedge the whole bus.
        ++runtime_stats_.rx_invalid;
        continue;
      }
      if (result != TransportResult::Ok) {
        return fault_for(result);
      }
      ++runtime_stats_.rx_frames;
      std::array<std::uint16_t, 32U> destinations{};
      std::size_t destination_count = 0U;
      if (!router_.route_into(frame, destinations, destination_count)) {
        ++runtime_stats_.transport_errors;
        state_ = RuntimeState::Fault;
        return RuntimeResult::Fault;
      }
      if (destination_count == 0U) {
        ++runtime_stats_.rx_unrouted;
      }
      for (std::size_t index = 0U; index < destination_count; ++index) {
        if (!snapshots_.publish(destinations[index], frame)) {
          // Distinct from transport_errors: this is a configuration
          // mismatch (snapshot capacity smaller than the number of routes
          // actually published to), not a transport fault.
          ++runtime_stats_.snapshot_overflow;
        }
      }
    }
    bool backpressure = false;
    const std::size_t slot_count = command_slots_.size();
    for (std::size_t offset = 0U; offset < slot_count; ++offset) {
      // Rotate the starting slot each cycle so sustained backpressure on
      // one slot cannot starve the slots that follow it: every slot gets a
      // turn to be "first" once every slot_count cycles.
      auto& slot = command_slots_[(next_slot_ + offset) % slot_count];
      if (slot.has_expired(now)) {
        ++runtime_stats_.expired_commands;
        slot.clear();
        continue;
      }
      const auto pending = slot.pending(now);
      if (!pending.has_value()) {
        continue;
      }
      const auto result = transport_.try_send(pending->frame);
      if (result == TransportResult::Ok) {
        slot.mark_sent();
        ++runtime_stats_.tx_frames;
      } else if (result == TransportResult::QueueFull) {
        ++runtime_stats_.queue_full;
        backpressure = true;
      } else if (result == TransportResult::WouldBlock) {
        // Backpressure, not a fault: leave the lease pending (mark_sent()
        // is NOT called) so it is retried on a later cycle, exactly like
        // QueueFull.
        ++runtime_stats_.would_block;
        backpressure = true;
      } else {
        return fault_for(result);
      }
    }
    if (slot_count != 0U) {
      next_slot_ = (next_slot_ + 1U) % slot_count;
    }
    return backpressure ? RuntimeResult::QueueFull : RuntimeResult::Ok;
  }

  [[nodiscard]] RuntimeState state() const noexcept { return state_; }
  [[nodiscard]] const SnapshotStore& snapshots() const noexcept {
    return snapshots_;
  }
  [[nodiscard]] const RuntimeStats& stats() const noexcept {
    return runtime_stats_;
  }

 private:
  [[nodiscard]] RuntimeResult fault_for(TransportResult result) noexcept {
    if (result == TransportResult::Disconnected) {
      ++runtime_stats_.bus_faults;
      ++runtime_stats_.transport_errors;
      state_ = RuntimeState::Fault;
      return RuntimeResult::Disconnected;
    }
    ++runtime_stats_.transport_errors;
    state_ = RuntimeState::Fault;
    return RuntimeResult::Fault;
  }

  std::uint16_t logical_bus_;
  std::string physical_channel_;
  Transport& transport_;
  FrameRouter& router_;
  BusOwnershipRegistry& ownership_;
  RuntimeState state_{RuntimeState::Stopped};
  SnapshotStore snapshots_;
  RuntimeStats runtime_stats_;
  std::vector<CommandSlot> command_slots_;
  std::size_t next_slot_{0U};
};

}  // namespace mech::mech_control_core
