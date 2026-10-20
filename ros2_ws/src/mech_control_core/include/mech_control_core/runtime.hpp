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
// mutex, no atomics). publish() (via BusRuntime::poll()/receive()) and
// clear() (via stop()/fault/recover epoch reset) must run on the bus poller
// thread. read()/size() may run from another thread ONLY with external
// synchronization against these mutations. Without it, concurrent access
// is a data race.
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

  void clear() noexcept { entries_.clear(); }

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

  [[nodiscard]] bool was_sent(std::uint64_t generation) const noexcept {
    return generation != 0U && sent_generation_ == generation;
  }

  void mark_sent() noexcept {
    if (lease_.has_value()) {
      sent_generation_ = lease_->generation;
    }
  }

  void clear() noexcept { lease_.reset(); }

  void reset_epoch() noexcept {
    route_id_ = 0U;
    lease_.reset();
    last_generation_ = 0U;
    sent_generation_ = 0U;
  }

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
// which owns start()/stop()/recover()/poll()/receive()/transmit() and must call them serially
// (never concurrently with each other, and never re-entrantly). submit()
// is expected to be called from a *different* thread -- typically a
// ros2_control read/write cycle -- but submit() and poll()/start()/stop()/
// recover()/receive()/transmit()/cancel() on the SAME BusRuntime instance MUST NOT be invoked
// concurrently without external synchronization added by the caller.
// There is no lock-free guarantee here: CommandSlot/SnapshotStore state is
// read and written without ordering constraints, so a caller that wants
// submit() to run on a different thread than poll() must add its own
// mutex (or equivalent) around both call sites. Similarly, snapshots() and
// stats() expose mutable runtime state; reading them from another thread while
// receive()/poll() publishes or stop()/fault/recover clears an epoch is a data
// race unless externally synchronized.
// -----------------------------------------------------------------------
class BusRuntime final {
 public:
  // A synchronous, non-owning callback. Returning false faults the runtime
  // and prevents transmit. It must not call back into this runtime; all
  // runtime entry points are non-reentrant and require external synchronization
  // when used from different threads.
  struct RxObserver final {
    void* context{nullptr};
    bool (*on_frame)(void*, std::uint16_t, const RawCanFrame&) noexcept{nullptr};
  };

  // Optional synchronous guard for each pending TX attempt. The callback may
  // supply a newer monotonic time and reject the send. It must not call any
  // driving/mutating BusRuntime method; the read-only was_sent() query is
  // permitted for checking a peer's accepted generation. A rejection or
  // clock rollback faults the bus epoch.
  struct TxGuard final {
    void* context{nullptr};
    bool (*before_send)(void*, const CommandLease&,
                        MonotonicTime&) noexcept{nullptr};
  };

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
    reset_epoch();
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

  // Cancels only a bound route's host-side pending lease. The generation
  // watermark remains until stop/fault, so a stale lease cannot be replayed.
  // This does not stop a physical device or retract a frame already sent.
  [[nodiscard]] RuntimeResult cancel(std::uint16_t route_id) noexcept {
    if (state_ != RuntimeState::Running) {
      return RuntimeResult::NotRunning;
    }
    for (auto& slot : command_slots_) {
      if (slot.route_id() == route_id && route_id != 0U) {
        slot.clear();
        return RuntimeResult::Ok;
      }
    }
    return RuntimeResult::InvalidCommand;
  }

  // May only be called from the bus-poller thread. See the thread-affinity
  // contract above BusRuntime.
  [[nodiscard]] RuntimeResult poll(MonotonicTime now) noexcept {
    // Legacy path keeps its historical count-only snapshot overflow behavior.
    const auto result = receive_impl(RxObserver{}, false);
    return result == RuntimeResult::Ok ? transmit(now) : result;
  }

  // Guarded receive phase. Each routed frame reaches the observer in receive
  // order, even if a later frame would overwrite its route's snapshot.
  // Snapshot capacity failure faults this path before any subsequent TX.
  // The caller must validate session feedback freshness before transmit().
  [[nodiscard]] RuntimeResult receive(MonotonicTime now) noexcept {
    return receive(now, RxObserver{});
  }

  [[nodiscard]] RuntimeResult receive(
      MonotonicTime /*now*/, RxObserver observer) noexcept {
    return receive_impl(observer, true);
  }

  [[nodiscard]] RuntimeResult transmit(MonotonicTime now) noexcept {
    return transmit(now, TxGuard{});
  }

  [[nodiscard]] RuntimeResult transmit(MonotonicTime now,
                                       TxGuard guard) noexcept {
    if (state_ != RuntimeState::Running) {
      return RuntimeResult::NotRunning;
    }
    bool backpressure = false;
    auto last_attempt_time = now;
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
      auto attempt_time = now;
      if (guard.before_send != nullptr) {
        if (!guard.before_send(guard.context, *pending, attempt_time) ||
            attempt_time < last_attempt_time) {
          return fault_runtime();
        }
        last_attempt_time = attempt_time;
        if (slot.has_expired(attempt_time)) {
          ++runtime_stats_.expired_commands;
          slot.clear();
          continue;
        }
      }
      const auto result = transport_.try_send(pending->frame);
      if (result == TransportResult::Ok) {
        slot.mark_sent();
        ++runtime_stats_.tx_frames;
      } else if (result == TransportResult::QueueFull) {
        ++runtime_stats_.queue_full;
        backpressure = true;
      } else if (result == TransportResult::WouldBlock) {
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
  // True only when the most recent receive observed an empty nonblocking RX
  // queue. A caller must defer TX when the bounded receive budget was used up.
  [[nodiscard]] bool last_receive_drained() const noexcept {
    return last_receive_drained_;
  }
  // Actual accepted TX, scoped to one route and generation. Aggregate frame
  // counts cannot tell a caller which member entered a holding watchdog.
  [[nodiscard]] bool was_sent(std::uint16_t route_id,
                              std::uint64_t generation) const noexcept {
    if (route_id == 0U || generation == 0U) return false;
    for (const auto& slot : command_slots_) {
      if (slot.route_id() == route_id) return slot.was_sent(generation);
    }
    return false;
  }

 private:
  [[nodiscard]] RuntimeResult receive_impl(
      RxObserver observer, bool fail_on_snapshot_overflow) noexcept {
    if (state_ != RuntimeState::Running) {
      return RuntimeResult::NotRunning;
    }
    last_receive_drained_ = false;
    RawCanFrame frame{};
    constexpr std::size_t kReceiveBudget = 64U;
    for (std::size_t received = 0U; received < kReceiveBudget; ++received) {
      const auto result = transport_.try_receive(frame);
      if (result == TransportResult::WouldBlock) {
        last_receive_drained_ = true;
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
        return fault_runtime();
      }
      if (destination_count == 0U) {
        ++runtime_stats_.rx_unrouted;
      }
      for (std::size_t index = 0U; index < destination_count; ++index) {
        if (observer.on_frame != nullptr &&
            !observer.on_frame(observer.context, destinations[index], frame)) {
          return fault_runtime();
        }
        if (!snapshots_.publish(destinations[index], frame)) {
          // Distinct from transport_errors: this is a configuration
          // mismatch (snapshot capacity smaller than the number of routes
          // actually published to), not a transport fault.
          ++runtime_stats_.snapshot_overflow;
          if (fail_on_snapshot_overflow) {
            return fault_runtime();
          }
        }
      }
    }
    return RuntimeResult::Ok;
  }

  void reset_epoch() noexcept {
    for (auto& slot : command_slots_) {
      slot.reset_epoch();
    }
    snapshots_.clear();
    next_slot_ = 0U;
    last_receive_drained_ = false;
  }

  [[nodiscard]] RuntimeResult fault_runtime() noexcept {
    state_ = RuntimeState::Fault;
    reset_epoch();
    return RuntimeResult::Fault;
  }

  [[nodiscard]] RuntimeResult fault_for(TransportResult result) noexcept {
    if (result == TransportResult::Disconnected) {
      ++runtime_stats_.bus_faults;
      ++runtime_stats_.transport_errors;
      state_ = RuntimeState::Fault;
      reset_epoch();
      return RuntimeResult::Disconnected;
    }
    ++runtime_stats_.transport_errors;
    state_ = RuntimeState::Fault;
    reset_epoch();
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
  bool last_receive_drained_{false};
};

}  // namespace mech::mech_control_core
