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

class BusOwnershipRegistry final {
 public:
  explicit BusOwnershipRegistry(std::size_t capacity = 8U)
      : capacity_(capacity) {
    owners_.reserve(capacity_);
  }

  [[nodiscard]] bool acquire(const std::string& physical_channel) {
    if (physical_channel.empty() || owns(physical_channel) ||
        owners_.size() >= capacity_) {
      return false;
    }
    owners_.push_back(physical_channel);
    return true;
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
  TransportOpenFailed,
  InvalidTransport,
  InvalidCommand,
  QueueFull,
  Disconnected,
  Fault,
};

struct RuntimeStats final {
  std::uint64_t rx_frames{0U};
  std::uint64_t rx_unrouted{0U};
  std::uint64_t tx_frames{0U};
  std::uint64_t expired_commands{0U};
  std::uint64_t queue_full{0U};
  std::uint64_t transport_errors{0U};
  std::uint64_t bus_faults{0U};
};

class BusRuntime final {
 public:
  BusRuntime(std::uint16_t logical_bus, std::string physical_channel,
             Transport& transport, FrameRouter& router,
             BusOwnershipRegistry& ownership, std::size_t command_capacity = 32U)
      : logical_bus_(logical_bus),
        physical_channel_(std::move(physical_channel)),
        transport_(transport),
        router_(router),
        ownership_(ownership),
        command_slots_(command_capacity) {}

  ~BusRuntime() { stop(); }

  [[nodiscard]] RuntimeResult start() noexcept {
    if (state_ != RuntimeState::Stopped) {
      return RuntimeResult::Fault;
    }
    if (!transport_.capabilities().is_valid()) {
      return RuntimeResult::InvalidTransport;
    }
    if (!ownership_.acquire(physical_channel_)) {
      return RuntimeResult::AlreadyOwned;
    }
    if (!transport_.open()) {
      ownership_.release(physical_channel_);
      return RuntimeResult::TransportOpenFailed;
    }
    state_ = RuntimeState::Running;
    return RuntimeResult::Ok;
  }

  void stop() noexcept {
    if (state_ != RuntimeState::Stopped) {
      transport_.close();
      ownership_.release(physical_channel_);
    }
    state_ = RuntimeState::Stopped;
  }

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
    if (target == nullptr || !target->bind(lease.route_id) ||
        !target->replace(lease)) {
      return RuntimeResult::InvalidCommand;
    }
    return RuntimeResult::Ok;
  }

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
          ++runtime_stats_.transport_errors;
        }
      }
    }
    for (auto& slot : command_slots_) {
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
        return RuntimeResult::QueueFull;
      } else {
        return fault_for(result);
      }
    }
    return RuntimeResult::Ok;
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
};

}  // namespace mech::mech_control_core
