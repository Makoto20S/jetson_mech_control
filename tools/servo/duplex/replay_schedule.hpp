#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace duplex {
struct ReplayEvent {
  std::uint64_t offset_ns{};
  unsigned port{}, lane{};
};
struct ReplaySchedule {
  std::string profile;
  std::vector<ReplayEvent> events;
  void validate() const;
};
constexpr std::uint64_t kReplayDurationNs = 5000000000ULL;
constexpr std::uint64_t kReplayMaxLateNs = 250000ULL;
constexpr std::uint64_t kReplayReadGuardNs = 200000ULL;
ReplaySchedule load_replay_schedule(const std::string& path);
std::string replay_json(const ReplaySchedule&);
// Checked at the actual write boundary; an overdue event is never submitted.
bool replay_deadline_allowed(std::uint64_t deadline, std::uint64_t now) noexcept;
}  // namespace duplex
