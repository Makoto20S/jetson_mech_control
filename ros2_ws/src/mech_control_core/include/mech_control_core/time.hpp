#pragma once

#include <cstdint>
#include <optional>

namespace mech::mech_control_core {

// Host monotonic time is the only time domain used for freshness and TTL.
class MonotonicTime final {
 public:
  constexpr MonotonicTime() noexcept : nanoseconds_(0) {}

  [[nodiscard]] static std::optional<MonotonicTime> from_nanoseconds(
      std::int64_t nanoseconds) noexcept {
    if (nanoseconds < 0) {
      return std::nullopt;
    }
    return MonotonicTime(nanoseconds);
  }

  [[nodiscard]] constexpr std::int64_t nanoseconds() const noexcept {
    return nanoseconds_;
  }

  friend constexpr bool operator==(MonotonicTime lhs,
                                   MonotonicTime rhs) noexcept {
    return lhs.nanoseconds_ == rhs.nanoseconds_;
  }

  friend constexpr bool operator<(MonotonicTime lhs,
                                  MonotonicTime rhs) noexcept {
    return lhs.nanoseconds_ < rhs.nanoseconds_;
  }

  friend constexpr bool operator<=(MonotonicTime lhs,
                                   MonotonicTime rhs) noexcept {
    return lhs.nanoseconds_ <= rhs.nanoseconds_;
  }

 private:
  explicit constexpr MonotonicTime(std::int64_t nanoseconds) noexcept
      : nanoseconds_(nanoseconds) {}

  std::int64_t nanoseconds_;
};

class MonotonicDuration final {
 public:
  constexpr MonotonicDuration() noexcept : nanoseconds_(0) {}

  [[nodiscard]] static std::optional<MonotonicDuration> from_nanoseconds(
      std::int64_t nanoseconds) noexcept {
    if (nanoseconds < 0) {
      return std::nullopt;
    }
    return MonotonicDuration(nanoseconds);
  }

  [[nodiscard]] constexpr std::int64_t nanoseconds() const noexcept {
    return nanoseconds_;
  }

  friend constexpr bool operator==(MonotonicDuration lhs,
                                   MonotonicDuration rhs) noexcept {
    return lhs.nanoseconds_ == rhs.nanoseconds_;
  }

  friend constexpr bool operator<=(MonotonicDuration lhs,
                                   MonotonicDuration rhs) noexcept {
    return lhs.nanoseconds_ <= rhs.nanoseconds_;
  }

 private:
  explicit constexpr MonotonicDuration(std::int64_t nanoseconds) noexcept
      : nanoseconds_(nanoseconds) {}

  std::int64_t nanoseconds_;
};

[[nodiscard]] inline std::optional<MonotonicDuration> elapsed_since(
    MonotonicTime start, MonotonicTime end) noexcept {
  if (end < start) {
    return std::nullopt;
  }
  return MonotonicDuration::from_nanoseconds(end.nanoseconds() -
                                             start.nanoseconds());
}

enum class SourceClockDomain : std::uint8_t {
  Transport,
  Device,
};

// Source timestamps retain raw ticks because their unit and epoch are backend- or
// device-specific until an explicit clock mapping has been proven.
struct SourceTimestamp final {
  SourceClockDomain domain;
  std::uint64_t ticks;
};

}  // namespace mech::mech_control_core
