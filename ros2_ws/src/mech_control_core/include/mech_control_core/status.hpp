#pragma once

#include <cstdint>
#include <optional>

#include "mech_control_core/time.hpp"

namespace mech::mech_control_core {

enum class SampleQuality : std::uint8_t {
  Unknown,
  Valid,
  Degraded,
  Stale,
  Invalid,
};

enum class DeviceState : std::uint8_t {
  Unknown,
  Offline,
  Ready,
  Active,
  Fault,
};

// Metadata shared by canonical state snapshots. It carries evidence about a
// sample without converting an unavailable device timestamp into host time.
struct StatusSnapshot final {
  SampleQuality quality;
  DeviceState device_state;
  std::uint32_t raw_fault_code;
  std::uint64_t sequence;
  std::optional<MonotonicTime> host_rx_time;
  std::optional<SourceTimestamp> source_timestamp;

  [[nodiscard]] static std::optional<StatusSnapshot> create(
      SampleQuality quality, DeviceState device_state,
      std::uint32_t raw_fault_code, std::uint64_t sequence,
      std::optional<MonotonicTime> host_rx_time,
      std::optional<SourceTimestamp> source_timestamp = std::nullopt) noexcept {
    if (quality == SampleQuality::Unknown && host_rx_time.has_value()) {
      return std::nullopt;
    }
    if (quality != SampleQuality::Unknown && !host_rx_time.has_value()) {
      return std::nullopt;
    }
    return StatusSnapshot{quality, device_state, raw_fault_code, sequence,
                          host_rx_time, source_timestamp};
  }

  [[nodiscard]] constexpr bool has_sample() const noexcept {
    return quality != SampleQuality::Unknown && host_rx_time.has_value();
  }
};

}  // namespace mech::mech_control_core
