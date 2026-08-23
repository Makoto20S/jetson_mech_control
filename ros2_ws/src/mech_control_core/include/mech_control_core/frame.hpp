#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "mech_control_core/time.hpp"

namespace mech::mech_control_core {

constexpr std::size_t kMaxCanPayloadBytes = 64U;
constexpr std::uint32_t kMaxStandardCanId = 0x7FFU;
constexpr std::uint32_t kMaxExtendedCanId = 0x1FFFFFFFU;

enum class CanFrameFormat : std::uint8_t {
  Standard,
  Extended,
};

enum class CanFrameType : std::uint8_t {
  Classic,
  FlexibleDataRate,
};

enum class FrameDirection : std::uint8_t {
  Rx,
  Tx,
};

struct CanId final {
  std::uint32_t value;
  CanFrameFormat format;

  [[nodiscard]] static std::optional<CanId> create(
      std::uint32_t value, CanFrameFormat format) noexcept {
    const auto maximum = format == CanFrameFormat::Standard
                             ? kMaxStandardCanId
                             : kMaxExtendedCanId;
    if (value > maximum) {
      return std::nullopt;
    }
    return CanId{value, format};
  }

  [[nodiscard]] constexpr bool is_valid() const noexcept {
    const auto maximum = format == CanFrameFormat::Standard
                             ? kMaxStandardCanId
                             : kMaxExtendedCanId;
    return value <= maximum;
  }
};

struct RawCanFrame final {
  std::uint16_t logical_bus;
  CanId id;
  CanFrameType type;
  FrameDirection direction;
  std::uint8_t payload_size;
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload;
  MonotonicTime host_arrival;
  std::optional<SourceTimestamp> source_timestamp;
  bool error_frame{false};
  bool bitrate_switch{false};

  [[nodiscard]] static std::optional<RawCanFrame> create(
      std::uint16_t logical_bus, CanId id, CanFrameType type,
      FrameDirection direction, std::uint8_t payload_size,
      const std::array<std::uint8_t, kMaxCanPayloadBytes>& payload,
      MonotonicTime host_arrival,
      std::optional<SourceTimestamp> source_timestamp = std::nullopt,
      bool bitrate_switch = false) noexcept {
    const auto maximum = type == CanFrameType::Classic ? 8U
                                                        : kMaxCanPayloadBytes;
    if (!id.is_valid() || payload_size > maximum ||
        (bitrate_switch && type != CanFrameType::FlexibleDataRate)) {
      return std::nullopt;
    }
    return RawCanFrame{logical_bus, id, type, direction, payload_size, payload,
                       host_arrival, source_timestamp, false, bitrate_switch};
  }

  [[nodiscard]] constexpr bool is_valid() const noexcept {
    const auto maximum = type == CanFrameType::Classic ? 8U
                                                        : kMaxCanPayloadBytes;
    return id.is_valid() && payload_size <= maximum &&
           (!bitrate_switch || type == CanFrameType::FlexibleDataRate);
  }
};

}  // namespace mech::mech_control_core
