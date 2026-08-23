#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "mech_control_core/frame.hpp"

namespace mech::mech_control_core {

struct SchemaVersion final {
  std::uint16_t major;
  std::uint16_t minor;
};

inline constexpr SchemaVersion kSchemaV1{1U, 0U};

enum class TransportKind : std::uint8_t {
  Unknown,
  Fake,
  SocketCan,
  HighTorqueUsbCdc,
};

enum class ProtocolProfile : std::uint8_t {
  Unknown,
  LoopbackV1,
  L02ServoExtended,
  L02MitStandard,
  Hi12J1939,
  Hi12Canopen,
};

struct TransportCapabilities final {
  bool supports_classic_can{false};
  bool supports_can_fd{false};
  bool supports_brs{false};
  bool supports_standard_frames{false};
  bool supports_extended_frames{false};
  bool supports_filters{false};
  bool supports_error_frames{false};
  bool supports_timestamps{false};
  bool supports_non_blocking_io{false};
  bool nominal_bitrate_configurable{false};
  std::uint32_t nominal_bitrate_hz{0U};
  std::uint8_t max_payload_bytes{0U};
  std::uint16_t queue_capacity{0U};

  [[nodiscard]] bool is_valid() const noexcept {
    if (max_payload_bytes == 0U || max_payload_bytes > kMaxCanPayloadBytes ||
        queue_capacity == 0U ||
        (!supports_classic_can && !supports_can_fd) ||
        (!supports_standard_frames && !supports_extended_frames) ||
        !supports_non_blocking_io || nominal_bitrate_hz == 0U ||
        (supports_brs && !supports_can_fd)) {
      return false;
    }
    if (supports_classic_can && max_payload_bytes < 8U) {
      return false;
    }
    if (supports_can_fd && max_payload_bytes < 8U) {
      return false;
    }
    return true;
  }
};

struct BusConfig final {
  std::uint16_t logical_bus{0U};
  std::string physical_channel;
  TransportKind transport{TransportKind::Unknown};
  TransportCapabilities capabilities;
};

struct DeviceConfig final {
  std::uint16_t device_id{0U};
  std::string name;
  std::uint16_t logical_bus{0U};
  ProtocolProfile profile{ProtocolProfile::Unknown};
  CanFrameType frame_type{CanFrameType::Classic};
  CanFrameFormat frame_format{CanFrameFormat::Standard};
  std::optional<CanId> command_id;
  std::optional<CanId> feedback_id;
  std::uint8_t command_payload_bytes{0U};
  std::uint8_t feedback_payload_bytes{0U};
  bool writable{false};
};

struct DeploymentConfig final {
  SchemaVersion schema_version{0U, 0U};
  std::vector<BusConfig> buses;
  std::vector<DeviceConfig> devices;
};

enum class ConfigErrorCode : std::uint8_t {
  SchemaMismatch,
  MissingField,
  DuplicateBus,
  DuplicatePhysicalChannel,
  InvalidTransport,
  InvalidCapability,
  DuplicateDevice,
  DuplicateDeviceName,
  UnknownProfile,
  UnknownBus,
  DuplicateRouteId,
  InvalidFrameId,
  InvalidPayloadSize,
  IncompatibleFrameFormat,
  IncompatibleCapability,
};

struct ConfigError final {
  ConfigErrorCode code;
  std::string subject;
};

struct ConfigValidation final {
  std::vector<ConfigError> errors;

  [[nodiscard]] bool valid() const noexcept { return errors.empty(); }

  [[nodiscard]] bool has(ConfigErrorCode code) const noexcept {
    for (const auto& error : errors) {
      if (error.code == code) {
        return true;
      }
    }
    return false;
  }
};

struct ProfileRequirements final {
  std::optional<CanFrameType> frame_type;
  std::optional<CanFrameFormat> frame_format;
};

[[nodiscard]] inline std::optional<ProfileRequirements> profile_requirements(
    ProtocolProfile profile) noexcept {
  switch (profile) {
    case ProtocolProfile::LoopbackV1:
      return ProfileRequirements{};
    case ProtocolProfile::L02ServoExtended:
      return ProfileRequirements{CanFrameType::Classic,
                                 CanFrameFormat::Extended};
    case ProtocolProfile::L02MitStandard:
      return ProfileRequirements{CanFrameType::Classic,
                                 CanFrameFormat::Standard};
    case ProtocolProfile::Hi12J1939:
      return ProfileRequirements{CanFrameType::Classic,
                                 CanFrameFormat::Extended};
    case ProtocolProfile::Hi12Canopen:
      return ProfileRequirements{CanFrameType::Classic,
                                 CanFrameFormat::Standard};
    case ProtocolProfile::Unknown:
      return std::nullopt;
  }
  return std::nullopt;
}

[[nodiscard]] inline bool same_route_id(const CanId& lhs,
                                        const CanId& rhs) noexcept {
  return lhs.value == rhs.value && lhs.format == rhs.format;
}

[[nodiscard]] inline ConfigValidation validate_deployment(
    const DeploymentConfig& config) {
  ConfigValidation result;
  const auto add = [&result](ConfigErrorCode code, std::string subject) {
    result.errors.push_back(ConfigError{code, std::move(subject)});
  };

  if (config.schema_version.major != kSchemaV1.major ||
      config.schema_version.minor != kSchemaV1.minor) {
    add(ConfigErrorCode::SchemaMismatch, "schema_version");
  }
  if (config.buses.empty()) {
    add(ConfigErrorCode::MissingField, "buses");
  }

  for (std::size_t index = 0U; index < config.buses.size(); ++index) {
    const auto& bus = config.buses[index];
    const auto subject = "buses[" + std::to_string(index) + "]";
    if (bus.logical_bus == 0U || bus.physical_channel.empty()) {
      add(ConfigErrorCode::MissingField, subject);
    }
    if (bus.transport == TransportKind::Unknown) {
      add(ConfigErrorCode::InvalidTransport, subject);
    }
    if (!bus.capabilities.is_valid()) {
      add(ConfigErrorCode::InvalidCapability, subject);
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (config.buses[previous].logical_bus == bus.logical_bus) {
        add(ConfigErrorCode::DuplicateBus, subject);
      }
      if (!bus.physical_channel.empty() &&
          config.buses[previous].physical_channel == bus.physical_channel) {
        add(ConfigErrorCode::DuplicatePhysicalChannel, subject);
      }
    }
  }

  for (std::size_t index = 0U; index < config.devices.size(); ++index) {
    const auto& device = config.devices[index];
    const auto subject = "devices[" + std::to_string(index) + "]";
    if (device.device_id == 0U || device.name.empty() ||
        device.logical_bus == 0U || !device.feedback_id.has_value() ||
        (device.writable && !device.command_id.has_value())) {
      add(ConfigErrorCode::MissingField, subject);
    }
    if (profile_requirements(device.profile) == std::nullopt) {
      add(ConfigErrorCode::UnknownProfile, subject);
    }
    if (device.command_id.has_value() &&
        !device.command_id->is_valid()) {
      add(ConfigErrorCode::InvalidFrameId, subject + ".command_id");
    }
    if (device.feedback_id.has_value() &&
        !device.feedback_id->is_valid()) {
      add(ConfigErrorCode::InvalidFrameId, subject + ".feedback_id");
    }
    if (device.command_id.has_value() && device.feedback_id.has_value() &&
        same_route_id(*device.command_id, *device.feedback_id)) {
      add(ConfigErrorCode::DuplicateRouteId, subject);
    }
    const auto maximum = device.frame_type == CanFrameType::Classic
                             ? 8U
                             : kMaxCanPayloadBytes;
    if (device.feedback_payload_bytes == 0U ||
        device.feedback_payload_bytes > maximum ||
        (device.writable && (device.command_payload_bytes == 0U ||
                             device.command_payload_bytes > maximum)) ||
        (!device.writable && (device.command_id.has_value() ||
                              device.command_payload_bytes != 0U))) {
      add(ConfigErrorCode::InvalidPayloadSize, subject);
    }
    const auto requirements = profile_requirements(device.profile);
    if (requirements.has_value() &&
        ((requirements->frame_type.has_value() &&
          requirements->frame_type != device.frame_type) ||
         (requirements->frame_format.has_value() &&
          requirements->frame_format != device.frame_format))) {
      add(ConfigErrorCode::IncompatibleFrameFormat, subject);
    }
    const auto bus = [&config, &device]() -> const BusConfig* {
      for (const auto& candidate : config.buses) {
        if (candidate.logical_bus == device.logical_bus) {
          return &candidate;
        }
      }
      return nullptr;
    }();
    if (bus == nullptr) {
      add(ConfigErrorCode::UnknownBus, subject);
    } else {
      const auto& capabilities = bus->capabilities;
      if ((device.frame_type == CanFrameType::Classic &&
           !capabilities.supports_classic_can) ||
          (device.frame_type == CanFrameType::FlexibleDataRate &&
           !capabilities.supports_can_fd) ||
          (device.frame_format == CanFrameFormat::Standard &&
           !capabilities.supports_standard_frames) ||
          (device.frame_format == CanFrameFormat::Extended &&
           !capabilities.supports_extended_frames) ||
          (device.writable &&
           device.command_payload_bytes > capabilities.max_payload_bytes) ||
          device.feedback_payload_bytes > capabilities.max_payload_bytes) {
        add(ConfigErrorCode::IncompatibleCapability, subject);
      }
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (config.devices[previous].device_id == device.device_id) {
        add(ConfigErrorCode::DuplicateDevice, subject);
      }
      if (!device.name.empty() && config.devices[previous].name == device.name) {
        add(ConfigErrorCode::DuplicateDeviceName, subject);
      }
      if (config.devices[previous].logical_bus == device.logical_bus) {
        const std::optional<CanId> current_ids[] = {device.command_id,
                                                    device.feedback_id};
        const std::optional<CanId> previous_ids[] = {
            config.devices[previous].command_id,
            config.devices[previous].feedback_id};
        for (const auto& current_id : current_ids) {
          for (const auto& previous_id : previous_ids) {
            if (current_id.has_value() && previous_id.has_value() &&
                same_route_id(*current_id, *previous_id)) {
              add(ConfigErrorCode::DuplicateRouteId, subject);
            }
          }
        }
      }
    }
  }
  return result;
}

}  // namespace mech::mech_control_core
