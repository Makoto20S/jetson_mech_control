#include "mech_bringup/ak30_runtime_params.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mech::mech_bringup {
namespace {

// Every parameter this parser accepts. An entry not in this set rejects the
// configuration: unknown keys are how a renamed parameter silently loses its
// value in a deployment file.
const std::set<std::string>& known_keys() noexcept {
  static const std::set<std::string> keys{
      "device_path",
      "logical_bus",
      "drive_id",
      "device_id",
      "sub_mode",
      "kp",
      "kd",
      "control_period_ns",
      "command_ttl_ns",
      "command_hard_ttl_ns",
      "feedback_ttl_ns",
      "zero_offset_rad",
      "position_is_output_shaft",
  };
  return keys;
}

[[nodiscard]] bool parse_uint64(const std::string& text,
                                std::uint64_t& output) noexcept {
  if (text.empty() || text.find('-') != std::string::npos) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const unsigned long long value = std::strtoull(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0') {
    return false;
  }
  output = static_cast<std::uint64_t>(value);
  return true;
}

[[nodiscard]] bool parse_double(const std::string& text,
                                double& output) noexcept {
  if (text.empty()) {
    return false;
  }
  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || *end != '\0' || !std::isfinite(value)) {
    return false;
  }
  output = value;
  return true;
}

[[nodiscard]] bool parse_bool(const std::string& text,
                              bool& output) noexcept {
  if (text == "true" || text == "1") {
    output = true;
    return true;
  }
  if (text == "false" || text == "0") {
    output = false;
    return true;
  }
  return false;
}

}  // namespace

const char* expected_command_interface_name(
    mech::mech_protocol_cubemars::ForceControlSubMode sub_mode) noexcept {
  using mech::mech_protocol_cubemars::ForceControlSubMode;
  switch (sub_mode) {
    case ForceControlSubMode::Position:
      return hardware_interface::HW_IF_POSITION;
    case ForceControlSubMode::Velocity:
      return hardware_interface::HW_IF_VELOCITY;
    case ForceControlSubMode::Torque:
      return hardware_interface::HW_IF_EFFORT;
  }
  return hardware_interface::HW_IF_POSITION;
}

std::optional<Ak30RuntimeParams> Ak30RuntimeParams::parse(
    const std::map<std::string, std::string>& params) noexcept {
  for (const auto& entry : params) {
    if (known_keys().count(entry.first) == 0U) {
      return std::nullopt;
    }
  }

  Ak30RuntimeParams parsed{};
  Ak30RuntimeConfig& config = parsed.config;

  const auto device_path = params.find("device_path");
  if (device_path == params.end() || device_path->second.empty()) {
    // Mandatory with no default: the serial device must be named by the
    // deployment, never guessed.
    return std::nullopt;
  }
  parsed.device_path = device_path->second;

  if (const auto it = params.find("logical_bus"); it != params.end()) {
    std::uint64_t value = 0U;
    if (!parse_uint64(it->second, value) || value == 0U ||
        value > std::numeric_limits<std::uint32_t>::max()) {
      return std::nullopt;
    }
    config.logical_bus = static_cast<std::uint32_t>(value);
  }
  if (const auto it = params.find("drive_id"); it != params.end()) {
    std::uint64_t value = 0U;
    if (!parse_uint64(it->second, value) || value > 255U) {
      return std::nullopt;
    }
    config.drive_id = static_cast<std::uint16_t>(value);
  }
  if (const auto it = params.find("device_id"); it != params.end()) {
    std::uint64_t value = 0U;
    if (!parse_uint64(it->second, value) || value == 0U ||
        value > std::numeric_limits<std::uint32_t>::max()) {
      return std::nullopt;
    }
    config.device_id = static_cast<std::uint32_t>(value);
  }
  if (const auto it = params.find("sub_mode"); it != params.end()) {
    // Explicit configuration, never inferred from payload or interface
    // claims (ADR-004): the deployment states which sub-mode the device
    // runs, and the URDF command interface must match it.
    using mech::mech_protocol_cubemars::ForceControlSubMode;
    if (it->second == "position") {
      config.sub_mode = ForceControlSubMode::Position;
    } else if (it->second == "velocity") {
      config.sub_mode = ForceControlSubMode::Velocity;
    } else if (it->second == "torque") {
      config.sub_mode = ForceControlSubMode::Torque;
    } else {
      return std::nullopt;
    }
  }
  if (const auto it = params.find("kp"); it != params.end()) {
    double value = 0.0;
    if (!parse_double(it->second, value) || value < 0.0) {
      return std::nullopt;
    }
    config.gains.kp = value;
  }
  if (const auto it = params.find("kd"); it != params.end()) {
    double value = 0.0;
    if (!parse_double(it->second, value) || value < 0.0) {
      return std::nullopt;
    }
    config.gains.kd = value;
  }
  if (const auto it = params.find("control_period_ns"); it != params.end()) {
    std::uint64_t value = 0U;
    if (!parse_uint64(it->second, value) ||
        value > static_cast<std::uint64_t>(
                    std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    config.control_period_nanoseconds = static_cast<std::int64_t>(value);
  }
  if (const auto it = params.find("command_ttl_ns"); it != params.end()) {
    std::uint64_t value = 0U;
    if (!parse_uint64(it->second, value) ||
        value > static_cast<std::uint64_t>(
                    std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    config.command_ttl_nanoseconds = static_cast<std::int64_t>(value);
  }
  if (const auto it = params.find("command_hard_ttl_ns");
      it != params.end()) {
    std::uint64_t value = 0U;
    if (!parse_uint64(it->second, value) ||
        value > static_cast<std::uint64_t>(
                    std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    config.command_hard_ttl_nanoseconds = static_cast<std::int64_t>(value);
  }
  if (const auto it = params.find("feedback_ttl_ns"); it != params.end()) {
    std::uint64_t value = 0U;
    if (!parse_uint64(it->second, value) ||
        value > static_cast<std::uint64_t>(
                    std::numeric_limits<std::int64_t>::max())) {
      return std::nullopt;
    }
    config.feedback_ttl_nanoseconds = static_cast<std::int64_t>(value);
  }
  if (const auto it = params.find("zero_offset_rad"); it != params.end()) {
    double value = 0.0;
    if (!parse_double(it->second, value)) {
      return std::nullopt;
    }
    config.mapping.zero_offset_rad = {value, true};
  }
  if (const auto it = params.find("position_is_output_shaft");
      it != params.end()) {
    bool value = false;
    if (!parse_bool(it->second, value)) {
      return std::nullopt;
    }
    config.mapping.position_is_output_shaft = value;
  }

  // Cross-field validation the session would also catch, but failing here
  // keeps the error at the deployment-parameter level where it belongs.
  if (config.control_period_nanoseconds <= 0 ||
      config.command_ttl_nanoseconds <= 0 ||
      config.command_hard_ttl_nanoseconds <= config.command_ttl_nanoseconds ||
      config.feedback_ttl_nanoseconds <= 0) {
    return std::nullopt;
  }
  // ADR-012: the whole staged watchdog fits <=3 control cycles (<=6 ms at
  // the documented 500 Hz).
  if (config.command_hard_ttl_nanoseconds >
      mech::mech_protocol_cubemars::kMaxHardTtlNanoseconds) {
    return std::nullopt;
  }

  return parsed;
}

}  // namespace mech::mech_bringup
