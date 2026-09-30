#include "mech_bringup/ak30_servo_runtime_params.hpp"

#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <unordered_map>
#include <set>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_protocol_cubemars/ak30_servo_position_session.hpp"

namespace mech::mech_bringup {
namespace {

using Parameters = std::unordered_map<std::string, std::string>;
using mech_protocol_cubemars::Ak30ServoPositionSession;
using mech_protocol_cubemars::ServoPositionSessionConfig;

const std::set<std::string>& hardware_keys() {
  static const std::set<std::string> keys{
      "profile", "device_path", "logical_bus", "control_period_ns",
      "command_ttl_ns", "command_hard_ttl_ns", "feedback_ttl_ns"};
  return keys;
}

const std::set<std::string>& joint_keys() {
  static const std::set<std::string> keys{
      "drive_id", "target_scale", "target_offset", "target_mapping_verified",
      "feedback_scale", "feedback_offset", "feedback_mapping_verified",
      "speed_erpm", "acceleration_raw", "position_min_rad",
      "position_max_rad", "position_max_error_rad"};
  return keys;
}

bool exactly_keys(const Parameters& params,
                  const std::set<std::string>& keys) noexcept {
  if (params.size() != keys.size()) return false;
  for (const auto& key : keys)
    if (params.count(key) != 1U) return false;
  return true;
}

bool parse_uint(const std::string& source, std::uint64_t& value) noexcept {
  if (source.empty()) return false;
  const auto result = std::from_chars(source.data(),
                                      source.data() + source.size(), value);
  return result.ec == std::errc{} && result.ptr == source.data() + source.size();
}

bool parse_positive_ns(const Parameters& params, const char* key,
                       std::int64_t& value) noexcept {
  std::uint64_t parsed = 0U;
  if (!parse_uint(params.at(key), parsed) || parsed == 0U ||
      parsed > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    return false;
  value = static_cast<std::int64_t>(parsed);
  return true;
}

bool parse_real(const Parameters& params, const char* key,
                double& value) noexcept {
  const auto& source = params.at(key);
  if (source.empty() ||
      source.find_first_not_of("0123456789+-.eE") != std::string::npos)
    return false;
  errno = 0;
  char* end = nullptr;
  const double parsed = std::strtod(source.c_str(), &end);
  if (errno != 0 || end == source.c_str() ||
      end != source.c_str() + source.size() || !std::isfinite(parsed))
    return false;
  value = parsed;
  return true;
}

bool parse_bool(const Parameters& params, const char* key,
                bool& value) noexcept {
  const auto& source = params.at(key);
  if (source == "true") {
    value = true;
    return true;
  }
  if (source == "false") {
    value = false;
    return true;
  }
  return false;
}

bool valid_interfaces(const hardware_interface::ComponentInfo& joint) noexcept {
  if (joint.state_interfaces.size() != 1U ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.command_interfaces.size() != 2U) return false;
  bool has_position = false;
  bool has_generation = false;
  for (const auto& command : joint.command_interfaces) {
    if (command.name == hardware_interface::HW_IF_POSITION && !has_position)
      has_position = true;
    else if (command.name ==
        mech_hardware_ros2_control::kCommandGenerationInterface &&
        !has_generation)
      has_generation = true;
    else
      return false;
  }
  return has_position && has_generation;
}

bool parse_joint(const Parameters& params, std::uint16_t bus,
                 std::int64_t ttl, std::int64_t hard_ttl,
                 std::int64_t feedback_ttl,
                 ServoPositionSessionConfig& config) noexcept {
  if (!exactly_keys(params, joint_keys())) return false;
  std::uint64_t id = 0U;
  if (!parse_uint(params.at("drive_id"), id) || id > 255U) return false;
  config.drive_id = static_cast<std::uint16_t>(id);
  config.logical_bus = bus;
  config.command_ttl_ns = ttl;
  config.command_hard_ttl_ns = hard_ttl;
  config.feedback_ttl_ns = feedback_ttl;
  if (!parse_real(params, "target_scale", config.target_rad_to_deg.scale) ||
      !parse_real(params, "target_offset", config.target_rad_to_deg.offset) ||
      !parse_bool(params, "target_mapping_verified",
                  config.target_rad_to_deg.evidence_declared) ||
      !parse_real(params, "feedback_scale", config.feedback_deg_to_rad.scale) ||
      !parse_real(params, "feedback_offset", config.feedback_deg_to_rad.offset) ||
      !parse_bool(params, "feedback_mapping_verified",
                  config.feedback_deg_to_rad.evidence_declared) ||
      !parse_real(params, "speed_erpm", config.speed_erpm) ||
      !parse_real(params, "acceleration_raw", config.acceleration_raw) ||
      !parse_real(params, "position_min_rad", config.position_min_rad) ||
      !parse_real(params, "position_max_rad", config.position_max_rad) ||
      !parse_real(params, "position_max_error_rad", config.max_target_error_rad))
    return false;
  Ak30ServoPositionSession session;
  return session.configure(config) == mech_control_core::AdapterResult::Ok;
}

}  // namespace

std::optional<Ak30ServoRuntimeParams> Ak30ServoRuntimeParams::parse(
    const hardware_interface::HardwareInfo& info) noexcept {
  if (info.name.empty() || info.type != "system" ||
      !info.sensors.empty() || !info.gpios.empty() ||
      info.joints.empty() || info.joints.size() > 6U ||
      !exactly_keys(info.hardware_parameters, hardware_keys()) ||
      info.hardware_parameters.at("profile") != "ak30_servo_extended")
    return std::nullopt;

  Ak30ServoRuntimeParams parsed;
  parsed.device_path = info.hardware_parameters.at("device_path");
  if (parsed.device_path.empty()) return std::nullopt;
  parsed.config.physical_bus = parsed.device_path;
  std::uint64_t bus = 0U;
  if (!parse_uint(info.hardware_parameters.at("logical_bus"), bus) ||
      bus == 0U || bus > std::numeric_limits<std::uint16_t>::max())
    return std::nullopt;
  parsed.config.logical_bus = static_cast<std::uint16_t>(bus);
  if (!parse_positive_ns(info.hardware_parameters, "control_period_ns",
                         parsed.config.control_period_ns)) return std::nullopt;
  std::int64_t ttl = 0;
  std::int64_t hard_ttl = 0;
  std::int64_t feedback_ttl = 0;
  if (!parse_positive_ns(info.hardware_parameters, "command_ttl_ns", ttl) ||
      !parse_positive_ns(info.hardware_parameters, "command_hard_ttl_ns",
                         hard_ttl) ||
      !parse_positive_ns(info.hardware_parameters, "feedback_ttl_ns",
                         feedback_ttl) ||
      hard_ttl > 6000000 || ttl >= hard_ttl ||
      parsed.config.control_period_ns < (hard_ttl + 2) / 3)
    return std::nullopt;

  std::set<std::string> names;
  std::set<std::uint16_t> ids;
  for (const auto& joint : info.joints) {
    if (joint.name.empty() || joint.type != "joint" ||
        !names.insert(joint.name).second || !valid_interfaces(joint))
      return std::nullopt;
    ServoPositionSessionConfig config;
    if (!parse_joint(joint.parameters, parsed.config.logical_bus,
                     ttl, hard_ttl, feedback_ttl, config) ||
        !ids.insert(config.drive_id).second)
      return std::nullopt;
    parsed.config.joints.push_back(config);
  }
  return parsed;
}

}  // namespace mech::mech_bringup
