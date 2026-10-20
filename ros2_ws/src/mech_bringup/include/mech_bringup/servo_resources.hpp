#pragma once

#include <filesystem>
#include <set>
#include <stdexcept>
#include <vector>
#include "mech_bringup/ak30_servo_runtime_params.hpp"

namespace mech::mech_bringup {
// Validate the entire shutdown/observation plan before opening any port.
inline std::vector<Ak30ServoRuntimeParams> parse_servo_resources(
    const std::vector<hardware_interface::HardwareInfo>& resources) {
  if (resources.empty() || resources.size() > 2)
    throw std::runtime_error("expected one or two Ak30ServoSystem resources");
  std::vector<Ak30ServoRuntimeParams> result;
  std::set<std::string> paths, names;
  std::set<unsigned> buses, ids;
  for (const auto& resource : resources) {
    if (resource.hardware_class_type != "mech_bringup/Ak30ServoSystem")
      throw std::runtime_error("expected Ak30ServoSystem");
    auto parsed = Ak30ServoRuntimeParams::parse(resource);
    if (!parsed || parsed->config.joints.empty())
      throw std::runtime_error("invalid servo configuration");
    if (!paths.insert(std::filesystem::weakly_canonical(parsed->device_path).string()).second ||
        !buses.insert(parsed->config.logical_bus).second || !names.insert(resource.name).second)
      throw std::runtime_error("duplicate servo port, bus or component");
    for (const auto& joint : parsed->config.joints)
      if (!ids.insert(joint.drive_id).second)
        throw std::runtime_error("duplicate servo motor ID");
    result.push_back(std::move(*parsed));
  }
  return result;
}
}  // namespace mech::mech_bringup
