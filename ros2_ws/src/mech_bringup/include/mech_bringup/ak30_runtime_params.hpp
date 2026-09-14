#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>

#include "mech_bringup/ak30_force_runtime.hpp"

namespace mech::mech_bringup {

// Parses ros2_control hardware parameters (the URDF ros2_control block's
// <param> entries arrive as a string map) into Ak30RuntimeConfig plus the
// serial device path. Fails closed: any missing mandatory key, unknown key,
// or out-of-range value rejects the whole configuration - a deployment with
// a typo must fail at configure, never run on guessed values.
//
// Since ADR-014 the sub-mode is an explicit parameter ("position",
// "velocity", "torque"; default "position"). The deployment's URDF command
// interface must match it (Position->position, Velocity->velocity,
// Torque->effort); the deployment-files structure test pins that
// correspondence offline.
struct Ak30RuntimeParams final {
  Ak30RuntimeConfig config{};
  std::string device_path;

  // Returns std::nullopt on any invalid or unknown parameter.
  [[nodiscard]] static std::optional<Ak30RuntimeParams> parse(
      const std::map<std::string, std::string>& params) noexcept;
};

// The ros2_control command-interface name a joint must declare for the
// given sub-mode (ADR-014): the interface shape and the sub-mode are two
// spellings of the same choice, and the deployment files must agree.
[[nodiscard]] const char* expected_command_interface_name(
    mech::mech_protocol_cubemars::ForceControlSubMode sub_mode) noexcept;

}  // namespace mech::mech_bringup
