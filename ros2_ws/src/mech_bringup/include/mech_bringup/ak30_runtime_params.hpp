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
// The sub-mode is deliberately not a parameter: this slice is Position-only
// because that is the interface shape CompositeSystem exports. Torque/
// Velocity command interfaces would be a canonical contract change requiring
// an ADR first (adapter_contract_v1.md item 7).
struct Ak30RuntimeParams final {
  Ak30RuntimeConfig config{};
  std::string device_path;

  // Returns std::nullopt on any invalid or unknown parameter.
  [[nodiscard]] static std::optional<Ak30RuntimeParams> parse(
      const std::map<std::string, std::string>& params) noexcept;
};

}  // namespace mech::mech_bringup
