#pragma once

#include <optional>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "mech_bringup/ak30_servo_runtime.hpp"

namespace mech::mech_bringup {

// A complete deployment declaration. Parsing performs no transport I/O.
struct Ak30ServoRuntimeParams final {
  Ak30ServoRuntimeConfig config{};
  std::string device_path;

  [[nodiscard]] static std::optional<Ak30ServoRuntimeParams> parse(
      const hardware_interface::HardwareInfo& info) noexcept;
};

}  // namespace mech::mech_bringup
