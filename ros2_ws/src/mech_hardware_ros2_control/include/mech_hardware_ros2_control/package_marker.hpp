#pragma once

#include <string_view>

namespace mech::mech_hardware_ros2_control {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_hardware_ros2_control";
}

}  // namespace mech::mech_hardware_ros2_control
