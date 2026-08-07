#pragma once

#include <string_view>

namespace mech::mech_control_core {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_control_core";
}

}  // namespace mech::mech_control_core
