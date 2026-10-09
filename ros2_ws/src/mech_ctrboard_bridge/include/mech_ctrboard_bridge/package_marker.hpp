#pragma once

#include <string_view>

namespace mech::mech_ctrboard_bridge {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_ctrboard_bridge";
}

}  // namespace mech::mech_ctrboard_bridge
