#pragma once

#include <string_view>

namespace mech::mech_bringup {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_bringup";
}

}  // namespace mech::mech_bringup
