#pragma once

#include <string_view>

namespace mech::mech_controllers {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_controllers";
}

}  // namespace mech::mech_controllers
