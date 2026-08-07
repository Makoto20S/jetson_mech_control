#pragma once

#include <string_view>

namespace mech::mech_simulation {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_simulation";
}

}  // namespace mech::mech_simulation
