#pragma once

#include <string_view>

namespace mech::mech_protocol_cubemars {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_protocol_cubemars";
}

}  // namespace mech::mech_protocol_cubemars
