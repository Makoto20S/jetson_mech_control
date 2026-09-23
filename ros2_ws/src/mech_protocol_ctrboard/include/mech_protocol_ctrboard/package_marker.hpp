#pragma once

#include <string_view>

namespace mech::mech_protocol_ctrboard {

[[nodiscard]] constexpr std::string_view package_name() noexcept {
  return "mech_protocol_ctrboard";
}

}  // namespace mech::mech_protocol_ctrboard
