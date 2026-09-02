#include "mech_protocol_cubemars/package_marker.hpp"

namespace mech::mech_protocol_cubemars {

// Translation unit exists so the package produces a library target.
static_assert(package_name() == "mech_protocol_cubemars");

}  // namespace mech::mech_protocol_cubemars
