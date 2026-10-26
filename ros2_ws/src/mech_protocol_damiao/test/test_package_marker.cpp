#include "mech_protocol_damiao/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechProtocolDamiao, PackageMarker) {
  EXPECT_EQ(mech::mech_protocol_damiao::package_name(),
            "mech_protocol_damiao");
}
