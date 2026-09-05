#include "mech_protocol_cubemars/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechProtocolCubemars, PackageMarker) {
  EXPECT_EQ(mech::mech_protocol_cubemars::package_name(),
            "mech_protocol_cubemars");
}
