#include "mech_ctrboard_bridge/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechCtrboardBridge, PackageMarker) {
  EXPECT_EQ(mech::mech_ctrboard_bridge::package_name(),
            "mech_ctrboard_bridge");
}
