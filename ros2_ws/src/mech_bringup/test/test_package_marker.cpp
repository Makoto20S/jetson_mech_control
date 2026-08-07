#include "mech_bringup/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechBringup, FoundationPackageMarker) {
  EXPECT_EQ(mech::mech_bringup::package_name(), "mech_bringup");
}
