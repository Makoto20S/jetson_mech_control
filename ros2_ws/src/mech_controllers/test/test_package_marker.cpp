#include "mech_controllers/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechControllers, FoundationPackageMarker) {
  EXPECT_EQ(mech::mech_controllers::package_name(), "mech_controllers");
}
