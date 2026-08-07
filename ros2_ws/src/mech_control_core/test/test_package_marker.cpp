#include "mech_control_core/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechControlCore, FoundationPackageMarker) {
  EXPECT_EQ(mech::mech_control_core::package_name(), "mech_control_core");
}
