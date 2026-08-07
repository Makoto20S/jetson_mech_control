#include "mech_hardware_ros2_control/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechHardwareRos2Control, FoundationPackageMarker) {
  EXPECT_EQ(
    mech::mech_hardware_ros2_control::package_name(),
    "mech_hardware_ros2_control");
}
