#include "mech_simulation/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechSimulation, FoundationPackageMarker) {
  EXPECT_EQ(mech::mech_simulation::package_name(), "mech_simulation");
}
