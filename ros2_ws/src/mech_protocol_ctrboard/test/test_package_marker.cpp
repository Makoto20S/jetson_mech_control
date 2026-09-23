#include "mech_protocol_ctrboard/package_marker.hpp"

#include <gtest/gtest.h>

TEST(MechProtocolCtrboard, PackageMarker) {
  EXPECT_EQ(mech::mech_protocol_ctrboard::package_name(),
            "mech_protocol_ctrboard");
}
