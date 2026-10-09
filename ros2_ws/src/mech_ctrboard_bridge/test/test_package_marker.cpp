#include "mech_ctrboard_bridge/package_marker.hpp"
#include "mech_ctrboard_bridge/topic_names.hpp"

#include <gtest/gtest.h>
#include <rcl/validate_topic_name.h>

#include <array>

TEST(MechCtrboardBridge, PackageMarker) {
  EXPECT_EQ(mech::mech_ctrboard_bridge::package_name(),
            "mech_ctrboard_bridge");
}

TEST(MechCtrboardBridge, PublishedTopicNamesAreValid) {
  constexpr std::array<const char*, 4U> topics{
      mech::mech_ctrboard_bridge::kImu1DataTopic,
      mech::mech_ctrboard_bridge::kImu2DataTopic,
      mech::mech_ctrboard_bridge::kImu1EulerTopic,
      mech::mech_ctrboard_bridge::kImu2EulerTopic,
  };

  for (const char* topic : topics) {
    int validation_result = RCL_TOPIC_NAME_INVALID_IS_EMPTY_STRING;
    std::size_t invalid_index = 0U;
    ASSERT_EQ(rcl_validate_topic_name(topic, &validation_result, &invalid_index),
              RCL_RET_OK);
    EXPECT_EQ(validation_result, RCL_TOPIC_NAME_VALID) << topic;
  }
}
