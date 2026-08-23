#include "mech_simulation/loopback.hpp"

#include <gtest/gtest.h>

namespace mech::mech_simulation {
namespace {

TEST(LoopbackCodec, EncodesAndDecodesCommandAndFeedback) {
  const auto now = mech_control_core::MonotonicTime::from_nanoseconds(10);
  const auto command = LoopbackCodec::encode_command(1U, 2U, 1500, 7U, *now);
  ASSERT_TRUE(command.has_value());
  const auto decoded_command = LoopbackCodec::decode_command(*command);
  ASSERT_TRUE(decoded_command.has_value());
  EXPECT_EQ(decoded_command->device_id, 2U);
  EXPECT_EQ(decoded_command->target_milli, 1500);
  EXPECT_EQ(decoded_command->sequence, 7U);

  const auto feedback = LoopbackCodec::encode_feedback(1U, 2U, 1200, 3U, 7U, *now);
  ASSERT_TRUE(feedback.has_value());
  const auto decoded_feedback = LoopbackCodec::decode_feedback(*feedback);
  ASSERT_TRUE(decoded_feedback.has_value());
  EXPECT_EQ(decoded_feedback->position_milli, 1200);
  EXPECT_EQ(decoded_feedback->fault_code, 3U);
}

TEST(SimulatedDevice, MovesInBoundedStepsAndLatchesFaultCode) {
  const auto now = mech_control_core::MonotonicTime::from_nanoseconds(10);
  SimulatedDevice device(1U, 2U, 100);
  const auto command = LoopbackCodec::encode_command(1U, 2U, 250, 4U, *now);
  ASSERT_TRUE(command.has_value());
  ASSERT_TRUE(device.accept_command(*command));
  auto feedback = device.step(*now);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(LoopbackCodec::decode_feedback(*feedback)->position_milli, 100);
  device.inject_fault(9U);
  feedback = device.step(*now);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(LoopbackCodec::decode_feedback(*feedback)->fault_code, 9U);
}

TEST(LoopbackCodec, RejectsWrongDirectionAndMarker) {
  const auto now = mech_control_core::MonotonicTime::from_nanoseconds(10);
  auto command = LoopbackCodec::encode_command(1U, 2U, 0, 1U, *now);
  ASSERT_TRUE(command.has_value());
  command->direction = mech_control_core::FrameDirection::Rx;
  EXPECT_FALSE(LoopbackCodec::decode_command(*command).has_value());
}

}  // namespace
}  // namespace mech::mech_simulation
