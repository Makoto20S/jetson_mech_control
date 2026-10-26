#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace mech::mech_protocol_damiao {
namespace {

using mech_control_core::CanFrameFormat;
using mech_control_core::CanFrameType;
using mech_control_core::CanId;
using mech_control_core::FrameDirection;
using mech_control_core::MonotonicTime;
using mech_control_core::RawCanFrame;

RawCanFrame feedback_frame() {
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> bytes{};
  // Vendor SDK feedback layout: fault/id, p16, v12, torque12, temperatures.
  bytes[0] = 0x21U;
  bytes[1] = 0x7FU;
  bytes[2] = 0xFFU;
  bytes[3] = 0x7FU;
  bytes[4] = 0xF7U;
  bytes[5] = 0xFFU;
  bytes[6] = 35U;
  bytes[7] = 36U;
  return RawCanFrame::create(2U, *CanId::create(0x11U,
                             CanFrameFormat::Standard),
                             CanFrameType::Classic, FrameDirection::Rx, 8U,
                             bytes, MonotonicTime{}).value();
}

TEST(Dm4340MitWire, SdkZeroCommandVector) {
  std::array<std::uint8_t, 8> payload{};
  ASSERT_TRUE(encode_mit({0.0, 0.0, 0.0, 0.0, 0.0},
                         kSdkDm4340Limits, payload));
  const std::array<std::uint8_t, 8> expected{
      0x7FU, 0xFFU, 0x7FU, 0xF0U, 0x00U, 0x00U, 0x07U, 0xFFU};
  EXPECT_EQ(payload, expected);
}

TEST(Dm4340MitWire, SdkPositiveAndNegativeEndpoints) {
  std::array<std::uint8_t, 8> payload{};
  ASSERT_TRUE(encode_mit({12.5, 10.0, 500.0, 5.0, 28.0},
                         kSdkDm4340Limits, payload));
  EXPECT_EQ(payload, (std::array<std::uint8_t, 8>{
                         0xFFU, 0xFFU, 0xFFU, 0xFFU,
                         0xFFU, 0xFFU, 0xFFU, 0xFFU}));
  ASSERT_TRUE(encode_mit({-12.5, -10.0, 0.0, 0.0, -28.0},
                         kSdkDm4340Limits, payload));
  EXPECT_EQ(payload, (std::array<std::uint8_t, 8>{
                         0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U}));
}

TEST(Dm4340MitWire, RejectsUnsafeOrUnknownRangesWithoutChangingOutput) {
  std::array<std::uint8_t, 8> payload{};
  payload.fill(0xA5U);
  EXPECT_FALSE(encode_mit({0, 0, 0, 0, 0}, {0, 10, 28}, payload));
  EXPECT_FALSE(encode_mit({12.6, 0, 0, 0, 0}, kSdkDm4340Limits,
                          payload));
  EXPECT_FALSE(encode_mit({0, 0, 0, 0, 28.1}, kSdkDm4340Limits,
                          payload));
  EXPECT_FALSE(encode_mit({0, 0, 501, 0, 0}, kSdkDm4340Limits,
                          payload));
  EXPECT_FALSE(encode_mit({0, 0, 0, 5.1, 0}, kSdkDm4340Limits,
                          payload));
  EXPECT_FALSE(encode_mit({std::numeric_limits<double>::quiet_NaN(),
                           0, 0, 0, 0}, kSdkDm4340Limits, payload));
  EXPECT_FALSE(encode_mit({0, 0, 0, 0,
                           std::numeric_limits<double>::infinity()},
                          kSdkDm4340Limits, payload));
  EXPECT_EQ(payload, (std::array<std::uint8_t, 8>{
                         0xA5U, 0xA5U, 0xA5U, 0xA5U,
                         0xA5U, 0xA5U, 0xA5U, 0xA5U}));
}

TEST(Dm4340MitWire, DecodesFeedbackAndKeepsRawFault) {
  MitFeedback feedback{};
  ASSERT_TRUE(decode_mit_feedback(feedback_frame(), 2U, 0x11U, 1U,
                                  kSdkDm4340Limits, feedback));
  EXPECT_NEAR(feedback.position_rad, 0.0, 0.001);
  EXPECT_NEAR(feedback.velocity_rad_s, 0.0, 0.01);
  EXPECT_NEAR(feedback.torque_nm, 0.0, 0.02);
  EXPECT_EQ(feedback.raw_fault, 2U);
  EXPECT_EQ(feedback.raw_motor_id, 1U);
  EXPECT_EQ(feedback.raw_byte_6, 35U);
  EXPECT_EQ(feedback.raw_byte_7, 36U);
}

TEST(Dm4340MitWire, StatusNibbleDistinguishesEnabledFromUndervoltage) {
  EXPECT_TRUE(feedback_status_ok(0U));
  EXPECT_TRUE(feedback_status_ok(1U));
  EXPECT_TRUE(feedback_motor_enabled(1U));
  EXPECT_FALSE(feedback_motor_enabled(0U));
  EXPECT_FALSE(feedback_status_ok(2U));
  EXPECT_FALSE(feedback_status_ok(8U));
  EXPECT_FALSE(feedback_status_ok(9U));
}

TEST(Dm4340MitWire, RejectsForeignAndMalformedFrames) {
  MitFeedback feedback{};
  const auto original = feedback_frame();
  EXPECT_FALSE(decode_mit_feedback(original, 3U, 0x11U, 1U,
                                   kSdkDm4340Limits, feedback));
  EXPECT_FALSE(decode_mit_feedback(original, 2U, 0x12U, 1U,
                                   kSdkDm4340Limits, feedback));
  EXPECT_FALSE(decode_mit_feedback(original, 2U, 0x11U, 2U,
                                   kSdkDm4340Limits, feedback));
  auto frame = original;
  frame.payload_size = 7U;
  EXPECT_FALSE(decode_mit_feedback(frame, 2U, 0x11U, 1U,
                                   kSdkDm4340Limits, feedback));
  frame = original;
  frame.direction = FrameDirection::Tx;
  EXPECT_FALSE(decode_mit_feedback(frame, 2U, 0x11U, 1U,
                                   kSdkDm4340Limits, feedback));
  frame = original;
  frame.error_frame = true;
  EXPECT_FALSE(decode_mit_feedback(frame, 2U, 0x11U, 1U,
                                   kSdkDm4340Limits, feedback));
  frame = original;
  frame.remote_request = true;
  EXPECT_FALSE(decode_mit_feedback(frame, 2U, 0x11U, 1U,
                                   kSdkDm4340Limits, feedback));
  frame = original;
  frame.id.format = CanFrameFormat::Extended;
  EXPECT_FALSE(decode_mit_feedback(frame, 2U, 0x11U, 1U,
                                   kSdkDm4340Limits, feedback));
}

}  // namespace
}  // namespace mech::mech_protocol_damiao
