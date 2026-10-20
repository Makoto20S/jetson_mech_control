#include "mech_protocol_cubemars/ak30_servo_wire.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace {
using namespace mech::mech_protocol_cubemars;
using namespace mech::mech_control_core;

RawCanFrame frame(std::uint32_t id, std::array<std::uint8_t, 8> bytes) {
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  for (std::size_t i = 0; i < bytes.size(); ++i) payload[i] = bytes[i];
  return RawCanFrame::create(0, *CanId::create(id, CanFrameFormat::Extended),
                             CanFrameType::Classic, FrameDirection::Rx, 8,
                             payload, MonotonicTime{}).value();
}

void expect_same_frame(const RawCanFrame& actual, const RawCanFrame& expected) {
  EXPECT_EQ(actual.logical_bus, expected.logical_bus);
  EXPECT_EQ(actual.id.value, expected.id.value);
  EXPECT_EQ(actual.id.format, expected.id.format);
  EXPECT_EQ(actual.type, expected.type);
  EXPECT_EQ(actual.direction, expected.direction);
  EXPECT_EQ(actual.payload_size, expected.payload_size);
  EXPECT_EQ(actual.payload, expected.payload);
  EXPECT_EQ(actual.host_arrival.nanoseconds(), expected.host_arrival.nanoseconds());
  EXPECT_EQ(actual.source_timestamp.has_value(), expected.source_timestamp.has_value());
  if (actual.source_timestamp && expected.source_timestamp) {
    EXPECT_EQ(actual.source_timestamp->domain, expected.source_timestamp->domain);
    EXPECT_EQ(actual.source_timestamp->ticks, expected.source_timestamp->ticks);
  }
  EXPECT_EQ(actual.error_frame, expected.error_frame);
  EXPECT_EQ(actual.bitrate_switch, expected.bitrate_switch);
  EXPECT_EQ(actual.remote_request, expected.remote_request);
}

void expect_same_feedback(const ServoFeedback& actual, const ServoFeedback& expected) {
  EXPECT_EQ(actual.position_deg, expected.position_deg);
  EXPECT_EQ(actual.electrical_speed_erpm, expected.electrical_speed_erpm);
  EXPECT_EQ(actual.current_iq_a, expected.current_iq_a);
  EXPECT_EQ(actual.board_temperature_c, expected.board_temperature_c);
  EXPECT_EQ(actual.raw_status, expected.raw_status);
  EXPECT_EQ(actual.status, expected.status);
}

TEST(Ak30ServoWire, EncodesGoldenFramesFor104And105) {
  RawCanFrame out{};
  ASSERT_TRUE(encode_servo_position_speed(104, {12.5, 1230, 450}, 0,
                                          MonotonicTime{}, out));
  EXPECT_EQ(out.id.value, 0x668U);
  EXPECT_EQ(out.id.format, CanFrameFormat::Extended);
  EXPECT_EQ(out.type, CanFrameType::Classic);
  EXPECT_EQ(out.direction, FrameDirection::Tx);
  EXPECT_EQ(out.payload_size, 8U);
  EXPECT_EQ((std::array<std::uint8_t, 8>{out.payload[0],out.payload[1],out.payload[2],out.payload[3],out.payload[4],out.payload[5],out.payload[6],out.payload[7]}),
            (std::array<std::uint8_t, 8>{0x00,0x01,0xE8,0x48,0x00,0x7B,0x00,0x2D}));
  ASSERT_TRUE(encode_servo_position_speed(105, {-1.23459, 19.9, 29.9}, 0,
                                          MonotonicTime{}, out));
  EXPECT_EQ(out.id.value, 0x669U);
  EXPECT_EQ((std::array<std::uint8_t, 8>{out.payload[0],out.payload[1],out.payload[2],out.payload[3],out.payload[4],out.payload[5],out.payload[6],out.payload[7]}),
            (std::array<std::uint8_t, 8>{0xFF,0xFF,0xCF,0xC7,0x00,0x01,0x00,0x02}));
}

TEST(Ak30ServoWire, RejectsInvalidNumericInputsWithoutChangingOutput) {
  RawCanFrame out = frame(0x2968, {1,2,3,4,5,6,7,8});
  const auto before = out;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const ServoPositionSpeedCommand command : {
           ServoPositionSpeedCommand{nan, 10, 10}, {inf, 10, 10},
           {36000.0001, 10, 10}, {-36000.0001, 10, 10},
           {214748.3648, 10, 10}, {-214748.3649, 10, 10},
           {0, 0, 10}, {0, -10, 10}, {0, 9.99, 10},
           {0, inf, 10}, {0, 327680, 10},
           {0, 10, 0}, {0, 10, -10}, {0, 10, 9.99},
           {0, 10, nan}, {0, 10, 327680}}) {
    EXPECT_FALSE(encode_servo_position_speed(104, command, 0,
                                             MonotonicTime{}, out));
    expect_same_frame(out, before);
  }
  for (int id : {-1, 256, 1000}) {
    EXPECT_FALSE(encode_servo_position_speed(id, {0,10,10}, 0,
                                             MonotonicTime{}, out));
    expect_same_frame(out, before);
  }
}

TEST(Ak30ServoWire, EncodesNumericBoundariesAndSeparateDisable) {
  RawCanFrame out{};
  ASSERT_TRUE(encode_servo_position_speed(104, {36000, 327670, 327670}, 2,
                                          MonotonicTime{}, out));
  EXPECT_EQ((std::array<std::uint8_t, 8>{out.payload[0],out.payload[1],out.payload[2],out.payload[3],out.payload[4],out.payload[5],out.payload[6],out.payload[7]}),
            (std::array<std::uint8_t, 8>{0x15,0x75,0x2A,0x00,0x7F,0xFF,0x7F,0xFF}));
  ASSERT_TRUE(encode_servo_position_speed(104, {-36000, 10, 10}, 2,
                                          MonotonicTime{}, out));
  EXPECT_EQ((std::array<std::uint8_t, 4>{out.payload[0],out.payload[1],out.payload[2],out.payload[3]}),
            (std::array<std::uint8_t, 4>{0xEA,0x8A,0xD6,0x00}));
  ASSERT_TRUE(encode_servo_disable(105, 2, MonotonicTime{}, out));
  EXPECT_EQ(out.id.value, 0xF69U);
  EXPECT_EQ(out.payload_size, 0U);
  EXPECT_EQ(out.logical_bus, 2U);
  const auto before = out;
  EXPECT_FALSE(encode_servo_disable(256, 2, MonotonicTime{}, out));
  expect_same_frame(out, before);
}

TEST(Ak30ServoWire, DecodesFeedbackStatusAndSignedFields) {
  ServoFeedback out{};
  auto input = frame(0x2968, {0xFF,0x85,0x00,0x7B,0xFE,0x38,0xD8,0x00});
  ASSERT_TRUE(decode_servo_feedback(104, input, out));
  EXPECT_NEAR(out.position_deg, -12.3, 1e-12);
  EXPECT_EQ(out.electrical_speed_erpm, 1230);
  EXPECT_NEAR(out.current_iq_a, -4.56, 1e-12);
  EXPECT_EQ(out.board_temperature_c, -40);
  EXPECT_EQ(out.raw_status, 0U);
  EXPECT_EQ(out.status, ServoFeedbackStatus::Normal);
  EXPECT_TRUE(out.usable_position_sample());
  input.payload[7] = 7;
  ASSERT_TRUE(decode_servo_feedback(104, input, out));
  EXPECT_EQ(out.status, ServoFeedbackStatus::KnownFault);
  input.payload[7] = 0x77;
  ASSERT_TRUE(decode_servo_feedback(104, input, out));
  EXPECT_EQ(out.status, ServoFeedbackStatus::DisableAcknowledged);
  EXPECT_FALSE(out.usable_position_sample());
  input.payload[7] = 0x80;
  ASSERT_TRUE(decode_servo_feedback(104, input, out));
  EXPECT_EQ(out.status, ServoFeedbackStatus::Unknown);
  EXPECT_EQ(out.raw_status, 0x80U);
}

TEST(Ak30ServoWire, RejectsWrongShapeWithoutChangingFeedback) {
  ServoFeedback out{3,4,5,6,7,ServoFeedbackStatus::KnownFault};
  const auto before = out;
  const auto valid = frame(0x2968, {0,1,0,2,0,3,4,0});
  auto check = [&](RawCanFrame bad) {
    EXPECT_FALSE(decode_servo_feedback(104, bad, out));
    expect_same_feedback(out, before);
  };
  auto bad = valid; bad.id.value = 0x2969; check(bad);
  bad = valid; bad.id.format = CanFrameFormat::Standard; check(bad);
  bad = valid; bad.type = CanFrameType::FlexibleDataRate; check(bad);
  bad = valid; bad.direction = FrameDirection::Tx; check(bad);
  bad = valid; bad.remote_request = true; check(bad);
  bad = valid; bad.error_frame = true; check(bad);
  bad = valid; bad.payload_size = 7; check(bad);
  bad = valid; bad.payload_size = 9; check(bad);
  EXPECT_FALSE(decode_servo_feedback(256, valid, out));
  expect_same_feedback(out, before);
}
}  // namespace
