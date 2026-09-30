#include "mech_bringup/servo_feedback_reader.hpp"
#include <gtest/gtest.h>
TEST(ServoFeedbackReader, MissingSamplesAreExplicitNullAndInvalid) {
  const auto line = mech::mech_bringup::format_servo_feedback(123, 0, {104, 105}, {std::nullopt, std::nullopt});
  EXPECT_NE(line.find("\"valid\":false"), std::string::npos);
  EXPECT_NE(line.find("\"position_rad\":null"), std::string::npos);
  EXPECT_NE(line.find("\"availability\":\"unknown\""), std::string::npos);
}
TEST(ServoFeedbackReader, FreshSamplesRetainWireAndMappedMeasurements) {
  mech::mech_protocol_cubemars::ServoPositionSnapshot sample;
  sample.position_rad = 1.2;
  sample.feedback_position_deg = 30.0;
  sample.temperature_c = 43.0;
  sample.availability = mech::mech_protocol_cubemars::ServoPositionAvailability::Fresh;
  const auto line = mech::mech_bringup::format_servo_feedback(123, 0, {104}, {sample});
  EXPECT_NE(line.find("\"valid\":true"), std::string::npos);
  EXPECT_NE(line.find("\"temperature_c\":43"), std::string::npos);
  EXPECT_NE(line.find("\"feedback_position_deg\":30"), std::string::npos);
}
