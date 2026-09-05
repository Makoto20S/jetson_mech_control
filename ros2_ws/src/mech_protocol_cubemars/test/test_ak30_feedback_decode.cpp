#include "mech_protocol_cubemars/ak30_force_wire.hpp"

#include <cstdint>

#include <gtest/gtest.h>

namespace {

using mech::mech_protocol_cubemars::classify_status;
using mech::mech_protocol_cubemars::decode_feedback;
using mech::mech_protocol_cubemars::ForceControlFeedback;
using mech::mech_protocol_cubemars::ForceControlPayload;
using mech::mech_protocol_cubemars::StatusMeaning;

// L07 section 4.3.1: position int16 x 0.1 deg, speed int16 x 10 ERPM,
// Iq int16 x 0.01 A, temperature Data[6] int8 deg C, Data[7] status.
TEST(Ak30Feedback, DecodesBigEndianFieldsWithDocumentedScaling) {
  // position 0x0BB8 = 3000 -> 300.0 deg
  // speed    0x03E8 = 1000 -> 10000 ERPM
  // Iq       0x0064 = 100  -> 1.0 A
  // temp     0x2D   = 45   -> 45 deg C
  const ForceControlPayload payload{0x0B, 0xB8, 0x03, 0xE8,
                                    0x00, 0x64, 0x2D, 0x00};
  ForceControlFeedback feedback{};
  decode_feedback(payload, feedback);

  EXPECT_DOUBLE_EQ(feedback.position_deg, 300.0);
  EXPECT_DOUBLE_EQ(feedback.electrical_speed_erpm, 10000.0);
  EXPECT_DOUBLE_EQ(feedback.current_iq_a, 1.0);
  EXPECT_DOUBLE_EQ(feedback.board_temperature_c, 45.0);
  EXPECT_EQ(feedback.raw_status, 0x00U);
}

TEST(Ak30Feedback, DecodesNegativeSignedFields) {
  // position 0xF448 = -3000 -> -300.0 deg
  // speed    0xFC18 = -1000 -> -10000 ERPM
  // Iq       0xFF9C = -100  -> -1.0 A
  // temp     0xEC   = -20   -> -20 deg C, the documented minimum
  const ForceControlPayload payload{0xF4, 0x48, 0xFC, 0x18,
                                    0xFF, 0x9C, 0xEC, 0x00};
  ForceControlFeedback feedback{};
  decode_feedback(payload, feedback);

  EXPECT_DOUBLE_EQ(feedback.position_deg, -300.0);
  EXPECT_DOUBLE_EQ(feedback.electrical_speed_erpm, -10000.0);
  EXPECT_DOUBLE_EQ(feedback.current_iq_a, -1.0);
  EXPECT_DOUBLE_EQ(feedback.board_temperature_c, -20.0);
}

TEST(Ak30Feedback, DecodesDocumentedRangeExtremes) {
  // The manual states position -32000..32000 maps to -3200..3200 degrees,
  // speed -32000..32000 maps to -320000..320000 ERPM, and Iq -6000..6000
  // maps to -60..60 A.
  ForceControlFeedback feedback{};
  decode_feedback(ForceControlPayload{0x7D, 0x00, 0x7D, 0x00,
                                      0x17, 0x70, 0x7F, 0x00},
                  feedback);
  EXPECT_DOUBLE_EQ(feedback.position_deg, 3200.0);
  EXPECT_DOUBLE_EQ(feedback.electrical_speed_erpm, 320000.0);
  EXPECT_DOUBLE_EQ(feedback.current_iq_a, 60.0);
  EXPECT_DOUBLE_EQ(feedback.board_temperature_c, 127.0);
}

// Data[7] carries two disjoint meanings. Fault codes are 0-7. 0x77 is the
// disable-succeeded acknowledgement returned once after control mode 15
// (L07 section 4.1.8). Decoding 0x77 as a fault would misreport the safety
// path as a failure. Anything else is unknown and must surface as unknown
// rather than being silently mapped onto a fault.
TEST(Ak30Feedback, ClassifiesTheStatusByteIntoItsThreeDisjointMeanings) {
  EXPECT_EQ(classify_status(0x00U), StatusMeaning::NoFault);
  for (std::uint8_t code = 1U; code <= 7U; ++code) {
    EXPECT_EQ(classify_status(code), StatusMeaning::Fault)
        << "fault code " << static_cast<int>(code);
  }
  EXPECT_EQ(classify_status(0x77U), StatusMeaning::DisableAcknowledged);
  EXPECT_EQ(classify_status(0x08U), StatusMeaning::Unknown);
  EXPECT_EQ(classify_status(0x76U), StatusMeaning::Unknown);
  EXPECT_EQ(classify_status(0x78U), StatusMeaning::Unknown);
  EXPECT_EQ(classify_status(0xFFU), StatusMeaning::Unknown);
}

TEST(Ak30Feedback, PreservesTheRawStatusByteAlongsideItsClassification) {
  ForceControlFeedback feedback{};
  decode_feedback(ForceControlPayload{0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x77},
                  feedback);
  EXPECT_EQ(feedback.raw_status, 0x77U);
  EXPECT_EQ(classify_status(feedback.raw_status),
            StatusMeaning::DisableAcknowledged);
}

}  // namespace
