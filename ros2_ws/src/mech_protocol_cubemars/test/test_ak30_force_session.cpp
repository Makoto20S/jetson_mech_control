#include "mech_protocol_cubemars/ak30_force_session.hpp"

#include <gtest/gtest.h>

#include "ak30_test_fixtures.hpp"

namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::ProtocolProfile;
using mech::mech_protocol_cubemars::Ak30ForceControlSession;
using mech::mech_protocol_cubemars::ForceControlSubMode;
namespace fixtures = mech::mech_protocol_cubemars::testing;

TEST(Ak30SessionConfigure, AcceptsAFullyEvidencedConfiguration) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::Ok);
}

// The state the repository is actually in today. If this test ever passes,
// either evidence landed or the gate broke; both need a human to look.
TEST(Ak30SessionConfigure, RefusesMotor1sCurrentEvidenceForEverySubMode) {
  for (const auto sub_mode :
       {ForceControlSubMode::Torque, ForceControlSubMode::Velocity,
        ForceControlSubMode::Position}) {
    fixtures::RecordingTransport transport{
        fixtures::classic_extended_capabilities()};
    auto config = fixtures::valid_session_config();
    config.sub_mode = sub_mode;
    config.mapping = mech::mech_protocol_cubemars::Ak30Mapping{};  // defaults
    Ak30ForceControlSession session{transport, config};
    EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                                fixtures::classic_extended_capabilities()),
              AdapterResult::InvalidConfiguration);
  }
}

TEST(Ak30SessionConfigure, RejectsAnyProfileOtherThanForceControlExtended) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  for (const auto profile :
       {ProtocolProfile::Unknown, ProtocolProfile::LoopbackV1,
        ProtocolProfile::Ak30ServoExtended, ProtocolProfile::Hi12J1939,
        ProtocolProfile::Hi12Canopen}) {
    auto config = fixtures::valid_device_config();
    config.profile = profile;
    EXPECT_EQ(session.configure(config, fixtures::classic_extended_capabilities()),
              AdapterResult::InvalidConfiguration);
  }
}

// The superseded AK2.0 MIT enum required Standard here. Accepting a Standard
// frame format is the exact inversion ADR-013 removed, so it gets its own test.
TEST(Ak30SessionConfigure, RejectsStandardFormatAndNonClassicFrameTypes) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto standard = fixtures::valid_device_config();
  standard.frame_format = CanFrameFormat::Standard;
  EXPECT_EQ(session.configure(standard, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto flexible = fixtures::valid_device_config();
  flexible.frame_type = CanFrameType::FlexibleDataRate;
  EXPECT_EQ(session.configure(flexible, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

// The extended identifier reserves bits [7:0] for the drive ID, so 255 is the
// hard ceiling and a CubeMarsTool reading of "104" can only ever be decimal.
TEST(Ak30SessionConfigure, RejectsADriveIdThatDoesNotFitEightBits) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  auto config = fixtures::valid_session_config();
  config.drive_id = 256U;
  Ak30ForceControlSession session{transport, config};
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsIdentifiersThatDoNotMatchTheDriveId) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto missing_command = fixtures::valid_device_config();
  missing_command.command_id.reset();
  EXPECT_EQ(session.configure(missing_command,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto missing_feedback = fixtures::valid_device_config();
  missing_feedback.feedback_id.reset();
  EXPECT_EQ(session.configure(missing_feedback,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  // Control mode 9 instead of force control's 8.
  auto wrong_mode = fixtures::valid_device_config();
  wrong_mode.command_id = CanId::create(0x0968U, CanFrameFormat::Extended);
  EXPECT_EQ(session.configure(wrong_mode, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  // Feedback pointing at a different drive.
  auto wrong_drive = fixtures::valid_device_config();
  wrong_drive.feedback_id = CanId::create(0x2969U, CanFrameFormat::Extended);
  EXPECT_EQ(session.configure(wrong_drive, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsPayloadSizesThatAreNotEightBytes) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto short_command = fixtures::valid_device_config();
  short_command.command_payload_bytes = 4U;
  EXPECT_EQ(session.configure(short_command,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto long_feedback = fixtures::valid_device_config();
  long_feedback.feedback_payload_bytes = 16U;
  EXPECT_EQ(session.configure(long_feedback,
                              fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsAReadOnlyDeviceBecauseForceControlCommands) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  auto config = fixtures::valid_device_config();
  config.writable = false;
  EXPECT_EQ(session.configure(config, fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

// Reject unavailable capabilities rather than synthesizing them.
TEST(Ak30SessionConfigure, RejectsBackendsThatCannotCarryClassicExtendedFrames) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};

  auto no_extended = fixtures::classic_extended_capabilities();
  no_extended.supports_extended_frames = false;
  no_extended.supports_standard_frames = true;
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), no_extended),
            AdapterResult::InvalidConfiguration);

  auto no_classic = fixtures::classic_extended_capabilities();
  no_classic.supports_classic_can = false;
  no_classic.supports_can_fd = true;
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), no_classic),
            AdapterResult::InvalidConfiguration);

  auto too_small = fixtures::classic_extended_capabilities();
  too_small.max_payload_bytes = 4U;
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), too_small),
            AdapterResult::InvalidConfiguration);

  auto invalid = fixtures::classic_extended_capabilities();
  invalid.queue_capacity = 0U;
  ASSERT_FALSE(invalid.is_valid());
  EXPECT_EQ(session.configure(fixtures::valid_device_config(), invalid),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsAFirmwareIdOutsideTheAcceptedRange) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};

  auto below = fixtures::valid_session_config();
  below.firmware_id = 0x02FFU;
  Ak30ForceControlSession below_session{transport, below};
  EXPECT_EQ(below_session.configure(fixtures::valid_device_config(),
                                    fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto above = fixtures::valid_session_config();
  above.firmware_id = 0x0400U;
  Ak30ForceControlSession above_session{transport, above};
  EXPECT_EQ(above_session.configure(fixtures::valid_device_config(),
                                    fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto inverted = fixtures::valid_session_config();
  inverted.firmware_id_min = 0x0400U;
  inverted.firmware_id_max = 0x0300U;
  Ak30ForceControlSession inverted_session{transport, inverted};
  EXPECT_EQ(inverted_session.configure(fixtures::valid_device_config(),
                                       fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

// ADR-012 caps the whole watchdog at <=3 control cycles, which is <=6 ms at
// 500 Hz per 03_mvp_delivery_plan.md:215. The 2026-08-27 review found a
// shipped TTL 16x over that budget and the first fix made it 33x; nothing in
// the build catches a number that contradicts a planning document, so it is
// checked here.
TEST(Ak30SessionConfigure, RejectsWatchdogTimingsOutsideTheDocumentedBudget) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};

  auto too_long = fixtures::valid_session_config();
  too_long.command_hard_ttl_nanoseconds = 6000001;
  Ak30ForceControlSession too_long_session{transport, too_long};
  EXPECT_EQ(too_long_session.configure(fixtures::valid_device_config(),
                                       fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto inverted = fixtures::valid_session_config();
  inverted.command_ttl_nanoseconds = 5000000;
  inverted.command_hard_ttl_nanoseconds = 4000000;
  Ak30ForceControlSession inverted_session{transport, inverted};
  EXPECT_EQ(inverted_session.configure(fixtures::valid_device_config(),
                                       fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto zero = fixtures::valid_session_config();
  zero.command_ttl_nanoseconds = 0;
  Ak30ForceControlSession zero_session{transport, zero};
  EXPECT_EQ(zero_session.configure(fixtures::valid_device_config(),
                                   fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, RejectsGainsOutsideTheModelsDocumentedRange) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};

  auto high_kp = fixtures::valid_session_config();
  high_kp.sub_mode = ForceControlSubMode::Position;
  high_kp.gains.kp = 500.1;
  Ak30ForceControlSession kp_session{transport, high_kp};
  EXPECT_EQ(kp_session.configure(fixtures::valid_device_config(),
                                 fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);

  auto high_kd = fixtures::valid_session_config();
  high_kd.gains.kd = 5.1;
  Ak30ForceControlSession kd_session{transport, high_kd};
  EXPECT_EQ(kd_session.configure(fixtures::valid_device_config(),
                                 fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST(Ak30SessionConfigure, IsRepeatable) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::Ok);
  EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                              fixtures::classic_extended_capabilities()),
            AdapterResult::Ok);
}

}  // namespace
