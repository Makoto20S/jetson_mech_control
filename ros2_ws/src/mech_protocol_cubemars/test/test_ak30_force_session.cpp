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

// The default mapping is motor1's evidenced state (direction_sign verified on
// the bench 2026-09-03: positive command -> clockwise rotation -> feedback
// position increase), so torque and velocity configure on defaults. Position
// position now uses the owner-approved provisional 330.07 deg/output-shaft
// mapping. The dedicated position bring-up test can revise that mapping.
TEST(Ak30SessionConfigure, DefaultEvidenceConfiguresAllThreeSubModes) {
  const std::pair<ForceControlSubMode, AdapterResult> cases[] = {
      {ForceControlSubMode::Torque, AdapterResult::Ok},
      {ForceControlSubMode::Velocity, AdapterResult::Ok},
      {ForceControlSubMode::Position, AdapterResult::Ok}};
  for (const auto& [sub_mode, expected] : cases) {
    fixtures::RecordingTransport transport{
        fixtures::classic_extended_capabilities()};
    auto config = fixtures::valid_session_config();
    config.sub_mode = sub_mode;
    config.mapping = mech::mech_protocol_cubemars::Ak30Mapping{};  // defaults
    Ak30ForceControlSession session{transport, config};
    EXPECT_EQ(session.configure(fixtures::valid_device_config(),
                                fixtures::classic_extended_capabilities()),
              expected);
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

using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::DeviceState;
using mech::mech_control_core::SampleQuality;
using mech::mech_control_core::TransportResult;
using mech::mech_protocol_cubemars::CommandStage;

// Configures and activates, so runtime tests start from a known state.
class Ak30SessionRuntime : public ::testing::Test {
 protected:
  Ak30SessionRuntime()
      : transport_(fixtures::classic_extended_capabilities()),
        session_(transport_, fixtures::valid_session_config()) {}

  void SetUp() override {
    ASSERT_EQ(session_.configure(fixtures::valid_device_config(),
                                 fixtures::classic_extended_capabilities()),
              AdapterResult::Ok);
    ASSERT_EQ(session_.activate(), AdapterResult::Ok);
  }

  [[nodiscard]] static CanonicalDeviceCommand torque_command(double effort,
                                                             std::int64_t deadline) {
    CanonicalDeviceCommand command{};
    command.effort = effort;
    command.deadline = fixtures::at(deadline);
    return command;
  }

  fixtures::RecordingTransport transport_;
  Ak30ForceControlSession session_;
};

TEST_F(Ak30SessionRuntime, SendsOneFrameToTheBorrowedTransportPerCommand) {
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_EQ(transport_.sent().size(), 1U);
  EXPECT_EQ(transport_.sent().front().id.value, fixtures::kCommandId);
  EXPECT_EQ(transport_.sent().front().payload[7], 0x10U);
}

TEST(Ak30SessionLifecycle, RefusesToActivateBeforeConfigure) {
  fixtures::RecordingTransport transport{fixtures::classic_extended_capabilities()};
  Ak30ForceControlSession session{transport, fixtures::valid_session_config()};
  EXPECT_EQ(session.activate(), AdapterResult::InvalidConfiguration);
  EXPECT_EQ(session.submit(CanonicalDeviceCommand{}, fixtures::at(1000)),
            AdapterResult::InvalidConfiguration);
}

TEST_F(Ak30SessionRuntime, RefusesToReconfigureWhileActive) {
  EXPECT_EQ(session_.configure(fixtures::valid_device_config(),
                               fixtures::classic_extended_capabilities()),
            AdapterResult::InvalidConfiguration);
}

TEST_F(Ak30SessionRuntime, IsRepeatableAcrossDeactivateAndActivate) {
  session_.deactivate();
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::InvalidConfiguration);
  EXPECT_EQ(session_.activate(), AdapterResult::Ok);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
}

TEST_F(Ak30SessionRuntime, ReportsAnEmptySnapshotBeforeAnyFeedbackArrives) {
  const auto state = session_.snapshot(fixtures::at(1000));
  EXPECT_EQ(state.status.quality, SampleQuality::Unknown);
  EXPECT_FALSE(state.status.host_rx_time.has_value());
  EXPECT_FALSE(state.status.has_sample());
}

TEST_F(Ak30SessionRuntime, AcceptsACommandBeforeAnyFeedbackHasArrived) {
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
}

TEST_F(Ak30SessionRuntime, SequencesAcceptedFeedbackFrames) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.sequence, 1U);

  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(2000)),
                             fixtures::at(2000)),
            AdapterResult::Ok);
  EXPECT_EQ(session_.snapshot(fixtures::at(2000)).status.sequence, 2U);
}

TEST_F(Ak30SessionRuntime, IgnoresFramesThatAreNotThisDevicesFeedback) {
  auto foreign = fixtures::feedback_frame(0x00U, fixtures::at(1000));
  foreign.id = mech::mech_control_core::CanId::create(
                   0x2969U, CanFrameFormat::Extended)
                   .value();
  EXPECT_EQ(session_.process(foreign, fixtures::at(1000)),
            AdapterResult::InvalidCommand);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.sequence, 0U);
}

TEST_F(Ak30SessionRuntime, MarksTheSampleStaleOnceFeedbackExceedsItsTtl) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.quality,
            SampleQuality::Valid);
  // feedback_ttl is 6 ms.
  EXPECT_EQ(session_.snapshot(fixtures::at(6000999)).status.quality,
            SampleQuality::Valid);
  // The comparison is `age > feedback_ttl_nanoseconds`, so an age exactly at
  // the TTL (6,000,000 ns) is still Valid, and only the first nanosecond past
  // it (6,000,001 ns) becomes Stale.
  EXPECT_EQ(session_.snapshot(fixtures::at(6001000)).status.quality,
            SampleQuality::Valid);
  EXPECT_EQ(session_.snapshot(fixtures::at(6001001)).status.quality,
            SampleQuality::Stale);
  EXPECT_EQ(session_.snapshot(fixtures::at(7001000)).status.quality,
            SampleQuality::Stale);
}

// ADR-012's staged watchdog: follow, then freeze the last valid command, then
// an explicit error past the hard TTL. ttl 4 ms, hard_ttl 6 ms.
TEST_F(Ak30SessionRuntime, StagesTheCommandWatchdogFollowingHoldingExpired) {
  ASSERT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);

  EXPECT_EQ(session_.command_stage(fixtures::at(1000)), CommandStage::Following);
  EXPECT_EQ(session_.command_stage(fixtures::at(4000999)), CommandStage::Following);
  EXPECT_EQ(session_.command_stage(fixtures::at(4001000)), CommandStage::Holding);
  EXPECT_EQ(session_.command_stage(fixtures::at(6000999)), CommandStage::Holding);
  EXPECT_EQ(session_.command_stage(fixtures::at(6001000)), CommandStage::Expired);
}

TEST_F(Ak30SessionRuntime, ReportsExpiredBeforeAnyCommandHasBeenSubmitted) {
  EXPECT_EQ(session_.command_stage(fixtures::at(1000)), CommandStage::Expired);
}

// The failure this guards is a position command resolving to 0.0, which on a
// position interface is a commanded move to the zero position. The session
// emits nothing it was not given, so an expired watchdog produces silence.
TEST_F(Ak30SessionRuntime, NeverSynthesizesACommandWhenTheWatchdogExpires) {
  ASSERT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_EQ(transport_.sent().size(), 1U);

  ASSERT_EQ(session_.command_stage(fixtures::at(9000000)), CommandStage::Expired);
  (void)session_.snapshot(fixtures::at(9000000));
  (void)session_.command_stage(fixtures::at(9000000));

  EXPECT_EQ(transport_.sent().size(), 1U);
}

TEST_F(Ak30SessionRuntime, RejectsACommandWhoseDeadlineHasAlreadyPassed) {
  EXPECT_EQ(session_.submit(torque_command(2.0, 1000), fixtures::at(2000)),
            AdapterResult::InvalidCommand);
  EXPECT_TRUE(transport_.sent().empty());
}

TEST_F(Ak30SessionRuntime, RejectsAnUnrepresentableCommandWithoutSending) {
  EXPECT_EQ(session_.submit(torque_command(99.0, 10000000), fixtures::at(1000)),
            AdapterResult::InvalidCommand);
  EXPECT_TRUE(transport_.sent().empty());
}

// A transient backpressure result must not fault the bus. The RC shipped a
// defect where one WouldBlock permanently faulted it; the lease is retried
// instead.
TEST_F(Ak30SessionRuntime, TreatsTransientBackpressureAsRetryableNotFatal) {
  transport_.inject_send_result(TransportResult::WouldBlock);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::WouldBlock);
  EXPECT_FALSE(session_.fault_latched());

  transport_.inject_send_result(TransportResult::QueueFull);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(2000)),
            AdapterResult::WouldBlock);
  EXPECT_FALSE(session_.fault_latched());

  transport_.inject_send_result(TransportResult::Ok);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(3000)),
            AdapterResult::Ok);
  EXPECT_EQ(transport_.sent().size(), 1U);
}

TEST_F(Ak30SessionRuntime, SurfacesADisconnectedTransport) {
  transport_.inject_send_result(TransportResult::Disconnected);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(1000)),
            AdapterResult::Disconnected);
}

TEST_F(Ak30SessionRuntime, LatchesAFaultAndRefusesFurtherCommands) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x05U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_TRUE(session_.fault_latched());
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.device_state,
            DeviceState::Fault);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.raw_fault_code, 0x05U);
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(2000)),
            AdapterResult::Fault);
}

// Latched means latched: a subsequent clean frame is not a recovery signal,
// because the condition that caused the fault is not observable from one frame.
TEST_F(Ak30SessionRuntime, DoesNotClearALatchedFaultOnTheNextCleanFrame) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x02U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_TRUE(session_.fault_latched());

  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(2000)),
                             fixtures::at(2000)),
            AdapterResult::Ok);
  EXPECT_TRUE(session_.fault_latched());
}

TEST_F(Ak30SessionRuntime, ClearsALatchedFaultOnlyThroughTheLifecycle) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x07U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_TRUE(session_.fault_latched());

  session_.deactivate();
  ASSERT_EQ(session_.activate(), AdapterResult::Ok);
  EXPECT_FALSE(session_.fault_latched());
  EXPECT_EQ(session_.submit(torque_command(2.0, 10000000), fixtures::at(3000)),
            AdapterResult::Ok);
}

// A pre-deactivation sample must never be reported as a current measurement
// after reactivation: deactivate-then-activate is the fault-recovery path,
// and re-presenting stale, pre-fault state as live would mask exactly the
// condition recovery is meant to reveal.
TEST_F(Ak30SessionRuntime, DoesNotReportAPreDeactivationSampleAfterReactivation) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x00U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  ASSERT_TRUE(session_.snapshot(fixtures::at(1000)).status.has_sample());

  session_.deactivate();
  ASSERT_EQ(session_.activate(), AdapterResult::Ok);

  const auto state = session_.snapshot(fixtures::at(2000));
  EXPECT_EQ(state.status.quality, SampleQuality::Unknown);
  EXPECT_FALSE(state.status.has_sample());
  EXPECT_FALSE(state.status.host_rx_time.has_value());
  EXPECT_EQ(state.status.sequence, 0U);
}

// 0x77 is the disable-succeeded acknowledgement. Latching a fault on it would
// turn a successful, deliberate disable into an error state requiring recovery.
TEST_F(Ak30SessionRuntime, DoesNotLatchAFaultOnTheDisableAcknowledgement) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x77U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_FALSE(session_.fault_latched());
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.raw_fault_code, 0x77U);
}

TEST_F(Ak30SessionRuntime, DoesNotLatchAFaultOnAnUnknownStatusByte) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x42U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_FALSE(session_.fault_latched());
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.quality,
            SampleQuality::Degraded);
}

// Precedence rule at the Stale assignment in snapshot(): once a sample is
// stale, staleness wins over a Degraded classification, because an old
// sample's classification is itself no longer trustworthy. The raw status
// byte is unaffected either way.
TEST_F(Ak30SessionRuntime, StalenessOutranksADegradedClassification) {
  ASSERT_EQ(session_.process(fixtures::feedback_frame(0x42U, fixtures::at(1000)),
                             fixtures::at(1000)),
            AdapterResult::Ok);
  EXPECT_EQ(session_.snapshot(fixtures::at(1000)).status.quality,
            SampleQuality::Degraded);

  // feedback_ttl is 6 ms; the first nanosecond past it becomes Stale.
  const auto state = session_.snapshot(fixtures::at(6001001));
  EXPECT_EQ(state.status.quality, SampleQuality::Stale);
  EXPECT_EQ(state.status.raw_fault_code, 0x42U);
}

}  // namespace
