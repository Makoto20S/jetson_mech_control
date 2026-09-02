#include "mech_protocol_cubemars/ak30_force_codec.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>

namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::CanonicalDeviceState;
using mech::mech_control_core::DeviceState;
using mech::mech_control_core::FrameDirection;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::SampleQuality;
using mech::mech_protocol_cubemars::Ak30ForceControlCodec;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::ForceControlGains;
using mech::mech_protocol_cubemars::ForceControlSubMode;

constexpr std::uint8_t kDriveId = 104U;  // motor1, decimal 104 = 0x68
constexpr std::uint16_t kLogicalBus = 0U;

[[nodiscard]] Ak30Mapping fully_verified() {
  Ak30Mapping mapping{};
  mapping.pole_pairs = {14.0, true};
  mapping.gear_ratio = {8.0, true};
  mapping.zero_offset_rad = {0.0, true};
  mapping.direction_sign = {1.0, true};
  mapping.torque_constant_nm_per_a = {0.7382, true};
  mapping.position_source_known = true;
  mapping.position_is_output_shaft = true;
  return mapping;
}

[[nodiscard]] Ak30ForceControlCodec torque_codec() {
  return Ak30ForceControlCodec{kDriveId, ForceControlSubMode::Torque,
                               fully_verified(), ForceControlGains{}};
}

[[nodiscard]] MonotonicTime at(std::int64_t nanoseconds) {
  return MonotonicTime::from_nanoseconds(nanoseconds).value();
}

// Builds a well-formed 0x29 feedback frame, which individual tests then break
// one field at a time.
[[nodiscard]] RawCanFrame feedback_frame() {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03U;  // position 900 -> 90.0 deg
  payload[1] = 0x84U;
  payload[2] = 0x03U;  // speed 1000 -> 10000 ERPM
  payload[3] = 0xE8U;
  payload[4] = 0x00U;  // Iq 200 -> 2.0 A
  payload[5] = 0xC8U;
  payload[6] = 0x28U;  // 40 deg C
  payload[7] = 0x00U;  // no fault
  return RawCanFrame::create(kLogicalBus,
                             CanId::create(0x2968U, CanFrameFormat::Extended).value(),
                             CanFrameType::Classic, FrameDirection::Rx, 8U,
                             payload, at(1000))
      .value();
}

TEST(Ak30Codec, DeclaresTheForceControlExtendedProfile) {
  EXPECT_EQ(torque_codec().profile(),
            mech::mech_control_core::ProtocolProfile::Ak30ForceControlExtended);
}

TEST(Ak30Codec, EncodesToAnExtendedClassicFrameAtTheCommandIdentifier) {
  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  RawCanFrame frame{};
  ASSERT_EQ(torque_codec().encode(command, kLogicalBus, at(500), frame),
            AdapterResult::Ok);

  EXPECT_EQ(frame.id.value, 0x0868U);
  EXPECT_EQ(frame.id.format, CanFrameFormat::Extended);
  EXPECT_EQ(frame.type, CanFrameType::Classic);
  EXPECT_EQ(frame.direction, FrameDirection::Tx);
  EXPECT_EQ(frame.payload_size, 8U);
  EXPECT_FALSE(frame.bitrate_switch);
  EXPECT_FALSE(frame.remote_request);
  EXPECT_EQ(frame.logical_bus, kLogicalBus);
  // Same bytes as the wire-layer AKE60-8 torque golden vector.
  EXPECT_EQ(frame.payload[6], 0xF9U);
  EXPECT_EQ(frame.payload[7], 0x10U);
}

TEST(Ak30Codec, RejectsCommandsTheWireLayerCannotRepresent) {
  CanonicalDeviceCommand command{};
  command.effort = 15.1;  // beyond AKE60-8's +/-15 N.m
  RawCanFrame frame{};
  EXPECT_EQ(torque_codec().encode(command, kLogicalBus, at(500), frame),
            AdapterResult::InvalidCommand);

  command.effort = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(torque_codec().encode(command, kLogicalBus, at(500), frame),
            AdapterResult::InvalidCommand);
}

TEST(Ak30Codec, DecodesAWellFormedFeedbackFrame) {
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(feedback_frame(), state), AdapterResult::Ok);

  EXPECT_NEAR(state.effort, 1.4764, 1e-12);
  EXPECT_EQ(state.status.quality, SampleQuality::Valid);
  EXPECT_EQ(state.status.raw_fault_code, 0U);
  ASSERT_TRUE(state.status.host_rx_time.has_value());
  EXPECT_EQ(state.status.host_rx_time.value(), at(1000));
}

TEST(Ak30Codec, RejectsFramesThatAreNotThisDevicesFeedback) {
  CanonicalDeviceState state{};

  // Wrong function ID: the servo start frame 0x2C, not 0x29.
  auto wrong_function = feedback_frame();
  wrong_function.id = CanId::create(0x2C68U, CanFrameFormat::Extended).value();
  EXPECT_EQ(torque_codec().decode(wrong_function, state),
            AdapterResult::InvalidCommand);

  // Right function, different drive.
  auto other_drive = feedback_frame();
  other_drive.id = CanId::create(0x2969U, CanFrameFormat::Extended).value();
  EXPECT_EQ(torque_codec().decode(other_drive, state),
            AdapterResult::InvalidCommand);

  // A standard-format frame carrying the same numeric value. This is the AK2.0
  // MIT shape; accepting it would be the exact defect ADR-013 removed.
  auto standard = feedback_frame();
  standard.id = CanId::create(0x268U, CanFrameFormat::Standard).value();
  EXPECT_EQ(torque_codec().decode(standard, state),
            AdapterResult::InvalidCommand);
}

TEST(Ak30Codec, RejectsMalformedFeedbackFrames) {
  CanonicalDeviceState state{};

  auto short_dlc = feedback_frame();
  short_dlc.payload_size = 7U;
  EXPECT_EQ(torque_codec().decode(short_dlc, state), AdapterResult::InvalidCommand);

  auto flexible = feedback_frame();
  flexible.type = CanFrameType::FlexibleDataRate;
  EXPECT_EQ(torque_codec().decode(flexible, state), AdapterResult::InvalidCommand);

  auto brs = feedback_frame();
  brs.bitrate_switch = true;
  EXPECT_EQ(torque_codec().decode(brs, state), AdapterResult::InvalidCommand);

  auto remote = feedback_frame();
  remote.remote_request = true;
  EXPECT_EQ(torque_codec().decode(remote, state), AdapterResult::InvalidCommand);

  auto errored = feedback_frame();
  errored.error_frame = true;
  EXPECT_EQ(torque_codec().decode(errored, state), AdapterResult::InvalidCommand);
}

TEST(Ak30Codec, SurfacesAFaultCodeWithoutOverwritingTheRawByte) {
  auto frame = feedback_frame();
  frame.payload[7] = 0x05U;  // encoder fault
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  EXPECT_EQ(state.status.raw_fault_code, 0x05U);
  EXPECT_EQ(state.status.device_state, DeviceState::Fault);
  EXPECT_EQ(state.status.quality, SampleQuality::Valid);
}

// 0x77 is the disable-succeeded acknowledgement, not fault code 0x77. Treating
// it as a fault would misreport the safety path as a failure.
TEST(Ak30Codec, DoesNotTreatTheDisableAcknowledgementAsAFault) {
  auto frame = feedback_frame();
  frame.payload[7] = 0x77U;
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  EXPECT_EQ(state.status.raw_fault_code, 0x77U);
  EXPECT_NE(state.status.device_state, DeviceState::Fault);
}

// An uninterpretable status byte is a genuine per-sample condition, so the
// sample is Degraded rather than silently mapped onto "no fault".
TEST(Ak30Codec, MarksAnUnknownStatusByteDegradedRatherThanGuessing) {
  auto frame = feedback_frame();
  frame.payload[7] = 0x42U;
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  EXPECT_EQ(state.status.raw_fault_code, 0x42U);
  EXPECT_EQ(state.status.quality, SampleQuality::Degraded);
  EXPECT_NE(state.status.device_state, DeviceState::Fault);
}

TEST(Ak30Codec, PreservesASourceTimestampWhenTheBackendSuppliedOne) {
  auto frame = feedback_frame();
  frame.source_timestamp = mech::mech_control_core::SourceTimestamp{
      mech::mech_control_core::SourceClockDomain::Transport, 4242U};
  CanonicalDeviceState state{};
  ASSERT_EQ(torque_codec().decode(frame, state), AdapterResult::Ok);

  ASSERT_TRUE(state.status.source_timestamp.has_value());
  EXPECT_EQ(state.status.source_timestamp.value().ticks, 4242U);
}

}  // namespace
