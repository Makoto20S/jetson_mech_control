#include "mech_protocol_damiao/dm4340_mit_session.hpp"

#include <array>
#include <gtest/gtest.h>

namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::MonotonicTime;
using mech::mech_protocol_damiao::Dm4340MitSession;
using mech::mech_protocol_damiao::Dm4340MitSessionConfig;
using mech::mech_protocol_damiao::encode_mit;
using mech::mech_protocol_damiao::kSdkDm4340Limits;

Dm4340MitSessionConfig fixture() {
  Dm4340MitSessionConfig c{};
  c.profile = mech::mech_control_core::ProtocolProfile::Dm4340MitStandard;
  c.logical_bus = 2U;
  c.command_can_id = 1U;
  c.feedback_can_id = 0x11U;
  c.motor_id = 1U;
  c.firmware_id = 1U;
  c.firmware_min = 1U;
  c.firmware_max = 1U;
  c.firmware_evidence_declared = true;
  c.limits = kSdkDm4340Limits;
  c.limits_evidence_declared = true;
  c.joint = {1.0, 0.0, true};
  c.position_min_rad = -1.0;
  c.position_max_rad = 1.0;
  c.max_target_error_rad = 0.2;
  c.kp = 1.0;
  c.kd = 0.1;
  c.command_ttl_ns = 4000000;
  c.feedback_ttl_ns = 6000000;
  return c;
}

mech::mech_control_core::RawCanFrame feedback(std::uint8_t status,
                                                std::int64_t time_ns) {
  std::array<std::uint8_t, mech::mech_control_core::kMaxCanPayloadBytes> bytes{};
  bytes[0] = static_cast<std::uint8_t>((status << 4U) | 1U);
  bytes[1] = 0x7FU;
  bytes[2] = 0xFFU;
  bytes[3] = 0x7FU;
  bytes[4] = 0xF7U;
  bytes[5] = 0xFFU;
  return *mech::mech_control_core::RawCanFrame::create(
      2U, *mech::mech_control_core::CanId::create(
                0U, mech::mech_control_core::CanFrameFormat::Standard),
      mech::mech_control_core::CanFrameType::Classic,
      mech::mech_control_core::FrameDirection::Rx, 8U, bytes,
      *MonotonicTime::from_nanoseconds(time_ns));
}

TEST(Dm4340Session, MissingEvidenceAndIdentifiersFailClosed) {
  Dm4340MitSession session;
  auto c = fixture();
  c.firmware_evidence_declared = false;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c = fixture();
  c.limits_evidence_declared = false;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c = fixture();
  c.joint.evidence_declared = false;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c = fixture();
  c.command_can_id = c.feedback_can_id;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c = fixture();
  c.profile = mech::mech_control_core::ProtocolProfile::Ak30ServoExtended;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c = fixture();
  c.command_ttl_ns = 0;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c = fixture();
  c.max_command_velocity_rad_s = 1.5;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
}

TEST(Dm4340Session, ActivationCannotProduceACommand) {
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(fixture()), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
  mech::mech_control_core::RawCanFrame frame{};
  auto deadline = *MonotonicTime::from_nanoseconds(4000000);
  EXPECT_EQ(session.prepare_position({0.0, 0.0, 0.0, 1U, deadline},
                                     MonotonicTime{}, frame),
            AdapterResult::Stale);
  EXPECT_FALSE(session.pending_target_still_authorized(
      frame, 1U, deadline, MonotonicTime{}));
}

TEST(Dm4340Session, EnabledFeedbackIsFreshButUndervoltageLatchesFault) {
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(fixture()), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);

  std::array<std::uint8_t, mech::mech_control_core::kMaxCanPayloadBytes> bytes{};
  bytes[0] = 0x11U;  // Status 1, motor ID 1.
  bytes[1] = 0x7FU;
  bytes[2] = 0xFFU;
  bytes[3] = 0x7FU;
  bytes[4] = 0xF7U;
  bytes[5] = 0xFFU;
  const auto t1 = *MonotonicTime::from_nanoseconds(1);
  auto frame = *mech::mech_control_core::RawCanFrame::create(
      2U, *mech::mech_control_core::CanId::create(
                0x11U, mech::mech_control_core::CanFrameFormat::Standard),
      mech::mech_control_core::CanFrameType::Classic,
      mech::mech_control_core::FrameDirection::Rx, 8U, bytes, t1);

  EXPECT_EQ(session.accept_feedback(frame, t1), AdapterResult::Ok);
  EXPECT_EQ(session.snapshot(t1).availability,
            mech::mech_protocol_damiao::DmFeedbackAvailability::Fresh);
  EXPECT_EQ(session.snapshot(t1).raw_fault, 1U);

  frame.payload[0] = 0x91U;  // Status 9 is undervoltage.
  frame.host_arrival = *MonotonicTime::from_nanoseconds(2);
  EXPECT_EQ(session.accept_feedback(frame, frame.host_arrival), AdapterResult::Fault);
  EXPECT_TRUE(session.fault_latched());
}

TEST(Dm4340Session, AcceptsEqualBatchArrivalButRejectsTimeRegression) {
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(fixture()), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
  const auto arrival = *MonotonicTime::from_nanoseconds(1000000);
  auto frame = feedback(1U, 1000000);
  frame.id = *mech::mech_control_core::CanId::create(
      0x11U, mech::mech_control_core::CanFrameFormat::Standard);
  EXPECT_EQ(session.accept_feedback(frame, arrival), AdapterResult::Ok);
  EXPECT_EQ(session.accept_feedback(frame, arrival), AdapterResult::Ok);
  EXPECT_EQ(session.snapshot(arrival).sequence, 2U);
  frame.host_arrival = *MonotonicTime::from_nanoseconds(999999);
  EXPECT_EQ(session.accept_feedback(frame, arrival), AdapterResult::Stale);
}

TEST(Dm4340Session, VelocityCommandUsesRealZeroFeedbackIdAndRequiresEnable) {
  auto c = fixture();
  c.feedback_can_id = 0U;
  c.kp = 0.0;
  c.kd = 1.0;
  c.max_command_velocity_rad_s = 1.5;
  c.max_feedback_velocity_rad_s = 2.0;
  c.max_feedback_torque_nm = 5.0;
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(c), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
  auto frame = feedback(0U, 1);
  ASSERT_EQ(session.accept_feedback(frame, frame.host_arrival), AdapterResult::Ok);
  mech::mech_control_core::RawCanFrame command_frame{};
  const auto now = *MonotonicTime::from_nanoseconds(2);
  const auto deadline = *MonotonicTime::from_nanoseconds(4000002);
  EXPECT_EQ(session.prepare_velocity({0.0, 0.8, 0.0, 1U, deadline},
                                     now, command_frame), AdapterResult::InvalidCommand);
  frame = feedback(1U, 3);
  ASSERT_EQ(session.accept_feedback(frame, frame.host_arrival), AdapterResult::Ok);
  const auto enabled_now = *MonotonicTime::from_nanoseconds(4);
  const auto enabled_deadline = *MonotonicTime::from_nanoseconds(4000004);
  EXPECT_EQ(session.prepare_velocity({0.0, 0.8, 0.0, 1U, enabled_deadline},
                                     enabled_now, command_frame), AdapterResult::Ok);
  EXPECT_EQ(command_frame.id.value, 1U);
  EXPECT_TRUE(session.pending_target_still_authorized(
      command_frame, 1U, enabled_deadline, enabled_now));
  EXPECT_FALSE(session.pending_target_still_authorized(
      command_frame, 1U, enabled_deadline, enabled_deadline));
  EXPECT_EQ(session.prepare_velocity({0.0, 1.6, 0.0, 2U,
                                      *MonotonicTime::from_nanoseconds(4000005)},
                                     *MonotonicTime::from_nanoseconds(5),
                                     command_frame), AdapterResult::InvalidCommand);
}

TEST(Dm4340Session, VelocityFeedbackLimitLatchesFault) {
  auto c = fixture();
  c.feedback_can_id = 0U;
  c.kp = 0.0;
  c.kd = 1.0;
  c.max_command_velocity_rad_s = 1.5;
  c.max_feedback_velocity_rad_s = 0.1;
  c.max_feedback_torque_nm = 5.0;
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(c), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
  auto frame = feedback(1U, 1);
  frame.payload[3] = 0xFFU;
  EXPECT_EQ(session.accept_feedback(frame, frame.host_arrival), AdapterResult::Fault);
  EXPECT_TRUE(session.fault_latched());
}

TEST(Dm4340Session, EffortCommandRequiresFreshEnabledFeedbackAndBounds) {
  auto c = fixture();
  c.feedback_can_id = 0U;
  c.kp = 0.0;
  c.kd = 0.0;
  c.max_command_effort_nm = 0.3;
  c.max_feedback_velocity_rad_s = 2.0;
  c.max_feedback_torque_nm = 1.0;
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(c), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
  mech::mech_control_core::RawCanFrame output{};
  const auto now = *MonotonicTime::from_nanoseconds(2);
  const auto deadline = *MonotonicTime::from_nanoseconds(4000002);
  EXPECT_EQ(session.prepare_effort({0.0, 0.0, 0.2, 1U, deadline}, now, output),
            AdapterResult::Stale);
  auto sample = feedback(0U, 1);
  ASSERT_EQ(session.accept_feedback(sample, sample.host_arrival), AdapterResult::Ok);
  EXPECT_EQ(session.prepare_effort({0.0, 0.0, 0.2, 1U, deadline}, now, output),
            AdapterResult::InvalidCommand);
  sample = feedback(1U, 3);
  ASSERT_EQ(session.accept_feedback(sample, sample.host_arrival), AdapterResult::Ok);
  const auto enabled_now = *MonotonicTime::from_nanoseconds(4);
  const auto enabled_deadline = *MonotonicTime::from_nanoseconds(4000004);
  ASSERT_EQ(session.prepare_effort({0.0, 0.0, 0.2, 1U, enabled_deadline},
                                   enabled_now, output), AdapterResult::Ok);
  EXPECT_TRUE(session.pending_target_still_authorized(
      output, 1U, enabled_deadline, enabled_now));
  std::array<std::uint8_t, 8> expected{};
  ASSERT_TRUE(encode_mit({0.0, 0.0, 0.0, 0.0, 0.2}, c.limits, expected));
  for (std::size_t i = 0; i < expected.size(); ++i)
    EXPECT_EQ(output.payload[i], expected[i]);
  const auto next = *MonotonicTime::from_nanoseconds(5);
  const auto next_deadline = *MonotonicTime::from_nanoseconds(4000005);
  EXPECT_EQ(session.prepare_effort({0.0, 0.0, 0.4, 2U, next_deadline},
                                   next, output), AdapterResult::InvalidCommand);
  EXPECT_FALSE(session.pending_target_still_authorized(
      output, 1U, enabled_deadline, next));
}

TEST(Dm4340Session, VelocityFeedforwardFollowsMotorDirectionAndStopsAtZero) {
  auto c = fixture();
  c.feedback_can_id = 0U;
  c.kp = 0.0;
  c.kd = 2.0;
  c.max_command_velocity_rad_s = 1.5;
  c.max_feedback_velocity_rad_s = 2.0;
  c.max_feedback_torque_nm = 5.0;
  c.velocity_feedforward_nm = 0.4;
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(c), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
  auto sample = feedback(1U, 1);
  ASSERT_EQ(session.accept_feedback(sample, sample.host_arrival), AdapterResult::Ok);

  const std::array<double, 3> velocities{0.8, -0.8, 0.0};
  const std::array<double, 3> torques{0.4, -0.4, 0.0};
  for (std::size_t index = 0; index < velocities.size(); ++index) {
    const auto now = *MonotonicTime::from_nanoseconds(index + 2U);
    const auto deadline = *MonotonicTime::from_nanoseconds(4000002U + index);
    mech::mech_control_core::RawCanFrame actual{};
    ASSERT_EQ(session.prepare_velocity(
        {0.0, velocities[index], 0.0, index + 1U, deadline}, now, actual),
        AdapterResult::Ok);
    std::array<std::uint8_t, 8> expected{};
    ASSERT_TRUE(encode_mit({0.0, velocities[index], 0.0, c.kd,
                            torques[index]}, c.limits, expected));
    for (std::size_t byte = 0; byte < expected.size(); ++byte)
      EXPECT_EQ(actual.payload[byte], expected[byte]);
  }

  c.joint.scale = -1.0;
  Dm4340MitSession reversed;
  ASSERT_EQ(reversed.configure(c), AdapterResult::Ok);
  ASSERT_EQ(reversed.activate(MonotonicTime{}), AdapterResult::Ok);
  ASSERT_EQ(reversed.accept_feedback(sample, sample.host_arrival), AdapterResult::Ok);
  mech::mech_control_core::RawCanFrame actual{};
  const auto now = *MonotonicTime::from_nanoseconds(2);
  const auto deadline = *MonotonicTime::from_nanoseconds(4000002);
  ASSERT_EQ(reversed.prepare_velocity({0.0, 0.8, 0.0, 1U, deadline}, now, actual),
            AdapterResult::Ok);
  std::array<std::uint8_t, 8> expected{};
  ASSERT_TRUE(encode_mit({0.0, -0.8, 0.0, c.kd, -0.4}, c.limits, expected));
  for (std::size_t byte = 0; byte < expected.size(); ++byte)
    EXPECT_EQ(actual.payload[byte], expected[byte]);
}

TEST(Dm4340Session, RejectsUnsafeVelocityFeedforwardConfiguration) {
  auto c = fixture();
  c.kp = 0.0;
  c.kd = 2.0;
  c.max_command_velocity_rad_s = 1.5;
  c.max_feedback_velocity_rad_s = 2.0;
  c.max_feedback_torque_nm = 5.0;
  c.velocity_feedforward_nm = 2.1;
  Dm4340MitSession session;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c.velocity_feedforward_nm = -0.1;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c.velocity_feedforward_nm = 0.4;
  c.max_command_velocity_rad_s = 0.0;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
}

}  // namespace
