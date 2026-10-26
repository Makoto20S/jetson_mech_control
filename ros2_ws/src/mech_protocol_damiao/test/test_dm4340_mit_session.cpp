#include "mech_protocol_damiao/dm4340_mit_session.hpp"

#include <array>
#include <cmath>
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

TEST(Dm4340Session, LowerLayerPositionGuardBlocksOutwardForBothModesAndMappings) {
  for (const bool effort : {false, true}) {
    for (const double scale : {-1.0, 1.0}) {
      for (const double sign : {-1.0, 1.0}) {
        auto c = fixture();
        c.feedback_can_id = 0U;
        c.joint.scale = scale;
        c.kp = 0.0;
        c.kd = effort ? 0.0 : 1.0;
        c.max_command_velocity_rad_s = effort ? 0.0 : 1.5;
        c.max_command_effort_nm = effort ? 0.3 : 0.0;
        c.max_feedback_velocity_rad_s = 2.0;
        c.max_feedback_torque_nm = 5.0;
        Dm4340MitSession session;
        ASSERT_EQ(session.configure(c), AdapterResult::Ok);
        ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
        auto sample = feedback(1U, 1);
        const auto code = static_cast<std::uint16_t>(
            ((sign*.99/scale+12.5)/25.0)*65535.0);
        sample.payload[1] = static_cast<std::uint8_t>(code >> 8U);
        sample.payload[2] = static_cast<std::uint8_t>(code);
        ASSERT_EQ(session.accept_feedback(sample, sample.host_arrival), AdapterResult::Ok);
        const auto now = *MonotonicTime::from_nanoseconds(2);
        const auto deadline = *MonotonicTime::from_nanoseconds(4000002);
        mech::mech_control_core::RawCanFrame out{};
        const auto prepare = [&](double value, std::uint64_t generation) {
          return effort ? session.prepare_effort({0,0,value,generation,deadline},now,out)
                        : session.prepare_velocity({0,value,0,generation,deadline},now,out);
        };
        EXPECT_EQ(prepare(sign*.1,1), AdapterResult::InvalidCommand);
        EXPECT_EQ(prepare(-sign*.1,1), AdapterResult::Ok);
        EXPECT_EQ(prepare(0,2), AdapterResult::Ok);
      }
    }
  }
}

TEST(Dm4340Session, NewBoundaryFeedbackRevokesAlreadyPreparedCommand) {
  auto c = fixture(); c.feedback_can_id=0; c.kp=0; c.kd=0;
  c.max_command_effort_nm=.3; c.max_feedback_torque_nm=5;
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(c),AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}),AdapterResult::Ok);
  auto sample=feedback(1,1);
  ASSERT_EQ(session.accept_feedback(sample,sample.host_arrival),AdapterResult::Ok);
  mech::mech_control_core::RawCanFrame out{};
  auto now=*MonotonicTime::from_nanoseconds(2);
  auto deadline=*MonotonicTime::from_nanoseconds(4000002);
  ASSERT_EQ(session.prepare_effort({0,0,.1,1,deadline},now,out),AdapterResult::Ok);
  ASSERT_TRUE(session.pending_target_still_authorized(out,1,deadline,now));
  sample=feedback(1,3);
  const auto code=static_cast<std::uint16_t>(((.99+12.5)/25)*65535);
  sample.payload[1]=static_cast<std::uint8_t>(code>>8U);
  sample.payload[2]=static_cast<std::uint8_t>(code);
  ASSERT_EQ(session.accept_feedback(sample,sample.host_arrival),AdapterResult::Ok);
  EXPECT_FALSE(session.pending_target_still_authorized(out,1,deadline,sample.host_arrival));
}

TEST(Dm4340Session, PositionViolationLatchesUntilExplicitLifecycleRecovery) {
  auto c=fixture();c.feedback_can_id=0;
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(c),AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}),AdapterResult::Ok);
  auto sample=feedback(1,1);sample.payload[1]=0xFF;sample.payload[2]=0xFF;
  EXPECT_EQ(session.accept_feedback(sample,sample.host_arrival),AdapterResult::InvalidCommand);
  EXPECT_TRUE(session.fault_latched());
  sample=feedback(1,2);
  EXPECT_EQ(session.accept_feedback(sample,sample.host_arrival),AdapterResult::Fault);
}

TEST(Dm4340Session, InvalidHardBoundaryMarginFailsClosed) {
  for (const double margin : {0.0, -0.01, 1.0, 2.0}) {
    auto c=fixture();c.position_guard_margin_rad=margin;
    Dm4340MitSession session;
    EXPECT_EQ(session.configure(c),AdapterResult::InvalidConfiguration);
  }
}

TEST(Dm4310Session, MeasuredOffsetBoundsProtectBothAxesAndBothCommandModes) {
  using namespace mech::mech_control_core;
  using namespace mech::mech_protocol_damiao;
  for (unsigned axis : {1U, 2U}) {
    for (bool effort : {false, true}) {
      for (double sign : {-1., 1.}) {
        auto c = fixture();
        c.profile = ProtocolProfile::Dm4310MitStandard;
        c.limits = kSdkDm4310Limits;
        c.command_can_id = axis; c.motor_id = axis; c.feedback_can_id = axis+3;
        c.position_min_rad = axis == 1 ? -.83 : -2.06;
        c.position_max_rad = axis == 1 ? .82 : -.74;
        c.kp = 0; c.kd = effort ? 0 : 1;
        c.max_command_velocity_rad_s = effort ? 0 : .3;
        c.max_command_effort_nm = effort ? .2 : 0;
        c.max_feedback_velocity_rad_s = 2; c.max_feedback_torque_nm = 5;
        Dm4340MitSession session;
        ASSERT_EQ(session.configure(c), AdapterResult::Ok);
        ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
        auto sample = feedback(1,1);
        sample.id = *CanId::create(axis+3, CanFrameFormat::Standard);
        sample.payload[0] = 0x10 | axis;
        const double q = sign < 0 ? c.position_min_rad+.01 : c.position_max_rad-.01;
        auto code = static_cast<std::uint16_t>((q+12.5)*65535/25);
        sample.payload[1] = code >> 8; sample.payload[2] = code & 255;
        ASSERT_EQ(session.accept_feedback(sample,sample.host_arrival), AdapterResult::Ok);
        const auto deadline = *MonotonicTime::from_nanoseconds(4000002);
        const auto now = *MonotonicTime::from_nanoseconds(2);
        RawCanFrame output{};
        auto prepare = [&](double value) {
          return effort ? session.prepare_effort({0,0,value,1,deadline},now,output)
                        : session.prepare_velocity({0,value,0,1,deadline},now,output);
        };
        EXPECT_EQ(prepare(sign*.1), AdapterResult::InvalidCommand);
        EXPECT_EQ(prepare(-sign*.1), AdapterResult::Ok);
        // Actual position outside either bound must invalidate and latch.
        const double outside = sign < 0 ? c.position_min_rad-.01 : c.position_max_rad+.01;
        code = static_cast<std::uint16_t>((outside+12.5)*65535/25);
        sample.payload[1] = code >> 8; sample.payload[2] = code & 255;
        sample.host_arrival = *MonotonicTime::from_nanoseconds(3);
        EXPECT_EQ(session.accept_feedback(sample,sample.host_arrival), AdapterResult::InvalidCommand);
        EXPECT_TRUE(session.fault_latched());
        EXPECT_FALSE(session.pending_target_still_authorized(output,1,deadline,sample.host_arrival));
      }
    }
  }
}

namespace {

TEST(Dm4340Session, PositionImpedanceRejectsDisabledAndRechecksBeforeSend) {
  auto c = fixture(); c.feedback_can_id = 0;
  Dm4340MitSession s;
  ASSERT_EQ(s.configure(c), AdapterResult::Ok);
  ASSERT_EQ(s.activate(MonotonicTime{}), AdapterResult::Ok);
  auto now = *MonotonicTime::from_nanoseconds(1);
  auto deadline = *MonotonicTime::from_nanoseconds(4000001);
  mech::mech_control_core::RawCanFrame out{};
  ASSERT_EQ(s.accept_feedback(feedback(0,1), now), AdapterResult::Ok);
  EXPECT_EQ(s.prepare_position({.1,0,0,1,deadline},now,out), AdapterResult::InvalidCommand);
  ASSERT_EQ(s.accept_feedback(feedback(1,1), now), AdapterResult::Ok);
  ASSERT_EQ(s.prepare_position({.1,0,0,1,deadline},now,out), AdapterResult::Ok);
  EXPECT_TRUE(s.pending_target_still_authorized(out,1,deadline,now));
  ASSERT_EQ(s.accept_feedback(feedback(0,2), *MonotonicTime::from_nanoseconds(2)), AdapterResult::Ok);
  EXPECT_FALSE(s.pending_target_still_authorized(out,1,deadline,*MonotonicTime::from_nanoseconds(2)));
}

TEST(Dm4340Session, PositionImpedanceMapsNegativeScaleAndUsesNativeGains) {
  auto c = fixture(); c.feedback_can_id = 0;
  c.joint = {-2.0, .1, true};
  Dm4340MitSession s;
  ASSERT_EQ(s.configure(c), AdapterResult::Ok);
  ASSERT_EQ(s.activate(MonotonicTime{}), AdapterResult::Ok);
  const auto now = *MonotonicTime::from_nanoseconds(1);
  ASSERT_EQ(s.accept_feedback(feedback(1,1), now), AdapterResult::Ok);
  mech::mech_control_core::RawCanFrame out{};
  ASSERT_EQ(s.prepare_position({.2,0,0,1,*MonotonicTime::from_nanoseconds(4000001)},now,out), AdapterResult::Ok);
  // Independent MIT packing: p=-.05 in +/-12.5 -> 32636; v=0 ->2047;
  // Kp=1 in 0..500 ->8; Kd=.1 in 0..5 ->81; t=0 ->2047.
  const std::array<std::uint8_t,8> expected{0x7f,0x7c,0x7f,0xf0,0x08,0x05,0x17,0xff};
  for (std::size_t i=0;i<8;++i) EXPECT_EQ(out.payload[i],expected[i]);
}

TEST(Dm4340Session, PositionImpedanceGuardRechecksQuantizedTargetAndError) {
  auto c=fixture(); c.feedback_can_id=0;
  c.position_min_rad=-.1; c.position_max_rad=.1;
  c.max_target_error_rad=.09;
  Dm4340MitSession s;
  ASSERT_EQ(s.configure(c),AdapterResult::Ok);
  ASSERT_EQ(s.activate(MonotonicTime{}),AdapterResult::Ok);
  auto now=*MonotonicTime::from_nanoseconds(1);
  auto deadline=*MonotonicTime::from_nanoseconds(4000001);
  auto fb=feedback(1,1);
  ASSERT_EQ(s.accept_feedback(fb,now),AdapterResult::Ok);
  mech::mech_control_core::RawCanFrame out{};
  ASSERT_EQ(s.prepare_position({.085,0,0,1,deadline},now,out),AdapterResult::Ok);
  // Move into the upper guard (.085); same target is now behind the measured
  // position, so it remains allowed. A larger outward target must be rejected.
  fb.payload[1]=0x80; fb.payload[2]=0xdf; // ~.085 rad
  fb.host_arrival=*MonotonicTime::from_nanoseconds(2);
  ASSERT_EQ(s.accept_feedback(fb,fb.host_arrival),AdapterResult::Ok);
  EXPECT_EQ(s.prepare_position({.095,0,0,2,deadline},fb.host_arrival,out),AdapterResult::InvalidCommand);
  EXPECT_EQ(s.prepare_position({0,0,0,3,deadline},fb.host_arrival,out),AdapterResult::Ok);
  // A fresh feedback change can invalidate an otherwise unexpired pending frame.
  fb.payload[1]=0x7f; fb.payload[2]=0x0a; // ~-.094 rad
  fb.host_arrival=*MonotonicTime::from_nanoseconds(3);
  ASSERT_EQ(s.accept_feedback(fb,fb.host_arrival),AdapterResult::Ok);
  EXPECT_FALSE(s.pending_target_still_authorized(out,3,deadline,fb.host_arrival));
}

TEST(Dm4340Session, RejectedFrameRetainsNumbersWithoutPublishingInvalidState) {
  using mech::mech_protocol_damiao::DmFeedbackRejection;
  for (int scenario = 0; scenario < 8; ++scenario) {
    SCOPED_TRACE(scenario);
    auto c = fixture();
    c.max_feedback_velocity_rad_s = 0.1;
    c.max_feedback_torque_nm = 1.0;
    Dm4340MitSession session;
    ASSERT_EQ(session.configure(c), AdapterResult::Ok);
    ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
    auto frame = feedback(1U, 1000000);
    frame.id.value = c.feedback_can_id;
    auto now = frame.host_arrival;
    auto expected = DmFeedbackRejection::Velocity;
    if (scenario == 0 || scenario == 7) {
      frame.payload[3] = 0xFFU;  // Independently encoded high positive velocity.
    } else if (scenario == 1) {
      expected = DmFeedbackRejection::Torque;
    } else if (scenario == 2) {
      frame.payload[1] = frame.payload[2] = 0xFFU;
      expected = DmFeedbackRejection::Position;
    } else if (scenario == 3) {
      now = *MonotonicTime::from_nanoseconds(7000000); // TTL equality rejects.
      expected = DmFeedbackRejection::Expired;
    } else if (scenario == 4) {
      now = *MonotonicTime::from_nanoseconds(999999);
      expected = DmFeedbackRejection::TimeOrder;
    } else if (scenario == 5) {
      frame.payload_size = 2U;
      expected = DmFeedbackRejection::Decode;
    } else if (scenario == 6) {
      frame.payload[0] = 0x91U;
      expected = DmFeedbackRejection::DeviceStatus;
    }
    if (scenario == 1 || scenario == 7) {
      frame.payload[4] |= 0x0FU;
      frame.payload[5] = 0xFFU;  // +28 Nm; both exceeded flags for scenario7.
    }
    EXPECT_NE(session.accept_feedback(frame, now), AdapterResult::Ok);
    ASSERT_TRUE(session.feedback_diagnostic());
    const auto saved = *session.feedback_diagnostic();
    EXPECT_EQ(saved.reason, expected);
    EXPECT_EQ(saved.frame.payload, frame.payload);
    EXPECT_EQ(saved.decoded, scenario != 5);
    EXPECT_EQ(saved.velocity_exceeded, scenario == 0 || scenario == 7);
    EXPECT_EQ(saved.torque_exceeded, scenario == 1 || scenario == 7);
    if (saved.decoded) {
      EXPECT_TRUE(std::isfinite(saved.joint_position_rad));
      EXPECT_TRUE(std::isfinite(saved.velocity_rad_s));
      EXPECT_TRUE(std::isfinite(saved.torque_nm));
    }
    EXPECT_TRUE(std::isnan(session.snapshot(now).joint_position_rad));
    frame.payload_size = 8U;
    frame.payload[0] = 0x91U;
    (void)session.accept_feedback(frame, now);
    EXPECT_EQ(session.feedback_diagnostic()->reason, saved.reason);
    EXPECT_EQ(session.feedback_diagnostic()->frame.payload, saved.frame.payload);
    session.deactivate();
    EXPECT_FALSE(session.feedback_diagnostic());
    ASSERT_EQ(session.activate(now), AdapterResult::Ok);
    EXPECT_FALSE(session.feedback_diagnostic());
  }
}

TEST(Dm4340Session, NearPointOneSpeedRejectsNextWireStepAndRetainsIt) {
  auto c = fixture();
  c.max_feedback_velocity_rad_s = 0.1;
  c.max_feedback_torque_nm = 1.0;
  Dm4340MitSession session;
  ASSERT_EQ(session.configure(c), AdapterResult::Ok);
  ASSERT_EQ(session.activate(MonotonicTime{}), AdapterResult::Ok);
  auto frame = feedback(1U, 1000000);
  frame.id.value = c.feedback_can_id;
  frame.payload[3] = 0x81U;
  frame.payload[4] = 0x37U; // velocity code2067 -> +0.0952380952rad/s
  ASSERT_EQ(session.accept_feedback(frame, frame.host_arrival), AdapterResult::Ok);
  EXPECT_NEAR(session.snapshot(frame.host_arrival).raw_motor_velocity_rad_s,
              0.0952380952380952, 1e-12);
  EXPECT_FALSE(session.feedback_diagnostic());
  frame.payload[4] = 0x47U; // next code2068 -> +0.1001221001rad/s
  frame.host_arrival = *MonotonicTime::from_nanoseconds(2000000);
  EXPECT_EQ(session.accept_feedback(frame, frame.host_arrival), AdapterResult::Fault);
  ASSERT_TRUE(session.feedback_diagnostic());
  EXPECT_EQ(session.feedback_diagnostic()->reason,
            mech::mech_protocol_damiao::DmFeedbackRejection::Velocity);
  EXPECT_NEAR(session.feedback_diagnostic()->velocity_rad_s,
              0.100122100122100, 1e-12);
  EXPECT_TRUE(std::isnan(session.snapshot(frame.host_arrival).raw_motor_velocity_rad_s));
}

}  // namespace

TEST(Dm4340Session, EffortWireQuantizationStaysInsideMappedLimit) {
  for(double scale : {1.0,-1.0,0.5,-2.0}) {
    auto c=fixture();
    c.feedback_can_id=0;
    c.kp=0;c.kd=0;
    c.joint.scale=scale;
    c.max_command_effort_nm=8.0/std::abs(scale);
    c.max_feedback_velocity_rad_s=2;
    c.max_feedback_torque_nm=8;
    Dm4340MitSession session;
    ASSERT_EQ(session.configure(c),AdapterResult::Ok);
    ASSERT_EQ(session.activate(MonotonicTime{}),AdapterResult::Ok);
    auto sample=feedback(1,1);
    ASSERT_EQ(session.accept_feedback(sample,sample.host_arrival),AdapterResult::Ok);
    unsigned generation=0;
    for(double sign : {-1.0,1.0}) {
      mech::mech_control_core::RawCanFrame frame{};
      ASSERT_EQ(session.prepare_effort({0,0,sign*c.max_command_effort_nm,++generation,
          *MonotonicTime::from_nanoseconds(4000002)},
          *MonotonicTime::from_nanoseconds(2),frame),AdapterResult::Ok);
      const auto code=((frame.payload[6]&15U)<<8U)|frame.payload[7];
      const double joint=(code*56.0/4095.0-28.0)/scale;
      EXPECT_LE(std::abs(joint),c.max_command_effort_nm+1e-12);
      EXPECT_NEAR(joint,sign*c.max_command_effort_nm,56.0/4095.0/std::abs(scale));
    }
  }
}
