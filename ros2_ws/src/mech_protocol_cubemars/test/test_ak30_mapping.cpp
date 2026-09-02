#include "mech_protocol_cubemars/ak30_mapping.hpp"

#include <gtest/gtest.h>

namespace {

using mech::mech_control_core::CanonicalDeviceCommand;
using mech::mech_control_core::CanonicalDeviceState;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::evidenced_state_fields;
using mech::mech_protocol_cubemars::ForceControlCommand;
using mech::mech_protocol_cubemars::ForceControlFeedback;
using mech::mech_protocol_cubemars::ForceControlGains;
using mech::mech_protocol_cubemars::ForceControlSubMode;
using mech::mech_protocol_cubemars::mapping_is_sufficient;
using mech::mech_protocol_cubemars::to_canonical_state;
using mech::mech_protocol_cubemars::to_device_command;

// Everything verified. Individual tests knock out one parameter at a time so a
// rejection can only be attributed to that parameter.
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

// motor1 as the repository actually evidences it today: pole_pairs, gear_ratio
// and torque_constant are verified. This is the state the evidence gate must
// still refuse, and it is why no sub-mode configures yet.
[[nodiscard]] Ak30Mapping motor1_as_evidenced() {
  Ak30Mapping mapping{};
  mapping.pole_pairs = {14.0, true};
  mapping.torque_constant_nm_per_a = {0.7382, true};
  // Three agreeing sources, 2026-09-02; see the header's comment.
  mapping.gear_ratio = {8.0, true};
  mapping.zero_offset_rad = {0.0, false};
  mapping.direction_sign = {1.0, false};
  mapping.position_source_known = false;
  return mapping;
}

TEST(Ak30Mapping, RefusesEverySubModeWithMotor1sCurrentEvidence) {
  const auto mapping = motor1_as_evidenced();
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));
}

// The design's central claim: direction_sign is the last parameter standing
// between the project and a configurable adapter. Since gear_ratio was verified
// on 2026-09-02 it now unblocks velocity as well as torque; only the position
// chain (zero offset and encoder shaft) remains beyond it. If this test ever
// needs changing, the shortest path to a usable adapter has moved and the
// design document is stale.
TEST(Ak30Mapping, VerifyingDirectionSignUnblocksTorqueAndVelocityButNotPosition) {
  auto mapping = motor1_as_evidenced();
  mapping.direction_sign = {1.0, true};

  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));
}

TEST(Ak30Mapping, TorqueSubModeConsumesOnlyDirectionSignAndTorqueConstant) {
  auto mapping = fully_verified();
  mapping.gear_ratio.verified = false;
  mapping.zero_offset_rad.verified = false;
  mapping.position_source_known = false;
  mapping.pole_pairs.verified = false;
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping.direction_sign.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.torque_constant_nm_per_a.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));
}

TEST(Ak30Mapping, VelocitySubModeAlsoConsumesPolePairsAndGearRatio) {
  auto mapping = fully_verified();
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  mapping.pole_pairs.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  mapping = fully_verified();
  mapping.gear_ratio.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  // Velocity does not need the position chain.
  mapping = fully_verified();
  mapping.zero_offset_rad.verified = false;
  mapping.position_source_known = false;
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
}

TEST(Ak30Mapping, PositionSubModeAlsoConsumesZeroOffsetAndShaftSource) {
  auto mapping = fully_verified();
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));

  mapping.position_source_known = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));

  mapping = fully_verified();
  mapping.zero_offset_rad.verified = false;
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Position));
}

// A verified-but-nonsensical value is still a rejection. "Verified" means an
// evidence source was read, not that whatever number is present is usable.
TEST(Ak30Mapping, RejectsVerifiedButPhysicallyImpossibleValues) {
  auto mapping = fully_verified();
  mapping.direction_sign = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.direction_sign = {2.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.direction_sign = {-1.0, true};
  EXPECT_TRUE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.torque_constant_nm_per_a = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Torque));

  mapping = fully_verified();
  mapping.gear_ratio = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));

  mapping = fully_verified();
  mapping.pole_pairs = {0.0, true};
  EXPECT_FALSE(mapping_is_sufficient(mapping, ForceControlSubMode::Velocity));
}

// Fields a sub-mode cannot evidence stay at 0.0 and must not be exported as
// ros2_control interfaces by the later slice. This table is that contract.
TEST(Ak30Mapping, ReportsWhichCanonicalFieldsEachSubModeCanEvidence) {
  const auto torque = evidenced_state_fields(ForceControlSubMode::Torque);
  EXPECT_FALSE(torque.position);
  EXPECT_FALSE(torque.velocity);
  EXPECT_TRUE(torque.effort);

  const auto velocity = evidenced_state_fields(ForceControlSubMode::Velocity);
  EXPECT_FALSE(velocity.position);
  EXPECT_TRUE(velocity.velocity);
  EXPECT_TRUE(velocity.effort);

  const auto position = evidenced_state_fields(ForceControlSubMode::Position);
  EXPECT_TRUE(position.position);
  EXPECT_TRUE(position.velocity);
  EXPECT_TRUE(position.effort);
}

TEST(Ak30Mapping, ConvertsFeedbackToCanonicalSiForTheOutputShaft) {
  // position 900  -> 90.0 deg -> pi/2 rad
  // speed    1000 -> 10000 ERPM -> 10000/14/8 rpm -> 9.3499781... rad/s
  // Iq        200 -> 2.0 A -> 2.0 * 0.7382 = 1.4764 N.m
  const ForceControlFeedback feedback{90.0, 10000.0, 2.0, 40.0, 0x00U};
  CanonicalDeviceState state{};
  to_canonical_state(fully_verified(), ForceControlSubMode::Position, feedback,
                     state);

  EXPECT_NEAR(state.position, 1.5707963267948966, 1e-12);
  EXPECT_NEAR(state.velocity, 9.349978135683909, 1e-12);
  EXPECT_NEAR(state.effort, 1.4764, 1e-12);
}

TEST(Ak30Mapping, LeavesUnevidencedCanonicalFieldsUntouched) {
  const ForceControlFeedback feedback{90.0, 10000.0, 2.0, 40.0, 0x00U};
  CanonicalDeviceState state{};
  to_canonical_state(fully_verified(), ForceControlSubMode::Torque, feedback,
                     state);

  EXPECT_DOUBLE_EQ(state.position, 0.0);
  EXPECT_DOUBLE_EQ(state.velocity, 0.0);
  EXPECT_NEAR(state.effort, 1.4764, 1e-12);
}

TEST(Ak30Mapping, AppliesDirectionSignAndZeroOffsetToDecodedPosition) {
  const ForceControlFeedback feedback{90.0, 10000.0, 2.0, 40.0, 0x00U};

  auto inverted = fully_verified();
  inverted.direction_sign = {-1.0, true};
  CanonicalDeviceState state{};
  to_canonical_state(inverted, ForceControlSubMode::Position, feedback, state);
  EXPECT_NEAR(state.position, -1.5707963267948966, 1e-12);
  EXPECT_NEAR(state.velocity, -9.349978135683909, 1e-12);
  EXPECT_NEAR(state.effort, -1.4764, 1e-12);

  auto offset = fully_verified();
  offset.zero_offset_rad = {0.5, true};
  CanonicalDeviceState offset_state{};
  to_canonical_state(offset, ForceControlSubMode::Position, feedback,
                     offset_state);
  EXPECT_NEAR(offset_state.position, 1.5707963267948966 - 0.5, 1e-12);
}

// When the encoder reports on the motor side, the reduction must be divided
// out. Which side it actually reports is vendor question B4, which is why
// position_source_known gates the whole sub-mode.
TEST(Ak30Mapping, DividesOutTheReductionWhenPositionIsMotorSide) {
  auto mapping = fully_verified();
  mapping.position_is_output_shaft = false;
  const ForceControlFeedback feedback{90.0, 0.0, 0.0, 40.0, 0x00U};
  CanonicalDeviceState state{};
  to_canonical_state(mapping, ForceControlSubMode::Position, feedback, state);
  EXPECT_NEAR(state.position, 0.19634954084936207, 1e-12);
}

TEST(Ak30Mapping, BuildsTorqueSubModeCommandsFromEffortAlone) {
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.position = 1.0;   // ignored by this sub-mode, see below
  command.velocity = 3.0;   // ignored by this sub-mode, see below
  to_device_command(fully_verified(), ForceControlSubMode::Torque,
                    ForceControlGains{}, command, wire);

  EXPECT_DOUBLE_EQ(wire.kp, 0.0);
  EXPECT_DOUBLE_EQ(wire.kd, 0.0);
  EXPECT_DOUBLE_EQ(wire.torque_nm, 2.0);
}

// Deliberate and documented: in torque sub-mode the canonical command's
// position and velocity are ignored rather than rejected, because the later
// ros2_control slice claims only the effort interface for a torque-mode
// device, so no consumer can send a position it expects to be honoured.
// Named so it reads as intent, not as a bug someone should "fix".
TEST(Ak30Mapping, TorqueSubModeIgnoresPositionAndVelocityByDesign) {
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.effort = 2.0;
  command.position = 5.0;
  command.velocity = 7.0;
  to_device_command(fully_verified(), ForceControlSubMode::Torque,
                    ForceControlGains{}, command, wire);

  EXPECT_DOUBLE_EQ(wire.position_rad, 0.0);
  EXPECT_DOUBLE_EQ(wire.velocity_rad_s, 0.0);
}

TEST(Ak30Mapping, BuildsPositionSubModeCommandsWithGainsSignAndOffset) {
  auto mapping = fully_verified();
  mapping.zero_offset_rad = {0.5, true};
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.position = 1.0;
  command.velocity = 2.0;
  command.effort = 0.25;  // rides along as feedforward torque, the field's role
  to_device_command(mapping, ForceControlSubMode::Position,
                    ForceControlGains{100.0, 2.0}, command, wire);

  EXPECT_DOUBLE_EQ(wire.kp, 100.0);
  EXPECT_DOUBLE_EQ(wire.kd, 2.0);
  EXPECT_DOUBLE_EQ(wire.position_rad, 1.5);
  EXPECT_DOUBLE_EQ(wire.velocity_rad_s, 2.0);
  EXPECT_DOUBLE_EQ(wire.torque_nm, 0.25);
}

TEST(Ak30Mapping, VelocitySubModeCarriesKdButNotKp) {
  ForceControlCommand wire{};
  CanonicalDeviceCommand command{};
  command.velocity = 6.0;
  to_device_command(fully_verified(), ForceControlSubMode::Velocity,
                    ForceControlGains{100.0, 2.0}, command, wire);

  EXPECT_DOUBLE_EQ(wire.kp, 0.0);
  EXPECT_DOUBLE_EQ(wire.kd, 2.0);
  EXPECT_DOUBLE_EQ(wire.velocity_rad_s, 6.0);
  EXPECT_DOUBLE_EQ(wire.position_rad, 0.0);
}

// Encode and decode must be exact inverses, otherwise a commanded position and
// the position reported back would disagree by a constant nobody notices.
TEST(Ak30Mapping, PositionEncodeAndDecodeAreInverses) {
  auto mapping = fully_verified();
  mapping.direction_sign = {-1.0, true};
  mapping.zero_offset_rad = {0.25, true};

  CanonicalDeviceCommand command{};
  command.position = 1.0;
  ForceControlCommand wire{};
  to_device_command(mapping, ForceControlSubMode::Position, ForceControlGains{},
                    command, wire);

  // Feed the encoded shaft angle back through the decoder as degrees.
  ForceControlFeedback feedback{};
  feedback.position_deg = wire.position_rad * 180.0 / 3.14159265358979323846;
  CanonicalDeviceState state{};
  to_canonical_state(mapping, ForceControlSubMode::Position, feedback, state);

  EXPECT_NEAR(state.position, command.position, 1e-12);
}

}  // namespace
