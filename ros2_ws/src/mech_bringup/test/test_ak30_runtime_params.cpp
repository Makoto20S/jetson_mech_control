#include "mech_bringup/ak30_runtime_params.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <string>

#include "mech_protocol_cubemars/ak30_mapping.hpp"

namespace mech::mech_bringup {
namespace {

using mech::mech_protocol_cubemars::ForceControlSubMode;

using Params = std::map<std::string, std::string>;

TEST(Ak30RuntimeParams, ParsesFullParameterSet) {
  Params params;
  params["device_path"] = "/dev/ttyACM0";
  params["logical_bus"] = "1";
  params["drive_id"] = "104";
  params["kp"] = "1.0";
  params["kd"] = "1.0";
  params["control_period_ns"] = "2000000";
  params["command_ttl_ns"] = "4000000";
  params["command_hard_ttl_ns"] = "6000000";
  params["feedback_ttl_ns"] = "2000000";
  params["zero_offset_rad"] = "5.760604931781636";
  params["position_is_output_shaft"] = "true";

  const auto parsed = Ak30RuntimeParams::parse(params);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->config.drive_id, 104U);
  EXPECT_EQ(parsed->config.logical_bus, 1U);
  EXPECT_EQ(parsed->config.sub_mode, ForceControlSubMode::Position);
  EXPECT_DOUBLE_EQ(parsed->config.gains.kp, 1.0);
  EXPECT_DOUBLE_EQ(parsed->config.gains.kd, 1.0);
  EXPECT_EQ(parsed->config.control_period_nanoseconds, 2000000);
  EXPECT_EQ(parsed->config.command_ttl_nanoseconds, 4000000);
  EXPECT_EQ(parsed->config.command_hard_ttl_nanoseconds, 6000000);
  EXPECT_EQ(parsed->config.feedback_ttl_nanoseconds, 2000000);
  EXPECT_DOUBLE_EQ(parsed->config.mapping.zero_offset_rad.value,
                   5.760604931781636);
  EXPECT_TRUE(parsed->config.mapping.position_is_output_shaft);
  EXPECT_EQ(parsed->device_path, "/dev/ttyACM0");
}

// Since ADR-014 the sub-mode is an explicit parameter; the default stays
// Position so existing deployments parse unchanged. The URDF command
// interface must match (pinned offline by test_deployment_files.cpp).
TEST(Ak30RuntimeParams, SubModeDefaultsToPositionAndParsesAllThreeValues) {
  Params defaults;
  defaults["device_path"] = "/dev/ttyACM0";
  const auto parsed = Ak30RuntimeParams::parse(defaults);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->config.sub_mode, ForceControlSubMode::Position);

  Params velocity;
  velocity["device_path"] = "/dev/ttyACM0";
  velocity["sub_mode"] = "velocity";
  const auto parsed_velocity = Ak30RuntimeParams::parse(velocity);
  ASSERT_TRUE(parsed_velocity.has_value());
  EXPECT_EQ(parsed_velocity->config.sub_mode, ForceControlSubMode::Velocity);

  Params torque;
  torque["device_path"] = "/dev/ttyACM0";
  torque["sub_mode"] = "torque";
  const auto parsed_torque = Ak30RuntimeParams::parse(torque);
  ASSERT_TRUE(parsed_torque.has_value());
  EXPECT_EQ(parsed_torque->config.sub_mode, ForceControlSubMode::Torque);

  Params explicit_position;
  explicit_position["device_path"] = "/dev/ttyACM0";
  explicit_position["sub_mode"] = "position";
  const auto parsed_position = Ak30RuntimeParams::parse(explicit_position);
  ASSERT_TRUE(parsed_position.has_value());
  EXPECT_EQ(parsed_position->config.sub_mode, ForceControlSubMode::Position);

  // Explicit configuration, never guessed: an unrecognized value rejects
  // the whole config rather than falling back to the default.
  Params bad;
  bad["device_path"] = "/dev/ttyACM0";
  bad["sub_mode"] = "servo";
  EXPECT_FALSE(Ak30RuntimeParams::parse(bad).has_value());

  Params empty;
  empty["device_path"] = "/dev/ttyACM0";
  empty["sub_mode"] = "";
  EXPECT_FALSE(Ak30RuntimeParams::parse(empty).has_value());
}

// The command-interface name a deployment's URDF must declare for each
// sub-mode (ADR-014): the interface shape and the sub-mode are two
// spellings of the same choice.
TEST(Ak30RuntimeParams, ExpectedCommandInterfaceNameMatchesSubMode) {
  EXPECT_STREQ(expected_command_interface_name(ForceControlSubMode::Position),
               "position");
  EXPECT_STREQ(expected_command_interface_name(ForceControlSubMode::Velocity),
               "velocity");
  EXPECT_STREQ(expected_command_interface_name(ForceControlSubMode::Torque),
               "effort");
}

TEST(Ak30RuntimeParams, DefaultsAreTheBenchEvidencedMotor1Values) {
  Params params;
  params["device_path"] = "/dev/ttyACM0";
  const auto parsed = Ak30RuntimeParams::parse(params);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->config.drive_id, 104U);
  EXPECT_EQ(parsed->config.logical_bus, 1U);
  EXPECT_EQ(parsed->config.control_period_nanoseconds, 2000000);
  EXPECT_EQ(parsed->config.command_ttl_nanoseconds, 4000000);
  EXPECT_EQ(parsed->config.command_hard_ttl_nanoseconds, 6000000);
  // ADR-012: the whole watchdog fits <=3 control cycles (<=6 ms at 500 Hz).
  EXPECT_LE(parsed->config.command_hard_ttl_nanoseconds, 6000000);
  EXPECT_DOUBLE_EQ(parsed->config.mapping.zero_offset_rad.value,
                    5.760604931781636);  // 330.07 deg, owner-approved
  EXPECT_DOUBLE_EQ(parsed->config.mapping.direction_sign.value, 1.0);
}

// Fail closed: any invalid or unknown parameter rejects the whole config.
TEST(Ak30RuntimeParams, RejectsInvalidValues) {
  const auto check_invalid = [](const Params& params) {
    const auto parsed = Ak30RuntimeParams::parse(params);
    EXPECT_FALSE(parsed.has_value());
  };
  Params base;
  base["drive_id"] = "300";  // wire field is 8 bits
  check_invalid(base);

  Params negative_bus;
  negative_bus["logical_bus"] = "0";
  check_invalid(negative_bus);

  Params bad_ttl;
  bad_ttl["command_ttl_ns"] = "0";
  check_invalid(bad_ttl);

  Params inverted_ttl;
  inverted_ttl["command_ttl_ns"] = "6000000";
  inverted_ttl["command_hard_ttl_ns"] = "4000000";  // hard < soft
  check_invalid(inverted_ttl);

  Params over_budget;
  over_budget["command_ttl_ns"] = "5000000";
  over_budget["command_hard_ttl_ns"] = "8000000";  // > 6 ms budget
  check_invalid(over_budget);

  Params bad_period;
  bad_period["control_period_ns"] = "-2000000";
  check_invalid(bad_period);

  Params not_a_number;
  not_a_number["drive_id"] = "104abc";
  check_invalid(not_a_number);

  Params bad_double;
  bad_double["kp"] = "nan";
  check_invalid(bad_double);

  Params negative_gain;
  negative_gain["kp"] = "-1.0";
  check_invalid(negative_gain);

  Params unknown;
  unknown["completely_unknown_key"] = "1";
  check_invalid(unknown);
}

// device_path has no default: a deployment that never names its serial
// device must not silently fall back to a guessed /dev/ttyACM*.
TEST(Ak30RuntimeParams, DevicePathIsMandatory) {
  const auto parsed = Ak30RuntimeParams::parse(Params{});
  ASSERT_FALSE(parsed.has_value());

  Params only_path;
  only_path["device_path"] = "/dev/ttyACM0";
  const auto ok = Ak30RuntimeParams::parse(only_path);
  ASSERT_TRUE(ok.has_value());
  EXPECT_EQ(ok->device_path, "/dev/ttyACM0");
}

}  // namespace
}  // namespace mech::mech_bringup
