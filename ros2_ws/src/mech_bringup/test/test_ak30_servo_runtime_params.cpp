#include "mech_bringup/ak30_servo_runtime_params.hpp"

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"

namespace {

hardware_interface::InterfaceInfo interface(const std::string& name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  result.size = 1;
  return result;
}

hardware_interface::ComponentInfo joint(const std::string& name,
                                        const std::string& id) {
  hardware_interface::ComponentInfo result;
  result.name = name;
  result.type = "joint";
  result.state_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
  result.command_interfaces = {
      interface(hardware_interface::HW_IF_POSITION),
      interface(mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
  result.parameters = {
      {"drive_id", id},
      {"target_scale", "2.0"},
      {"target_offset", "-5.0"},
      {"target_mapping_verified", "true"},
      {"feedback_scale", "0.5"},
      {"feedback_offset", "1.0"},
      {"feedback_mapping_verified", "true"},
      {"speed_erpm", "1000"},
      {"acceleration_raw", "500"},
      {"position_min_rad", "-2"},
      {"position_max_rad", "2"},
      {"position_max_error_rad", "0.25"},
  };
  return result;
}

hardware_interface::HardwareInfo valid_info() {
  hardware_interface::HardwareInfo info;
  info.name = "servo_pair";
  info.type = "system";
  info.hardware_class_type = "mech_bringup/Ak30ServoSystem";
  info.hardware_parameters = {
      {"profile", "ak30_servo_extended"},
      {"device_path", "/dev/ttyTEST0"},
      {"logical_bus", "42"},
      {"control_period_ns", "2000000"},
      {"command_ttl_ns", "3000000"},
      {"command_hard_ttl_ns", "6000000"},
      {"feedback_ttl_ns", "60000000"},
  };
  info.joints.push_back(joint("left", "104"));
  info.joints.push_back(joint("right", "105"));
  return info;
}

using Params = mech::mech_bringup::Ak30ServoRuntimeParams;

TEST(Ak30ServoRuntimeParams, OptionalTraceNameCannotEscapeItsCaptureDirectory) {
  auto info = valid_info();
  info.hardware_parameters["trace_name"] = "bus-2";
  EXPECT_TRUE(Params::parse(info));
  for (const auto& name : {"", "../other", "bus/2", "bus_2"}) {
    info.hardware_parameters["trace_name"] = name;
    EXPECT_FALSE(Params::parse(info));
  }
}

TEST(Ak30ServoRuntimeParams, ParsesIndependentExplicitMappingsInJointOrder) {
  const auto parsed = Params::parse(valid_info());
  ASSERT_TRUE(parsed);
  EXPECT_EQ(parsed->device_path, "/dev/ttyTEST0");
  EXPECT_EQ(parsed->config.physical_bus, "/dev/ttyTEST0");
  EXPECT_EQ(parsed->config.logical_bus, 42U);
  EXPECT_EQ(parsed->config.control_period_ns, 2000000);
  ASSERT_EQ(parsed->config.joints.size(), 2U);
  EXPECT_EQ(parsed->config.joints[0].drive_id, 104U);
  EXPECT_EQ(parsed->config.joints[1].drive_id, 105U);
  EXPECT_DOUBLE_EQ(parsed->config.joints[0].target_rad_to_deg.scale, 2.0);
  EXPECT_DOUBLE_EQ(parsed->config.joints[0].target_rad_to_deg.offset, -5.0);
  EXPECT_DOUBLE_EQ(parsed->config.joints[0].feedback_deg_to_rad.scale, 0.5);
  EXPECT_DOUBLE_EQ(parsed->config.joints[0].feedback_deg_to_rad.offset, 1.0);
  EXPECT_EQ(parsed->config.joints[0].command_ttl_ns, 3000000);
  EXPECT_EQ(parsed->config.joints[1].command_hard_ttl_ns, 6000000);
  EXPECT_EQ(parsed->config.joints[1].feedback_ttl_ns, 60000000);
}

TEST(Ak30ServoRuntimeParams, RequiresGenerationDeclarationForWeakClaims) {
  auto info = valid_info();
  info.joints[0].command_interfaces.pop_back();
  EXPECT_FALSE(Params::parse(info));
}

TEST(Ak30ServoRuntimeParams, RejectsUnknownMissingAndMalformedHardwareParameters) {
  for (const char* key : {"profile", "device_path", "logical_bus",
       "control_period_ns", "command_ttl_ns", "command_hard_ttl_ns",
       "feedback_ttl_ns"}) {
    auto info = valid_info();
    info.hardware_parameters.erase(key);
    EXPECT_FALSE(Params::parse(info)) << key;
  }
  for (const auto& entry : std::map<std::string, std::string>{
       {"profile", "ak30_force"}, {"device_path", ""},
       {"logical_bus", "0"}, {"control_period_ns", "2ms"},
       {"command_ttl_ns", "-1"}, {"command_hard_ttl_ns", "6000001"},
       {"feedback_ttl_ns", "18446744073709551616"}}) {
    auto info = valid_info();
    info.hardware_parameters[entry.first] = entry.second;
    EXPECT_FALSE(Params::parse(info)) << entry.first;
  }
  auto info = valid_info();
  info.hardware_parameters["typo"] = "1";
  EXPECT_FALSE(Params::parse(info));
}

TEST(Ak30ServoRuntimeParams, RejectsUnknownMissingMalformedAndUnsafeJointParameters) {
  const auto baseline = valid_info();
  for (const auto& entry : baseline.joints.front().parameters) {
    auto info = valid_info();
    info.joints[0].parameters.erase(entry.first);
    EXPECT_FALSE(Params::parse(info)) << entry.first;
  }
  for (const auto& entry : std::map<std::string, std::string>{
       {"drive_id", "256"}, {"target_scale", "NaN"},
       {"target_offset", "1junk"}, {"target_mapping_verified", "false"},
       {"feedback_scale", "0"}, {"feedback_offset", "inf"},
       {"feedback_mapping_verified", "yes"}, {"speed_erpm", "0"},
       {"acceleration_raw", "-1"}, {"position_min_rad", "2"},
       {"position_max_rad", "-2"}, {"position_max_error_rad", "0"}}) {
    auto info = valid_info();
    info.joints[0].parameters[entry.first] = entry.second;
    EXPECT_FALSE(Params::parse(info)) << entry.first;
  }
  auto info = valid_info();
  info.joints[0].parameters["extra"] = "1";
  EXPECT_FALSE(Params::parse(info));
}

TEST(Ak30ServoRuntimeParams, RejectsNonDecimalOrWhitespacePrefixedNumbers) {
  auto info = valid_info();
  info.joints[0].parameters["target_scale"] = "\r2";
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.hardware_parameters["logical_bus"] = " 42";
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.joints[0].parameters["target_scale"] = "0x1p0";
  EXPECT_FALSE(Params::parse(info));
}

TEST(Ak30ServoRuntimeParams, RejectsDuplicateIdsNamesAndMoreThanSixJoints) {
  auto info = valid_info();
  info.joints[1].parameters["drive_id"] = "104";
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.joints[1].name = "left";
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  for (int id = 106; id <= 110; ++id)
    info.joints.push_back(joint("extra" + std::to_string(id), std::to_string(id)));
  EXPECT_FALSE(Params::parse(info));
}

TEST(Ak30ServoRuntimeParams, RejectsInvalidInterfaceShapesAndAncillaryComponents) {
  auto info = valid_info();
  info.joints[0].state_interfaces.push_back(interface(hardware_interface::HW_IF_VELOCITY));
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.joints[0].command_interfaces.push_back(interface(hardware_interface::HW_IF_EFFORT));
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.joints[0].command_interfaces.push_back(interface(hardware_interface::HW_IF_POSITION));
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.joints[0].command_interfaces.clear();
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.sensors.push_back(hardware_interface::ComponentInfo{});
  EXPECT_FALSE(Params::parse(info));
}

TEST(Ak30ServoRuntimeParams, RejectsHardTtlBeyondThreeCycles) {
  auto info = valid_info();
  info.hardware_parameters["control_period_ns"] = "1000000";
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.hardware_parameters["command_ttl_ns"] = "6000001";
  EXPECT_FALSE(Params::parse(info));
  info = valid_info();
  info.hardware_parameters["command_ttl_ns"] = "6000000";
  EXPECT_FALSE(Params::parse(info));
}

}  // namespace
