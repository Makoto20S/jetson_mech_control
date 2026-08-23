#include "mech_hardware_ros2_control/composite_system.hpp"

#include <limits>
#include <memory>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <gtest/gtest.h>

namespace mech::mech_hardware_ros2_control {
namespace {

hardware_interface::HardwareInfo info(std::size_t joints = 2U) {
  hardware_interface::HardwareInfo result;
  result.name = "foundation_system";
  result.type = "system";
  result.hardware_class_type = "mech_hardware_ros2_control/CompositeSystem";
  const auto interface = [](const std::string& name) {
    hardware_interface::InterfaceInfo value;
    value.name = name;
    value.size = 1;
    return value;
  };
  for (std::size_t index = 0U; index < joints; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(index + 1U);
    joint.type = "joint";
    joint.command_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
    joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                              interface(hardware_interface::HW_IF_VELOCITY),
                              interface(hardware_interface::HW_IF_EFFORT)};
    result.joints.push_back(joint);
  }
  return result;
}

rclcpp_lifecycle::State state() {
  return rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test");
}

TEST(CompositeSystem, ExportsCanonicalInterfacesAndLoopsBackNonBlocking) {
  CompositeSystem system;
  ASSERT_EQ(system.on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(states.size(), 6U);
  ASSERT_EQ(commands.size(), 2U);
  EXPECT_EQ(commands[0].get_name(), "joint_1/position");
  EXPECT_EQ(states[2].get_name(), "joint_1/effort");
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.prepare_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  commands[0].set_value(0.25);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.01);
}

TEST(CompositeSystem, RejectsInvalidInterfacesAndStrictSwitchConflicts) {
  auto invalid = info(1U);
  invalid.joints[0].command_interfaces[0].name = hardware_interface::HW_IF_EFFORT;
  CompositeSystem rejected;
  EXPECT_EQ(rejected.on_init(invalid), hardware_interface::CallbackReturn::ERROR);

  CompositeSystem system;
  ASSERT_EQ(system.on_init(info(1U)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.prepare_command_mode_switch({"unknown/position"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.prepare_command_mode_switch({"joint_1/position"},
                                                {"joint_1/position"}),
            hardware_interface::return_type::ERROR);
  ASSERT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::ERROR);
}

TEST(CompositeSystem, RepeatsLifecycleAndLatchesInvalidCommandFault) {
  CompositeSystem system;
  ASSERT_EQ(system.on_init(info(1U)), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  for (int iteration = 0; iteration < 100; ++iteration) {
    ASSERT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(system.on_deactivate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_cleanup(state()),
              hardware_interface::CallbackReturn::SUCCESS);
  }
  ASSERT_EQ(system.on_configure(state()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(system.fault_latched());
}

}  // namespace
}  // namespace mech::mech_hardware_ros2_control
