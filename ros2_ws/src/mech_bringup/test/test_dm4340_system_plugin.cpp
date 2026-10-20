#include "mech_bringup/dm4340_system.hpp"

#include <memory>
#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_simulation/fake_serial.hpp"

namespace {
using mech::mech_bringup::Dm4340System;

class EnableReplySerial final : public mech::mech_control_core::CdcSerialPort {
 public:
  bool is_open() const noexcept override { return serial_.is_open(); }
  bool open() noexcept override { return serial_.open(); }
  void close() noexcept override { serial_.close(); }
  mech::mech_control_core::TransportResult read_some(
      std::uint8_t* data, std::size_t capacity,
      std::size_t& size) noexcept override {
    return serial_.read_some(data, capacity, size);
  }
  mech::mech_control_core::TransportResult write_all(
      const std::uint8_t* data, std::size_t size) noexcept override {
    const auto result = serial_.write_all(data, size);
    if (result != mech::mech_control_core::TransportResult::Ok || size != 21U)
      return result;
    if (data[7] == 0xFFU && data[8] == 0x07U && data[13] == 2U &&
        data[14] == 0U) {
      if (data[16] == 9U && (data[15] == 0x33U || data[15] == 0x55U))
        ++timeout_accesses;
      if (data[15] == 0xCCU) (void)inject_status(status);
      return result;
    }
    if (data[7] != 2U || data[8] != 0U || data[20] == 0U) return result;
    if (data[20] == 0xFCU) {
      ++enable_count;
      if (reply_to_enable) {
        status = 1U;
        (void)inject_status(status);
      }
    } else if (data[20] == 0xFDU) {
      ++disable_count;
      status = 0U;
    }
    return result;
  }

  bool inject_status(std::uint8_t status) {
    using namespace mech::mech_control_core;
    std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
    payload[0] = static_cast<std::uint8_t>((status << 4U) | 2U);
    payload[1] = 0x7FU;
    payload[2] = 0xFFU;
    payload[3] = 0x7FU;
    payload[4] = 0xF7U;
    payload[5] = 0xFFU;
    const auto frame = RawCanFrame::create(
        1U, *CanId::create(0U, CanFrameFormat::Standard),
        CanFrameType::Classic, FrameDirection::Tx, 8U, payload,
        *MonotonicTime::from_nanoseconds(0));
    std::array<std::uint8_t, 528U> encoded{};
    std::size_t size = 0U;
    return frame && UsbCdcCodec::encode(*frame, encoded, size) &&
           serial_.inject_rx({encoded.begin(), encoded.begin() + size});
  }

  int enable_count{0};
  int disable_count{0};
  int timeout_accesses{0};
  std::uint8_t status{0};
  bool reply_to_enable{true};

 private:
  mech::mech_simulation::FakeSerial serial_;
};

hardware_interface::InterfaceInfo interface(const std::string& name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  result.size = 1;
  return result;
}

hardware_interface::HardwareInfo info() {
  hardware_interface::HardwareInfo result;
  result.name = "dm4340_test";
  result.type = "system";
  result.hardware_class_type = "mech_bringup/Dm4340System";
  result.hardware_parameters = {
      {"profile", "dm4340_mit_standard"},
      {"device_path", "/dev/ttyACM1"},
      {"logical_bus", "1"},
      {"control_period_ns", "2000000"},
      {"command_ttl_ns", "4000000"},
      {"command_hard_ttl_ns", "6000000"},
      {"feedback_ttl_ns", "40000000"},
      {"query_period_ns", "20000000"},
      {"motor_side_timeout_verified", "true"},
      {"enable_on_activate", "false"},
      {"board_version_major", "4"},
      {"board_version_minor", "8"},
      {"board_version_patch", "8"},
      {"board_version_verified", "true"},
      {"can_bitrate_verified", "true"}};
  hardware_interface::ComponentInfo joint;
  joint.name = "knee_joint";
  joint.type = "joint";
  joint.command_interfaces = {
      interface(hardware_interface::HW_IF_VELOCITY),
      interface(mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
  joint.state_interfaces = {
      interface(hardware_interface::HW_IF_POSITION),
      interface(hardware_interface::HW_IF_VELOCITY),
      interface(hardware_interface::HW_IF_EFFORT)};
  joint.parameters = {
      {"command_can_id", "2"}, {"feedback_can_id", "0"},
      {"motor_id", "2"}, {"firmware_id", "5010107"},
      {"firmware_min", "5010107"}, {"firmware_max", "5010107"},
      {"firmware_evidence_declared", "true"},
      {"limit_position_rad", "12.5"},
      {"limit_velocity_rad_s", "10"}, {"limit_torque_nm", "28"},
      {"limits_evidence_declared", "true"},
      {"joint_scale", "1"}, {"joint_offset_rad", "0"},
      {"joint_mapping_verified", "true"},
      {"position_min_rad", "-12.5"}, {"position_max_rad", "12.5"},
      {"max_target_error_rad", "0.3"},
      {"kp", "0"}, {"kd", "1"},
      {"max_command_velocity_rad_s", "1.5"},
      {"max_feedback_velocity_rad_s", "2"},
      {"max_feedback_torque_nm", "5"},
      {"velocity_feedforward_nm", "0"}};
  result.joints.push_back(std::move(joint));
  return result;
}

TEST(Dm4340System, ConfigurationIsFailClosedBeforePortOpen) {
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  auto invalid = info();
  invalid.hardware_parameters["motor_side_timeout_verified"] = "false";
  EXPECT_EQ(system.on_init(invalid), hardware_interface::CallbackReturn::ERROR);
  EXPECT_FALSE(serial->is_open());
}

TEST(Dm4340System, InvalidEnableSettingFailsBeforePortOpen) {
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  auto invalid = info();
  invalid.hardware_parameters["enable_on_activate"] = "yes";
  EXPECT_EQ(system.on_init(invalid), hardware_interface::CallbackReturn::ERROR);
  EXPECT_FALSE(serial->is_open());
}

TEST(Dm4340System, ExplicitActivationWithoutFeedbackFailsClosed) {
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  auto declaration = info();
  declaration.hardware_parameters["enable_on_activate"] = "true";
  const rclcpp_lifecycle::State state{
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test"};
  ASSERT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::ERROR);
  EXPECT_FALSE(serial->is_open());
}

TEST(Dm4340System, ExplicitHardwareActivationEnablesAndDeactivationDisables) {
  auto serial = std::make_shared<EnableReplySerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  auto declaration = info();
  declaration.hardware_parameters["enable_on_activate"] = "true";
  const rclcpp_lifecycle::State state{
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test"};
  ASSERT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(serial->inject_status(0U));
  ASSERT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(serial->enable_count, 1);
  EXPECT_EQ(serial->timeout_accesses, 0);
  EXPECT_EQ(serial->disable_count, 0);
  ASSERT_EQ(system.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(serial->disable_count, 1);
  EXPECT_FALSE(serial->is_open());
}

TEST(Dm4340System, AlreadyEnabledHardwareIsAdoptedWithoutAnotherEnable) {
  auto serial = std::make_shared<EnableReplySerial>();
  serial->status = 1U;
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  auto declaration = info();
  declaration.hardware_parameters["enable_on_activate"] = "true";
  const rclcpp_lifecycle::State state{
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test"};
  ASSERT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(serial->inject_status(1U));
  ASSERT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(serial->enable_count, 0);
  EXPECT_EQ(serial->disable_count, 0);
  ASSERT_EQ(system.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(serial->disable_count, 1);
}

TEST(Dm4340System, FaultedHardwareIsNotAdoptedOrEnabled) {
  auto serial = std::make_shared<EnableReplySerial>();
  serial->status = 9U;
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  auto declaration = info();
  declaration.hardware_parameters["enable_on_activate"] = "true";
  const rclcpp_lifecycle::State state{
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test"};
  ASSERT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(serial->inject_status(9U));
  EXPECT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(serial->enable_count, 0);
  EXPECT_FALSE(serial->is_open());
}

TEST(Dm4340System, MissingEnableAcknowledgementAttemptsDisable) {
  auto serial = std::make_shared<EnableReplySerial>();
  serial->reply_to_enable = false;
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  auto declaration = info();
  declaration.hardware_parameters["enable_on_activate"] = "true";
  const rclcpp_lifecycle::State state{
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test"};
  ASSERT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(serial->inject_status(0U));
  EXPECT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(serial->enable_count, 1);
  EXPECT_GE(serial->disable_count, 1);
  EXPECT_FALSE(serial->is_open());
}

TEST(Dm4340System, ValidSyntheticConfigurationExportsCommonInterfaces) {
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  ASSERT_EQ(system.on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_command_interfaces().size(), 2U);
  EXPECT_EQ(system.export_state_interfaces().size(), 3U);
  EXPECT_FALSE(serial->is_open());
  EXPECT_EQ(system.prepare_command_mode_switch(
                {"knee_joint/velocity"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.prepare_command_mode_switch(
                {}, {"knee_joint/command_generation"}),
            hardware_interface::return_type::ERROR);
}

TEST(Dm4340System, EffortConfigurationExportsOnlyEffortAndGeneration) {
  auto declaration = info();
  declaration.joints[0].command_interfaces[0] =
      interface(hardware_interface::HW_IF_EFFORT);
  auto& params = declaration.joints[0].parameters;
  params["kd"] = "0";
  params["max_command_velocity_rad_s"] = "0";
  params["max_command_effort_nm"] = "0.3";
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  ASSERT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(commands.size(), 2U);
  EXPECT_EQ(commands[0].get_name(), "knee_joint/effort");
  EXPECT_EQ(commands[1].get_name(), "knee_joint/command_generation");
  EXPECT_FALSE(serial->is_open());
  EXPECT_EQ(system.prepare_command_mode_switch({"knee_joint/effort"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.prepare_command_mode_switch({"knee_joint/velocity",
                                                "knee_joint/command_generation"}, {}),
            hardware_interface::return_type::ERROR);
}

TEST(Dm4340System, SupervisedBenchAllowsUnverifiedTimeoutOnlyForBareEffortAxis) {
  auto declaration = info();
  declaration.hardware_parameters["profile"] =
      "dm4340_mit_supervised_effort_bench";
  declaration.hardware_parameters["motor_side_timeout_verified"] = "false";
  declaration.joints[0].command_interfaces[0] =
      interface(hardware_interface::HW_IF_EFFORT);
  auto& params = declaration.joints[0].parameters;
  params["joint_mapping_verified"] = "false";
  params["kd"] = "0";
  params["max_command_velocity_rad_s"] = "0";
  params["max_command_effort_nm"] = "0.1";
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  EXPECT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(serial->is_open());
  params["max_command_effort_nm"] = "0.3";
  Dm4340System too_much;
  EXPECT_EQ(too_much.on_init(declaration), hardware_interface::CallbackReturn::ERROR);
}

TEST(Dm4340System, TwoJointsExportIndependentInterfacesOnOnePort) {
  auto declaration = info();
  auto second = declaration.joints[0];
  second.name = "ankle_joint";
  second.parameters["command_can_id"] = "3";
  second.parameters["motor_id"] = "3";
  declaration.joints.push_back(std::move(second));
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  ASSERT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_command_interfaces().size(), 4U);
  EXPECT_EQ(system.export_state_interfaces().size(), 6U);
  EXPECT_FALSE(serial->is_open());
  EXPECT_EQ(system.prepare_command_mode_switch(
                {"knee_joint/velocity", "ankle_joint/velocity",
                 "ankle_joint/command_generation"}, {}),
            hardware_interface::return_type::ERROR);
}

TEST(Dm4340System, DuplicateMotorIdFailsBeforePortOpen) {
  auto declaration = info();
  auto second = declaration.joints[0];
  second.name = "ankle_joint";
  second.parameters["command_can_id"] = "3";
  declaration.joints.push_back(std::move(second));
  auto serial = std::make_shared<mech::mech_simulation::FakeSerial>();
  Dm4340System system;
  system.set_serial_factory_for_testing(
      [serial](const std::string&) { return serial; });
  EXPECT_EQ(system.on_init(declaration), hardware_interface::CallbackReturn::ERROR);
  EXPECT_FALSE(serial->is_open());
}
}  // namespace
