#include "mech_bringup/ak30_servo_system.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_simulation/fake_serial.hpp"

namespace {
using mech::mech_bringup::Ak30ServoSystem;
using mech::mech_simulation::FakeSerial;
using hardware_interface::CallbackReturn;

hardware_interface::InterfaceInfo interface(const std::string& name) {
  hardware_interface::InterfaceInfo value;
  value.name = name;
  value.size = 1;
  return value;
}

hardware_interface::HardwareInfo info() {
  hardware_interface::HardwareInfo result;
  result.name = "synthetic_servo_pair";
  result.type = "system";
  result.hardware_class_type = "mech_bringup/Ak30ServoSystem";
  result.hardware_parameters = {
      {"profile", "ak30_servo_extended"},
      {"device_path", "/dev/ttySYNTHETIC"},
      {"logical_bus", "42"},
      {"control_period_ns", "2000000"},
      {"command_ttl_ns", "3000000"},
      {"command_hard_ttl_ns", "6000000"},
      {"feedback_ttl_ns", "60000000"}};
  for (const auto& [name, id] :
       {std::pair<const char*, const char*>{"left", "104"}, {"right", "105"}}) {
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";
    joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
    joint.command_interfaces = {
        interface(hardware_interface::HW_IF_POSITION),
        interface(mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
    joint.parameters = {
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
        {"position_max_error_rad", "0.25"}};
    result.joints.push_back(std::move(joint));
  }
  result.joints[1].parameters["target_scale"] = "-3.0";
  result.joints[1].parameters["target_offset"] = "1.0";
  result.joints[1].parameters["feedback_scale"] = "0.25";
  result.joints[1].parameters["feedback_offset"] = "0.5";
  return result;
}

std::vector<std::uint8_t> feedback_wire(std::uint8_t id,
                                        std::uint8_t status = 0U) {
  using namespace mech::mech_control_core;
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  payload[6] = 43U;
  payload[7] = status;
  const auto frame = RawCanFrame::create(
      42U, *CanId::create(0x2900U | id, CanFrameFormat::Extended),
      CanFrameType::Classic, FrameDirection::Tx, 8U, payload,
      *MonotonicTime::from_nanoseconds(1000));
  std::array<std::uint8_t, 528U> bytes{};
  std::size_t length = 0;
  EXPECT_TRUE(UsbCdcCodec::encode(*frame, bytes, length));
  return {bytes.begin(), bytes.begin() + length};
}

void expect_mode6_packet(const std::vector<std::uint8_t>& bytes,
                         std::size_t offset, std::uint8_t drive,
                         std::array<std::uint8_t, 4U> position) {
  ASSERT_GE(bytes.size(), offset + 21U);
  EXPECT_EQ(bytes[offset], 0xF7U);
  EXPECT_EQ(bytes[offset + 1U], 0x12U);
  EXPECT_EQ(bytes[offset + 2U], 14U);
  EXPECT_EQ(bytes[offset + 7U], drive);
  EXPECT_EQ(bytes[offset + 8U], 0x06U);
  EXPECT_EQ(bytes[offset + 11U], 0x0CU);
  EXPECT_EQ(bytes[offset + 12U], 8U);
  EXPECT_TRUE(std::equal(position.begin(), position.end(),
                         bytes.begin() + offset + 13U));
  EXPECT_EQ(bytes[offset + 17U], 0U);
  EXPECT_EQ(bytes[offset + 18U], 100U);  // 1000 eRPM / 10
  EXPECT_EQ(bytes[offset + 19U], 0U);
  EXPECT_EQ(bytes[offset + 20U], 50U);   // raw acceleration 500 / 10
}

TEST(Ak30ServoSystemPlugin, InvalidDeclarationNeverConstructsSerial) {
  Ak30ServoSystem plugin;
  unsigned requests = 0;
  plugin.set_serial_port_factory_for_testing(
      [&requests](const std::string&) {
        ++requests;
        return std::make_shared<FakeSerial>();
      });
  auto bad = info();
  bad.joints[1].parameters["drive_id"] = "104";
  EXPECT_EQ(plugin.on_init(bad), CallbackReturn::ERROR);
  EXPECT_EQ(requests, 0U);
}

TEST(Ak30ServoSystemPlugin, ConfigureDoesNotOpenAndEachActivationInitializesGateway) {
  auto serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem plugin;
  unsigned requests = 0;
  plugin.set_serial_port_factory_for_testing(
      [&requests, serial](const std::string& path) {
        EXPECT_EQ(path, "/dev/ttySYNTHETIC");
        ++requests;
        return serial;
      });
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(requests, 1U);
  EXPECT_FALSE(serial->is_open());
  EXPECT_TRUE(serial->take_tx().empty());
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  EXPECT_TRUE(serial->is_open());
  EXPECT_EQ(plugin.pass_through_frames(), 1U);
  EXPECT_EQ(serial->take_tx().size(), 13U);
  ASSERT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
  EXPECT_FALSE(serial->is_open());
  serial->clear_tx();
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(plugin.pass_through_frames(), 2U);
  EXPECT_EQ(serial->take_tx().size(), 13U);
  ASSERT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_cleanup(state), CallbackReturn::SUCCESS);
  EXPECT_FALSE(serial->is_open());
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  serial->clear_tx();
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(plugin.pass_through_frames(), 3U);
  EXPECT_EQ(serial->take_tx().size(), 13U);
  EXPECT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
}

TEST(Ak30ServoSystemPlugin, TwoIndependentClaimsProduceMode6AndReleaseIsLocal) {
  auto serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem plugin;
  plugin.set_serial_port_factory_for_testing(
      [serial](const std::string&) { return serial; });
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  auto states = plugin.export_state_interfaces();
  auto commands = plugin.export_command_interfaces();
  ASSERT_EQ(states.size(), 2U);
  ASSERT_EQ(commands.size(), 4U);
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  serial->clear_tx();
  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};
  EXPECT_EQ(plugin.prepare_command_mode_switch({"left/position"}, {}),
            hardware_interface::return_type::ERROR);
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 1.0);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 0.5);
  ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  EXPECT_TRUE(serial->take_tx().empty());

  const std::vector<std::string> left{"left/position"};
  const std::vector<std::string> right{"right/position",
      "right/command_generation"};
  ASSERT_EQ(plugin.prepare_command_mode_switch(left, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.perform_command_mode_switch(left, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.prepare_command_mode_switch(right, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.perform_command_mode_switch(right, {}),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(commands[0].get_value(), 1.0);  // weak measured seed
  commands[0].set_value(1.125);
  commands[2].set_value(0.625);
  commands[3].set_value(1.0);
  ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  const auto tx = serial->take_tx();
  ASSERT_EQ(tx.size(), 42U);
  // 1.125 rad -> -2.75 deg; 0.625 rad -> -0.875 deg.
  expect_mode6_packet(tx, 0U, 104U, {0xFF, 0xFF, 0x94, 0x94});
  expect_mode6_packet(tx, 21U, 105U, {0xFF, 0xFF, 0xDD, 0xD2});
  EXPECT_EQ(plugin.motor_command_frames(), 2U);
  EXPECT_EQ(plugin.pass_through_frames(), 1U);

  ASSERT_EQ(plugin.prepare_command_mode_switch({}, right),
            hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.perform_command_mode_switch({}, right),
            hardware_interface::return_type::OK);
  serial->clear_tx();
  ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  const auto after_release = serial->take_tx();
  ASSERT_EQ(after_release.size(), 21U);
  expect_mode6_packet(after_release, 0U, 104U,
                      {0xFF, 0xFF, 0x94, 0x94});
  ASSERT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
  serial->clear_tx();
  EXPECT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  EXPECT_TRUE(serial->take_tx().empty());
}

TEST(Ak30ServoSystemPlugin, GatewayInitFailureClosesAndReleasesBus) {
  mech::mech_control_core::BusOwnershipRegistry registry;
  auto first_serial = std::make_shared<FakeSerial>();
  auto second_serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem first;
  Ak30ServoSystem second;
  first.set_ownership_registry_for_testing(registry);
  second.set_ownership_registry_for_testing(registry);
  first.set_serial_port_factory_for_testing(
      [first_serial](const std::string&) { return first_serial; });
  second.set_serial_port_factory_for_testing(
      [second_serial](const std::string&) { return second_serial; });
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(first.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(first.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_configure(state), CallbackReturn::SUCCESS);
  first_serial->force_next_write(
      mech::mech_control_core::TransportResult::Disconnected);
  EXPECT_EQ(first.on_activate(state), CallbackReturn::ERROR);
  EXPECT_FALSE(first_serial->is_open());
  EXPECT_EQ(first.pass_through_frames(), 0U);
  EXPECT_FALSE(registry.owns("/dev/ttySYNTHETIC"));
  ASSERT_EQ(second.on_activate(state), CallbackReturn::SUCCESS);
  EXPECT_TRUE(second_serial->is_open());
  EXPECT_EQ(second.on_deactivate(state), CallbackReturn::SUCCESS);
  EXPECT_FALSE(second_serial->is_open());
}

TEST(Ak30ServoSystemPlugin, SharedRegistryRefusesSecondPhysicalOpen) {
  mech::mech_control_core::BusOwnershipRegistry registry;
  auto first_serial = std::make_shared<FakeSerial>();
  auto second_serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem first;
  Ak30ServoSystem second;
  first.set_ownership_registry_for_testing(registry);
  second.set_ownership_registry_for_testing(registry);
  first.set_serial_port_factory_for_testing(
      [first_serial](const std::string&) { return first_serial; });
  second.set_serial_port_factory_for_testing(
      [second_serial](const std::string&) { return second_serial; });
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(first.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(first.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(second.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(first.on_activate(state), CallbackReturn::SUCCESS);
  EXPECT_EQ(second.on_activate(state), CallbackReturn::ERROR);
  EXPECT_FALSE(second_serial->is_open());
  EXPECT_TRUE(second_serial->take_tx().empty());
  EXPECT_TRUE(registry.owns("/dev/ttySYNTHETIC"));
  EXPECT_EQ(first.on_deactivate(state), CallbackReturn::SUCCESS);
  EXPECT_FALSE(registry.owns("/dev/ttySYNTHETIC"));
}

TEST(Ak30ServoSystemPlugin, ActiveDestructorReleasesSerialAndRegistry) {
  mech::mech_control_core::BusOwnershipRegistry registry;
  auto serial = std::make_shared<FakeSerial>();
  {
    auto plugin = std::make_unique<Ak30ServoSystem>();
    plugin->set_ownership_registry_for_testing(registry);
    plugin->set_serial_port_factory_for_testing(
        [serial](const std::string&) { return serial; });
    const rclcpp_lifecycle::State state;
    ASSERT_EQ(plugin->on_init(info()), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin->on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin->on_activate(state), CallbackReturn::SUCCESS);
    EXPECT_TRUE(serial->is_open());
  }
  EXPECT_FALSE(serial->is_open());
  EXPECT_FALSE(registry.owns("/dev/ttySYNTHETIC"));
}

TEST(Ak30ServoSystemPlugin, FeedbackFaultCancelsBothPendingTargetsAndRecoveryNeedsFreshClaim) {
  auto serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem plugin;
  plugin.set_serial_port_factory_for_testing(
      [serial](const std::string&) { return serial; });
  const rclcpp_lifecycle::State state;
  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};
  ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  serial->clear_tx();
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  const std::vector<std::string> claims{"left/position", "right/position"};
  ASSERT_EQ(plugin.prepare_command_mode_switch(claims, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.perform_command_mode_switch(claims, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U, 1U)));
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
  EXPECT_EQ(plugin.read(time, period), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(serial->take_tx().empty());
  ASSERT_EQ(plugin.on_error(state), CallbackReturn::SUCCESS);
  EXPECT_FALSE(serial->is_open());
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  serial->clear_tx();
  EXPECT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
  EXPECT_TRUE(serial->take_tx().empty());
  EXPECT_EQ(plugin.prepare_command_mode_switch(claims, {}),
            hardware_interface::return_type::ERROR);
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(plugin.prepare_command_mode_switch(claims, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
}
}  // namespace

TEST(Ak30ServoSystemDiagnostics, ReceiveOnlyIncludesTemperatureAndNeverTransmitsMotorFrames) {
  auto serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem plugin;
  plugin.set_serial_port_factory_for_testing([serial](const std::string&) { return serial; });
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  EXPECT_FALSE(plugin.diagnostic_snapshot(0).has_value());
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  serial->clear_tx();
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
  ASSERT_EQ(plugin.read(rclcpp::Time(0), rclcpp::Duration(std::chrono::milliseconds(2))),
            hardware_interface::return_type::OK);
  const auto sample = plugin.diagnostic_snapshot(0);
  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->temperature_c, 43.0);
  EXPECT_DOUBLE_EQ(sample->feedback_position_deg, 0.0);
  EXPECT_EQ(sample->sequence, 1U);
  EXPECT_EQ(plugin.motor_command_frames(), 0U);
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U, 1U)));
  EXPECT_EQ(plugin.read(rclcpp::Time(0), rclcpp::Duration(std::chrono::milliseconds(2))),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(plugin.motor_command_frames(), 0U);
  EXPECT_EQ(plugin.on_error(state), CallbackReturn::SUCCESS);
  EXPECT_TRUE(serial->take_tx().empty());
}
