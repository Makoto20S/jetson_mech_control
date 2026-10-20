#include "mech_bringup/ak30_servo_system.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <cstdint>
#include <memory>
#include <optional>
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
                                        std::uint8_t status = 0U,
                                        std::int16_t position_tenths = 0) {
  using namespace mech::mech_control_core;
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  const auto position = static_cast<std::uint16_t>(position_tenths);
  payload[0] = static_cast<std::uint8_t>(position >> 8U);
  payload[1] = static_cast<std::uint8_t>(position);
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

TEST(Ak30ServoSystemPlugin, NamedCommandHandlesAndFeedbackCannotAliasOtherRoute) {
  for (const bool reverse_declaration : {false, true}) {
    SCOPED_TRACE(reverse_declaration ? "right-first hardware" : "left-first hardware");
    auto serial = std::make_shared<FakeSerial>();
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([serial](const std::string&) { return serial; });
    auto declaration = info();
    if (reverse_declaration)
      std::reverse(declaration.joints.begin(), declaration.joints.end());
    const rclcpp_lifecycle::State state;
    const rclcpp::Time time(0);
    const rclcpp::Duration period(0, 2000000);
    ASSERT_EQ(plugin.on_init(declaration), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
    auto commands = plugin.export_command_interfaces();
    auto states = plugin.export_state_interfaces();
    auto left = std::find_if(commands.begin(), commands.end(), [](const auto& c) {
      return c.get_name() == "left/position";
    });
    auto right = std::find_if(commands.begin(), commands.end(), [](const auto& c) {
      return c.get_name() == "right/position";
    });
    ASSERT_NE(left, commands.end());
    ASSERT_NE(right, commands.end());
    ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
    serial->clear_tx();
    ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
    ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
    ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(plugin.perform_command_mode_switch({"right/position", "left/position"}, {}),
              hardware_interface::return_type::OK);

    const std::array<std::array<double, 2>, 4> targets{{
        {1.125, 0.625}, {1.1875, 0.625}, {1.1875, 0.4375}, {1.0625, 0.5625}}};
    // Golden raw mode6 fields from the documented affine and 10000 scale;
    // no production codec is used to construct the oracle.
    const std::array<std::array<std::int32_t, 2>, 4> raw_targets{{
        {-27500, -8750}, {-26250, -8750}, {-26250, -3125}, {-28750, -6875}}};
    for (std::size_t stage = 0; stage < targets.size(); ++stage) {
      for (unsigned sample = 0; sample < 128U; ++sample) {
        left->set_value(targets[stage][0]);
        EXPECT_DOUBLE_EQ(right->get_value(), sample == 0U && stage == 0U
            ? 0.5 : 0.68);  // writing the left handle cannot alter the right handle
        right->set_value(targets[stage][1]);
        EXPECT_DOUBLE_EQ(left->get_value(), targets[stage][0]);
        ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
        // A dispatch is a value snapshot. Mutating the exported buffers after
        // write() must not alter the pending runtime command before read().
        left->set_value(1.18);
        right->set_value(0.68);
        // Non-constant fresh feedback arrives in changing ID order. read()
        // may update state but must never seed/rewrite an existing claim.
        const auto first = sample % 2U == 0U ? 104U : 105U;
        const auto second = first == 104U ? 105U : 104U;
        for (const auto id : {first, second}) {
          const auto tenths = static_cast<std::int16_t>(id == 104U
              ? sample % 3U : sample % 5U);
          ASSERT_TRUE(serial->inject_rx(feedback_wire(id, 0U, tenths)));
        }
        ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
        EXPECT_DOUBLE_EQ(left->get_value(), 1.18);
        EXPECT_DOUBLE_EQ(right->get_value(), 0.68);
        const auto bytes = serial->take_tx();
        serial->clear_tx();
        ASSERT_EQ(bytes.size(), 42U);
        std::array<bool, 2> seen{};
        for (std::size_t offset = 0; offset < bytes.size(); offset += 21U) {
          const auto id = bytes[offset + 7U];
          ASSERT_TRUE(id == 104U || id == 105U);
          const auto index = id == 104U ? 0U : 1U;
          ASSERT_FALSE(seen[index]);
          seen[index] = true;
          const auto raw = static_cast<std::uint32_t>(raw_targets[stage][index]);
          expect_mode6_packet(bytes, offset, id,
              {static_cast<std::uint8_t>(raw >> 24U),
               static_cast<std::uint8_t>(raw >> 16U),
               static_cast<std::uint8_t>(raw >> 8U), static_cast<std::uint8_t>(raw)});
        }
        for (const auto& s : states) {
          const auto expected = s.get_name() == "left/position"
              ? 1.0 + (sample % 3U) * 0.05 : 0.5 + (sample % 5U) * 0.025;
          EXPECT_NEAR(s.get_value(), expected, 1e-12);
        }
      }
    }
    ASSERT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
  }
}

TEST(Ak30ServoSystemPlugin, FeedbackAtPeerTargetCannotRewriteHeldCommand) {
  auto serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem plugin;
  plugin.set_serial_port_factory_for_testing([serial](const std::string&) { return serial; });
  auto declaration = info();
  for (auto& joint : declaration.joints) {
    joint.parameters["target_scale"] = "1.0";
    joint.parameters["target_offset"] = "0.0";
    joint.parameters["feedback_scale"] = "0.1";
    joint.parameters["feedback_offset"] = "0.0";
  }
  const rclcpp_lifecycle::State state;
  const rclcpp::Time time(0);
  const rclcpp::Duration period(0, 2000000);
  ASSERT_EQ(plugin.on_init(declaration), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  auto commands = plugin.export_command_interfaces();
  auto states = plugin.export_state_interfaces();
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  serial->clear_tx();
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U, 0U, 100)));
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U, 0U, 120)));
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(plugin.perform_command_mode_switch({"left/position", "right/position"}, {}),
            hardware_interface::return_type::OK);
  commands[0].set_value(1.1);
  commands[2].set_value(1.25);
  for (int repeat = 0; repeat < 20; ++repeat) {
    ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
    // The 105 feedback deliberately equals 104's canonical command. A
    // receive-path alias or read-time command seeding would now be exposed.
    ASSERT_TRUE(serial->inject_rx(feedback_wire(105U, 0U, 110)));
    ASSERT_TRUE(serial->inject_rx(feedback_wire(104U, 0U, 100)));
    ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
    EXPECT_NEAR(states[1].get_value(), 1.1, 1e-12);
    EXPECT_DOUBLE_EQ(commands[0].get_value(), 1.1);
    EXPECT_DOUBLE_EQ(commands[2].get_value(), 1.25);
    const auto bytes = serial->take_tx();
    serial->clear_tx();
    ASSERT_EQ(bytes.size(), 42U);
    const auto left_offset = bytes[7] == 104U ? 0U : 21U;
    expect_mode6_packet(bytes, left_offset, 104U, {0x00, 0x00, 0x2A, 0xF8});
    expect_mode6_packet(bytes, 21U - left_offset, 105U, {0x00, 0x00, 0x30, 0xD4});
  }
  ASSERT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
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

TEST(Ak30ServoSystemPlugin, InjectedClockAcceptsFeedbackAndStillEnforcesCommandExpiry) {
  using namespace mech::mech_control_core;
  using mech::mech_bringup::Ak30ServoFaultReason;
  // Both timestamp producers must observe this same externally owned clock.
  // Its epoch deliberately differs from the host steady clock.
  for (const auto elapsed : {3000000LL, 6000000LL}) {
    SCOPED_TRACE(elapsed);
    std::int64_t now_ns = 1000000000;
    auto serial = std::make_shared<FakeSerial>();
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing(
        [serial](const std::string&) { return serial; });
    plugin.set_clock_for_testing(
        [&now_ns] { return *MonotonicTime::from_nanoseconds(now_ns); });
    ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
    const rclcpp_lifecycle::State state;
    ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
    serial->clear_tx();
    const rclcpp::Time time(0);
    const rclcpp::Duration period(0, 2000000);
    ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
    ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
    ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
    for (std::size_t index = 0; index < 2U; ++index) {
      const auto feedback = plugin.diagnostic_snapshot(index);
      ASSERT_TRUE(feedback);
      EXPECT_EQ(feedback->availability,
                mech::mech_protocol_cubemars::ServoPositionAvailability::Fresh);
      ASSERT_TRUE(feedback->host_rx_time);
      EXPECT_EQ(feedback->host_rx_time->nanoseconds(), now_ns);
    }
    ASSERT_EQ(plugin.perform_command_mode_switch(
                  {"left/position", "right/position"}, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
    // Leave this generation unsent, then cross its real configured TTL in
    // logical time. Fresh feedback cannot revive the expired target.
    now_ns += elapsed;
    ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
    ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
    EXPECT_EQ(plugin.read(time, period), hardware_interface::return_type::ERROR);
    ASSERT_TRUE(plugin.first_fault());
    EXPECT_EQ(plugin.first_fault()->reason,
              elapsed == 3000000LL ? Ak30ServoFaultReason::UnsentSoftDeadline
                                   : Ak30ServoFaultReason::HardDeadline);
    EXPECT_TRUE(serial->take_tx().empty());
  }
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

TEST(Ak30ServoSystemDiagnostics, ReportsFirstCauseOnceAndRetainsItThroughOnError) {
  auto serial = std::make_shared<FakeSerial>();
  Ak30ServoSystem plugin;
  plugin.set_serial_port_factory_for_testing([serial](const std::string&) { return serial; });
  const rclcpp_lifecycle::State state;
  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::milliseconds(2)};
  ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  ASSERT_TRUE(serial->inject_rx(feedback_wire(104U)));
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U)));
  ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
  ASSERT_TRUE(serial->inject_rx(feedback_wire(105U, 1U)));
  testing::internal::CaptureStderr();
  EXPECT_EQ(plugin.read(time, period), hardware_interface::return_type::ERROR);
  EXPECT_EQ(plugin.read(time, period), hardware_interface::return_type::ERROR);
  EXPECT_EQ(plugin.on_error(state), CallbackReturn::SUCCESS);
  const auto log = testing::internal::GetCapturedStderr();
  const auto first = log.find("AK30 servo first fault");
  ASSERT_NE(first, std::string::npos) << log;
  EXPECT_EQ(log.find("AK30 servo first fault", first + 1U), std::string::npos);
  EXPECT_NE(log.find("reason=FeedbackRejected phase=ReceiveObserver index=1 drive_id=105"), std::string::npos);
  ASSERT_TRUE(plugin.first_fault());
  EXPECT_EQ(plugin.first_fault()->drive_id, 105U);
  ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
  EXPECT_FALSE(plugin.first_fault());
}

TEST(Ak30ServoSystemPlugin, OptionalSerialTraceFlushesAfterShutdown) {
  char path[] = "/tmp/servo-trace-test-XXXXXX";
  const auto fd = mkstemp(path);
  ASSERT_GE(fd, 0);
  close(fd);
  unlink(path);
  const auto prior = std::getenv("MECH_SERVO_TRACE_PATH");
  const std::string old = prior ? prior : "";
  const bool had_prior = prior != nullptr;
  auto serial = std::make_shared<FakeSerial>();
  {
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([serial](const std::string&) { return serial; });
    setenv("MECH_SERVO_TRACE_PATH", path, 1);
    const std::string chain_path = std::string(path) + ".chain";
    setenv("MECH_SERVO_CHAIN_PATH", chain_path.c_str(), 1);
    const auto initialized = plugin.on_init(info());
    unsetenv("MECH_SERVO_CHAIN_PATH");
    if (had_prior) setenv("MECH_SERVO_TRACE_PATH", old.c_str(), 1);
    else unsetenv("MECH_SERVO_TRACE_PATH");
    ASSERT_EQ(initialized, CallbackReturn::SUCCESS);
    const rclcpp_lifecycle::State state;
    ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
    ASSERT_TRUE(serial->inject_rx(feedback_wire(104)));
    ASSERT_TRUE(serial->inject_rx(feedback_wire(105)));
    ASSERT_EQ(plugin.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
              hardware_interface::return_type::OK);
    auto commands = plugin.export_command_interfaces();
    ASSERT_EQ(plugin.perform_command_mode_switch({"left/position", "right/position"}, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(plugin.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
              hardware_interface::return_type::OK);
    ASSERT_EQ(plugin.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
              hardware_interface::return_type::OK);
    EXPECT_FALSE(std::ifstream(path).good());
  }
  EXPECT_FALSE(serial->is_open());
  std::ifstream input(path);
  std::ostringstream output; output << input.rdbuf();
  EXPECT_NE(output.str().find("\"direction\":\"tx\""), std::string::npos);
  EXPECT_NE(output.str().find("\"direction\":\"rx\""), std::string::npos);
  std::ifstream chain(std::string(path) + ".chain");
  std::ostringstream records; records << chain.rdbuf();
  for (const auto* stage : {"claim_after", "interface", "dispatch", "stored",
                             "prepared", "selected", "transport_send", "serial_request"})
    EXPECT_NE(records.str().find(std::string("\"stage\":\"") + stage + "\""), std::string::npos) << stage;
  EXPECT_NE(records.str().find("\"hex\":\"ffff8ad000640032\""), std::string::npos);
  unlink((std::string(path) + ".chain").c_str());
  unlink(path);
}

namespace {
class TraceEnvironment final {
 public:
  TraceEnvironment() {
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto value = ::getenv(names[i]);
      if (value) prior[i] = value;
      ::unsetenv(names[i]);
    }
  }
  ~TraceEnvironment() {
    for (std::size_t i = 0; i < names.size(); ++i) {
      if (prior[i]) ::setenv(names[i], prior[i]->c_str(), 1);
      else ::unsetenv(names[i]);
    }
  }
 private:
  const std::array<const char*, 5> names{{"MECH_SERVO_SNAPSHOT_DIR", "MECH_SERVO_CHAIN_CAPACITY",
      "MECH_SERVO_RAW_CAPACITY", "MECH_SERVO_CHAIN_PATH", "MECH_SERVO_TRACE_PATH"}};
  std::array<std::optional<std::string>, 5> prior;
};
}

TEST(Ak30ServoSystemPlugin, TraceAdmissionFailsBeforeSerialFactoryOrDeviceOpen) {
  for (const auto bad : {"", "0", "-1", "garbage", "1.5", "1000000000"}) {
    SCOPED_TRACE(bad);
    TraceEnvironment environment;
    ::setenv("MECH_SERVO_SNAPSHOT_DIR", "/dev/shm", 1);
    ::setenv("MECH_SERVO_CHAIN_CAPACITY", bad, 1);
    ::setenv("MECH_SERVO_RAW_CAPACITY", "2", 1);
    bool factory_called = false;
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([&](const std::string&) {
      factory_called = true; return std::make_shared<FakeSerial>();
    });
    EXPECT_EQ(plugin.on_init(info()), CallbackReturn::ERROR);
    EXPECT_FALSE(factory_called);
  }
  for (const auto directory : {"", "/tmp", "/dev/shm/no-such-servo-directory"}) {
    TraceEnvironment environment;
    ::setenv("MECH_SERVO_SNAPSHOT_DIR", directory, 1);
    ::setenv("MECH_SERVO_CHAIN_CAPACITY", "2", 1);
    ::setenv("MECH_SERVO_RAW_CAPACITY", "2", 1);
    bool factory_called = false;
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([&](const std::string&) {
      factory_called = true; return std::make_shared<FakeSerial>();
    });
    EXPECT_EQ(plugin.on_init(info()), CallbackReturn::ERROR);
    EXPECT_FALSE(factory_called);
  }
  TraceEnvironment environment;
  ::setenv("MECH_SERVO_CHAIN_CAPACITY", "2", 1);
  bool factory_called = false;
  Ak30ServoSystem plugin;
  plugin.set_serial_port_factory_for_testing([&](const std::string&) {
    factory_called = true; return std::make_shared<FakeSerial>();
  });
  EXPECT_EQ(plugin.on_init(info()), CallbackReturn::ERROR);
  EXPECT_FALSE(factory_called);
}

TEST(Ak30ServoSystemPlugin, SnapshotShutdownSealsWithoutLegacyJsonDump) {
  TraceEnvironment environment;
  char directory[] = "/dev/shm/mech-plugin-trace-XXXXXX";
  ASSERT_NE(::mkdtemp(directory), nullptr);
  const std::string chain = std::string(directory) + "/chain.snapshot";
  const std::string raw = std::string(directory) + "/serial.snapshot";
  const std::string legacy = std::string(directory) + "/unexpected.jsonl";
  ::setenv("MECH_SERVO_SNAPSHOT_DIR", directory, 1);
  ::setenv("MECH_SERVO_CHAIN_CAPACITY", "32", 1);
  ::setenv("MECH_SERVO_RAW_CAPACITY", "8", 1);
  ::setenv("MECH_SERVO_CHAIN_PATH", legacy.c_str(), 1);
  ::setenv("MECH_SERVO_TRACE_PATH", legacy.c_str(), 1);
  auto serial = std::make_shared<FakeSerial>();
  {
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([&](const std::string&) {
      // Both complete mappings already exist before constructing the port.
      EXPECT_TRUE(std::ifstream(chain).good()); EXPECT_TRUE(std::ifstream(raw).good());
      return serial;
    });
    ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  }
  EXPECT_FALSE(serial->is_open());
  EXPECT_FALSE(std::ifstream(legacy).good());
  for (const auto& path : {chain, raw}) {
    mech::mech_bringup::TraceSnapshotHeader h{};
    std::ifstream input(path, std::ios::binary);
    input.read(reinterpret_cast<char*>(&h), sizeof(h));
    ASSERT_TRUE(input.good());
    EXPECT_EQ(std::string(h.magic, 8), "MCHTRC01"); EXPECT_EQ(h.closed, 1U);
    ::unlink(path.c_str());
  }
  ::rmdir(directory);
}

TEST(Ak30ServoSystemPlugin, TerminalShutdownSealsBeforeDestructionButDeactivateDoesNot) {
  TraceEnvironment environment;
  char directory[] = "/dev/shm/mech-plugin-terminal-trace-XXXXXX";
  ASSERT_NE(::mkdtemp(directory), nullptr);
  ::setenv("MECH_SERVO_SNAPSHOT_DIR", directory, 1);
  ::setenv("MECH_SERVO_CHAIN_CAPACITY", "64", 1);
  ::setenv("MECH_SERVO_RAW_CAPACITY", "16", 1);
  auto serial = std::make_shared<FakeSerial>();
  const auto headers = [&](std::uint64_t expected) {
    for (const auto* name : {"chain.snapshot", "serial.snapshot"}) {
      mech::mech_bringup::TraceSnapshotHeader header{};
      std::ifstream input(std::string(directory) + "/" + name, std::ios::binary);
      input.read(reinterpret_cast<char*>(&header), sizeof(header));
      EXPECT_TRUE(input.good());
      EXPECT_EQ(header.closed, expected);
    }
  };
  {
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([&](const std::string&) { return serial; });
    const rclcpp_lifecycle::State state;
    ASSERT_EQ(plugin.on_init(info()), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
    headers(0);
    ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
    headers(0);
    ASSERT_EQ(plugin.on_shutdown(state), CallbackReturn::SUCCESS);
    EXPECT_FALSE(serial->is_open());
    headers(1);  // Must be published while the plugin object still exists.
  }
  headers(1);
  ::unlink((std::string(directory) + "/chain.snapshot").c_str());
  ::unlink((std::string(directory) + "/serial.snapshot").c_str());
  ::rmdir(directory);
}

TEST(Ak30ServoSystemPlugin, FailedSnapshotInitializationNeverClaimsClosedEvidence) {
  TraceEnvironment environment;
  char directory[] = "/dev/shm/mech-plugin-failed-trace-XXXXXX";
  ASSERT_NE(::mkdtemp(directory), nullptr);
  ::setenv("MECH_SERVO_SNAPSHOT_DIR", directory, 1);
  ::setenv("MECH_SERVO_CHAIN_CAPACITY", "2", 1);
  ::setenv("MECH_SERVO_RAW_CAPACITY", "2", 1);
  {
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([](const std::string&) {
      return std::shared_ptr<mech::mech_control_core::CdcSerialPort>{};
    });
    ASSERT_EQ(plugin.on_init(info()), CallbackReturn::ERROR);
  }
  for (const auto* name : {"chain.snapshot", "serial.snapshot"}) {
    const auto path = std::string(directory) + "/" + name;
    mech::mech_bringup::TraceSnapshotHeader h{};
    std::ifstream input(path, std::ios::binary);
    input.read(reinterpret_cast<char*>(&h), sizeof(h));
    ASSERT_TRUE(input.good()); EXPECT_EQ(h.closed, 0U);
    ::unlink(path.c_str());
  }
  ::rmdir(directory);
}

#include <fcntl.h>
#include <sys/wait.h>
#include "mech_bringup/posix_cdc_serial_port.hpp"

TEST(Ak30ServoSystemPlugin, FullChainAuditMatchesRealPtyBytesAndRejectsMutation) {
  const int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  ASSERT_GE(master, 0);
  ASSERT_EQ(grantpt(master), 0);
  ASSERT_EQ(unlockpt(master), 0);
  std::array<char, 256> name{};
  ASSERT_EQ(ptsname_r(master, name.data(), name.size()), 0);
  char directory[] = "/tmp/servo-chain-integration-XXXXXX";
  ASSERT_NE(mkdtemp(directory), nullptr);
  const std::string path = std::string(directory) + "/chain.jsonl";
  {
    Ak30ServoSystem plugin;
    plugin.set_serial_port_factory_for_testing([&](const std::string&) {
      return std::make_shared<mech::mech_bringup::PosixCdcSerialPort>(name.data());
    });
    setenv("MECH_SERVO_CHAIN_PATH", path.c_str(), 1);
    const auto initialized = plugin.on_init(info());
    unsetenv("MECH_SERVO_CHAIN_PATH");
    ASSERT_EQ(initialized, CallbackReturn::SUCCESS);
    const rclcpp_lifecycle::State state;
    ASSERT_EQ(plugin.on_configure(state), CallbackReturn::SUCCESS);
    ASSERT_EQ(plugin.on_activate(state), CallbackReturn::SUCCESS);
    std::array<std::uint8_t, 128> rx{};
    ASSERT_EQ(::read(master, rx.data(), rx.size()), 13);
    for (const auto id : {104, 105}) {
      const auto bytes = feedback_wire(id);
      ASSERT_EQ(::write(master, bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
    }
    const rclcpp::Time time(0);
    const rclcpp::Duration period(0, 2000000);
    // PTY delivery is asynchronous; no command is claimed during this wait.
    for (int i = 0; i < 100; ++i) {
      ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
      auto sample = plugin.diagnostic_snapshot(1);
      if (sample && sample->availability == mech::mech_protocol_cubemars::ServoPositionAvailability::Fresh) break;
      usleep(1000);
    }
    ASSERT_EQ(plugin.perform_command_mode_switch({"left/position", "right/position"}, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(plugin.write(time, period), hardware_interface::return_type::OK);
    ASSERT_EQ(plugin.read(time, period), hardware_interface::return_type::OK);
    // PTY is a nonblocking byte stream: one read can return only the first
    // of the two already-written frames. Accumulate with a bounded wait while
    // keeping the exact total and golden byte assertions below.
    std::vector<std::uint8_t> received;
    for (int guard = 0; guard < 100 && received.size() < 42U; ++guard) {
      const auto size = ::read(master, rx.data(), rx.size());
      if (size > 0) {
        received.insert(received.end(), rx.begin(), rx.begin() + size);
      } else if (size < 0) {
        ASSERT_TRUE(errno == EAGAIN || errno == EWOULDBLOCK);
      }
      if (received.size() < 42U) usleep(1000);
    }
    ASSERT_EQ(received.size(), 42U);
    const auto left_offset = received[7] == 104 ? 0U : 21U;
    expect_mode6_packet(received, left_offset, 104, {0xff, 0xff, 0x8a, 0xd0});
    expect_mode6_packet(received, 21U - left_offset, 105, {0xff, 0xff, 0xec, 0x78});
    EXPECT_EQ(plugin.on_deactivate(state), CallbackReturn::SUCCESS);
  }
  ::close(master);
  const std::string audit = std::string(MECH_BRINGUP_SOURCE_DIR) + "/../../../tools/servo/chain_audit.py";
  auto run_audit = [&] {
    const auto pid = fork();
    if (pid == 0) {
      execl("/usr/bin/python3", "python3", audit.c_str(), path.c_str(), "--hardware-only", static_cast<char*>(nullptr));
      _exit(127);
    }
    int status = 0;
    if (pid < 0 || waitpid(pid, &status, 0) != pid || !WIFEXITED(status)) return -1;
    return WEXITSTATUS(status);
  };
  EXPECT_EQ(run_audit(), 0);
  // Mutate a real captured boundary; production codec is not used as oracle.
  std::ifstream input(path);
  std::string contents((std::istreambuf_iterator<char>(input)), {});
  const auto at = contents.find("ffff8ad000640032");
  ASSERT_NE(at, std::string::npos);
  contents.replace(at, 16, "0000000000640032");
  { std::ofstream output(path); output << contents; }
  EXPECT_EQ(run_audit(), 1);
  unlink(path.c_str());
  rmdir(directory);
}
