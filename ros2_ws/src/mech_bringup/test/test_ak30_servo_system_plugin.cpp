#include "mech_bringup/ak30_servo_system.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>
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
    const auto size = ::read(master, rx.data(), rx.size());
    ASSERT_EQ(size, 42);
    std::vector<std::uint8_t> received(rx.begin(), rx.begin() + size);
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
