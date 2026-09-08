// Offline lifecycle tests for the Ak30System composition plugin: the
// production composition point between the launch path and the real
// serial/CAN chain. FakeSerial stands in for /dev/ttyACM*, so every test
// drives the plugin through the real UsbCdcCodec framing path and none ever
// opens a serial device.
//
// What is pinned here:
// - the URDF parameters that on_init parses (fail-closed on unknown keys
//   and a missing device_path);
// - the exact 0x12 pass-through init frame sent at on_configure, asserted
//   byte-for-byte against the bench-proven golden (never recomputed);
// - the full lifecycle round-trip with a feedback frame injected through
//   the codec, decoding into the exported state interfaces;
// - the default factory's fail-closed behaviour on a nonexistent device
//   (PosixCdcSerialPort::open returns false; on_configure must ERROR, not
//   hang).

#include "mech_bringup/ak30_system.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_bringup/pass_through_init.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include "mech_simulation/fake_serial.hpp"

namespace mech::mech_bringup {
namespace {

using mech::mech_control_core::UsbCdcCodec;
using mech::mech_simulation::FakeSerial;

constexpr std::uint32_t kDriveId = 104U;

// The bench-proven 0x12 pass-through init golden, asserted byte-for-byte
// against what the plugin sends. kPassThroughInitFrame is the shared
// production literal; this duplicate is deliberate - the test must fail if
// the shipped constant ever changes, and comparing a constant to itself
// would silently pass.
constexpr std::array<std::uint8_t, 13U> kPassThroughInitGolden{
    0xF7, 0x12, 0x06, 0x00, 0x7D, 0x70, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00};

[[nodiscard]] rclcpp_lifecycle::State lifecycle_state() {
  return rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test");
}

[[nodiscard]] hardware_interface::InterfaceInfo interface(
    const std::string& name) {
  hardware_interface::InterfaceInfo value;
  value.name = name;
  value.size = 1;
  return value;
}

// motor1's deployment-shaped HardwareInfo: the same URDF block the
// Position deployment xacro describes, including the bench-evidenced
// parameters (ADR-012 watchdog, zero offset, output-shaft positions).
[[nodiscard]] hardware_interface::HardwareInfo position_hardware_info(
    const std::map<std::string, std::string>& params) {
  hardware_interface::HardwareInfo info;
  info.name = "motor1_ak30_system";
  info.type = "system";
  info.hardware_class_type = "mech_bringup/Ak30System";
  info.hardware_parameters = {params.begin(), params.end()};
  hardware_interface::ComponentInfo joint;
  joint.name = "motor1_joint";
  joint.type = "joint";
  joint.command_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
  joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                            interface(hardware_interface::HW_IF_VELOCITY),
                            interface(hardware_interface::HW_IF_EFFORT)};
  info.joints.push_back(std::move(joint));
  return info;
}

[[nodiscard]] std::map<std::string, std::string> valid_params() {
  return {{"device_path", "/dev/ttyACM0"},
          {"logical_bus", "1"},
          {"drive_id", "104"},
          {"kp", "1.0"},
          {"kd", "1.0"},
          {"control_period_ns", "2000000"},
          {"command_ttl_ns", "4000000"},
          {"command_hard_ttl_ns", "6000000"},
          {"feedback_ttl_ns", "20000000"},
          {"zero_offset_rad", "5.760604931781636"},
          {"position_is_output_shaft", "true"}};
}

// Feedback payload 90.0 deg / 10000 ERPM / 2.0 A / 40 C / no fault - the
// same fixture payload as test_ak30_system_integration.cpp, wrapped in the
// on-wire CDC packet the 4.8.8 firmware actually emits (each 0x12 reply
// record prefixed with the CAN ID's low 24 bits, flags 0x0C, Classic DLC 8)
// so the frame travels the real UsbCdcCodec decode path. Both CRCs are
// recomputed exactly like the board does, mirroring the transport test's
// hand-built packet (test_transport_backends.cpp).
[[nodiscard]] std::vector<std::uint8_t> feedback_wire_bytes() {
  constexpr std::uint32_t kFeedbackId =
      mech::mech_protocol_cubemars::feedback_can_id(kDriveId);
  // 7-byte CDC header + 3-byte ID prefix + 14-byte record.
  std::vector<std::uint8_t> packet(7U + 17U, 0U);
  packet[0] = mech::mech_control_core::UsbCdcCodec::kHeader;
  packet[1] = mech::mech_control_core::UsbCdcCodec::kPassCommand;
  packet[2] = 17U;
  packet[3] = 0U;
  packet[4] = 0U;  // crc8, patched below
  packet[5] = 0U;  // crc16 low, patched below
  packet[6] = 0U;  // crc16 high, patched below
  std::size_t offset = 7U;
  // 24-bit ID prefix (LE).
  packet[offset] = static_cast<std::uint8_t>(kFeedbackId & 0xFFU);
  packet[offset + 1U] = static_cast<std::uint8_t>((kFeedbackId >> 8U) & 0xFFU);
  packet[offset + 2U] = static_cast<std::uint8_t>((kFeedbackId >> 16U) & 0xFFU);
  offset += 3U;
  // Record: id (LE u32), flags 0x0C (extended), dlc 8, payload 8.
  packet[offset] = static_cast<std::uint8_t>(kFeedbackId & 0xFFU);
  packet[offset + 1U] = static_cast<std::uint8_t>((kFeedbackId >> 8U) & 0xFFU);
  packet[offset + 2U] =
      static_cast<std::uint8_t>((kFeedbackId >> 16U) & 0xFFU);
  packet[offset + 3U] =
      static_cast<std::uint8_t>((kFeedbackId >> 24U) & 0xFFU);
  packet[offset + 4U] = 0x0CU;
  packet[offset + 5U] = 8U;
  const std::array<std::uint8_t, 8U> payload{0x03, 0x84, 0x03, 0xE8,
                                            0x00, 0xC8, 0x28, 0x00};
  std::copy(payload.begin(), payload.end(), packet.begin() + offset + 6U);

  std::uint8_t crc8 = 0xFFU;
  for (std::size_t index = 1U; index < 4U; ++index) {
    crc8 ^= packet[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc8 = (crc8 & 1U) != 0U
                 ? static_cast<std::uint8_t>((crc8 >> 1U) ^ 0x8CU)
                 : static_cast<std::uint8_t>(crc8 >> 1U);
    }
  }
  packet[4] = crc8;
  std::uint16_t crc16 = 0xFFFFU;
  for (std::size_t index = 7U; index < packet.size(); ++index) {
    crc16 ^= packet[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc16 = (crc16 & 1U) != 0U
                  ? static_cast<std::uint16_t>((crc16 >> 1U) ^ 0x8408U)
                  : static_cast<std::uint16_t>(crc16 >> 1U);
    }
  }
  packet[5] = static_cast<std::uint8_t>(crc16 & 0xFFU);
  packet[6] = static_cast<std::uint8_t>(crc16 >> 8U);
  return packet;
}

class Ak30SystemPluginTest : public ::testing::Test {
 protected:
  // Injects a factory producing one shared FakeSerial and asserts that the
  // plugin only ever asks for the device path its parameters named. shared_ptr
  // because the factory must be copy-constructible (std::function); the test
  // keeps its own reference for inject_rx/take_tx assertions.
  void use_fake_serial() {
    auto serial = std::make_shared<FakeSerial>(4096U);
    serial_ = serial.get();
    system_.set_serial_port_factory_for_testing(
        [owned = std::move(serial)](const std::string& device_path) {
          EXPECT_EQ(device_path, "/dev/ttyACM0");
          return std::shared_ptr<mech::mech_control_core::CdcSerialPort>(
              std::move(owned));
        });
  }

  FakeSerial* serial_{nullptr};
  Ak30System system_;
};

// The golden 0x12 init frame must be the first bytes on the wire after
// on_configure, byte-for-byte. This is the frame that arms pass-through
// mode on the board; a wrong CRC here would silently drop every reply.
TEST_F(Ak30SystemPluginTest, OnConfigureSendsTheGoldenPassThroughInit) {
  use_fake_serial();
  ASSERT_EQ(system_.on_init(position_hardware_info(valid_params())),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);

  const auto tx = serial_->take_tx();
  ASSERT_EQ(tx.size(), kPassThroughInitGolden.size());
  for (std::size_t index = 0U; index < tx.size(); ++index) {
    EXPECT_EQ(tx[index], kPassThroughInitGolden[index]) << "byte " << index;
  }
}

// The full production lifecycle through the plugin class, driving the real
// UsbCdcTransport: an injected vendor feedback record must decode through
// the codec into the exported state interfaces, and claim/switch must
// round-trip under the base's tested machinery.
TEST_F(Ak30SystemPluginTest, FullLifecycleRoundTripsFeedbackThroughCodec) {
  use_fake_serial();
  ASSERT_EQ(system_.on_init(position_hardware_info(valid_params())),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  auto states = system_.export_state_interfaces();
  auto commands = system_.export_command_interfaces();
  ASSERT_EQ(states.size(), 3U);
  ASSERT_EQ(commands.size(), 1U);
  ASSERT_EQ(system_.on_activate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);

  const std::vector<std::string> claim{"motor1_joint/position"};
  ASSERT_EQ(system_.prepare_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system_.perform_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);

  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};
  // read(): no feedback yet -> zeros (no sample, fail-safe publish).
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);

  // Inject one vendor feedback packet; the next read must decode it through
  // the real UsbCdcCodec path into the exported state interfaces.
  ASSERT_TRUE(serial_->inject_rx(feedback_wire_bytes()));
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  // 90 deg = 1.5708 rad output-shaft position; with the deployment's
  // zero_offset (5.7606 rad = 330.07 deg) and direction +1, the canonical
  // position is zero_offset - raw, matching the integration test's value.
  EXPECT_NEAR(states[0].get_value(),
              1.5707963267948966 - 5.760604931781636, 1e-6);

  ASSERT_EQ(system_.on_deactivate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_cleanup(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
}

// Deactivate -> activate re-entry must keep the serial port open and must
// NOT resend the 0x12 init (the port is already in pass-through mode; the
// frame is an init-time arm, not a per-activation requirement).
TEST_F(Ak30SystemPluginTest, ReactivationKeepsChannelAndDoesNotResendInit) {
  use_fake_serial();
  ASSERT_EQ(system_.on_init(position_hardware_info(valid_params())),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  serial_->clear_tx();
  ASSERT_EQ(system_.on_activate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_deactivate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_activate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(serial_->is_open());
  EXPECT_TRUE(serial_->take_tx().empty());
  // on_cleanup requires the INACTIVE state (the base refuses while active),
  // so deactivate first - the standard ros2_control teardown path.
  ASSERT_EQ(system_.on_deactivate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_cleanup(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(serial_->is_open());
}

// on_cleanup closes the port (the probe teardown order), so a second
// configure must reopen it and re-arm pass-through - full reconfiguration
// works offline, which is what a bench power-cycle recovery needs.
TEST_F(Ak30SystemPluginTest, ReconfigureAfterCleanupReopensAndReinits) {
  use_fake_serial();
  ASSERT_EQ(system_.on_init(position_hardware_info(valid_params())),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_activate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_deactivate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_cleanup(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  serial_->clear_tx();
  ASSERT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  const auto tx = serial_->take_tx();
  ASSERT_EQ(tx.size(), kPassThroughInitGolden.size());
  for (std::size_t index = 0U; index < tx.size(); ++index) {
    EXPECT_EQ(tx[index], kPassThroughInitGolden[index]) << "byte " << index;
  }
}

// Fail-closed parameter handling: an unknown key and a missing device_path
// both reject on_init with ERROR - the plugin must never fall back to
// guessed values.
TEST_F(Ak30SystemPluginTest, InvalidParametersRejectOnInit) {
  {
    auto params = valid_params();
    params["drive_idd"] = "104";  // typo'd key
    EXPECT_EQ(system_.on_init(position_hardware_info(params)),
              hardware_interface::CallbackReturn::ERROR);
  }
  {
    auto params = valid_params();
    params.erase("device_path");
    EXPECT_EQ(system_.on_init(position_hardware_info(params)),
              hardware_interface::CallbackReturn::ERROR);
  }
}

// The default factory builds a PosixCdcSerialPort; on_configure against a
// nonexistent device must fail closed (ERROR) rather than hang or pretend
// success. CI machines have no /dev/ttyACM0, which is exactly the point.
TEST_F(Ak30SystemPluginTest, DefaultFactoryFailsClosedOnMissingDevice) {
  auto params = valid_params();
  params["device_path"] = "/dev/nonexistent-mech-test-device";
  ASSERT_EQ(system_.on_init(position_hardware_info(params)),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::ERROR);
}

}  // namespace
}  // namespace mech::mech_bringup
