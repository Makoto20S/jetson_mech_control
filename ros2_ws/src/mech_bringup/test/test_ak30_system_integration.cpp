// End-to-end offline integration of the AK3.0 force-control runtime with
// CompositeSystem: the SystemInterface drives Ak30ForceControlRuntime through
// the exported command/state interfaces, exactly the wiring a deployment
// will use, with FakeTransport standing in for the USB-CDC chain and a test
// clock replacing steady_clock. No test opens a serial device.

#include "mech_bringup/ak30_force_runtime.hpp"
#include "mech_bringup/ak30_runtime_params.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_control_core/frame.hpp"
#include "mech_control_core/time.hpp"
#include "mech_control_core/transport.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include "mech_simulation/fake_transport.hpp"

namespace mech::mech_bringup {
namespace {

using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::FrameDirection;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::TransportResult;
using mech::mech_hardware_ros2_control::CompositeSystem;
using mech::mech_simulation::FakeTransport;

constexpr std::uint32_t kDriveId = 104U;
constexpr std::uint32_t kLogicalBus = 1U;

[[nodiscard]] MonotonicTime at(std::int64_t nanoseconds) {
  return MonotonicTime::from_nanoseconds(nanoseconds).value();
}

class TestClock final {
 public:
  [[nodiscard]] MonotonicTime now() const noexcept { return at(now_ns_); }
  void set(std::int64_t nanoseconds) noexcept { now_ns_ = nanoseconds; }

 private:
  std::int64_t now_ns_{0};
};

[[nodiscard]] hardware_interface::InterfaceInfo interface(
    const std::string& name) {
  hardware_interface::InterfaceInfo value;
  value.name = name;
  value.size = 1;
  return value;
}

[[nodiscard]] rclcpp_lifecycle::State lifecycle_state() {
  return rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test");
}

// Feedback payload: 90.0 deg, 10000 ERPM, 2.0 A, 40 C, no fault.
[[nodiscard]] RawCanFrame feedback_frame(std::int64_t arrival_ns) {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03;
  payload[1] = 0x84;
  payload[2] = 0x03;
  payload[3] = 0xE8;
  payload[4] = 0x00;
  payload[5] = 0xC8;
  payload[6] = 0x28;
  payload[7] = 0x00;
  return RawCanFrame::create(
             kLogicalBus,
             CanId::create(
                 mech::mech_protocol_cubemars::feedback_can_id(kDriveId),
                 CanFrameFormat::Extended)
                 .value(),
             CanFrameType::Classic, FrameDirection::Rx, 8U, payload,
             at(arrival_ns))
      .value();
}

// Builds a CompositeSystem with the AK3.0 runtime injected, mirroring how a
// deployment would construct it (set_runtime before on_init).
class Ak30SystemTest : public ::testing::Test {
 protected:
  void SetUp() override {
    transport_ = std::make_unique<FakeTransport>(16U);
    Ak30RuntimeConfig config{};
    config.drive_id = kDriveId;
    config.logical_bus = kLogicalBus;
    config.gains.kp = 1.0;
    config.gains.kd = 1.0;
    config.control_period_nanoseconds = 2000000;
    ASSERT_TRUE(system_.set_runtime(std::make_unique<Ak30ForceControlRuntime>(
        *transport_, [this]() { return clock_.now(); }, config)));

    info_.name = "ak30_system";
    info_.type = "system";
    info_.hardware_class_type =
        "mech_hardware_ros2_control/CompositeSystem";
    hardware_interface::ComponentInfo joint;
    joint.name = "motor1_joint";
    joint.type = "joint";
    joint.command_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
    joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                              interface(hardware_interface::HW_IF_VELOCITY),
                              interface(hardware_interface::HW_IF_EFFORT)};
    info_.joints.push_back(std::move(joint));
    ASSERT_EQ(system_.on_init(info_),
              hardware_interface::CallbackReturn::SUCCESS);
  }

  std::unique_ptr<FakeTransport> transport_;
  TestClock clock_;
  CompositeSystem system_;
  hardware_interface::HardwareInfo info_;
};

TEST_F(Ak30SystemTest, ClaimWriteReadRoundTripsThroughTheForceRuntime) {
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

  // Cycle 1: read (nothing to submit yet), write stores the command.
  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};
  clock_.set(2000000);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  commands[0].set_value(0.25);
  EXPECT_EQ(system_.write(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  // Cycle 2: read submits the stored command onto the wire.
  clock_.set(4000000);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));
  EXPECT_EQ(sent.id.value,
            mech::mech_protocol_cubemars::force_control_can_id(kDriveId));

  // Feedback decodes into the exported state interfaces.
  ASSERT_EQ(transport_->inject_receive(feedback_frame(clock_.now().nanoseconds())),
            TransportResult::Ok);
  commands[0].set_value(0.25);
  EXPECT_EQ(system_.write(time, period), hardware_interface::return_type::OK);
  clock_.set(6000000);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  EXPECT_NEAR(states[0].get_value(),
              1.5707963267948966 - 5.760604931781636, 1e-6);
  EXPECT_NEAR(states[1].get_value(), 9.3499, 1e-3);
  EXPECT_NEAR(states[2].get_value(), 1.4764, 1e-3);

  ASSERT_EQ(system_.on_deactivate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system_.on_cleanup(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
}

// The staged watchdog surfaces through the SystemInterface: a controller
// that stops refreshing must produce ERROR (fault-latched) within the
// 3-cycle budget, and recovery is the lifecycle path (cleanup/configure/
// activate), exactly like the Foundation integration test.
TEST_F(Ak30SystemTest, StaleCommandFaultsTheSystemWithinThreeCycles) {
  ASSERT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system_.export_command_interfaces();
  ASSERT_EQ(system_.on_activate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  const std::vector<std::string> claim{"motor1_joint/position"};
  ASSERT_EQ(system_.prepare_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system_.perform_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);

  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};

  // t=2 ms: read submits nothing (no command yet); write stores one.
  clock_.set(2000000);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  commands[0].set_value(0.5);
  EXPECT_EQ(system_.write(time, period), hardware_interface::return_type::OK);

  // t=4 ms: the command is submitted (age 0 at submit).
  clock_.set(4000000);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));

  // The controller goes silent: no more set_value + write.
  // t=6 ms: Holding (age 2 ms < ttl 4 ms is Following; 2 ms from submit at
  // t=4 ms is age 2 ms -> Following). No re-submit of the stale command.
  clock_.set(6000000);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  // t=8.000001 ms: age 4,000,001 ns -> Holding. Still no fault.
  clock_.set(8000001);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);

  // t=10.000001 ms: age 6,000,001 ns -> Expired: read() ERRORs and the
  // fault latches. This is within 3 cycles (6 ms) of the last submit at
  // t=4 ms - the ADR-012 budget.
  clock_.set(10000001);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(system_.fault_latched());
}

TEST_F(Ak30SystemTest, RepeatsFullLifecycleOneHundredTimes) {
  for (int iteration = 0; iteration < 100; ++iteration) {
    FakeTransport transport{16U};
    TestClock clock;
    CompositeSystem system;
    Ak30RuntimeConfig config{};
    config.drive_id = kDriveId;
    config.logical_bus = kLogicalBus;
    config.gains.kp = 1.0;
    config.gains.kd = 1.0;
    config.control_period_nanoseconds = 2000000;
    ASSERT_TRUE(system.set_runtime(std::make_unique<Ak30ForceControlRuntime>(
        transport, [&clock]() { return clock.now(); }, config)));
    ASSERT_EQ(system.on_init(info_),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(lifecycle_state()),
              hardware_interface::CallbackReturn::SUCCESS);
    auto commands = system.export_command_interfaces();
    ASSERT_EQ(system.on_activate(lifecycle_state()),
              hardware_interface::CallbackReturn::SUCCESS);
    const std::vector<std::string> claim{"motor1_joint/position"};
    ASSERT_EQ(system.prepare_command_mode_switch(claim, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(system.perform_command_mode_switch(claim, {}),
              hardware_interface::return_type::OK);
    const rclcpp::Time time(0);
    const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};
    clock.set(2000000);
    ASSERT_EQ(system.read(time, period), hardware_interface::return_type::OK);
    commands[0].set_value(0.25);
    ASSERT_EQ(system.write(time, period),
              hardware_interface::return_type::OK);
    clock.set(4000000);
    ASSERT_EQ(system.read(time, period),
              hardware_interface::return_type::OK);
    ASSERT_EQ(transport.pending_transmit(), 1U);
    ASSERT_EQ(system.on_deactivate(lifecycle_state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_cleanup(lifecycle_state()),
              hardware_interface::CallbackReturn::SUCCESS);
  }
}

// Non-finite commands are rejected by CompositeSystem before they reach the
// runtime; the runtime-level parity test exists, this pins the composite
// path with the AK runtime attached.
TEST_F(Ak30SystemTest, NonFiniteCommandIsRejectedWithRuntimeAttached) {
  ASSERT_EQ(system_.on_configure(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system_.export_command_interfaces();
  ASSERT_EQ(system_.on_activate(lifecycle_state()),
            hardware_interface::CallbackReturn::SUCCESS);
  const std::vector<std::string> claim{"motor1_joint/position"};
  ASSERT_EQ(system_.prepare_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system_.perform_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);

  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};
  clock_.set(2000000);
  EXPECT_EQ(system_.read(time, period), hardware_interface::return_type::OK);
  commands[0].set_value(std::nan(""));
  EXPECT_EQ(system_.write(time, period), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(system_.fault_latched());
}

}  // namespace
}  // namespace mech::mech_bringup
