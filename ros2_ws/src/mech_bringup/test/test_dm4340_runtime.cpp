#include "mech_bringup/dm4340_runtime.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_simulation/fake_transport.hpp"
#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

namespace {
using namespace mech::mech_bringup;
using namespace mech::mech_control_core;
using namespace mech::mech_hardware_ros2_control;
using mech::mech_simulation::FakeTransport;

MonotonicTime at(std::int64_t ns) {
  return *MonotonicTime::from_nanoseconds(ns);
}

hardware_interface::InterfaceInfo interface(const std::string& name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  result.size = 1;
  return result;
}

Dm4340RuntimeConfig config() {
  Dm4340RuntimeConfig c{};
  c.physical_bus = "fake-dm4340";
  c.logical_bus = 1U;
  c.control_period_ns = 2000000;
  c.command_hard_ttl_ns = 6000000;
  c.query_period_ns = 20000000;
  c.motor_side_timeout_verified = true;
  c.joints.emplace_back();
  auto& s = c.joints.back();
  s.profile = ProtocolProfile::Dm4340MitStandard;
  s.logical_bus = 1U;
  s.command_can_id = 2U;
  s.feedback_can_id = 0U;
  s.motor_id = 2U;
  s.firmware_id = 5010107U;
  s.firmware_min = 5010107U;
  s.firmware_max = 5010107U;
  s.firmware_evidence_declared = true;
  s.limits = mech::mech_protocol_damiao::kSdkDm4340Limits;
  s.limits_evidence_declared = true;
  s.joint = {1.0, 0.0, true};
  s.position_min_rad = -12.5;
  s.position_max_rad = 12.5;
  s.max_target_error_rad = 0.3;
  s.kp = 0.0;
  s.kd = 1.0;
  s.max_command_velocity_rad_s = 1.5;
  s.max_feedback_velocity_rad_s = 2.0;
  s.max_feedback_torque_nm = 5.0;
  s.command_ttl_ns = 4000000;
  s.feedback_ttl_ns = 40000000;
  return c;
}

RawCanFrame feedback(std::int64_t ns, std::uint8_t status = 1U,
                     std::uint8_t motor_id = 2U) {
  std::array<std::uint8_t, kMaxCanPayloadBytes> bytes{};
  bytes[0] = static_cast<std::uint8_t>((status << 4U) | motor_id);
  bytes[1] = 0x7FU;
  bytes[2] = 0xFFU;
  bytes[3] = 0x7FU;
  bytes[4] = 0xF7U;
  bytes[5] = 0xFFU;
  return *RawCanFrame::create(
      1U, *CanId::create(0U, CanFrameFormat::Standard),
      CanFrameType::Classic, FrameDirection::Rx, 8U, bytes, at(ns));
}

RawCanFrame other_motor_feedback(std::int64_t ns) {
  return feedback(ns, 1U, 3U);
}

struct Fixture final {
  FakeTransport transport{32U};
  BusOwnershipRegistry ownership;
  std::int64_t now{0};
  Dm4340Runtime runtime{
      transport, [this] { return at(now); }, config(), ownership};
  CanonicalState state{};

  void start() {
    ASSERT_TRUE(runtime.configure(1U));
    ASSERT_TRUE(runtime.start());
  }
  void enabled_feedback() {
    now = 1000000;
    ASSERT_EQ(transport.inject_receive(feedback(now)), TransportResult::Ok);
    ASSERT_TRUE(runtime.read(&state, 1U));
    ASSERT_TRUE(runtime.has_valid_sample());
    RawCanFrame query{};
    ASSERT_TRUE(transport.take_transmit(query));
    EXPECT_EQ(query.id.value, 0x7FFU);
    EXPECT_FALSE(transport.take_transmit(query));
  }
  void send_velocity() {
    CommandDispatch command{};
    command.authorized = true;
    command.fresh = true;
    command.command.velocity = 0.8;
    ASSERT_TRUE(runtime.write(&command, 1U));
    ASSERT_TRUE(runtime.read(&state, 1U));
  }
};

TEST(Dm4340Runtime, RefusesMissingMotorWatchdogEvidence) {
  FakeTransport transport;
  BusOwnershipRegistry ownership;
  auto c = config();
  c.motor_side_timeout_verified = false;
  Dm4340Runtime runtime{
      transport, [] { return at(0); }, c, ownership};
  EXPECT_FALSE(runtime.configure(1U));
  EXPECT_FALSE(transport.is_open());
}

TEST(Dm4340Runtime, SupervisedBenchOnlyAllowsEffortMode) {
  FakeTransport transport;
  BusOwnershipRegistry ownership;
  auto c = config();
  c.motor_side_timeout_verified = false;
  c.supervised_effort_bench = true;
  Dm4340Runtime velocity{
      transport, [] { return at(0); }, c, ownership};
  EXPECT_FALSE(velocity.configure(1U));
  c.command_mode = Dm4340CommandMode::Effort;
  c.joints[0].kd = 0.0;
  c.joints[0].max_command_velocity_rad_s = 0.0;
  c.joints[0].max_command_effort_nm = 0.1;
  Dm4340Runtime effort{
      transport, [] { return at(0); }, c, ownership};
  EXPECT_TRUE(effort.configure(1U));
  EXPECT_FALSE(transport.is_open());
}

TEST(Dm4340Runtime, EffortModeUsesSameBusAndRejectsVelocity) {
  FakeTransport transport{32U};
  BusOwnershipRegistry ownership;
  std::int64_t now = 0;
  auto c = config();
  c.command_mode = Dm4340CommandMode::Effort;
  c.joints[0].kd = 0.0;
  c.joints[0].max_command_velocity_rad_s = 0.0;
  c.joints[0].max_command_effort_nm = 0.3;
  Dm4340Runtime runtime{
      transport, [&] { return at(now); }, c, ownership};
  ASSERT_TRUE(runtime.configure(1U));
  ASSERT_TRUE(runtime.start());
  CanonicalState state{};
  now = 1000000;
  ASSERT_EQ(transport.inject_receive(feedback(now)), TransportResult::Ok);
  ASSERT_TRUE(runtime.read(&state, 1U));
  RawCanFrame sent{};
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x7FFU);
  CommandDispatch effort{};
  effort.authorized = true;
  effort.fresh = true;
  effort.command.effort = -0.2;
  ASSERT_TRUE(runtime.write(&effort, 1U));
  ASSERT_TRUE(runtime.read(&state, 1U));
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  std::array<std::uint8_t, 8> expected{};
  ASSERT_TRUE(mech::mech_protocol_damiao::encode_mit(
      {0.0, 0.0, 0.0, 0.0, -0.2}, c.joints[0].limits, expected));
  for (std::size_t i = 0; i < expected.size(); ++i)
    EXPECT_EQ(sent.payload[i], expected[i]);
  effort.command.velocity = 0.1;
  EXPECT_FALSE(runtime.write(&effort, 1U));
  EXPECT_EQ(runtime.fault_reason(), Dm4340FaultReason::CommandRejected);
}

TEST(Dm4340Runtime, RejectsControlCycleNotShorterThanLease) {
  FakeTransport transport;
  BusOwnershipRegistry ownership;
  auto c = config();
  c.control_period_ns = c.joints[0].command_ttl_ns;
  Dm4340Runtime runtime{
      transport, [] { return at(0); }, c, ownership};
  EXPECT_FALSE(runtime.configure(1U));
  EXPECT_FALSE(transport.is_open());
}

TEST(Dm4340Runtime, NoFeedbackNeverAuthorizesMotorCommand) {
  Fixture f;
  f.start();
  ASSERT_TRUE(f.runtime.read(&f.state, 1U));
  EXPECT_FALSE(f.runtime.has_valid_sample());
  CommandDispatch command{};
  command.authorized = true;
  command.fresh = true;
  command.command.velocity = 0.8;
  EXPECT_FALSE(f.runtime.write(&command, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::CommandRejected);
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x7FFU);
  EXPECT_FALSE(f.transport.take_transmit(sent));
}

TEST(Dm4340Runtime, DisabledFeedbackNeverAuthorizesMotorCommand) {
  Fixture f;
  f.start();
  f.now = 1000000;
  ASSERT_EQ(f.transport.inject_receive(feedback(f.now, 0U)),
            TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(&f.state, 1U));
  EXPECT_FALSE(f.runtime.has_valid_sample());
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x7FFU);
  CommandDispatch command{};
  command.authorized = true;
  command.fresh = true;
  command.command.velocity = 0.8;
  EXPECT_FALSE(f.runtime.write(&command, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::CommandRejected);
  EXPECT_FALSE(f.transport.take_transmit(sent));
}

TEST(Dm4340Runtime, PreservesFirstRejectedFeedbackStatus) {
  Fixture f;
  f.start();
  f.now = 1000000;
  ASSERT_EQ(f.transport.inject_receive(feedback(f.now, 9U)),
            TransportResult::Ok);
  EXPECT_FALSE(f.runtime.read(&f.state, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::FeedbackRejected);
  EXPECT_EQ(f.runtime.rejected_status(), 9U);
  EXPECT_FALSE(f.runtime.read(&f.state, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::FeedbackRejected);
}

TEST(Dm4340Runtime, DisabledFeedbackTriggersOneExplicitEnable) {
  Fixture f;
  f.start();
  EXPECT_FALSE(f.runtime.adopt_or_enable_from_feedback());
  f.now = 1000000;
  ASSERT_EQ(f.transport.inject_receive(feedback(f.now, 0U)),
            TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(&f.state, 1U));
  ASSERT_TRUE(f.runtime.has_fresh_ready_feedback());
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x7FFU);
  ASSERT_TRUE(f.runtime.adopt_or_enable_from_feedback());
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  EXPECT_EQ(sent.payload[7], 0xFCU);
  EXPECT_FALSE(f.transport.take_transmit(sent));
  EXPECT_FALSE(f.runtime.adopt_or_enable_from_feedback());

  f.now = 2000000;
  ASSERT_EQ(f.transport.inject_receive(feedback(f.now, 1U)),
            TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(&f.state, 1U));
  EXPECT_TRUE(f.runtime.has_valid_sample());
  f.runtime.stop();
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  EXPECT_EQ(sent.payload[7], 0xFDU);
  EXPECT_FALSE(f.transport.is_open());
}

TEST(Dm4340Runtime, EnabledFeedbackIsAdoptedWithoutEnableFrame) {
  Fixture f;
  f.start();
  f.enabled_feedback();
  ASSERT_TRUE(f.runtime.has_fresh_ready_feedback());
  ASSERT_TRUE(f.runtime.adopt_or_enable_from_feedback());
  RawCanFrame sent{};
  EXPECT_FALSE(f.transport.take_transmit(sent));
  EXPECT_FALSE(f.runtime.adopt_or_enable_from_feedback());
  f.runtime.stop();
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  EXPECT_EQ(sent.payload[7], 0xFDU);
}

TEST(Dm4340Runtime, CompositeSystemClaimsAndReleasesVelocity) {
  FakeTransport transport{32U};
  BusOwnershipRegistry ownership;
  std::int64_t now = 0;
  CompositeSystem system;
  ASSERT_TRUE(system.set_runtime(std::make_unique<Dm4340Runtime>(
      transport, [&now] { return at(now); }, config(), ownership)));

  hardware_interface::HardwareInfo info;
  info.name = "dm4340_integration";
  info.type = "system";
  info.hardware_class_type = "mech_hardware_ros2_control/CompositeSystem";
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
  info.joints.push_back(std::move(joint));
  const rclcpp_lifecycle::State lifecycle{
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test"};
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(lifecycle), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(commands.size(), 2U);
  ASSERT_EQ(system.on_activate(lifecycle), hardware_interface::CallbackReturn::SUCCESS);

  now = 1000000;
  ASSERT_EQ(transport.inject_receive(feedback(now)), TransportResult::Ok);
  const rclcpp::Time time(0);
  const rclcpp::Duration period{std::chrono::nanoseconds(2000000)};
  ASSERT_EQ(system.read(time, period), hardware_interface::return_type::OK);
  RawCanFrame sent{};
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x7FFU);

  const std::vector<std::string> claim{
      "knee_joint/velocity", "knee_joint/command_generation"};
  ASSERT_EQ(system.prepare_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch(claim, {}),
            hardware_interface::return_type::OK);
  now = 2000000;
  commands[0].set_value(0.8);
  commands[1].set_value(1.0);
  ASSERT_EQ(system.write(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(system.read(time, period), hardware_interface::return_type::OK);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  EXPECT_FALSE(transport.take_transmit(sent));

  now = 3000000;
  ASSERT_EQ(system.write(time, period), hardware_interface::return_type::OK);
  ASSERT_EQ(system.read(time, period), hardware_interface::return_type::OK);
  EXPECT_FALSE(transport.take_transmit(sent));

  ASSERT_EQ(system.prepare_command_mode_switch({}, claim),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch({}, claim),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[7], 0xFDU);
  EXPECT_FALSE(transport.take_transmit(sent));
  ASSERT_EQ(system.on_deactivate(lifecycle), hardware_interface::CallbackReturn::SUCCESS);
}

TEST(Dm4340Runtime, OtherMotorOnSharedFeedbackIdIsIgnored) {
  Fixture f;
  f.start();
  f.now = 1000000;
  ASSERT_EQ(f.transport.inject_receive(other_motor_feedback(f.now)),
            TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(&f.state, 1U));
  EXPECT_FALSE(f.runtime.has_valid_sample());
  ASSERT_EQ(f.transport.inject_receive(feedback(f.now + 1)),
            TransportResult::Ok);
  ++f.now;
  ASSERT_TRUE(f.runtime.read(&f.state, 1U));
  EXPECT_TRUE(f.runtime.has_valid_sample());
}

TEST(Dm4340Runtime, TwoMotorsShareOneBusWithoutCrossCommand) {
  auto c = config();
  auto second = c.joints[0];
  second.command_can_id = 3U;
  second.motor_id = 3U;
  c.joints.push_back(second);
  FakeTransport transport{32U};
  BusOwnershipRegistry ownership;
  std::int64_t now = 0;
  Dm4340Runtime runtime{
      transport, [&now] { return at(now); }, c, ownership};
  ASSERT_TRUE(runtime.configure(2U));
  ASSERT_TRUE(runtime.start());
  CanonicalState states[2]{};
  ASSERT_TRUE(runtime.read(states, 2U));
  RawCanFrame sent{};
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[0], 2U);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[0], 3U);
  now = 1000000;
  ASSERT_EQ(transport.inject_receive(feedback(now, 1U, 3U)),
            TransportResult::Ok);
  ASSERT_EQ(transport.inject_receive(feedback(now + 1, 1U, 2U)),
            TransportResult::Ok);
  ++now;
  ASSERT_TRUE(runtime.read(states, 2U));
  ASSERT_TRUE(runtime.has_valid_sample());

  CommandDispatch commands[2]{};
  commands[1].authorized = true;
  commands[1].fresh = true;
  commands[1].command.velocity = 0.8;
  ASSERT_TRUE(runtime.write(commands, 2U));
  ASSERT_TRUE(runtime.read(states, 2U));
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 3U);
  EXPECT_FALSE(transport.take_transmit(sent));

  runtime.cancel_pending(1U);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 3U);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 3U);
  EXPECT_EQ(sent.payload[7], 0xFDU);
  EXPECT_FALSE(transport.take_transmit(sent));

  commands[0].authorized = true;
  commands[0].fresh = true;
  commands[0].command.velocity = 0.4;
  commands[1] = {};
  ASSERT_TRUE(runtime.write(commands, 2U));
  ASSERT_TRUE(runtime.read(states, 2U));
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  runtime.stop();
}

TEST(Dm4340Runtime, FaultOnOneMotorHaltsBothOnSharedBus) {
  auto c = config();
  auto second = c.joints[0];
  second.command_can_id = 3U;
  second.motor_id = 3U;
  c.joints.push_back(second);
  FakeTransport transport{32U};
  BusOwnershipRegistry ownership;
  std::int64_t now = 0;
  Dm4340Runtime runtime{
      transport, [&now] { return at(now); }, c, ownership};
  ASSERT_TRUE(runtime.configure(2U));
  ASSERT_TRUE(runtime.start());
  CanonicalState states[2]{};
  ASSERT_TRUE(runtime.read(states, 2U));
  RawCanFrame sent{};
  ASSERT_TRUE(transport.take_transmit(sent));
  ASSERT_TRUE(transport.take_transmit(sent));
  now = 1000000;
  ASSERT_EQ(transport.inject_receive(feedback(now, 1U, 2U)),
            TransportResult::Ok);
  ASSERT_EQ(transport.inject_receive(feedback(now + 1, 1U, 3U)),
            TransportResult::Ok);
  ++now;
  ASSERT_TRUE(runtime.read(states, 2U));
  CommandDispatch commands[2]{};
  for (auto& command : commands) {
    command.authorized = true;
    command.fresh = true;
    command.command.velocity = 0.8;
  }
  ASSERT_TRUE(runtime.write(commands, 2U));
  ASSERT_TRUE(runtime.read(states, 2U));
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 3U);

  now = 2000000;
  ASSERT_EQ(transport.inject_receive(feedback(now, 9U, 3U)),
            TransportResult::Ok);
  EXPECT_FALSE(runtime.read(states, 2U));
  for (const std::uint32_t id : {2U, 3U}) {
    ASSERT_TRUE(transport.take_transmit(sent));
    EXPECT_EQ(sent.id.value, id);
    ASSERT_TRUE(transport.take_transmit(sent));
    EXPECT_EQ(sent.id.value, id);
    EXPECT_EQ(sent.payload[7], 0xFDU);
  }
  EXPECT_FALSE(transport.take_transmit(sent));
  EXPECT_FALSE(runtime.has_valid_sample());
}

TEST(Dm4340Runtime, RejectsDuplicateMotorOrCommandId) {
  auto c = config();
  c.joints.push_back(c.joints[0]);
  c.joints[1].motor_id = 3U;
  FakeTransport transport;
  BusOwnershipRegistry ownership;
  Dm4340Runtime runtime{
      transport, [] { return at(0); }, c, ownership};
  EXPECT_FALSE(runtime.configure(2U));
  EXPECT_FALSE(transport.is_open());
}

TEST(Dm4340Runtime, ReadOnlyUntilClaimedFreshVelocity) {
  Fixture f;
  f.start();
  f.enabled_feedback();
  f.send_velocity();
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  EXPECT_EQ(sent.payload_size, 8U);
  EXPECT_FALSE(f.transport.take_transmit(sent));
}

TEST(Dm4340Runtime, HardDeadlineNeutralizesAndDisables) {
  Fixture f;
  f.start();
  f.enabled_feedback();
  f.send_velocity();
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  f.now = 7000000;
  EXPECT_FALSE(f.runtime.read(&f.state, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::CommandDeadline);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[7], 0xFDU);
  EXPECT_FALSE(f.runtime.has_valid_sample());
}

TEST(Dm4340Runtime, ClaimReleaseDisablesAndRequiresNewEpoch) {
  Fixture f;
  f.start();
  f.enabled_feedback();
  f.send_velocity();
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  f.runtime.cancel_pending(0U);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[7], 0xFDU);
  EXPECT_FALSE(f.runtime.has_valid_sample());
  f.runtime.stop();
  EXPECT_FALSE(f.transport.is_open());
}

TEST(Dm4340Runtime, RetriesFailedDisableBeforeClosingPort) {
  Fixture f;
  f.start();
  f.enabled_feedback();
  f.send_velocity();
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  f.transport.force_next_send_results(
      {TransportResult::Ok, TransportResult::QueueFull});
  f.runtime.cancel_pending(0U);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[7], 0xFDU);
  EXPECT_FALSE(f.transport.take_transmit(sent));
  EXPECT_FALSE(f.runtime.has_valid_sample());
  f.runtime.stop();
  EXPECT_FALSE(f.transport.take_transmit(sent));
  EXPECT_FALSE(f.transport.is_open());
}

TEST(Dm4340Runtime, FaultFeedbackStopsMotion) {
  Fixture f;
  f.start();
  f.enabled_feedback();
  f.send_velocity();
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  f.now = 2000000;
  ASSERT_EQ(f.transport.inject_receive(feedback(f.now, 9U)), TransportResult::Ok);
  EXPECT_FALSE(f.runtime.read(&f.state, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::FeedbackRejected);
  EXPECT_EQ(f.runtime.rejected_status(), 9U);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[7], 0xFDU);
}

TEST(Dm4340Runtime, StaleFeedbackStopsMotion) {
  Fixture f;
  f.start();
  f.enabled_feedback();
  f.send_velocity();
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  f.now = 42000000;
  EXPECT_FALSE(f.runtime.read(&f.state, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::FeedbackExpired);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x7FFU);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.payload[7], 0xFDU);
  EXPECT_FALSE(f.runtime.has_valid_sample());
}
}  // namespace
