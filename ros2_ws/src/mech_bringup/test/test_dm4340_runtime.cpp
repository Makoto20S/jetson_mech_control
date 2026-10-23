#include "mech_bringup/dm4340_runtime.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
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
                     std::uint8_t motor_id = 2U, std::uint32_t can_id = 0U) {
  std::array<std::uint8_t, kMaxCanPayloadBytes> bytes{};
  bytes[0] = static_cast<std::uint8_t>((status << 4U) | motor_id);
  bytes[1] = 0x7FU;
  bytes[2] = 0xFFU;
  bytes[3] = 0x7FU;
  bytes[4] = 0xF7U;
  bytes[5] = 0xFFU;
  return *RawCanFrame::create(
      1U, *CanId::create(can_id, CanFrameFormat::Standard),
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
  ASSERT_TRUE(f.runtime.feedback_diagnostic());
  EXPECT_EQ(f.runtime.feedback_diagnostic()->joint_index, 0U);
  EXPECT_EQ(f.runtime.feedback_diagnostic()->rejected.reason,
            mech::mech_protocol_damiao::DmFeedbackRejection::DeviceStatus);
  EXPECT_TRUE(f.runtime.feedback_diagnostic()->rejected.decoded);
  f.runtime.stop();
  ASSERT_TRUE(f.runtime.feedback_diagnostic()); // Survives session cleanup.

  EXPECT_FALSE(f.runtime.read(&f.state, 1U));
  EXPECT_EQ(f.runtime.fault_reason(), Dm4340FaultReason::FeedbackRejected);
}

TEST(Dm4340Runtime, DisabledFeedbackTriggersOneExplicitEnable) {
  Fixture f;
  f.start();
  EXPECT_FALSE(f.runtime.fresh_feedback_status(0U));
  EXPECT_FALSE(f.runtime.fresh_feedback_status(1U));
  EXPECT_FALSE(f.runtime.adopt_or_enable_from_feedback());
  f.now = 1000000;
  ASSERT_EQ(f.transport.inject_receive(feedback(f.now, 0U)),
            TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(&f.state, 1U));
  ASSERT_TRUE(f.runtime.has_fresh_ready_feedback());
  EXPECT_EQ(f.runtime.fresh_feedback_status(0U), 0U);
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
  EXPECT_EQ(f.runtime.fresh_feedback_status(0U), 1U);
  f.runtime.stop();
  EXPECT_FALSE(f.runtime.fresh_feedback_status(0U));
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

TEST(Dm4340Runtime, HistoricalDualIdsRequireBothCanAndPayloadRouting) {
  auto c = config();
  c.joints[0].command_can_id = 1U;
  c.joints[0].motor_id = 1U;
  c.joints[0].feedback_can_id = 4U;
  auto second = c.joints[0];
  second.command_can_id = 2U;
  second.motor_id = 2U;
  second.feedback_can_id = 5U;
  c.joints.push_back(second);
  FakeTransport transport{32U};
  BusOwnershipRegistry ownership;
  std::int64_t now = 0;
  Dm4340Runtime runtime{transport, [&now] { return at(now); }, c, ownership};
  ASSERT_TRUE(runtime.configure(2U));
  ASSERT_TRUE(runtime.start());
  CanonicalState states[2]{};
  ASSERT_TRUE(runtime.read(states, 2U));
  RawCanFrame sent{};
  while (transport.take_transmit(sent)) {}
  now = 1000000;
  // Correct CAN ID with the wrong embedded motor ID cannot make either axis fresh.
  ASSERT_EQ(transport.inject_receive(feedback(now, 1U, 2U, 4U)), TransportResult::Ok);
  ASSERT_TRUE(runtime.read(states, 2U));
  EXPECT_FALSE(runtime.has_valid_sample());
  ++now;
  ASSERT_EQ(transport.inject_receive(feedback(now, 1U, 2U, 5U)), TransportResult::Ok);
  ++now;
  ASSERT_EQ(transport.inject_receive(feedback(now, 1U, 1U, 4U)), TransportResult::Ok);
  ASSERT_TRUE(runtime.read(states, 2U));
  ASSERT_TRUE(runtime.has_valid_sample());
  CommandDispatch commands[2]{};
  commands[1].authorized = commands[1].fresh = true;
  commands[1].command.velocity = -0.2;
  ASSERT_TRUE(runtime.write(commands, 2U));
  ASSERT_TRUE(runtime.read(states, 2U));
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 2U);
  EXPECT_FALSE(transport.take_transmit(sent));
  runtime.stop();
}

TEST(Dm4340Runtime, StaggersDualQueriesWithoutRelaxingFeedbackDeadline) {
  auto c = config();
  c.stagger_status_queries = true;
  auto second = c.joints[0]; second.command_can_id=3U; second.motor_id=3U;
  c.joints.push_back(second);
  FakeTransport transport{32U}; BusOwnershipRegistry ownership;
  std::int64_t now=0;
  Dm4340Runtime runtime{transport,[&now]{return at(now);},c,ownership};
  ASSERT_TRUE(runtime.configure(2U)); ASSERT_TRUE(runtime.start());
  CanonicalState states[2]{}; RawCanFrame sent{};
  ASSERT_TRUE(runtime.read(states,2U));
  ASSERT_TRUE(transport.take_transmit(sent)); EXPECT_EQ(sent.payload[0],2U);
  EXPECT_FALSE(transport.take_transmit(sent));
  now=9000000; ASSERT_TRUE(runtime.read(states,2U));
  EXPECT_FALSE(transport.take_transmit(sent));
  now=10000000; ASSERT_TRUE(runtime.read(states,2U));
  ASSERT_TRUE(transport.take_transmit(sent)); EXPECT_EQ(sent.payload[0],3U);
  EXPECT_FALSE(transport.take_transmit(sent));
  ASSERT_EQ(transport.inject_receive(feedback(now,1U,2U)),TransportResult::Ok);
  ASSERT_EQ(transport.inject_receive(feedback(now,1U,3U)),TransportResult::Ok);
  ASSERT_TRUE(runtime.read(states,2U));
  now=20000000; ASSERT_TRUE(runtime.read(states,2U));
  ASSERT_TRUE(transport.take_transmit(sent)); EXPECT_EQ(sent.payload[0],2U);
  EXPECT_FALSE(transport.take_transmit(sent));
  now=60000000; EXPECT_FALSE(runtime.read(states,2U));
  EXPECT_EQ(runtime.fault_reason(),Dm4340FaultReason::FeedbackExpired);
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
  ASSERT_TRUE(runtime.feedback_diagnostic());
  EXPECT_EQ(runtime.feedback_diagnostic()->joint_index, 1U);
  EXPECT_EQ(runtime.feedback_diagnostic()->rejected.frame.payload[0], 0x93U);

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

TEST(Dm4340Runtime, Dual120SecondReplayAndInjectedDeadlineStopBothAxes) {
  FakeTransport transport{64U};
  BusOwnershipRegistry ownership;
  auto c = config();
  c.stagger_status_queries = true;
  c.joints.push_back(c.joints.front());
  c.joints[0].motor_id = 1U;
  c.joints[0].command_can_id = 1U;
  std::int64_t now = 0;
  Dm4340Runtime runtime{transport, [&] { return at(now); }, c, ownership};
  ASSERT_TRUE(runtime.configure(2U));
  ASSERT_TRUE(runtime.start());
  CanonicalState states[2]{};
  CommandDispatch commands[2]{};
  for (auto& command : commands) {
    command.authorized = true;
    command.fresh = true;
  }
  RawCanFrame frame{};
  std::size_t motion_frames = 0U;
  // 60000 deterministic 2ms steps, not 120 seconds of wall-clock scheduling.
  for (std::int64_t cycle = 0; cycle < 60000; ++cycle) {
    now = 1000000 + cycle * 2000000;
    for (const auto id : {1U, 2U})
      ASSERT_EQ(transport.inject_receive(feedback(now, 1U, id)), TransportResult::Ok);
    ASSERT_TRUE(runtime.read(states, 2U)) << cycle;
    const auto phase = cycle % 5000;
    for (auto& command : commands)
      command.command.velocity = phase < 1500 ? -0.8 :
                                 phase >= 2500 && phase < 4000 ? 0.8 : 0.0;
    ASSERT_TRUE(runtime.write(commands, 2U)) << cycle;
    ASSERT_TRUE(runtime.read(states, 2U)) << cycle;
    while (transport.take_transmit(frame))
      if (frame.id.value != 0x7FFU) ++motion_frames;
  }
  EXPECT_EQ(motion_frames, 120000U);
  EXPECT_FALSE(runtime.deadline_diagnostic());
  now += 6500000;
  EXPECT_FALSE(runtime.read(states, 2U));
  ASSERT_TRUE(runtime.deadline_diagnostic());
  const auto snapshot = *runtime.deadline_diagnostic();
  EXPECT_EQ(snapshot.joint_index, 0U);
  EXPECT_FALSE(snapshot.during_write);
  EXPECT_EQ(snapshot.command_age_ns, 6500000);
  EXPECT_EQ(snapshot.overdue_ns, 500000);
  EXPECT_EQ(snapshot.read_gap_ns, 6500000);
  EXPECT_EQ(snapshot.read_work_ns, 0);
  EXPECT_EQ(snapshot.tx_age_ns, 6500000);
  std::size_t disables = 0U;
  while (transport.take_transmit(frame))
    if (frame.id.value != 0x7FFU && frame.payload[7] == 0xFDU) ++disables;
  EXPECT_EQ(disables, 2U);
  // New commands cannot erase the old fault or the diagnostic, even after stop.
  EXPECT_FALSE(runtime.write(commands, 2U));
  runtime.stop();
  ASSERT_TRUE(runtime.deadline_diagnostic());
  EXPECT_EQ(runtime.deadline_diagnostic()->overdue_ns, snapshot.overdue_ns);
  ASSERT_TRUE(runtime.start());
  EXPECT_FALSE(runtime.deadline_diagnostic());
}

TEST(Dm4340Runtime, DeadlineDiagnosticIdentifiesStaleSecondAxisDuringWrite) {
  FakeTransport transport{64U};
  BusOwnershipRegistry ownership;
  auto c = config();
  c.joints.push_back(c.joints.front());
  c.joints[0].motor_id = 1U;
  c.joints[0].command_can_id = 1U;
  std::int64_t now = 1000000;
  Dm4340Runtime runtime{transport, [&] { return at(now); }, c, ownership};
  ASSERT_TRUE(runtime.configure(2U));
  ASSERT_TRUE(runtime.start());
  CanonicalState states[2]{};
  for (const auto id : {1U, 2U})
    ASSERT_EQ(transport.inject_receive(feedback(now, 1U, id)), TransportResult::Ok);
  ASSERT_TRUE(runtime.read(states, 2U));
  CommandDispatch commands[2]{};
  for (auto& command : commands) {
    command.authorized = true;
    command.fresh = true;
    command.command.velocity = 0.8;
  }
  ASSERT_TRUE(runtime.write(commands, 2U));
  ASSERT_TRUE(runtime.read(states, 2U));
  now += 2000000;
  commands[1].fresh = false;
  ASSERT_TRUE(runtime.write(commands, 2U));
  now += 4000000;
  commands[1].fresh = true;
  EXPECT_FALSE(runtime.write(commands, 2U));
  ASSERT_TRUE(runtime.deadline_diagnostic());
  const auto& d = *runtime.deadline_diagnostic();
  EXPECT_EQ(d.joint_index, 1U);
  EXPECT_TRUE(d.during_write);
  EXPECT_EQ(d.command_age_ns, 6000000);
  EXPECT_EQ(d.overdue_ns, 0);
  EXPECT_EQ(d.write_gap_ns, 4000000);
  EXPECT_EQ(d.tx_age_ns, 6000000);
}

TEST(Dm4340Runtime, DeadlineDiagnosticSeparatesReadWorkFromEntryGap) {
  FakeTransport transport{32U};
  BusOwnershipRegistry ownership;
  std::int64_t now = 1000000;
  bool delay_inside_read = false;
  unsigned clock_calls = 0U;
  Dm4340Runtime runtime{transport, [&] {
    if (delay_inside_read && ++clock_calls == 2U) now += 6000000;
    return at(now);
  }, config(), ownership};
  ASSERT_TRUE(runtime.configure(1U));
  ASSERT_TRUE(runtime.start());
  CanonicalState state{};
  ASSERT_EQ(transport.inject_receive(feedback(now)), TransportResult::Ok);
  ASSERT_TRUE(runtime.read(&state, 1U));
  CommandDispatch command{};
  command.authorized = command.fresh = true;
  command.command.velocity = 0.8;
  ASSERT_TRUE(runtime.write(&command, 1U));
  ASSERT_TRUE(runtime.read(&state, 1U));
  now += 2000000;
  delay_inside_read = true;
  // No status query due and no RX packets: second clock call follows receive.
  EXPECT_FALSE(runtime.read(&state, 1U));
  ASSERT_TRUE(runtime.deadline_diagnostic());
  const auto& d = *runtime.deadline_diagnostic();
  EXPECT_EQ(d.read_gap_ns, 2000000);
  EXPECT_EQ(d.read_work_ns, 6000000);
  EXPECT_EQ(d.command_age_ns, 8000000);
  EXPECT_EQ(d.overdue_ns, 2000000);
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

TEST(Dm4340Runtime, PositionViolationDisablesWithoutApplicationForBothModes) {
  for (const bool effort : {false,true}) {
    auto c=config();c.joints[0].position_min_rad=-1;c.joints[0].position_max_rad=1;
    if (effort) {
      c.command_mode=Dm4340CommandMode::Effort;
      c.joints[0].kd=0;c.joints[0].max_command_velocity_rad_s=0;
      c.joints[0].max_command_effort_nm=.3;
    }
    FakeTransport transport{32};BusOwnershipRegistry ownership;std::int64_t now=0;
    Dm4340Runtime runtime(transport,[&]{return at(now);},c,ownership);
    ASSERT_TRUE(runtime.configure(1));ASSERT_TRUE(runtime.start());
    CanonicalState state{};
    now=1000000;ASSERT_EQ(transport.inject_receive(feedback(now)),TransportResult::Ok);
    ASSERT_TRUE(runtime.read(&state,1));
    RawCanFrame sent{};while(transport.take_transmit(sent)) {}
    CommandDispatch command{};command.authorized=true;command.fresh=true;
    if(effort)command.command.effort=.1;else command.command.velocity=.1;
    ASSERT_TRUE(runtime.write(&command,1));ASSERT_TRUE(runtime.read(&state,1));
    while(transport.take_transmit(sent)) {}
    now+=1000000;auto sample=feedback(now);sample.payload[1]=0xFF;sample.payload[2]=0xFF;
    ASSERT_EQ(transport.inject_receive(sample),TransportResult::Ok);
    EXPECT_FALSE(runtime.read(&state,1));
    EXPECT_EQ(runtime.fault_reason(),Dm4340FaultReason::FeedbackRejected);
    bool disabled=false;
    while(transport.take_transmit(sent))
      if(sent.id.value==2 && sent.payload[7]==0xFD)disabled=true;
    EXPECT_TRUE(disabled);
    EXPECT_FALSE(runtime.write(&command,1));
  }
}

namespace {

Dm4340RuntimeConfig position_config() {
  auto c=config(); c.command_mode=Dm4340CommandMode::Position;
  c.joints[0].kp=1; c.joints[0].kd=.2;
  c.joints[0].max_command_velocity_rad_s=0;
  return c;
}

TEST(Dm4340Runtime, PositionImpedanceValidatesGainsAndWorstCaseNominalTorque) {
  for (int bad=0;bad<5;++bad) {
    auto c=position_config();
    if(bad==0) c.joints[0].kp=0;
    if(bad==1) c.joints[0].kd=0;
    if(bad==2) c.joints[0].kp=100;
    if(bad==3) c.joints[0].velocity_feedforward_nm=.1;
    if(bad==4) c.joints[0].max_command_effort_nm=.1;
    FakeTransport tx; BusOwnershipRegistry ownership;
    Dm4340Runtime rt{tx,[]{return at(0);},c,ownership};
    EXPECT_FALSE(rt.configure(1)); EXPECT_FALSE(tx.is_open());
  }
}

TEST(Dm4340Runtime, DualPositionImpedanceRoutesIndependentTargetsAndStopsGroupOnBadTarget) {
  auto c=position_config();
  auto second=c.joints[0]; second.command_can_id=1; second.motor_id=1;
  second.joint.scale=-1; c.joints.push_back(second);
  FakeTransport tx{64}; BusOwnershipRegistry ownership; std::int64_t now=0;
  Dm4340Runtime rt{tx,[&]{return at(now);},c,ownership};
  ASSERT_TRUE(rt.configure(2)); ASSERT_TRUE(rt.start());
  CanonicalState state[2]{};
  now=1000000;
  ASSERT_EQ(tx.inject_receive(feedback(now,1,2)),TransportResult::Ok);
  ASSERT_EQ(tx.inject_receive(feedback(now,1,1)),TransportResult::Ok);
  ASSERT_TRUE(rt.read(state,2));
  RawCanFrame out{}; while(tx.take_transmit(out)) {}
  CommandDispatch cmd[2]{};
  for(auto& x:cmd) {x.authorized=true;x.fresh=true;}
  cmd[0].command.position=.1; cmd[1].command.position=-.15;
  ASSERT_TRUE(rt.write(cmd,2)); ASSERT_TRUE(rt.read(state,2));
  for(int i=0;i<2;++i) {
    ASSERT_TRUE(tx.take_transmit(out)); EXPECT_EQ(out.id.value,c.joints[i].command_can_id);
    const auto p=static_cast<unsigned>((out.payload[0]<<8)|out.payload[1]);
    const double motor_position=p*25.0/65535-.0-12.5;
    EXPECT_NEAR(motor_position,i==0?.1:.15,25.0/65535);
    EXPECT_EQ(((out.payload[3]&0xf)<<8)|out.payload[4],8U); // Kp=1
    EXPECT_EQ((out.payload[5]<<4)|(out.payload[6]>>4),163U); // Kd=.2
  }
  EXPECT_FALSE(tx.take_transmit(out));
  now+=2000000; cmd[1].command.position=1; // > .3rad target error
  ASSERT_TRUE(rt.write(cmd,2)); EXPECT_FALSE(rt.read(state,2));
  EXPECT_EQ(rt.fault_reason(),Dm4340FaultReason::CommandPreparation);
  int disables=0; while(tx.take_transmit(out)) if(out.payload[7]==0xfd) ++disables;
  EXPECT_EQ(disables,2);
}

TEST(Dm4340Runtime, PositionImpedanceKeepsLeaseExpiryAndRejectsOtherCommandFields) {
  for(bool expired:{false,true}) {
    auto c=position_config(); FakeTransport tx{32}; BusOwnershipRegistry ownership;
    std::int64_t now=0; Dm4340Runtime rt{tx,[&]{return at(now);},c,ownership};
    ASSERT_TRUE(rt.configure(1)); ASSERT_TRUE(rt.start());
    now=1000000; CanonicalState state{};
    ASSERT_EQ(tx.inject_receive(feedback(now)),TransportResult::Ok);
    ASSERT_TRUE(rt.read(&state,1));
    CommandDispatch cmd{};cmd.authorized=true;cmd.fresh=true;cmd.command.position=.1;
    if(!expired) cmd.command.velocity=.1;
    if(expired) {
      ASSERT_TRUE(rt.write(&cmd,1)); ASSERT_TRUE(rt.read(&state,1));
      now+=6000000; // A new target cannot hide an expired prior lease.
    }
    EXPECT_FALSE(rt.write(&cmd,1));
    EXPECT_EQ(rt.fault_reason(),expired?Dm4340FaultReason::CommandDeadline:Dm4340FaultReason::CommandRejected);
  }
}

}  // namespace

TEST(Dm4340Runtime, DualEffortSaturatesBeforeMappingAndKeepsFaultProtection) {
  FakeTransport transport{128U};
  BusOwnershipRegistry ownership;
  std::int64_t now=0;
  auto c=config();
  c.command_mode=Dm4340CommandMode::Effort;
  c.joints[0].kd=0;
  c.joints[0].max_command_velocity_rad_s=0;
  c.joints[0].max_command_effort_nm=8;
  c.joints[0].max_feedback_torque_nm=8;
  c.joints.push_back(c.joints[0]);
  c.joints[1].command_can_id=3;
  c.joints[1].motor_id=3;
  c.joints[1].joint.scale=-2;
  c.joints[1].max_command_effort_nm=3;
  Dm4340Runtime runtime{transport,[&]{return at(now);},c,ownership};
  ASSERT_TRUE(runtime.configure(2));
  ASSERT_TRUE(runtime.start());
  now=1000000;
  ASSERT_EQ(transport.inject_receive(feedback(now)),TransportResult::Ok);
  ASSERT_EQ(transport.inject_receive(feedback(now,1,3)),TransportResult::Ok);
  CanonicalState states[2]{};
  ASSERT_TRUE(runtime.read(states,2));
  RawCanFrame sent{};
  while(transport.take_transmit(sent)) {}
  CommandDispatch commands[2]{};
  for(auto& command:commands){command.authorized=true;command.fresh=true;}
  for(double request : {10.0,-10.0,1e300}) {
    for(auto& command:commands)command.command.effort=request;
    ASSERT_TRUE(runtime.write(commands,2));
    ASSERT_TRUE(runtime.read(states,2));
    unsigned seen=0;
    while(transport.take_transmit(sent)) {
      if(sent.id.value!=2 && sent.id.value!=3)continue;
      ++seen;
      const auto axis=sent.id.value==2?0U:1U;
      const auto code=((sent.payload[6]&15U)<<8U)|sent.payload[7];
      const double motor=code*56.0/4095.0-28.0;
      const double joint=motor/c.joints[axis].joint.scale;
      const double cap=c.joints[axis].max_command_effort_nm;
      EXPECT_LE(std::abs(joint),cap+1e-12);
      EXPECT_NEAR(joint,request>0?cap:-cap,56.0/4095.0);
      EXPECT_EQ(sent.payload[3]&15U,0U); // Kp=0
      EXPECT_EQ(sent.payload[4],0U);
      EXPECT_EQ(sent.payload[5],0U); // Kd=0
      EXPECT_EQ(sent.payload[6]&240U,0U);
    }
    EXPECT_EQ(seen,2U);
    EXPECT_EQ(runtime.fault_reason(),Dm4340FaultReason::None);
    now+=1000000;
  }
  commands[0].command.effort=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(runtime.write(commands,2));
  EXPECT_EQ(runtime.fault_reason(),Dm4340FaultReason::CommandRejected);
}
