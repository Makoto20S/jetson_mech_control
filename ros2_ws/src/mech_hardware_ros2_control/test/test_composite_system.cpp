#include "mech_hardware_ros2_control/composite_system.hpp"

#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <algorithm>
#include <stdexcept>

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
    // ADR-017 shape: one motion command interface plus the always-exported
    // command_generation interface.
    joint.command_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                                interface(kCommandGenerationInterface)};
    joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                              interface(hardware_interface::HW_IF_VELOCITY),
                              interface(hardware_interface::HW_IF_EFFORT)};
    result.joints.push_back(joint);
  }
  return result;
}

// Looks a command interface up by its full "<joint>/<interface>" name.
// Positional indexing broke the moment ADR-017 exported a second interface per
// joint, and it broke SILENTLY: commands[1] went from joint_2's position to
// joint_1's generation, so a test written to put a non-finite value on a
// motion interface was putting it on a generation interface that nothing
// validates - and it still "passed" its own setup. Look them up by name.
hardware_interface::CommandInterface& command_by_name(
    std::vector<hardware_interface::CommandInterface>& commands,
    const std::string& name) {
  for (auto& command : commands) {
    if (command.get_name() == name) return command;
  }
  throw std::runtime_error("no command interface named " + name);
}

// info() variant whose first joint commands a different canonical kind
// (effort or velocity) instead of position - the ADR-014 single-command-
// interface shape for Torque/Velocity devices.
hardware_interface::HardwareInfo info_with_command_interface(
    const std::string& command_interface) {
  auto result = info(1U);
  result.joints[0].command_interfaces[0].name = command_interface;
  return result;
}

// info() variant where the first joint's name itself contains a '/', to
// exercise interface-name resolution that must split on the *last* slash.
hardware_interface::HardwareInfo info_with_slash_joint_name() {
  auto result = info(2U);
  result.joints[0].name = "arm/j1";
  return result;
}

// RuntimePort double whose read() reports a NaN position, simulating a
// device adapter that decoded a corrupt frame.
class NanReadRuntime final : public RuntimePort {
 public:
  bool configure(std::size_t resource_count) noexcept override {
    count_ = resource_count;
    return resource_count > 0U;
  }
  bool start() noexcept override {
    running_ = count_ > 0U;
    return running_;
  }
  void stop() noexcept override { running_ = false; }
  bool read(CanonicalState* states, std::size_t count) noexcept override {
    if (!running_ || states == nullptr || count != count_) return false;
    for (std::size_t index = 0U; index < count; ++index) {
      states[index] = CanonicalState{};
    }
    states[0].position = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  bool write(const CommandDispatch*, std::size_t) noexcept override { return true; }
  void cancel_pending(std::size_t) noexcept override {}
  bool has_valid_sample() const noexcept override { return running_; }

 private:
  bool running_{false};
  std::size_t count_{0U};
};

rclcpp_lifecycle::State state() {
  return rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test");
}

// RuntimePort double that counts how many commands were handed down with
// transmit authorization, per joint. This is the in-package stand-in for a
// transport TX counter: ADR-015's exit criterion is stated in terms of
// "did a command cross the RuntimePort boundary", which is exactly what a
// device adapter would turn into a CAN frame.
class RecordingRuntime final : public RuntimePort {
 public:
  bool configure(std::size_t resource_count) noexcept override {
    count_ = resource_count;
    try {
      authorized_writes_.assign(resource_count, 0U);
      fresh_writes_.assign(resource_count, 0U);
      cancels_.assign(resource_count, 0U);
      last_command_.assign(resource_count, CanonicalCommand{});
    } catch (...) {
      return false;
    }
    return resource_count > 0U;
  }
  bool start() noexcept override {
    running_ = count_ > 0U;
    return running_;
  }
  void stop() noexcept override { running_ = false; }
  bool read(CanonicalState* states, std::size_t count) noexcept override {
    if (!running_ || states == nullptr || count != count_) return false;
    for (std::size_t index = 0U; index < count; ++index) {
      states[index] = CanonicalState{};
      states[index].position = measured_position;
    }
    return true;
  }
  bool write(const CommandDispatch* commands,
             std::size_t count) noexcept override {
    if (!running_ || commands == nullptr || count != count_) return false;
    ++write_calls_;
    for (std::size_t index = 0U; index < count; ++index) {
      if (!commands[index].authorized) continue;
      ++authorized_writes_[index];
      // Counted separately from authorized_writes_ so a test can tell
      // "the manager cycled and the claim is held" apart from "the controller
      // refreshed its target" (ADR-017) - the distinction the hardware could
      // not make before.
      if (commands[index].fresh) ++fresh_writes_[index];
      last_command_[index] = commands[index].command;
    }
    return true;
  }

  double measured_position{0.0};

  void cancel_pending(std::size_t index) noexcept override {
    if (index < count_) ++cancels_[index];
  }

  bool has_valid_sample() const noexcept override { return has_valid_sample_; }

  // ADR-016: tests that are not about the claim gate keep a valid sample so
  // claiming behaves as it did before; the gate has its own tests.
  void set_has_valid_sample(bool value) noexcept { has_valid_sample_ = value; }

  [[nodiscard]] std::size_t write_calls() const noexcept { return write_calls_; }
  [[nodiscard]] std::size_t authorized_writes(std::size_t index) const noexcept {
    return index < authorized_writes_.size() ? authorized_writes_[index] : 0U;
  }
  [[nodiscard]] std::size_t fresh_writes(std::size_t index) const noexcept {
    return index < fresh_writes_.size() ? fresh_writes_[index] : 0U;
  }
  [[nodiscard]] std::size_t cancels(std::size_t index) const noexcept {
    return index < cancels_.size() ? cancels_[index] : 0U;
  }
  [[nodiscard]] CanonicalCommand last_command(std::size_t index) const noexcept {
    return index < last_command_.size() ? last_command_[index] : CanonicalCommand{};
  }

 private:
  bool running_{false};
  std::size_t count_{0U};
  std::size_t write_calls_{0U};
  std::vector<std::size_t> authorized_writes_;
  std::vector<std::size_t> fresh_writes_;
  std::vector<std::size_t> cancels_;
  std::vector<CanonicalCommand> last_command_;
  bool has_valid_sample_{true};
};

// ADR-015 Decision 1/2: an unclaimed joint has no transmit authorization, so
// its default-constructed CanonicalCommand{0, 0, 0} must never be handed to
// the runtime - on a position interface that zero IS a commanded move to the
// zero position (ADR-012 Decision 3). The controller_manager keeps calling
// read()/write() regardless, so the loop below is the realistic shape: the
// hardware cycle running is not evidence that a controller commanded
// anything.
TEST(CompositeSystem, UnclaimedJointIsNeverHandedToTheRuntime) {
  CompositeSystem system;
  auto runtime = std::make_unique<RecordingRuntime>();
  auto* recorder = runtime.get();
  ASSERT_TRUE(system.set_runtime(std::move(runtime)));
  ASSERT_EQ(system.on_init(info(1U)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);

  // No perform_command_mode_switch(): nothing is claimed.
  for (int cycle = 0; cycle < 10; ++cycle) {
    ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
              hardware_interface::return_type::OK);
    ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
              hardware_interface::return_type::OK);
  }
  EXPECT_EQ(recorder->write_calls(), 0U);
  EXPECT_FALSE(system.fault_latched());
}

// Fixture-free helper: brings a RecordingRuntime-backed system to ACTIVE and
// hands back the recorder plus the exported command interfaces.
struct ActiveSystem final {
  CompositeSystem system;
  RecordingRuntime* recorder{nullptr};
  std::vector<hardware_interface::CommandInterface> commands;

  explicit ActiveSystem(std::size_t joints) {
    auto runtime = std::make_unique<RecordingRuntime>();
    recorder = runtime.get();
    EXPECT_TRUE(system.set_runtime(std::move(runtime)));
    EXPECT_EQ(system.on_init(info(joints)),
              hardware_interface::CallbackReturn::SUCCESS);
    commands = system.export_command_interfaces();
    EXPECT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
  }

  void cycle(int count) {
    for (int index = 0; index < count; ++index) {
      EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
                hardware_interface::return_type::OK);
      EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
                hardware_interface::return_type::OK);
    }
  }
};

// Exercise the switch-cycle write before the incoming controller updates.
TEST(CompositeSystem, WeakPositionTakeoverHoldsAcceptedNonzeroPosition) {
  ActiveSystem fixture(1U);
  fixture.recorder->measured_position = -4.2;
  ASSERT_EQ(fixture.system.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
            hardware_interface::return_type::OK);
  // Changing the runtime's next sample must not replace the accepted sample.
  fixture.recorder->measured_position = -3.9;
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(fixture.system.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(fixture.recorder->last_command(0U).position, -4.2);
  EXPECT_EQ(fixture.recorder->fresh_writes(0U), 1U);
}

TEST(CompositeSystem, WeakPositionReclaimHoldsNewFeedbackInsteadOfOldTarget) {
  ActiveSystem fixture(1U);
  fixture.recorder->measured_position = -4.2;
  fixture.cycle(1);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.commands[0].set_value(-4.0);
  fixture.cycle(1);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({}, {"joint_1/position"}),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(fixture.commands[0].get_value(), -4.0);
  EXPECT_EQ(fixture.recorder->cancels(0U), 1U);
  fixture.recorder->measured_position = -3.0;
  fixture.cycle(2);
  EXPECT_EQ(fixture.recorder->fresh_writes(0U), 1U);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(fixture.system.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(fixture.recorder->last_command(0U).position, -3.0);
  EXPECT_EQ(fixture.recorder->fresh_writes(0U), 2U);
}

TEST(CompositeSystem, RejectedTakeoverDoesNotSeedAnyCommand) {
  ActiveSystem fixture(2U);
  fixture.recorder->measured_position = -4.2;
  fixture.cycle(1);
  fixture.commands[0].set_value(0.7);
  fixture.commands[1].set_value(0.8);
  // The first start is valid, but the complete transaction is not.
  EXPECT_EQ(fixture.system.perform_command_mode_switch(
                {"joint_1/position", "joint_2/position", "joint_2/position"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(fixture.commands[0].get_value(), 0.7);
  EXPECT_DOUBLE_EQ(fixture.commands[1].get_value(), 0.8);
  EXPECT_FALSE(fixture.system.authorized(0U));
  fixture.recorder->set_has_valid_sample(false);
  EXPECT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(fixture.commands[0].get_value(), 0.7);
}

TEST(CompositeSystem, StrongPositionTakeoverPreservesTargetAndRequiresGeneration) {
  ActiveSystem fixture(1U);
  fixture.recorder->measured_position = -4.2;
  fixture.cycle(1);
  fixture.commands[0].set_value(0.7);
  const std::string generation = "joint_1/command_generation";
  command_by_name(fixture.commands, generation).set_value(8.0);
  ASSERT_EQ(fixture.system.perform_command_mode_switch(
                {generation, "joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.cycle(1);
  EXPECT_DOUBLE_EQ(fixture.commands[0].get_value(), 0.7);
  EXPECT_EQ(fixture.recorder->fresh_writes(0U), 0U);
  command_by_name(fixture.commands, generation).set_value(9.0);
  fixture.cycle(1);
  EXPECT_EQ(fixture.recorder->fresh_writes(0U), 1U);
  EXPECT_DOUBLE_EQ(fixture.recorder->last_command(0U).position, 0.7);
}

// ADR-016 Decision 3: a joint whose feedback has never arrived must not be
// claimable. This is the shared choke point that keeps a controller from ever
// seeding a hold from a position nobody measured - put in one layer rather
// than trusting every controller to check a quality field it cannot even see
// through a bare double.
TEST(CompositeSystem, JointWithoutAValidSampleCannotBeClaimed) {
  ActiveSystem fixture(1U);
  fixture.recorder->set_has_valid_sample(false);

  EXPECT_EQ(fixture.system.prepare_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_FALSE(fixture.system.authorized(0U));

  // The manager keeps cycling against an unknown position: still nothing goes
  // out, because without a claim there is no transmit authorization (ADR-015).
  fixture.cycle(10);
  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 0U);
  EXPECT_FALSE(fixture.system.fault_latched());

  // Feedback arrives; the joint becomes claimable and commands flow.
  fixture.recorder->set_has_valid_sample(true);
  ASSERT_EQ(fixture.system.prepare_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.commands[0].set_value(0.25);
  fixture.cycle(1);
  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 1U);
}

// Releasing a claim must stay possible after the device goes silent. Refusing
// a stop-only switch would strand the controller holding a joint it can no
// longer observe - the opposite of fail-closed.
TEST(CompositeSystem, ReleasingAClaimStillWorksWithoutAValidSample) {
  ActiveSystem fixture(1U);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);

  fixture.recorder->set_has_valid_sample(false);
  EXPECT_EQ(fixture.system.prepare_command_mode_switch({}, {"joint_1/position"}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(fixture.system.perform_command_mode_switch({}, {"joint_1/position"}),
            hardware_interface::return_type::OK);
  EXPECT_FALSE(fixture.system.authorized(0U));
  EXPECT_EQ(fixture.recorder->cancels(0U), 1U);
}

// ADR-015 Decision 1/3: releasing the claim revokes authorization immediately
// and cancels the pending command. The controller_manager keeps cycling
// afterwards - that must not resurrect the released target. This is the case
// the 2026-09-12 audit reproduced as "22 ms / 11 frames after release"; the
// old test only stopped calling hardware write(), which proves nothing.
TEST(CompositeSystem, ReleasingClaimRevokesAuthorizationAndCancelsPending) {
  ActiveSystem fixture(1U);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_TRUE(fixture.system.authorized(0U));
  fixture.commands[0].set_value(0.25);
  fixture.cycle(1);
  ASSERT_EQ(fixture.recorder->authorized_writes(0U), 1U);
  EXPECT_DOUBLE_EQ(fixture.recorder->last_command(0U).position, 0.25);

  ASSERT_EQ(fixture.system.perform_command_mode_switch({}, {"joint_1/position"}),
            hardware_interface::return_type::OK);
  EXPECT_FALSE(fixture.system.authorized(0U));
  EXPECT_EQ(fixture.recorder->cancels(0U), 1U);

  fixture.cycle(10);
  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 1U);
  EXPECT_FALSE(fixture.system.fault_latched());
}

// ADR-015 Decision 5: authorization is per joint. Claiming joint_1 must not
// give joint_2's default-constructed command a way onto the wire.
TEST(CompositeSystem, AuthorizationDoesNotLeakBetweenJoints) {
  ActiveSystem fixture(2U);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.commands[0].set_value(0.5);
  fixture.commands[1].set_value(0.75);
  fixture.cycle(5);

  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 5U);
  EXPECT_EQ(fixture.recorder->authorized_writes(1U), 0U);
  EXPECT_TRUE(fixture.system.authorized(0U));
  EXPECT_FALSE(fixture.system.authorized(1U));
  // joint_2's command value never reached the runtime, so the recorder still
  // holds its default - not the 0.75 the interface carries.
  EXPECT_DOUBLE_EQ(fixture.recorder->last_command(1U).position, 0.0);
}

// ADR-015 Decision 1/3: leaving ACTIVE revokes every joint and cancels every
// pending command, and re-activating requires a fresh claim before anything
// is transmitted again.
TEST(CompositeSystem, DeactivateRevokesAllAuthorizationAndRequiresReclaim) {
  ActiveSystem fixture(2U);
  ASSERT_EQ(fixture.system.perform_command_mode_switch(
                {"joint_1/position", "joint_2/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.commands[0].set_value(0.3);
  fixture.cycle(1);
  ASSERT_EQ(fixture.recorder->authorized_writes(0U), 1U);

  ASSERT_EQ(fixture.system.on_deactivate(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(fixture.system.authorized(0U));
  EXPECT_FALSE(fixture.system.authorized(1U));
  EXPECT_EQ(fixture.recorder->cancels(0U), 1U);
  EXPECT_EQ(fixture.recorder->cancels(1U), 1U);

  // Re-activate without re-claiming: the manager cycles, nothing transmits.
  ASSERT_EQ(fixture.system.on_activate(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  fixture.cycle(10);
  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 1U);

  // Re-claiming restores authorization.
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.cycle(1);
  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 2U);
}

// ADR-015 Decision 3: on_error revokes and cancels too. It also latches, so
// subsequent write() calls fail rather than transmit.
TEST(CompositeSystem, ErrorRevokesAuthorizationAndCancelsPending) {
  ActiveSystem fixture(1U);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.commands[0].set_value(0.1);
  fixture.cycle(1);
  ASSERT_EQ(fixture.recorder->authorized_writes(0U), 1U);

  ASSERT_EQ(fixture.system.on_error(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(fixture.system.authorized(0U));
  EXPECT_EQ(fixture.recorder->cancels(0U), 1U);
  EXPECT_TRUE(fixture.system.fault_latched());
  EXPECT_EQ(fixture.system.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 1U);
}

// A rejected switch must not cancel a live command: the strict-switch
// contract is all-or-nothing, so a bad stop request leaves the claim and its
// pending command exactly as they were.
TEST(CompositeSystem, RejectedSwitchLeavesAuthorizationAndPendingIntact) {
  ActiveSystem fixture(2U);
  ASSERT_EQ(fixture.system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  fixture.commands[0].set_value(0.2);
  fixture.cycle(1);

  // joint_2 was never claimed, so stopping it is invalid - and joint_1 is
  // named in the same request, which a naive implementation would revoke
  // before discovering the failure.
  EXPECT_EQ(fixture.system.perform_command_mode_switch(
                {}, {"joint_1/position", "joint_2/position"}),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(fixture.system.authorized(0U));
  EXPECT_EQ(fixture.recorder->cancels(0U), 0U);
  fixture.cycle(1);
  EXPECT_EQ(fixture.recorder->authorized_writes(0U), 2U);
}

TEST(CompositeSystem, ExportsCanonicalInterfacesAndLoopsBackNonBlocking) {
  CompositeSystem system;
  ASSERT_EQ(system.on_init(info()), hardware_interface::CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(states.size(), 6U);
  // Two joints x (one motion + one generation) = four (ADR-017).
  ASSERT_EQ(commands.size(), 4U);
  EXPECT_EQ(states[2].get_name(), "joint_1/effort");
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.prepare_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  command_by_name(commands, "joint_1/position").set_value(0.25);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.01);
}

// ADR-014: a joint's single command interface may be effort or velocity
// instead of position. The exported name carries the URDF's choice, and
// claiming goes through the same strict-switch machinery under that name.
TEST(CompositeSystem, ExportsEffortAndVelocityCommandInterfaces) {
  for (const char* command_interface :
       {hardware_interface::HW_IF_EFFORT, hardware_interface::HW_IF_VELOCITY}) {
    SCOPED_TRACE(command_interface);
    CompositeSystem system;
    ASSERT_EQ(system.on_init(info_with_command_interface(command_interface)),
              hardware_interface::CallbackReturn::SUCCESS);
    auto states = system.export_state_interfaces();
    auto commands = system.export_command_interfaces();
    ASSERT_EQ(states.size(), 3U);
    // One motion interface plus the always-exported generation interface.
    ASSERT_EQ(commands.size(), 2U);
    EXPECT_EQ(commands[0].get_name(), std::string("joint_1/") + command_interface);
    ASSERT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    const std::string claim = std::string("joint_1/") + command_interface;
    ASSERT_EQ(system.prepare_command_mode_switch({claim}, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(system.perform_command_mode_switch({claim}, {}),
              hardware_interface::return_type::OK);
    // A command written through the exported interface must be finite and
    // accepted; the loopback mirrors are covered by their dedicated test.
    commands[0].set_value(1.5);
    EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    EXPECT_EQ(system.perform_command_mode_switch({}, {claim}),
              hardware_interface::return_type::OK);
  }
}

TEST(CompositeSystem, WeakVelocityAndEffortTakeoverPreserveCommandBuffers) {
  for (const char* mode :
       {hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT}) {
    SCOPED_TRACE(mode);
    CompositeSystem system;
    auto runtime = std::make_unique<RecordingRuntime>();
    auto* recorder = runtime.get();
    recorder->measured_position = -4.2;
    ASSERT_TRUE(system.set_runtime(std::move(runtime)));
    ASSERT_EQ(system.on_init(info_with_command_interface(mode)),
              hardware_interface::CallbackReturn::SUCCESS);
    auto commands = system.export_command_interfaces();
    ASSERT_EQ(system.on_configure(state()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
              hardware_interface::return_type::OK);
    commands[0].set_value(0.7);
    ASSERT_EQ(system.perform_command_mode_switch({std::string("joint_1/") + mode}, {}),
              hardware_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(commands[0].get_value(), 0.7);
    ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
              hardware_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(recorder->last_command(0U).position, 0.0);
    EXPECT_EQ(recorder->fresh_writes(0U), 1U);
  }
}

// ADR-017 Decision 1: the generation interface is exported unconditionally,
// one per joint, alongside the motion interface. "Always exported, optionally
// claimed" is the whole difference between this design and the mandatory one
// that was rejected - if export were conditional, a deployment flag would
// decide whether a controller COULD protect itself, which is not the
// controller's choice to lose.
TEST(CompositeSystem, ExportsAGenerationCommandInterfacePerJoint) {
  CompositeSystem system;
  ASSERT_EQ(system.on_init(info(2U)),
            hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();

  ASSERT_EQ(commands.size(), 4U);
  std::vector<std::string> names;
  names.reserve(commands.size());
  for (const auto& command : commands) {
    names.push_back(command.get_name());
  }
  const auto has = [&names](const std::string& wanted) {
    return std::find(names.begin(), names.end(), wanted) != names.end();
  };
  EXPECT_TRUE(has(std::string("joint_1/") + kCommandGenerationInterface));
  EXPECT_TRUE(has(std::string("joint_2/") + kCommandGenerationInterface));
  EXPECT_TRUE(has(std::string("joint_1/") + hardware_interface::HW_IF_POSITION));
  EXPECT_TRUE(has(std::string("joint_2/") + hardware_interface::HW_IF_POSITION));

  // Each joint's generation interface must be its own storage: sharing one
  // cell between joints would make one controller's refresh look like every
  // joint's refresh.
  for (auto& command : commands) {
    if (command.get_interface_name() == kCommandGenerationInterface) {
      command.set_value(command.get_name() == std::string("joint_1/") +
                                                  kCommandGenerationInterface
                            ? 7.0
                            : 9.0);
    }
  }
  for (const auto& command : commands) {
    if (command.get_interface_name() != kCommandGenerationInterface) continue;
    const double expected =
        command.get_name() == std::string("joint_1/") + kCommandGenerationInterface
            ? 7.0
            : 9.0;
    EXPECT_DOUBLE_EQ(command.get_value(), expected) << command.get_name();
  }
}

// ADR-017 Decisions 2 and 5: both tiers must be claimable, and the generation
// interface must not be claimable on its own.
//
// The weak-tier case is the one that carries the project requirement: a
// standard ros2_control controller names only the motion interface, and it has
// to keep working. If that switch were ever refused, this hardware would stop
// being a general ros2_control target - which is exactly the design that was
// rejected before this ADR was accepted.
TEST(CompositeSystem, BothTiersAreClaimableButGenerationAloneIsNot) {
  const auto claim = [](const std::vector<std::string>& start) {
    CompositeSystem system;
    EXPECT_EQ(system.on_init(info(1U)),
              hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    const auto prepared = system.prepare_command_mode_switch(start, {});
    const auto performed = system.perform_command_mode_switch(start, {});
    return prepared == hardware_interface::return_type::OK &&
           performed == hardware_interface::return_type::OK;
  };
  const std::string motion = "joint_1/position";
  const std::string generation =
      std::string("joint_1/") + kCommandGenerationInterface;

  // Strong tier: a controller that opts into freshness protection.
  EXPECT_TRUE(claim({motion, generation}));
  // Weak tier: any stock ros2_control controller.
  EXPECT_TRUE(claim({motion}));
  // The generation interface modifies a motion command; alone it commands
  // nothing, so holding it alone is meaningless and must be refused.
  EXPECT_FALSE(claim({generation}));
}

// Drives a single-joint CompositeSystem with a RecordingRuntime so the tests
// below can talk about tiers without repeating twelve lines of lifecycle.
struct TierHarness {
  CompositeSystem system;
  RecordingRuntime* runtime{nullptr};
  std::vector<hardware_interface::CommandInterface> commands;

  explicit TierHarness() {
    auto owned = std::make_unique<RecordingRuntime>();
    runtime = owned.get();
    EXPECT_TRUE(system.set_runtime(std::move(owned)));
    EXPECT_EQ(system.on_init(info(1U)),
              hardware_interface::CallbackReturn::SUCCESS);
    commands = system.export_command_interfaces();
    EXPECT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
  }

  static std::string motion() { return "joint_1/position"; }
  static std::string generation() {
    return std::string("joint_1/") + kCommandGenerationInterface;
  }

  void claim(const std::vector<std::string>& names) {
    ASSERT_EQ(system.prepare_command_mode_switch(names, {}),
              hardware_interface::return_type::OK);
    ASSERT_EQ(system.perform_command_mode_switch(names, {}),
              hardware_interface::return_type::OK);
  }
  void release(const std::vector<std::string>& names) {
    ASSERT_EQ(system.perform_command_mode_switch({}, names),
              hardware_interface::return_type::OK);
  }
  void set_target(double value) {
    command_by_name(commands, motion()).set_value(value);
  }
  void set_generation(double value) {
    command_by_name(commands, generation()).set_value(value);
  }
  void cycle(std::size_t count) {
    for (std::size_t index = 0U; index < count; ++index) {
      ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 2000000)),
                hardware_interface::return_type::OK);
    }
  }
};

// ADR-017 Decision 3: the measured defect. A controller that holds its claim
// and stops writing used to produce one command per manager cycle, because
// "write() was called" was the only signal the hardware had. In the strong
// tier the generation interface supplies the missing one.
TEST(CompositeSystem, StrongTierSendsNothingNewWhileTheGenerationIsUnchanged) {
  TierHarness harness;
  harness.claim({TierHarness::motion(), TierHarness::generation()});
  harness.set_target(0.25);
  harness.set_generation(1.0);
  harness.cycle(5U);

  // The claim is held and the manager keeps cycling, so authorization is
  // handed down every cycle - that part is unchanged and is what ADR-015
  // governs.
  EXPECT_EQ(harness.runtime->authorized_writes(0U), 5U);
  // Exactly one of those cycles carried a command the controller refreshed.
  EXPECT_EQ(harness.runtime->fresh_writes(0U), 1U);
}

TEST(CompositeSystem, StrongTierSendsOncePerGenerationChange) {
  TierHarness harness;
  harness.claim({TierHarness::motion(), TierHarness::generation()});
  for (double generation = 1.0; generation <= 3.0; generation += 1.0) {
    harness.set_target(0.1 * generation);
    harness.set_generation(generation);
    harness.cycle(2U);
  }
  EXPECT_EQ(harness.runtime->authorized_writes(0U), 6U);
  EXPECT_EQ(harness.runtime->fresh_writes(0U), 3U);
}

// ADR-017 Decision 4, and the accepted risk made observable. A stock
// ros2_control controller names only the motion interface; it must keep
// working, and the freshness gap stays open for it. Written as a test rather
// than a sentence so that closing the gap later - alternative G - turns this
// red instead of passing silently.
TEST(CompositeSystem, WeakTierKeepsSendingWhileTheControllerIsSilent) {
  TierHarness harness;
  harness.claim({TierHarness::motion()});
  harness.set_target(0.25);
  harness.cycle(5U);

  EXPECT_EQ(harness.runtime->authorized_writes(0U), 5U);
  EXPECT_EQ(harness.runtime->fresh_writes(0U), 5U);
}

// ADR-017 Decision 3.4, the second symptom of the same root cause. Revocation
// deliberately keeps the last command value - zeroing it would be a commanded
// move to the calibrated zero on a position interface (ADR-012 Decision 3) -
// so re-claiming used to replay it. Resetting the baseline to whatever the
// buffer currently holds closes that with the same mechanism.
TEST(CompositeSystem, StrongTierDoesNotReplayTheOldTargetOnReclaim) {
  TierHarness harness;
  harness.claim({TierHarness::motion(), TierHarness::generation()});
  harness.set_target(0.25);
  harness.set_generation(1.0);
  harness.cycle(1U);
  ASSERT_EQ(harness.runtime->fresh_writes(0U), 1U);

  harness.release({TierHarness::motion(), TierHarness::generation()});
  harness.claim({TierHarness::motion(), TierHarness::generation()});
  // The new controller has not written anything yet: the buffers still hold
  // the previous controller's target and generation.
  harness.cycle(5U);
  EXPECT_EQ(harness.runtime->fresh_writes(0U), 1U);

  // Once it does speak, it is heard.
  harness.set_generation(2.0);
  harness.cycle(1U);
  EXPECT_EQ(harness.runtime->fresh_writes(0U), 2U);
}

// ADR-017 Decision 3.5: a non-finite generation is refused and is not a
// change. Deliberately NOT a latched fault - unlike a non-finite motion
// command, which is a defect that must stop the machine, an unreadable
// freshness marker only means "cannot tell", and the fail-closed answer to
// that is to send nothing, not to take the device down.
TEST(CompositeSystem, NonFiniteGenerationIsNotARefreshAndNotAFault) {
  for (const double broken : {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity()}) {
    TierHarness harness;
    harness.claim({TierHarness::motion(), TierHarness::generation()});
    harness.set_target(0.25);
    harness.set_generation(broken);
    harness.cycle(3U);

    EXPECT_EQ(harness.runtime->fresh_writes(0U), 0U);
    EXPECT_FALSE(harness.system.fault_latched());

    // And it must not have poisoned the baseline: a later finite generation
    // is still a change.
    harness.set_generation(1.0);
    harness.cycle(1U);
    EXPECT_EQ(harness.runtime->fresh_writes(0U), 1U);
  }
}

// ADR-017 negative space (revising ADR-014 Decision 2): exactly ONE motion
// command interface whose name is one of the three canonical kinds, plus
// exactly ONE command_generation interface. Widening the motion side into a
// permissive subset, and dropping or duplicating the generation side, both
// fail on_init.
//
// The generation interface being required in the URDF is not the same thing
// as a controller being required to claim it - the hardware always exports
// it, and claiming it stays the controller's choice. That is what keeps a
// standard ros2_control controller able to drive this hardware.
TEST(CompositeSystem, RejectsMalformedCommandInterfaceSets) {
  const auto named = [](const std::string& name) {
    hardware_interface::InterfaceInfo value;
    value.name = name;
    value.size = 1;
    return value;
  };
  const auto rejects = [](hardware_interface::HardwareInfo candidate) {
    CompositeSystem system;
    return system.on_init(candidate) ==
           hardware_interface::CallbackReturn::ERROR;
  };

  // Unknown motion-interface name.
  EXPECT_TRUE(rejects(info_with_command_interface("acceleration")));

  // Two motion interfaces (position + effort), the shape a permissive subset
  // would have let through.
  auto two_motion = info(1U);
  two_motion.joints[0].command_interfaces.push_back(
      named(hardware_interface::HW_IF_EFFORT));
  EXPECT_TRUE(rejects(two_motion));

  // No command interfaces at all.
  auto none = info(1U);
  none.joints[0].command_interfaces.clear();
  EXPECT_TRUE(rejects(none));

  // A motion interface with no generation interface - the pre-ADR-017 shape,
  // which a stale URDF would still carry. It must be refused rather than
  // silently running with an interface the hardware exports but the
  // deployment never declared.
  auto motion_only = info(1U);
  motion_only.joints[0].command_interfaces = {
      named(hardware_interface::HW_IF_POSITION)};
  EXPECT_TRUE(rejects(motion_only));

  // The generation interface alone commands nothing.
  auto generation_only = info(1U);
  generation_only.joints[0].command_interfaces = {
      named(kCommandGenerationInterface)};
  EXPECT_TRUE(rejects(generation_only));

  // Two generation interfaces on one joint.
  auto two_generations = info(1U);
  two_generations.joints[0].command_interfaces.push_back(
      named(kCommandGenerationInterface));
  EXPECT_TRUE(rejects(two_generations));
}

// ADR-017: the generation interface is ALWAYS exported, so the URDF shape a
// deployment declares is one motion command interface plus command_generation.
// Whether a controller claims the generation interface is its own choice -
// that is what keeps standard ros2_control controllers able to drive this
// hardware - but the hardware offers it unconditionally.
TEST(CompositeSystem, AcceptsAMotionInterfacePlusTheGenerationInterface) {
  CompositeSystem system;
  EXPECT_EQ(system.on_init(info(1U)),
            hardware_interface::CallbackReturn::SUCCESS);
}

// The loopback runtime mirrors a velocity/effort command into the matching
// state field deterministically (ADR-014): velocity commands hold the state
// velocity, effort commands hold the state effort, and neither invents
// values for the other fields.
TEST(CompositeSystem, LoopbackMirrorsVelocityAndEffortCommands) {
  {
    CompositeSystem system;
    ASSERT_EQ(system.on_init(info_with_command_interface(
                  hardware_interface::HW_IF_VELOCITY)),
              hardware_interface::CallbackReturn::SUCCESS);
    auto states = system.export_state_interfaces();
    auto commands = system.export_command_interfaces();
    ASSERT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(
        system.perform_command_mode_switch({"joint_1/velocity"}, {}),
        hardware_interface::return_type::OK);
    commands[0].set_value(0.4);
    ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(states[1].get_value(), 0.4);
    EXPECT_DOUBLE_EQ(states[2].get_value(), 0.0);
  }
  {
    CompositeSystem system;
    ASSERT_EQ(system.on_init(info_with_command_interface(
                  hardware_interface::HW_IF_EFFORT)),
              hardware_interface::CallbackReturn::SUCCESS);
    auto states = system.export_state_interfaces();
    auto commands = system.export_command_interfaces();
    ASSERT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(
        system.perform_command_mode_switch({"joint_1/effort"}, {}),
        hardware_interface::return_type::OK);
    commands[0].set_value(0.2);
    ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
              hardware_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(states[2].get_value(), 0.2);
    EXPECT_DOUBLE_EQ(states[1].get_value(), 0.0);
  }
}

// ADR-014: non-finite velocity/effort command values fault exactly like
// non-finite positions, on claimed and unclaimed joints alike.
TEST(CompositeSystem, RejectsNonFiniteVelocityAndEffortCommands) {
  for (const char* command_interface :
       {hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT}) {
    SCOPED_TRACE(command_interface);
    CompositeSystem system;
    ASSERT_EQ(system.on_init(info_with_command_interface(command_interface)),
              hardware_interface::CallbackReturn::SUCCESS);
    auto commands = system.export_command_interfaces();
    ASSERT_EQ(system.on_configure(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(state()),
              hardware_interface::CallbackReturn::SUCCESS);
    commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
    EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
              hardware_interface::return_type::ERROR);
    EXPECT_TRUE(system.fault_latched());
  }
}

TEST(CompositeSystem, RejectsInvalidInterfacesAndStrictSwitchConflicts) {
  auto invalid = info(1U);
  invalid.joints[0].command_interfaces[0].name = "acceleration";
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

// Regression test for a heap-buffer-overflow (ASan-confirmed): interface
// names are "<joint>/<interface>", so a joint name that itself contains '/'
// (e.g. "arm/j1" -> "arm/j1/position") made the old find('/')-based lookup
// resolve to the wrong joint substring, miss in joint_names_, and index
// claimed_ with joint_names_.size() (one past the end). This exercises both
// the start_interfaces and stop_interfaces loops in
// perform_command_mode_switch with a slash-bearing joint name.
TEST(CompositeSystem, HandlesJointNamesContainingSlashInModeSwitch) {
  CompositeSystem system;
  ASSERT_EQ(system.on_init(info_with_slash_joint_name()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);

  // start_interfaces loop: claim the slash-named joint's command interface.
  ASSERT_EQ(system.prepare_command_mode_switch({"arm/j1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch({"arm/j1/position"}, {}),
            hardware_interface::return_type::OK);

  // stop_interfaces loop: release it again via the same slash-named joint.
  ASSERT_EQ(system.prepare_command_mode_switch({}, {"arm/j1/position"}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(system.perform_command_mode_switch({}, {"arm/j1/position"}),
            hardware_interface::return_type::OK);
  EXPECT_FALSE(system.fault_latched());
}

TEST(CompositeSystem, RejectsNonFiniteOnUnclaimedCommandInterface) {
  CompositeSystem system;
  ASSERT_EQ(system.on_init(info(2U)), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);
  // Claim only joint_1; joint_2's command interface is left unclaimed.
  ASSERT_EQ(system.prepare_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);

  command_by_name(commands, "joint_2/position")
      .set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(system.fault_latched());
}

TEST(CompositeSystem, RejectsInfOnUnclaimedCommandInterface) {
  CompositeSystem system;
  ASSERT_EQ(system.on_init(info(2U)), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.prepare_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch({"joint_1/position"}, {}),
            hardware_interface::return_type::OK);

  command_by_name(commands, "joint_2/position")
      .set_value(std::numeric_limits<double>::infinity());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(system.fault_latched());
}

TEST(CompositeSystem, LatchesFaultWhenRuntimeReadReturnsNonFiniteState) {
  CompositeSystem system;
  ASSERT_TRUE(system.set_runtime(std::make_unique<NanReadRuntime>()));
  ASSERT_EQ(system.on_init(info(2U)), hardware_interface::CallbackReturn::SUCCESS);
  auto states = system.export_state_interfaces();
  ASSERT_EQ(system.on_configure(state()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(state()), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
            hardware_interface::return_type::ERROR);
  EXPECT_TRUE(system.fault_latched());
}

}  // namespace
}  // namespace mech::mech_hardware_ros2_control
