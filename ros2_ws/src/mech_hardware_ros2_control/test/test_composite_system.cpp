#include "mech_hardware_ros2_control/composite_system.hpp"

#include <limits>
#include <memory>
#include <vector>

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
      last_command_[index] = commands[index].command;
    }
    return true;
  }

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
    ASSERT_EQ(commands.size(), 1U);
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

// ADR-014 negative space: exactly ONE command interface per joint, and its
// name must be one of the three canonical kinds. Two interfaces (any pair)
// and unknown names both fail on_init - this pins the single-command rule
// against a future widening into a permissive subset.
TEST(CompositeSystem, RejectsMultipleOrUnknownCommandInterfaces) {
  // Unknown command-interface name.
  auto unknown = info_with_command_interface("acceleration");
  CompositeSystem rejected_unknown;
  EXPECT_EQ(rejected_unknown.on_init(unknown),
            hardware_interface::CallbackReturn::ERROR);

  // Two command interfaces on one joint (position + effort, the shape a
  // three-interface superset would have accepted).
  auto doubled = info(1U);
  hardware_interface::InterfaceInfo extra;
  extra.name = hardware_interface::HW_IF_EFFORT;
  extra.size = 1;
  doubled.joints[0].command_interfaces.push_back(extra);
  CompositeSystem rejected_doubled;
  EXPECT_EQ(rejected_doubled.on_init(doubled),
            hardware_interface::CallbackReturn::ERROR);

  // Zero command interfaces is equally invalid.
  auto none = info(1U);
  none.joints[0].command_interfaces.clear();
  CompositeSystem rejected_none;
  EXPECT_EQ(rejected_none.on_init(none),
            hardware_interface::CallbackReturn::ERROR);
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

  commands[1].set_value(std::numeric_limits<double>::quiet_NaN());
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

  commands[1].set_value(std::numeric_limits<double>::infinity());
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
