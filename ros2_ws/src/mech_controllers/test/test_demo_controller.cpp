#include "mech_controllers/demo_controller.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_control_core/command_contract.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace mech::mech_controllers {
namespace {

constexpr char kJoint[] = "motor1_joint";

[[nodiscard]] rclcpp_lifecycle::State lifecycle_state() {
  return rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, "test");
}

// Drives a real DemoController plugin instance with fake interfaces and a
// clock the test owns. Nothing here touches hardware or a controller_manager;
// what it pins is the controller's own contract.
class DemoControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    ASSERT_EQ(controller_.init("motor1_position_controller"),
              controller_interface::return_type::OK);
    auto node = controller_.get_node();
    node->set_parameter({"joint", std::string(kJoint)});
    node->set_parameter({"minimum", -1.0});
    node->set_parameter({"maximum", 1.0});
    node->set_parameter({"max_slew_per_second", 2.0});
    node->set_parameter({"ttl_nanoseconds", 4000000});
    node->set_parameter({"hard_ttl_nanoseconds", 6000000});
  }

  // Hands the controller the interfaces it claims: the joint's position command
  // and, per ADR-017, its command_generation - both backed by doubles the test
  // can read - plus the position state interface.
  void assign_interfaces() {
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(command_interface_);
    commands.emplace_back(generation_interface_);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(state_interface_);
    controller_.assign_interfaces(std::move(commands), std::move(states));
  }

  // now_ is the monotonic reading the controller sees; advance() steps it.
  void advance(std::int64_t nanoseconds) { now_ += nanoseconds; }

  // configure -> assign interfaces -> activate, with the test's clock.
  void activate_with_test_clock() {
    controller_.set_clock_for_testing([this]() { return now_; });
    ASSERT_EQ(controller_.on_configure(lifecycle_state()),
              controller_interface::CallbackReturn::SUCCESS);
    assign_interfaces();
    ASSERT_EQ(controller_.on_activate(lifecycle_state()),
              controller_interface::CallbackReturn::SUCCESS);
  }

  // Spins until `ready` holds or two seconds elapse. Bounded so a broken
  // subscription fails the test instead of hanging CI.
  [[nodiscard]] static bool spin_until(
      rclcpp::executors::SingleThreadedExecutor& executor,
      const std::function<bool()>& ready) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      if (ready()) return true;
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return ready();
  }

  // Activates the controller and stands up a publisher on its target topic,
  // so each test states only what it publishes and what it expects back.
  class Probe final {
   public:
    explicit Probe(DemoControllerTest& test) : test_(test) {
      test.activate_with_test_clock();
      node_ = std::make_shared<rclcpp::Node>("demo_controller_probe");
      publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
          "/motor1_position_controller/target_position",
          rclcpp::SystemDefaultsQoS());
      executor_.add_node(test.controller_.get_node()->get_node_base_interface());
      executor_.add_node(node_);
    }

    void publish(double value) {
      std_msgs::msg::Float64 message;
      message.data = value;
      publisher_->publish(message);
    }

    // Publishes and spins until the controller has accepted `expected`
    // targets in total, so tests never depend on delivery timing.
    void publish_and_wait(double value, std::uint64_t expected) {
      publish(value);
      ASSERT_TRUE(spin_until(executor_, [this, expected]() {
        return test_.controller_.target_generation() == expected;
      }));
    }

    // For asserting that nothing arrives: spins a bounded number of times so
    // a message that WAS going to be delivered has had its chance.
    void spin_briefly() {
      for (int index = 0; index < 50; ++index) {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

   private:
    DemoControllerTest& test_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::executors::SingleThreadedExecutor executor_;
  };

  controller_interface::return_type cycle() {
    return controller_.update(frozen_ros_time_, kPeriod);
  }

  static constexpr std::int64_t kPeriodNanoseconds = 2000000;
  const rclcpp::Duration kPeriod{std::chrono::nanoseconds(kPeriodNanoseconds)};
  // A deployment with a frozen use_sim_time hands update() the same stamp
  // forever. The watchdog must not depend on it.
  const rclcpp::Time frozen_ros_time_{1000000000};

  double command_value_{0.0};
  double generation_value_{0.0};
  double state_value_{0.25};
  hardware_interface::CommandInterface command_interface_{
      kJoint, hardware_interface::HW_IF_POSITION, &command_value_};
  hardware_interface::CommandInterface generation_interface_{
      kJoint, mech::mech_control_core::kCommandGenerationInterface,
      &generation_value_};
  hardware_interface::StateInterface state_interface_{
      kJoint, hardware_interface::HW_IF_POSITION, &state_value_};
  // A plausible steady_clock reading, so no test can pass by accident on a
  // small absolute time.
  std::int64_t now_{113000000000000};
  DemoController controller_;
};

TEST(TargetLimiter, ClampsSlewWhileFollowing) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  EXPECT_EQ(limiter.stage(50), WatchdogStage::Following);
  EXPECT_DOUBLE_EQ(limiter.update(0.0, 0.25, 50), 0.5);
  EXPECT_FALSE(limiter.submit(std::numeric_limits<double>::quiet_NaN(), 0));
}

TEST(TargetLimiter, RejectsInvalidLimitsAndOverflow) {
  TargetLimiter limiter;
  EXPECT_FALSE(limiter.configure(BoundedTarget{1.0, -1.0, 1.0, 10, 20}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 0.0, 10, 20}));
  EXPECT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 10, 20}));
  EXPECT_FALSE(limiter.submit(0.0, std::numeric_limits<std::int64_t>::max()));
}

TEST(TargetLimiter, RejectsHardTtlNotGreaterThanTtl) {
  TargetLimiter limiter;
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 100}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 50}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 0}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, -1}));
  EXPECT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 101}));
}

TEST(TargetLimiter, HoldsLastValidCommandBetweenTtlAndHardTtl) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  // Follow up to a non-zero, non-target value so "hold" and "go to zero" (the
  // old behaviour) and "keep following the target" are all distinguishable.
  const auto followed = limiter.update(0.2, 0.25, 50);
  EXPECT_DOUBLE_EQ(followed, 0.7);
  ASSERT_EQ(limiter.stage(100), WatchdogStage::Holding);
  const auto held = limiter.update(followed, 0.25, 100);
  EXPECT_DOUBLE_EQ(held, followed);
  EXPECT_NE(held, 0.0);
  // Holding must stay frozen across further cycles, not creep toward 0 or
  // resume slewing toward the stale target.
  const auto held_again = limiter.update(held, 0.25, 150);
  EXPECT_DOUBLE_EQ(held_again, followed);
}

TEST(TargetLimiter, ReportsExpiredStageAtHardTtl) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  EXPECT_EQ(limiter.stage(299), WatchdogStage::Holding);
  EXPECT_EQ(limiter.stage(300), WatchdogStage::Expired);
}

// The watchdog measures the staleness of a target that exists. A target that
// was never submitted cannot be stale, so "activated, waiting for the first
// command" must not be reported as Expired.
//
// This only shows up under a real clock. `now` is nanoseconds since boot, and
// the no-target window used to be measured from absolute zero, so any real
// reading was already past the 6 ms hard TTL: the very first update() after
// activation returned ERROR and the manager deactivated a controller that had
// done nothing wrong. With frozen test time at 0 the bug was invisible.
TEST(TargetLimiter, NoTargetUnderARealClockHoldsRatherThanExpiring) {
  TargetLimiter limiter;
  ASSERT_TRUE(
      limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 4000000, 6000000}));
  // A plausible steady_clock reading - about 31 hours of uptime.
  constexpr std::int64_t kUptimeNanoseconds = 113000000000000;
  EXPECT_EQ(limiter.stage(kUptimeNanoseconds), WatchdogStage::Holding);
  // And it must hold the activation seed rather than slew toward the
  // default-constructed target of 0.0, which on a position interface is a
  // commanded move to the calibrated zero.
  EXPECT_DOUBLE_EQ(limiter.update(0.42, 0.002, kUptimeNanoseconds), 0.42);
}

TEST(TargetLimiter, NonFiniteInputHoldsLastValidValueNotZero) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  const auto followed = limiter.update(0.2, 0.25, 50);
  EXPECT_DOUBLE_EQ(followed, 0.7);
  const auto held = limiter.update(std::numeric_limits<double>::quiet_NaN(), 0.25, 60);
  EXPECT_DOUBLE_EQ(held, followed);
  EXPECT_NE(held, 0.0);
  const auto held_negative_period = limiter.update(followed, -1.0, 70);
  EXPECT_DOUBLE_EQ(held_negative_period, followed);
}

TEST(DemoControllerConfigure, RejectsHardTtlNotGreaterThanTtl) {
  BoundedTarget limits{-1.0, 1.0, 1.0, 100, 100};
  TargetLimiter limiter;
  EXPECT_FALSE(limiter.configure(limits));
}

// The watchdog must be measured against a clock nobody can pause. update() is
// handed an rclcpp::Time that may be ROS time, and a deployment with a frozen
// use_sim_time hands it the same stamp on every cycle - so deriving the
// deadline from it means a stale target never lapses and stays alive on the
// motor indefinitely, while every TTL test still passes vacuously.
//
// Here ROS time is frozen and only the injected monotonic clock advances, well
// past the 6 ms hard TTL. The target must expire.
TEST_F(DemoControllerTest, FrozenRosTimeStillLetsAStaleTargetExpire) {
  controller_.set_clock_for_testing([this]() { return now_; });
  ASSERT_EQ(controller_.on_configure(lifecycle_state()),
            controller_interface::CallbackReturn::SUCCESS);
  assign_interfaces();
  ASSERT_EQ(controller_.on_activate(lifecycle_state()),
            controller_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(controller_.set_target(0.5));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);

  // Ten cycles at 2 ms = 20 ms of monotonic time, far past the 6 ms hard TTL,
  // while the ROS stamp never moves.
  controller_interface::return_type last = controller_interface::return_type::OK;
  for (int index = 0; index < 10; ++index) {
    advance(kPeriodNanoseconds);
    last = cycle();
  }
  EXPECT_EQ(last, controller_interface::return_type::ERROR);
}

// The controller's target must be reachable over ROS, not only through a C++
// call nothing in a deployment can make. The topic is controller-relative so
// its full name follows the controller's namespace.
TEST_F(DemoControllerTest, AcceptsATargetFromItsTopic) {
  activate_with_test_clock();

  auto publisher_node = std::make_shared<rclcpp::Node>("demo_controller_probe");
  auto publisher = publisher_node->create_publisher<std_msgs::msg::Float64>(
      "/motor1_position_controller/target_position",
      rclcpp::SystemDefaultsQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_.get_node()->get_node_base_interface());
  executor.add_node(publisher_node);

  std_msgs::msg::Float64 message;
  message.data = 0.5;
  publisher->publish(message);
  ASSERT_TRUE(spin_until(executor,
                         [this]() { return controller_.target_generation() == 1U; }));

  advance(kPeriodNanoseconds);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  // Seeded at 0.25 and slewing toward 0.5 at 2.0 rad/s over a 2 ms cycle, so
  // one step of 0.004. Asserting movement in the right direction rather than
  // the exact value keeps this about the topic entry, not the slew arithmetic.
  EXPECT_GT(command_value_, 0.25);
  EXPECT_LE(command_value_, 0.5);
}

// The activation seed becomes the held command, so it has to be a number that
// was really measured. ADR-016 already refuses the claim while no valid sample
// exists, but this controller must not depend on that reasoning holding
// elsewhere: the only alternative to a measured seed is a fabricated one, and
// on a position interface a fabricated 0.0 is a commanded move to the
// calibrated zero.
TEST_F(DemoControllerTest, RefusesToActivateOnNonFiniteFeedback) {
  controller_.set_clock_for_testing([this]() { return now_; });
  ASSERT_EQ(controller_.on_configure(lifecycle_state()),
            controller_interface::CallbackReturn::SUCCESS);
  state_value_ = std::numeric_limits<double>::quiet_NaN();
  assign_interfaces();
  EXPECT_EQ(controller_.on_activate(lifecycle_state()),
            controller_interface::CallbackReturn::ERROR);
}

// The TTL measures how long ago a controller last said something, so a message
// carrying the same number as the last one is still something said. If refresh
// were keyed on the value changing, a publisher holding a steady setpoint would
// look identical to a publisher that died.
TEST_F(DemoControllerTest, AnUnchangedValueStillRefreshesTheTtl) {
  Probe probe(*this);
  probe.publish_and_wait(0.5, 1U);

  // 4 ms in: past the soft TTL, inside the 6 ms hard TTL.
  advance(4000000);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);

  // The same value again. Without a refresh the original hard deadline lands
  // 2 ms from here and the cycles below would fail.
  probe.publish_and_wait(0.5, 2U);
  for (int index = 0; index < 2; ++index) {
    advance(kPeriodNanoseconds);
    EXPECT_EQ(cycle(), controller_interface::return_type::OK) << "cycle " << index;
  }
}

// The mirror of the rule above: only a message may refresh the TTL. Turning
// the control loop cannot, or a silent publisher would be indistinguishable
// from a live one for as long as the manager keeps cycling.
TEST_F(DemoControllerTest, BareControlCyclesDoNotRefreshTheTtl) {
  Probe probe(*this);
  probe.publish_and_wait(0.5, 1U);

  for (int index = 0; index < 4; ++index) {
    advance(kPeriodNanoseconds);
    (void)cycle();
  }
  // Eight milliseconds of cycling, no new message: past the 6 ms hard TTL.
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_EQ(controller_.target_generation(), 1U);
}

// A NaN or infinite target is a broken publisher. Clamping it to the limit
// would turn that bug into a full-scale motion command, so it is dropped
// before it can become a target at all. The trailing valid message is the
// sentinel that proves delivery really happened.
TEST_F(DemoControllerTest, RejectsNonFiniteTargetsFromTheTopic) {
  Probe probe(*this);
  probe.publish(std::numeric_limits<double>::quiet_NaN());
  probe.publish(std::numeric_limits<double>::infinity());
  probe.spin_briefly();
  // Nothing was accepted at all. Asserted on its own rather than alongside a
  // valid message: pairing them made the assertion depend on which message the
  // executor delivered first, and a mutation removing the filter still passed.
  EXPECT_EQ(controller_.target_generation(), 0U);

  // And the controller is still usable afterwards - the filter drops the
  // message, it does not wedge the entry.
  probe.publish_and_wait(0.5, 1U);
  advance(kPeriodNanoseconds);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_GT(command_value_, 0.25);
}

// A target published while the joint belonged to nobody is not a target for
// the next activation. Dropping it at the subscription is what stops a
// re-activation from replaying it.
TEST_F(DemoControllerTest, DiscardsTargetsPublishedWhileDeactivated) {
  Probe probe(*this);
  ASSERT_EQ(controller_.on_deactivate(lifecycle_state()),
            controller_interface::CallbackReturn::SUCCESS);

  probe.publish(0.9);
  probe.spin_briefly();
  EXPECT_EQ(controller_.target_generation(), 0U);

  ASSERT_EQ(controller_.on_activate(lifecycle_state()),
            controller_interface::CallbackReturn::SUCCESS);
  advance(kPeriodNanoseconds);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  // Still holding the activation seed: nothing was replayed.
  EXPECT_DOUBLE_EQ(command_value_, 0.25);
}

// ADR-017 Decision 9: this project's own controllers are in the strong tier, so
// DemoController must ASK for the generation interface. If it ever stops
// claiming it, the hardware silently drops this controller into the weak tier
// and a silent DemoController goes back to holding the motor on its last target
// with nothing able to notice. Names are checked, not just the count, because
// the hardware matches on the joint-qualified name.
TEST_F(DemoControllerTest, ClaimsTheGenerationInterfaceAlongsideTheMotionOne) {
  ASSERT_EQ(controller_.on_configure(lifecycle_state()),
            controller_interface::CallbackReturn::SUCCESS);
  const auto config = controller_.command_interface_configuration();
  EXPECT_EQ(config.type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(config.names,
            (std::vector<std::string>{
                std::string(kJoint) + "/" + hardware_interface::HW_IF_POSITION,
                std::string(kJoint) + "/" +
                    mech::mech_control_core::kCommandGenerationInterface}));
}

// The behaviour the whole ADR-017 mechanism rests on, stated from the
// controller's side: the generation changes when and only when this controller
// has something new to say. A manager cycle on its own must not move it.
//
// Read together with the hardware half. CompositeSystem treats an unchanged
// generation as "not refreshed" and the runtime then sends nothing
// (SilentControllerDoesNotRefreshItsCommand, in mech_bringup, drives that under
// a real ControllerManager). This test is what makes DemoController qualify for
// that protection rather than merely be eligible for it.
TEST_F(DemoControllerTest, GenerationChangesOnlyWhileFollowingALiveTarget) {
  Probe probe(*this);

  // Activated, no target yet. The controller writes its activation seed to hold
  // position, but that is a held value rather than a command - the limiter is
  // Holding - so the hardware must not be told anything was refreshed.
  advance(kPeriodNanoseconds);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const double before_any_target = generation_value_;
  advance(kPeriodNanoseconds);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(generation_value_, before_any_target)
      << "a bare manager cycle refreshed the generation";

  // A target arrives: every cycle spent slewing toward it is a real command, so
  // every one of them must read as new.
  probe.publish_and_wait(0.5, 1U);
  advance(kPeriodNanoseconds);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const double first = generation_value_;
  EXPECT_NE(first, before_any_target);
  advance(kPeriodNanoseconds);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NE(generation_value_, first) << "a following cycle was not a refresh";

  // Now the publisher falls silent past the soft TTL. The limiter freezes the
  // command, and the generation must freeze with it - otherwise this controller
  // would keep the hardware's watchdog satisfied while having nothing to say,
  // which is exactly the defect ADR-017 exists to close.
  //
  // The target was submitted two cycles ago, so this lands 4 ms after it: past
  // the 4 ms soft TTL and inside the 6 ms hard one. That is Holding. Going a
  // cycle further would reach Expired, where update() refuses outright and
  // never gets as far as the question this test asks.
  const double before_silence = generation_value_;
  const double frozen_command = command_value_;
  advance(2000000);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(generation_value_, before_silence)
      << "a silent controller still claimed a refresh";
  EXPECT_DOUBLE_EQ(command_value_, frozen_command);
}

}  // namespace
}  // namespace mech::mech_controllers
