#include "mech_controllers/resistance_training_controller.hpp"

#include <limits>
#include <cmath>
#include <utility>
#include <vector>
#include <gtest/gtest.h>
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "mech_control_core/command_contract.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mech::mech_controllers {
namespace {
class ResistanceControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    ASSERT_EQ(c.init("resistance_test"), controller_interface::return_type::OK);
    auto node = c.get_node();
    node->set_parameters({{"minimum", 0.0}, {"maximum", 1.0},
      {"max_slew_per_second", 1000000.0}, {"damping", 8.0},
      {"torque_limit_nm", 8.0}, {"buildup_nm_per_second", 8.0},
      {"direction_confirmation_s", 0.0}, // Individual law tests bypass qualification.
      {"position_min_rad", -1.0}, {"position_max_rad", 1.0}});
    c.set_clock_for_testing([this]() { return now; });
  }
  void activate() {
    ASSERT_EQ(c.on_configure(state), controller_interface::CallbackReturn::SUCCESS);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(effort_interface); commands.emplace_back(generation_interface);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(position_interface); states.emplace_back(velocity_interface);
    c.assign_interfaces(std::move(commands), std::move(states));
    ASSERT_EQ(c.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
  }
  controller_interface::return_type cycle(std::int64_t elapsed = 2000000) {
    now += elapsed;
    return c.update(rclcpp::Time(0), rclcpp::Duration(0, static_cast<std::uint32_t>(elapsed)));
  }
  rclcpp_lifecycle::State state;
  std::int64_t now{1000000000};
  double effort{9.0}, generation{17.0}, position{0.0}, velocity{0.0};
  hardware_interface::CommandInterface effort_interface{"joint_1", "effort", &effort};
  hardware_interface::CommandInterface generation_interface{
    "joint_1", mech::mech_control_core::kCommandGenerationInterface, &generation};
  hardware_interface::StateInterface position_interface{"joint_1", "position", &position};
  hardware_interface::StateInterface velocity_interface{"joint_1", "velocity", &velocity};
  ResistanceTrainingController c;
};

TEST_F(ResistanceControllerTest, ClaimsGenericEncoderAndEffortInterfaces) {
  activate();
  EXPECT_EQ(c.state_interface_configuration().names,
            (std::vector<std::string>{"joint_1/position", "joint_1/velocity"}));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0);
  EXPECT_DOUBLE_EQ(generation, 17.0);  // Activation is not an enable permission.
  EXPECT_FALSE(c.set_target(2.0)); EXPECT_FALSE(c.set_target(-1.0));
  EXPECT_FALSE(c.set_target(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(c.target_generation(), 0U);
}

TEST_F(ResistanceControllerTest, RecomputesEveryTwoMsBetweenFiftyHzPermissions) {
  activate();
  ASSERT_TRUE(c.set_target(1.0)); velocity = .5;
  double previous = 0.0;
  for (int i = 1; i <= 10; ++i) {
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_LT(effort, previous);
    EXPECT_NEAR(effort, -.016 * i, 1e-12);
    previous = effort;
  }
  EXPECT_EQ(c.target_generation(), 1U); // ONE upstream permission, TEN local decisions.
  EXPECT_DOUBLE_EQ(generation, 10.0);
}

TEST_F(ResistanceControllerTest, MatchesFilteredViscousLawWhenBuildupIsNotLimiting) {
  c.get_node()->set_parameter({"buildup_nm_per_second", 10000.0});
  activate(); ASSERT_TRUE(c.set_target(1.0)); velocity = -.1;
  for (int i = 1; i <= 20; ++i) {
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_NEAR(effort, .8 * (1.0 - std::exp(-.002 * i / .025)), 1e-12);
  }
}

TEST_F(ResistanceControllerTest, ProtocolRangeProfileAllowsTrainingAboveOneButRetainsTorqueAndPosition) {
  c.get_node()->set_parameters({{"max_velocity_rad_s", 50.0}, {"damping", 12.0},
    {"buildup_nm_per_second", 20.0}});
  activate(); velocity = 1.1;
  for (int i = 0; i < 250; ++i) {
    if (i % 10 == 0) { ASSERT_TRUE(c.set_target(1.0)); }
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_LE(std::abs(effort), 8.0);
    EXPECT_LE(effort * velocity, 0.0);
    if (i == 39) { EXPECT_NEAR(effort, -1.6, 1e-12); }
  }
  EXPECT_DOUBLE_EQ(effort, -8.0);
  const double prior_generation = generation;
  position = 1.01;
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, prior_generation);
}

TEST_F(ResistanceControllerTest, DoubleProfileDoublesUnsaturatedResistanceForIdenticalVelocity) {
  c.get_node()->set_parameters({{"max_velocity_rad_s", 50.0}, {"damping", 24.0},
    {"buildup_nm_per_second", 40.0}});
  activate(); velocity = 0.1;
  for (int i = 1; i <= 40; ++i) {
    if (i % 10 == 1) { ASSERT_TRUE(c.set_target(1.0)); }
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    // Closed-form step response, including the prior buildup bound. Torque
    // saturation is not reached, so scaling BOTH B and buildup doubles output.
    const double filtered = 1.2 * (1.0 - std::exp(-.002 * i / .025));
    const double prior = -std::min(filtered, .04 * i);
    EXPECT_NEAR(effort, 2.0 * prior, 1e-12);
    EXPECT_LE(std::abs(effort), 8.0);
  }
}

TEST_F(ResistanceControllerTest, RestReversalAndPermissionZeroWithdrawImmediately) {
  activate(); ASSERT_TRUE(c.set_target(1.0)); velocity = .5;
  for (int i = 0; i < 10; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  velocity = -.5;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); EXPECT_DOUBLE_EQ(effort, 0.0);
  for (int i = 0; i < 10; ++i) {
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_LE(effort * velocity, 0.0); // A lagging signed observer cannot assist.
  }
  EXPECT_GT(effort, 0.0);
  velocity = .01221;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); EXPECT_DOUBLE_EQ(effort, 0.0);
  velocity = .03663;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); EXPECT_DOUBLE_EQ(effort, 0.0);
  velocity = .5;
  for (int i = 0; i < 10; ++i) {
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_LE(effort * velocity, 0.0);
  }
  EXPECT_LT(effort, 0.0);
  ASSERT_TRUE(c.set_target(0.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); EXPECT_DOUBLE_EQ(effort, 0.0);
}

TEST_F(ResistanceControllerTest, RejectsShortAlternatingMotionButRetainsSteadyDamping) {
  c.get_node()->set_parameters({{"direction_confirmation_s", .02},
    {"damping", 24.0}, {"buildup_nm_per_second", 40.0}});
  activate();
  // 10 ms oscillatory half-cycles are not established training motion.
  for (int i = 0; i < 100; ++i) {
    if (i % 10 == 0) { ASSERT_TRUE(c.set_target(1.0)); }
    velocity = (i / 5) % 2 == 0 ? .3 : -.3;
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(effort, 0.0);
  }
  velocity = .3;
  for (int i = 0; i < 200; ++i) {
    if (i % 10 == 0) { ASSERT_TRUE(c.set_target(1.0)); }
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    if (i < 9) { EXPECT_DOUBLE_EQ(effort, 0.0); }
    EXPECT_LE(effort * velocity, 0.0);
  }
  EXPECT_NEAR(effort, -7.2, 1e-6); // B24 retained for sustained training motion.
  velocity = -.3;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0);
}

TEST_F(ResistanceControllerTest, DirectionConfirmationUsesElapsedTimeAndRestCancelsIt) {
  c.get_node()->set_parameter({"direction_confirmation_s", .02});
  activate(); ASSERT_TRUE(c.set_target(1.0)); velocity = .3;
  ASSERT_EQ(cycle(5000000), controller_interface::return_type::OK);
  ASSERT_EQ(cycle(5000000), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0);
  velocity = .01221; // 10 ms before rest cannot accumulate into the next gesture.
  ASSERT_EQ(cycle(5000000), controller_interface::return_type::OK);
  velocity = .3;
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(cycle(5000000), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(effort, 0.0);
  }
  ASSERT_EQ(cycle(5000000), controller_interface::return_type::OK);
  EXPECT_LT(effort, 0.0); // Four 5 ms samples; not a fixed ten-cycle counter.
}

TEST_F(ResistanceControllerTest, FadesBeforeRestInsteadOfRetainingLaggingTorque) {
  c.get_node()->set_parameter({"direction_confirmation_s", .02});
  activate(); velocity = .2;
  for (int i = 0; i < 100; ++i) {
    if (i % 10 == 0) { ASSERT_TRUE(c.set_target(1.0)); }
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  }
  double previous = std::abs(effort);
  for (const double speed : {.06, .05, .04, .03, .021, .01221}) {
    velocity = speed;
    ASSERT_TRUE(c.set_target(1.0));
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_LE(std::abs(effort), previous);
    EXPECT_LE(std::abs(effort), 8.0 * speed); // No stale high-speed force at low speed.
    EXPECT_LE(effort * velocity, 0.0);
    previous = std::abs(effort);
  }
  EXPECT_DOUBLE_EQ(effort, 0.0);
  velocity = .3;
  for (int i = 0; i < 9; ++i) {
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(effort, 0.0); // Rest requires renewed direction qualification.
  }
  ASSERT_TRUE(c.set_target(0.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0);
}

TEST_F(ResistanceControllerTest, RejectsInvalidDirectionQualificationTime) {
  for (const double value : {-.001, .101, std::numeric_limits<double>::quiet_NaN()}) {
    c.get_node()->set_parameter({"direction_confirmation_s", value});
    EXPECT_EQ(c.on_configure(state), controller_interface::CallbackReturn::ERROR);
  }
}

TEST_F(ResistanceControllerTest, PermissionExpiryStopsGenerationAndCannotAutomaticallyRecover) {
  activate(); ASSERT_TRUE(c.set_target(1.0)); velocity = .5;
  for (int i = 0; i < 49; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const double before = generation;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); // 100ms holding.
  EXPECT_DOUBLE_EQ(generation, before); EXPECT_DOUBLE_EQ(effort, 0.0);
  for (int i = 0; i < 2; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR); //106ms.
  ASSERT_TRUE(c.set_target(1.0));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, before);
}

TEST_F(ResistanceControllerTest, LimitsEffortAndAppliesDirectionalAngleMargin) {
  c.get_node()->set_parameters({{"damping", 100.0}, {"torque_limit_nm", 2.0},
                               {"buildup_nm_per_second", 10000.0}});
  activate(); ASSERT_TRUE(c.set_target(1.0)); velocity = -.5;
  for (int i = 0; i < 20; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 2.0);
  position = .999;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NEAR(effort, .1, 1e-12); // 2 * .001/.02; outward positive torque.
}

TEST_F(ResistanceControllerTest, InvalidFeedbackLatchesAndDoesNotRefreshGeneration) {
  activate(); ASSERT_TRUE(c.set_target(1.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const double before = generation;
  velocity = 1.01;
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  velocity = .1; ASSERT_TRUE(c.set_target(1.0));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, before);
}

TEST_F(ResistanceControllerTest, AbsoluteAngleAndNonFiniteFeedbackRejectBeforeWriting) {
  activate(); ASSERT_TRUE(c.set_target(1.0)); position = 1.01;
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, 17.0);
  ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
  position = 0.; ASSERT_EQ(c.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(c.set_target(1.0)); velocity = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, 17.0);
}

TEST_F(ResistanceControllerTest, ReconfigureAndActivationRequireMeasuredRangeAndStationaryEncoder) {
  c.get_node()->set_parameter({"position_max_rad", -1.0});
  EXPECT_EQ(c.on_configure(state), controller_interface::CallbackReturn::ERROR);
  c.get_node()->set_parameter({"position_max_rad", 1.0});
  activate(); ASSERT_TRUE(c.set_target(1.0)); ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
  velocity = .1;
  EXPECT_EQ(c.on_activate(state), controller_interface::CallbackReturn::ERROR);
  velocity = 0.;
  ASSERT_EQ(c.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0); // No replayed permission after a re-claim.
}

TEST_F(ResistanceControllerTest, MonotonicOverrunAndRunLimitIgnoreFrozenRosTime) {
  c.get_node()->set_parameter({"run_limit_s", .01});
  activate(); ASSERT_TRUE(c.set_target(1.0)); velocity = .1;
  for (int i = 0; i < 5; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
  velocity = 0.; ASSERT_EQ(c.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(c.set_target(1.0)); ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  now += 7000000; // Lie in the manager period; actual elapsed steady time still rejects.
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
}
}  // namespace
}  // namespace mech::mech_controllers
