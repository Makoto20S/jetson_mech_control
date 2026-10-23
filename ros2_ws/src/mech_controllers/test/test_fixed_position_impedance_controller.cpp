#include "mech_controllers/fixed_position_impedance_controller.hpp"

#include <chrono>
#include <thread>
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
class FixedImpedanceTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    ASSERT_EQ(c.init("resistance_test"), controller_interface::return_type::OK);
    auto node = c.get_node();
    node->set_parameters({{"minimum", 0.0}, {"maximum", 1.0},
      {"max_slew_per_second", 1000000.0},
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
  FixedPositionImpedanceController c;
};

TEST_F(FixedImpedanceTest, StructuredRosSubscriptionHasNoScalarBypass) {
  c.get_node()->set_parameter({"command_source", "training_command"}); activate();
  auto node = c.get_node();
  EXPECT_EQ(node->count_subscribers("/resistance_test/enable"), 0U);
  auto producer = std::make_shared<rclcpp::Node>("training_command_producer");
  auto publisher = producer->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/resistance_test/training_command", rclcpp::QoS(1));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(producer);
  std_msgs::msg::Float64MultiArray message;
  message.data = {1., 1., static_cast<double>(now)*1e-9, 1., 0., 10., .25, 2.};
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (c.target_generation() == 0U && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(message); executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(c.target_generation(), 1U);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  position = .1;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, -1.);
  // Repeated old frames cannot refresh the shared target lease.
  publisher->publish(message); executor.spin_some();
  EXPECT_EQ(c.target_generation(), 1U);
}

TEST_F(FixedImpedanceTest, StructuredPlanOwnsReferenceGainsAndSaturationAtomically) {
  c.get_node()->set_parameter({"command_source", "training_command"});
  position = .2; activate();
  auto frame = std::array<double, 8>{1., 1., static_cast<double>(now)*1e-9, 1., .205, 10., .25, 2.};
  EXPECT_FALSE(c.set_target(1.0)); // Scalar permission cannot bypass the contract.
  ASSERT_TRUE(c.set_training_command(frame));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NEAR(effort, .05, 1e-12); // Uses upper reference, not activation's .2.
  position = .3; velocity = .4;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NEAR(effort, -1.05, 1e-12); // Recomputed locally without another packet.
  position = .6;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, -2.);
  EXPECT_FALSE(c.set_training_command(frame)); // Replay does not renew lease.
  frame[1] = 2.; frame[2] = static_cast<double>(now)*1e-9; frame[3] = 0.;
  ASSERT_TRUE(c.set_training_command(frame));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.);
}

TEST_F(FixedImpedanceTest, StructuredPlanRejectsMalformedStaleAndExcessiveInputs) {
  c.get_node()->set_parameter({"command_source", "training_command"}); activate();
  const std::array<double, 8> valid{1., 1., static_cast<double>(now)*1e-9, 1., 0., 15., .5, 7.};
  for (const auto& mutation : std::vector<std::pair<std::size_t, double>>{
      {0, 2.}, {1, .5}, {1, 0.}, {2, .8}, {2, 1.1}, {3, .5}, {4, .99},
      {5, 16.}, {6, .6}, {7, 8.}, {5, std::numeric_limits<double>::quiet_NaN()}}) {
    auto frame = valid; frame[mutation.first] = mutation.second;
    EXPECT_FALSE(c.set_training_command(frame));
  }
  ASSERT_TRUE(c.set_training_command(valid));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  for (int i=0; i<49; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.);
  const auto before = generation;
  for (int i=0; i<3; ++i) (void)cycle();
  auto frame = valid; frame[1] = 2.; frame[2] = static_cast<double>(now)*1e-9;
  ASSERT_TRUE(c.set_training_command(frame));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, before);
}

TEST_F(FixedImpedanceTest, SourceAgeCannotBeExtendedByDelayedDelivery) {
  c.get_node()->set_parameter({"command_source", "training_command"}); activate();
  std::array<double, 8> frame{1., 1., static_cast<double>(now)*1e-9, 1., 0., 15., .5, 7.};
  now += 90000000; // Transport delivery 90 ms after the plan was issued.
  ASSERT_TRUE(c.set_training_command(frame));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  position = .1;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, -1.5);
  for (int i=0; i<5; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0); // Source age wins over a fresh arrival lease.
}

TEST_F(FixedImpedanceTest, StructuredPlanCannotMoveAnchorOrGainsDuringActivation) {
  c.get_node()->set_parameter({"command_source", "training_command"}); activate();
  std::array<double, 8> frame{1., 1., static_cast<double>(now)*1e-9, 1., 0., 15., .5, 7.};
  ASSERT_TRUE(c.set_training_command(frame));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  frame[1] = 2.; frame[2] = static_cast<double>(now)*1e-9; frame[5] = 14.;
  ASSERT_TRUE(c.set_training_command(frame));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  frame[1] = 3.; frame[2] = static_cast<double>(now)*1e-9; frame[5] = 15.;
  ASSERT_TRUE(c.set_training_command(frame));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
}

TEST_F(FixedImpedanceTest, StructuredPlanRequiresNearStationaryStartReference) {
  c.get_node()->set_parameter({"command_source", "training_command"}); activate();
  std::array<double, 8> frame{1., 1., static_cast<double>(now)*1e-9, 1., .1, 15., .5, 7.};
  ASSERT_TRUE(c.set_training_command(frame));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, 17.);
}

TEST_F(FixedImpedanceTest, StructuredPlanRejectsPacketsFromPreviousActivation) {
  c.get_node()->set_parameter({"command_source", "training_command"}); activate();
  std::array<double, 8> frame{1., 1., static_cast<double>(now)*1e-9, 1., 0., 15., .5, 7.};
  ASSERT_TRUE(c.set_training_command(frame));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(c.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
  frame[1] = 2.;
  EXPECT_FALSE(c.set_training_command(frame));
  frame[2] = static_cast<double>(now)*1e-9;
  ASSERT_TRUE(c.set_training_command(frame));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
}

TEST_F(FixedImpedanceTest, ActivationDoesNotAuthorizeAndPermissionIsNotPosition) {
  activate();
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0); EXPECT_DOUBLE_EQ(generation, 17.0);
  EXPECT_FALSE(c.set_target(.5)); EXPECT_FALSE(c.set_target(-1.0));
  EXPECT_FALSE(c.set_target(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(c.state_interface_configuration().names,
      (std::vector<std::string>{"joint_1/position", "joint_1/velocity"}));
}

TEST_F(FixedImpedanceTest, MatchesStm32FloatKernelAcrossPositionsAndSpeeds) {
  position = .125; activate();
  // Independent source-style float arithmetic (STM32 source clamp is 7 Nm).
  for (const double q : {-.8, -.1, .125, .3, .8}) {
    for (const double v : {-.8, 0.0, .8}) {
      position = q; velocity = v; ASSERT_TRUE(c.set_target(1.0));
      ASSERT_EQ(cycle(), controller_interface::return_type::OK);
      const float raw = 15.0f * (.125f - static_cast<float>(q)) - .5f * static_cast<float>(v);
      const float expected = std::fmax(-7.0f, std::fmin(7.0f, raw));
      EXPECT_NEAR(effort, expected, 2e-6);
    }
  }
}

TEST_F(FixedImpedanceTest, HoldsStaticDisplacementAndRestoresAnchorWithoutRecentering) {
  position = .2; activate(); ASSERT_TRUE(c.set_target(1.0));
  position = .4;
  for (int i = 0; i < 30; ++i) {
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_NEAR(effort, -3.0, 1e-12); // v=0 still produces restoring force.
  }
  velocity = -.1;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NEAR(effort, -2.95, 1e-12); // Spring returns stored energy toward anchor.
  EXPECT_GT(effort * velocity, 0.0); // Not a viscous-only no-assist law.
  ASSERT_TRUE(c.set_target(0.0)); ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0);
  ASSERT_TRUE(c.set_target(1.0)); ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NEAR(effort, -2.95, 1e-12); // Permission does not recapture q_ref.
  position = .2; velocity = 0.;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); EXPECT_DOUBLE_EQ(effort, 0.0);
}

TEST_F(FixedImpedanceTest, RecomputesBetweenPermissionsAndClampsBothDirections) {
  activate(); ASSERT_TRUE(c.set_target(1.0));
  position = .1; ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, -1.5);
  position = -.1; ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 1.5);
  position = .9; ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, -7.0);
  position = -.9; ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 7.0);
  EXPECT_EQ(c.target_generation(), 1U); EXPECT_DOUBLE_EQ(generation, 4.0);
}

TEST_F(FixedImpedanceTest, DirectionalMarginCapsOnlyOutwardForce) {
  activate(); ASSERT_TRUE(c.set_target(1.0));
  position = .999; velocity = -50.;
  // Reconfigure a wider valid velocity range before activation.
  ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
  c.get_node()->set_parameter({"max_velocity_rad_s", 50.0});
  ASSERT_EQ(c.on_configure(state), controller_interface::CallbackReturn::SUCCESS);
  position = 0.; velocity = 0.;
  ASSERT_EQ(c.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(c.set_target(1.0)); position = .999; velocity = -50.;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NEAR(effort, .35, 1e-12);
  velocity = 0.; ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, -7.0); // Inward spring is not faded.
}

TEST_F(FixedImpedanceTest, FeedbackFaultLatchesWithoutRefreshingGeneration) {
  activate(); ASSERT_TRUE(c.set_target(1.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const double before = generation;
  position = 1.01; EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  position = 0.; ASSERT_TRUE(c.set_target(1.0));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, before);
}

TEST_F(FixedImpedanceTest, NonFiniteAndSpeedFaultsRejectBeforeWriting) {
  for (const double v : {1.01, std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity()}) {
    activate(); ASSERT_TRUE(c.set_target(1.0)); velocity = v;
    EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
    EXPECT_DOUBLE_EQ(generation, 17.0);
    ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
    velocity = 0.;
  }
}

TEST_F(FixedImpedanceTest, PermissionTimeoutCannotAutomaticallyRearm) {
  activate(); ASSERT_TRUE(c.set_target(1.0)); position = .1;
  for (int i = 0; i < 49; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const double before = generation;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(effort, 0.0); EXPECT_DOUBLE_EQ(generation, before);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  ASSERT_TRUE(c.set_target(1.0)); EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, before);
}

TEST_F(FixedImpedanceTest, RunBudgetSurvivesZeroPermissionAndReactivationRecapturesAnchor) {
  c.get_node()->set_parameter({"run_limit_s", .01});
  activate(); ASSERT_TRUE(c.set_target(1.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  ASSERT_TRUE(c.set_target(0.0));
  for (int i = 0; i < 4; ++i) ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  ASSERT_TRUE(c.set_target(1.0)); EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
  position = .3;
  ASSERT_EQ(c.on_activate(state), controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); EXPECT_DOUBLE_EQ(effort, 0.0);
  ASSERT_TRUE(c.set_target(1.0)); position = .4;
  ASSERT_EQ(cycle(), controller_interface::return_type::OK); EXPECT_NEAR(effort, -1.5, 1e-12);
}

TEST_F(FixedImpedanceTest, RejectsDelayedMonotonicCycleEvenWithNominalReportedPeriod) {
  activate(); ASSERT_TRUE(c.set_target(1.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  now += 7000000;
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
}

TEST_F(FixedImpedanceTest, ContinuousIsExplicitAndRetainsWatchdogAndFaultLatch) {
  c.get_node()->set_parameter({"run_limit_s", 0.0});
  EXPECT_EQ(c.on_configure(state), controller_interface::CallbackReturn::ERROR);
  c.get_node()->set_parameter({"allow_continuous_operation", true});
  activate(); ASSERT_TRUE(c.set_target(1.0)); position = .1;
  for (int i = 0; i < 65000; ++i) { // 130 seconds, beyond bounded-mode ceiling.
    if (i % 10 == 0) { ASSERT_TRUE(c.set_target(1.0)); }
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(effort, -1.5);
  }
  for (int i = 0; i < 60; ++i) (void)cycle(); // Still rejects loss of permission.
  const auto before = generation;
  ASSERT_TRUE(c.set_target(1.0));
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, before);
}

TEST_F(FixedImpedanceTest, ContinuousDoesNotBypassAbsolutePositionProtection) {
  c.get_node()->set_parameters({{"allow_continuous_operation", true}, {"run_limit_s", 0.0}});
  activate(); ASSERT_TRUE(c.set_target(1.0)); position = 1.01;
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, 17.0);
}

TEST_F(FixedImpedanceTest, ArithmeticOverflowIsRejectedInsteadOfCommandingSaturation) {
  c.get_node()->set_parameter({"stiffness_nm_per_rad", std::numeric_limits<double>::max()});
  position = -.9; activate(); ASSERT_TRUE(c.set_target(1.0)); position = .9;
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(generation, 17.0);
}

TEST_F(FixedImpedanceTest, RejectsUnsafeParametersAndMovingOrEdgeActivation) {
  for (const auto& parameter : std::vector<rclcpp::Parameter>{
      {"stiffness_nm_per_rad", 0.0}, {"damping", -1.0}, {"torque_limit_nm", 0.0},
      {"position_max_rad", -1.0}, {"position_margin_rad", 1.1},
      {"run_limit_s", 121.0}, {"max_update_period_s", .007},
      {"activation_velocity_rad_s", 1.0}, {"maximum", 8.0},
      {"damping", std::numeric_limits<double>::quiet_NaN()}}) {
    SCOPED_TRACE(parameter.get_name());
    const auto original = c.get_node()->get_parameter(parameter.get_name());
    c.get_node()->set_parameter(parameter);
    EXPECT_EQ(c.on_configure(state), controller_interface::CallbackReturn::ERROR);
    c.get_node()->set_parameter(original);
  }
  activate(); ASSERT_EQ(c.on_deactivate(state), controller_interface::CallbackReturn::SUCCESS);
  velocity = .1; EXPECT_EQ(c.on_activate(state), controller_interface::CallbackReturn::ERROR);
  velocity = 0.; position = .99;
  EXPECT_EQ(c.on_activate(state), controller_interface::CallbackReturn::ERROR);
}
}  // namespace
}  // namespace mech::mech_controllers
