#include "mech_controllers/velocity_command_controller.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>
#include <utility>

#include <gtest/gtest.h>
#include "hardware_interface/loaned_command_interface.hpp"
#include "mech_control_core/command_contract.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mech::mech_controllers {
namespace {

class VelocityCommandControllerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    ASSERT_EQ(controller_.init("velocity_test"), controller_interface::return_type::OK);
    controller_.get_node()->set_parameter({"max_slew_per_second", 2.0});
    controller_.set_clock_for_testing([this]() { return now_; });
  }
  void activate() {
    ASSERT_EQ(controller_.on_configure(state_), controller_interface::CallbackReturn::SUCCESS);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(velocity_interface_);
    commands.emplace_back(generation_interface_);
    controller_.assign_interfaces(std::move(commands), {});
    ASSERT_EQ(controller_.on_activate(state_), controller_interface::CallbackReturn::SUCCESS);
  }
  controller_interface::return_type cycle(std::int64_t elapsed = 2000000) {
    now_ += elapsed;
    return controller_.update(rclcpp::Time(0), rclcpp::Duration(0, 2000000));
  }
  rclcpp_lifecycle::State state_;
  std::int64_t now_{1000000000000};
  double velocity_{9.0};
  double generation_{17.0};
  hardware_interface::CommandInterface velocity_interface_{"joint_1", "velocity", &velocity_};
  hardware_interface::CommandInterface generation_interface_{
      "joint_1", mech::mech_control_core::kCommandGenerationInterface, &generation_};
  VelocityCommandController controller_;
};

TEST_F(VelocityCommandControllerTest, RequiresVelocityAndGenerationButNoPositionState) {
  activate();
  EXPECT_EQ(controller_.command_interface_configuration().names,
            (std::vector<std::string>{"joint_1/velocity", "joint_1/command_generation"}));
  EXPECT_TRUE(controller_.state_interface_configuration().names.empty());
  EXPECT_EQ(cycle(500000000), controller_interface::return_type::OK);
  EXPECT_EQ(generation_, 17.0);  // No target: not even an implicit zero command refresh.
  ASSERT_TRUE(controller_.set_target(0.0));
  EXPECT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(velocity_, 0.0);
  EXPECT_NE(generation_, 17.0);  // Explicit fresh zero is a real command.
}

TEST_F(VelocityCommandControllerTest, ClampsSpeedAndBoundsAccelerationInBothDirections) {
  activate();
  ASSERT_TRUE(controller_.set_target(10.0));
  EXPECT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(velocity_, 0.004);  // 2 rad/s^2 times 2 ms, starting at zero.
  for (int i = 0; i < 300; ++i) {
    ASSERT_TRUE(controller_.set_target(10.0));
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  }
  EXPECT_DOUBLE_EQ(velocity_, 1.0);
  ASSERT_TRUE(controller_.set_target(-10.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(velocity_, 0.996);
  for (int i = 0; i < 600; ++i) {
    ASSERT_TRUE(controller_.set_target(-10.0));
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  }
  EXPECT_DOUBLE_EQ(velocity_, -1.0);
}

TEST_F(VelocityCommandControllerTest, FreshZeroRampsToRestBeforeDeactivation) {
  activate();
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(controller_.set_target(0.2));
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  }
  EXPECT_NEAR(velocity_, 0.2, 1e-12);
  ASSERT_TRUE(controller_.set_target(0.0));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_NEAR(velocity_, 0.196, 1e-12);
  for (int i = 0; i < 50; ++i) {
    ASSERT_TRUE(controller_.set_target(0.0));
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  }
  EXPECT_DOUBLE_EQ(velocity_, 0.0);
  const auto generation = generation_;
  ASSERT_EQ(controller_.on_deactivate(state_), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(cycle(), controller_interface::return_type::ERROR);
  EXPECT_EQ(generation_, generation);
}

TEST_F(VelocityCommandControllerTest, StaleTargetStopsRefreshWithoutSynthesizingZero) {
  activate();
  const auto arrival = now_;
  ASSERT_TRUE(controller_.set_target(0.5));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const auto command = velocity_;
  const auto generation = generation_;
  now_ = arrival + 98000000;
  EXPECT_EQ(cycle(), controller_interface::return_type::OK);  // Exactly 100 ms.
  EXPECT_EQ(velocity_, command);
  EXPECT_EQ(generation_, generation);
  EXPECT_EQ(cycle(6000000), controller_interface::return_type::ERROR);  // 106 ms.
  EXPECT_EQ(velocity_, command);
  EXPECT_EQ(generation_, generation);
}

TEST_F(VelocityCommandControllerTest, SameValueRefreshesButInvalidTargetsDoNotExtendLife) {
  activate();
  ASSERT_TRUE(controller_.set_target(0.5));
  for (int i = 0; i < 10; ++i) {
    now_ += 50000000;  // Frozen ROS time does not govern target age.
    ASSERT_TRUE(controller_.set_target(0.5));
    ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  }
  const auto count = controller_.target_generation();
  for (const auto invalid : {std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity(),
                            -std::numeric_limits<double>::infinity()}) {
    EXPECT_FALSE(controller_.set_target(invalid));
  }
  EXPECT_EQ(controller_.target_generation(), count);
  EXPECT_EQ(cycle(106000000), controller_interface::return_type::ERROR);
}

TEST_F(VelocityCommandControllerTest, ReactivationDiscardsOldTargetsAndRestartsTheRamp) {
  activate();
  ASSERT_TRUE(controller_.set_target(0.5));
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  const auto before = generation_;
  EXPECT_EQ(controller_.on_deactivate(state_), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(controller_.set_target(-1.0));
  EXPECT_EQ(controller_.on_activate(state_), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(cycle(500000000), controller_interface::return_type::OK);
  EXPECT_EQ(generation_, before);
  ASSERT_TRUE(controller_.set_target(-0.5));
  EXPECT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(velocity_, -0.004);
  EXPECT_GT(generation_, before);
}

TEST_F(VelocityCommandControllerTest, RejectsLimitsThatCannotRepresentAStop) {
  for (const auto& bounds : {std::pair<double, double>{0.1, 1.0}, {-1.0, -0.1}}) {
    controller_.get_node()->set_parameter({"minimum", bounds.first});
    controller_.get_node()->set_parameter({"maximum", bounds.second});
    EXPECT_EQ(controller_.on_configure(state_), controller_interface::CallbackReturn::ERROR);
  }
}

TEST_F(VelocityCommandControllerTest, RejectsPositionOrMissingGenerationClaims) {
  ASSERT_EQ(controller_.on_configure(state_), controller_interface::CallbackReturn::SUCCESS);
  hardware_interface::CommandInterface wrong{"joint_1", "position", &velocity_};
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  commands.emplace_back(wrong);
  commands.emplace_back(generation_interface_);
  controller_.assign_interfaces(std::move(commands), {});
  EXPECT_EQ(controller_.on_activate(state_), controller_interface::CallbackReturn::ERROR);
  controller_.release_interfaces();
  std::vector<hardware_interface::LoanedCommandInterface> missing;
  missing.emplace_back(velocity_interface_);
  controller_.assign_interfaces(std::move(missing), {});
  EXPECT_EQ(controller_.on_activate(state_), controller_interface::CallbackReturn::ERROR);
}

TEST_F(VelocityCommandControllerTest, ConsumesTheVelocityTopicWithoutAStateSubscription) {
  activate();
  auto node = std::make_shared<rclcpp::Node>("velocity_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::Float64>(
      "/velocity_test/target_velocity", rclcpp::SystemDefaultsQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller_.get_node()->get_node_base_interface());
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (publisher->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_EQ(publisher->get_subscription_count(), 1U);
  std_msgs::msg::Float64 target;
  target.data = -0.2;
  publisher->publish(target);
  while (controller_.target_generation() == 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_GT(controller_.target_generation(), 0U);
  ASSERT_EQ(cycle(), controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(velocity_, -0.004);
}

}  // namespace
}  // namespace mech::mech_controllers
