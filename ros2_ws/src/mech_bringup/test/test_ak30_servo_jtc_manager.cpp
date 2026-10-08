#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "controller_manager/controller_manager.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_bringup/ak30_servo_system.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_simulation/fake_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace {
using mech::mech_bringup::Ak30ServoSystem;
using mech::mech_simulation::FakeSerial;

constexpr char kController[] = "motor1_trajectory_controller";
constexpr std::int64_t kPeriodNs = 2000000;

hardware_interface::InterfaceInfo interface(const std::string& name) {
  hardware_interface::InterfaceInfo result;
  result.name = name;
  result.size = 1;
  return result;
}

hardware_interface::HardwareInfo servo_info() {
  hardware_interface::HardwareInfo result;
  result.name = "synthetic_servo_jtc";
  result.type = "system";
  result.hardware_class_type = "mech_bringup/Ak30ServoSystem";
  result.hardware_parameters = {
      {"profile", "ak30_servo_extended"},
      {"device_path", "/dev/ttySYNTHETIC-JTC"},
      {"logical_bus", "42"},
      {"control_period_ns", "2000000"},
      {"command_ttl_ns", "3000000"},
      {"command_hard_ttl_ns", "6000000"},
      {"feedback_ttl_ns", "60000000"}};
  hardware_interface::ComponentInfo joint;
  joint.name = "motor1_joint";
  joint.type = "joint";
  joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION)};
  joint.command_interfaces = {
      interface(hardware_interface::HW_IF_POSITION),
      interface(mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
  joint.parameters = {
      {"drive_id", "104"},
      {"target_scale", "2.0"},
      {"target_offset", "-5.0"},
      {"target_mapping_verified", "true"},
      {"feedback_scale", "0.5"},
      {"feedback_offset", "1.0"},
      {"feedback_mapping_verified", "true"},
      {"speed_erpm", "1000"},
      {"acceleration_raw", "500"},
      {"position_min_rad", "-2"},
      {"position_max_rad", "2"},
      {"position_max_error_rad", "0.25"}};
  result.joints.push_back(std::move(joint));
  return result;
}

std::vector<std::uint8_t> feedback_wire(std::uint8_t status = 0U,
                                      std::uint8_t id = 104U) {
  using namespace mech::mech_control_core;
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};  // 0 deg -> 1 rad
  payload[7] = status;
  const auto frame = RawCanFrame::create(
      42U, *CanId::create(0x2900U | id, CanFrameFormat::Extended),
      CanFrameType::Classic, FrameDirection::Tx, 8U, payload,
      *MonotonicTime::from_nanoseconds(0));
  std::array<std::uint8_t, 528U> bytes{};
  std::size_t length = 0U;
  EXPECT_TRUE(UsbCdcCodec::encode(*frame, bytes, length));
  return {bytes.begin(), bytes.begin() + length};
}

class ServoJtcManagerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (rclcpp::ok()) return;
    const std::string params = std::string(MECH_BRINGUP_SOURCE_DIR) +
        "/config/motor1_trajectory_controllers.yaml";
    std::vector<std::string> args{
        "servo_jtc_manager_test", "--ros-args", "--params-file", params};
    std::vector<char*> argv;
    for (auto& arg : args) argv.push_back(arg.data());
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  virtual hardware_interface::HardwareInfo hardware_info() { return servo_info(); }
  virtual std::vector<std::string> controller_joints() { return {"motor1_joint"}; }
  virtual void inject_feedback() { ASSERT_TRUE(serial_->inject_rx(feedback_wire())); }
  void configure_hardware_clock(Ak30ServoSystem& plugin) {
    // These functional tests run ROS callbacks on a non-RT host thread.
    // Runtime and transport share one logical instant per read/update/write
    // cycle; neither callback advances it. ROS/JTC and pacing retain their
    // real clocks. Host scheduling pauses must not become hardware faults.
    // Separate runtime/plugin tests cover the unchanged 3/6 ms deadlines.
    plugin.set_clock_for_testing([this] {
      return *mech::mech_control_core::MonotonicTime::from_nanoseconds(
          hardware_now_ns_.load());
    });
  }

  void advance_hardware_clock() { hardware_now_ns_.fetch_add(kPeriodNs); }

  void SetUp() override {
    serial_ = std::make_shared<FakeSerial>(65536U);
    auto plugin = std::make_unique<Ak30ServoSystem>();
    plugin->set_serial_port_factory_for_testing(
        [serial = serial_](const std::string&) { return serial; });
    configure_hardware_clock(*plugin);
    auto resources = std::make_unique<hardware_interface::ResourceManager>();
    resources_ = resources.get();
    resources->import_component(std::move(plugin), hardware_info());
    rclcpp_lifecycle::State active{
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE};
    ASSERT_EQ(resources->set_component_state("synthetic_servo_jtc", active),
              hardware_interface::return_type::OK);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto options = controller_manager::get_cm_node_options();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
        std::move(resources), executor_, "controller_manager", "", options);
    controller_ = manager_->load_controller(kController);
    ASSERT_NE(controller_, nullptr);
    ASSERT_TRUE(controller_->get_node()->set_parameter(
        rclcpp::Parameter("joints", controller_joints())).successful);
    ASSERT_EQ(manager_->configure_controller(kController),
              controller_interface::return_type::OK);
    publisher_node_ = std::make_shared<rclcpp::Node>("servo_jtc_publisher");
    publisher_ = publisher_node_->create_publisher<
        trajectory_msgs::msg::JointTrajectory>(
        std::string("/") + kController + "/joint_trajectory", 1);
    executor_->add_node(publisher_node_);
    next_cycle_ = std::chrono::steady_clock::now();
    serial_->clear_tx();  // discard the gateway init, keep motor evidence separate
  }

  void TearDown() override {
    if (controller_ && controller_->get_state().id() ==
                           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      (void)drive_switch({}, {kController});
    }
    publisher_.reset();
    publisher_node_.reset();
    controller_.reset();
    manager_.reset();
    executor_.reset();
    serial_.reset();
  }

  void cycle(int count) {
    for (int i = 0; i < count; ++i) {
      const auto current = std::chrono::steady_clock::now();
      if (next_cycle_ < current) next_cycle_ = current;
      next_cycle_ += std::chrono::nanoseconds(kPeriodNs);
      std::this_thread::sleep_until(next_cycle_);
      advance_hardware_clock();
      ASSERT_NO_FATAL_FAILURE(inject_feedback());
      const auto clock = controller_->get_node()->get_clock();
      const auto time = clock->now();
      const rclcpp::Duration period(0, kPeriodNs);
      manager_->read(time, period);
      manager_->update(time, period);
      manager_->write(time, period);
      executor_->spin_some();
    }
  }

  controller_interface::return_type drive_switch(
      const std::vector<std::string>& start,
      const std::vector<std::string>& stop) {
    auto result = std::async(std::launch::async, [this, start, stop] {
      return manager_->switch_controller(
          start, stop,
          controller_manager_msgs::srv::SwitchController::Request::STRICT);
    });
    for (int guard = 0; guard < 1000; ++guard) {
      if (result.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready) return result.get();
      cycle(1);
    }
    EXPECT_EQ(result.wait_for(std::chrono::seconds(2)),
              std::future_status::ready);
    return result.get();
  }

  hardware_interface::ResourceManager* resources_{nullptr};
  std::shared_ptr<FakeSerial> serial_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  controller_interface::ControllerInterfaceBaseSharedPtr controller_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point next_cycle_;
  std::atomic<std::int64_t> hardware_now_ns_{1000000000};
};

TEST_F(ServoJtcManagerTest, HostPauseDoesNotReplaceLogicalHardwareTime) {
  ASSERT_NO_FATAL_FAILURE(cycle(2));
  ASSERT_EQ(drive_switch({kController}, {}), controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(cycle(2));
  ASSERT_TRUE(resources_->command_interface_is_available("motor1_joint/position"));
  serial_->clear_tx();

  // Exceed the configured 6 ms hard TTL in host time only. This functional
  // fixture must still advance one 2 ms hardware cycle with fresh feedback.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_NO_FATAL_FAILURE(cycle(1));
  ASSERT_TRUE(resources_->command_interface_is_available("motor1_joint/position"));
  EXPECT_EQ(serial_->take_tx().size(), 21U);
}

TEST_F(ServoJtcManagerTest, DeviceFaultCanLeaveControllerActiveWithCachedPosition) {
  ASSERT_NO_FATAL_FAILURE(cycle(2));
  ASSERT_EQ(drive_switch({kController}, {}), controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(cycle(2));
  auto state = resources_->claim_state_interface("motor1_joint/position");
  const auto before = state.get_value();
  ASSERT_EQ(controller_->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(resources_->command_interface_is_available("motor1_joint/position"));

  // Inject a real servo status fault through the actual codec/session/plugin path.
  ASSERT_TRUE(serial_->inject_rx(feedback_wire(1U)));
  ASSERT_NO_FATAL_FAILURE(cycle(1));
  EXPECT_FALSE(resources_->command_interface_is_available("motor1_joint/position"));
  EXPECT_EQ(controller_->get_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_DOUBLE_EQ(state.get_value(), before);
  EXPECT_NE(drive_switch({}, {kController}), controller_interface::return_type::OK);
}

TEST_F(ServoJtcManagerTest, PositionTrajectoryProducesMode6AndDeactivateIsSilent) {
  ASSERT_NO_FATAL_FAILURE(cycle(2));  // establish actual position-only feedback
  ASSERT_EQ(drive_switch({kController}, {}), controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(cycle(5));
  serial_->clear_tx();

  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::seconds(2);
  while (publisher_->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < deadline) {
    ASSERT_NO_FATAL_FAILURE(cycle(1));
  }
  ASSERT_GT(publisher_->get_subscription_count(), 0U);
  trajectory_msgs::msg::JointTrajectory goal;
  goal.joint_names = {"motor1_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {1.1};
  point.time_from_start = rclcpp::Duration(0, 200000000);
  goal.points = {point};
  publisher_->publish(goal);
  ASSERT_NO_FATAL_FAILURE(cycle(130));
  const auto tx = serial_->take_tx();
  ASSERT_FALSE(tx.empty());
  ASSERT_EQ(tx.size() % 21U, 0U);
  bool saw_nontrivial_target = false;
  for (std::size_t offset = 0; offset < tx.size(); offset += 21U) {
    ASSERT_EQ(tx[offset], 0xF7U);
    ASSERT_EQ(tx[offset + 2U], 14U);
    EXPECT_EQ(tx[offset + 7U], 104U);
    EXPECT_EQ(tx[offset + 8U], 0x06U);
    EXPECT_EQ(tx[offset + 12U], 8U);
    const std::uint32_t raw =
        (static_cast<std::uint32_t>(tx[offset + 13U]) << 24U) |
        (static_cast<std::uint32_t>(tx[offset + 14U]) << 16U) |
        (static_cast<std::uint32_t>(tx[offset + 15U]) << 8U) |
        static_cast<std::uint32_t>(tx[offset + 16U]);
    const auto signed_raw = static_cast<std::int32_t>(raw);
    if (signed_raw > -30000) saw_nontrivial_target = true;
  }
  EXPECT_TRUE(saw_nontrivial_target)
      << "JTC must move the target past the measured-position hold";

  // Once the trajectory has ended, waiting for another user command must
  // still emit the final position every cycle through the real servo codec.
  serial_->clear_tx();
  ASSERT_NO_FATAL_FAILURE(cycle(100));
  const auto held = serial_->take_tx();
  ASSERT_EQ(held.size(), 100U * 21U);
  for (std::size_t offset = 0; offset < held.size(); offset += 21U) {
    EXPECT_EQ(held[offset + 8U], 0x06U);
    const std::uint32_t raw =
        (static_cast<std::uint32_t>(held[offset + 13U]) << 24U) |
        (static_cast<std::uint32_t>(held[offset + 14U]) << 16U) |
        (static_cast<std::uint32_t>(held[offset + 15U]) << 8U) |
        static_cast<std::uint32_t>(held[offset + 16U]);
    EXPECT_NEAR(static_cast<std::int32_t>(raw), -28000, 1);
  }

  ASSERT_EQ(drive_switch({}, {kController}), controller_interface::return_type::OK);
  serial_->clear_tx();
  ASSERT_NO_FATAL_FAILURE(cycle(20));
  EXPECT_TRUE(serial_->take_tx().empty());
}
}  // namespace

#include <fstream>
#include <sstream>
#include <unistd.h>

class ServoJtcChainTest : public ServoJtcManagerTest {
 protected:
  void SetUp() override {
    char directory[] = "/tmp/servo-jtc-chain-XXXXXX";
    ASSERT_NE(mkdtemp(directory), nullptr);
    directory_ = directory;
    path_ = directory_ + "/chain.jsonl";
    setenv("MECH_SERVO_CHAIN_PATH", path_.c_str(), 1);
    ServoJtcManagerTest::SetUp();
    unsetenv("MECH_SERVO_CHAIN_PATH");
  }
  void TearDown() override {
    ServoJtcManagerTest::TearDown();
    std::ifstream file(path_);
    std::ostringstream trace; trace << file.rdbuf();
    const auto records = trace.str();
    EXPECT_NE(records.find("\"stage\":\"claim_after\""), std::string::npos);
    EXPECT_NE(records.find("\"stage\":\"interface\""), std::string::npos);
    EXPECT_NE(records.find("\"stage\":\"dispatch\""), std::string::npos);
    EXPECT_NE(records.find("\"hex\":\"ffff8ad000640032\""), std::string::npos);
    EXPECT_NE(records.find("\"dropped\":0"), std::string::npos);
    unlink(path_.c_str());
    rmdir(directory_.c_str());
  }
  std::string directory_, path_;
};

TEST_F(ServoJtcChainTest, ActualUpstreamJtcHoldIsCapturedThroughUsbEncoder) {
  ASSERT_NO_FATAL_FAILURE(cycle(2));
  ASSERT_EQ(drive_switch({kController}, {}), controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(cycle(30));
  EXPECT_FALSE(serial_->take_tx().empty());
  ASSERT_EQ(drive_switch({}, {kController}), controller_interface::return_type::OK);
  serial_->clear_tx();
  ASSERT_NO_FATAL_FAILURE(cycle(3));
  EXPECT_TRUE(serial_->take_tx().empty());
}

// The application submits joint names in ID order; JTC and hardware need not
// use that same order. Exercise all three orderings through the production
// action -> JTC -> CompositeSystem -> session -> USB encoder chain.
class ServoPairJtcMappingTest : public ServoJtcManagerTest,
                               public ::testing::WithParamInterface<bool> {
 protected:
  using Action = control_msgs::action::FollowJointTrajectory;

  void SetUp() override {
    ASSERT_NO_FATAL_FAILURE(ServoJtcManagerTest::SetUp());
    // DDS entity creation/destruction is not bounded by the active 3/6 ms
    // command watchdog. Prepare one client before claiming motion interfaces
    // and retain it across all submissions and hold checks.
    action_client_ = rclcpp_action::create_client<Action>(
        publisher_node_, std::string("/") + kController + "/follow_joint_trajectory");
    for (int guard = 0; guard < 1000 && !action_client_->action_server_is_ready(); ++guard)
      ASSERT_NO_FATAL_FAILURE(cycle(1));
    ASSERT_TRUE(action_client_->action_server_is_ready());
  }

  hardware_interface::HardwareInfo hardware_info() override {
    auto result = servo_info();
    result.joints[0].name = "motor104_joint";
    auto right = result.joints[0];
    right.name = "motor105_joint";
    right.parameters["drive_id"] = "105";
    right.parameters["target_scale"] = "-3.0";
    right.parameters["target_offset"] = "1.0";
    right.parameters["feedback_scale"] = "0.25";
    right.parameters["feedback_offset"] = "0.5";
    right.parameters["speed_erpm"] = "1700";
    right.parameters["acceleration_raw"] = "900";
    result.joints.push_back(std::move(right));
    if (GetParam()) std::reverse(result.joints.begin(), result.joints.end());
    return result;
  }

  std::vector<std::string> controller_joints() override {
    return {"motor105_joint", "motor104_joint"};
  }

  void inject_feedback() override {
    // Reverse reception order too; feedback values stay distinct after mapping.
    ASSERT_TRUE(serial_->inject_rx(feedback_wire(0U, 105U)));
    ASSERT_TRUE(serial_->inject_rx(feedback_wire(0U, 104U)));
  }

  void submit(double left, double right, bool reverse_goal_order) {
    for (int guard = 0; guard < 1000 && !action_client_->action_server_is_ready(); ++guard)
      ASSERT_NO_FATAL_FAILURE(cycle(1));
    ASSERT_TRUE(action_client_->action_server_is_ready());
    Action::Goal goal;
    goal.trajectory.joint_names = {"motor104_joint", "motor105_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint start, finish;
    start.positions = {1.0, 0.5};
    start.velocities = {0.0, 0.0};
    finish.positions = {left, right};
    finish.velocities = {0.0, 0.0};
    finish.time_from_start = rclcpp::Duration(0, 80000000);
    if (reverse_goal_order) {
      std::reverse(goal.trajectory.joint_names.begin(), goal.trajectory.joint_names.end());
      std::reverse(start.positions.begin(), start.positions.end());
      std::reverse(finish.positions.begin(), finish.positions.end());
    }
    goal.trajectory.points = {start, finish};
    // Fake feedback is fixed at distinct centers. Wide action tolerances let
    // this test isolate routing, while the real hardware envelope remains on.
    for (const auto& name : goal.trajectory.joint_names) {
      control_msgs::msg::JointTolerance tolerance;
      tolerance.name = name;
      tolerance.position = 0.25;
      goal.goal_tolerance.push_back(tolerance);
    }
    auto accepted = action_client_->async_send_goal(goal);
    for (int guard = 0; guard < 1000 &&
         accepted.wait_for(std::chrono::seconds(0)) != std::future_status::ready; ++guard)
      ASSERT_NO_FATAL_FAILURE(cycle(1));
    ASSERT_EQ(accepted.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    auto handle = accepted.get();
    ASSERT_NE(handle, nullptr);
    auto result = action_client_->async_get_result(handle);
    for (int guard = 0; guard < 1000 &&
         result.wait_for(std::chrono::seconds(0)) != std::future_status::ready; ++guard)
      ASSERT_NO_FATAL_FAILURE(cycle(1));
    ASSERT_EQ(result.wait_for(std::chrono::seconds(0)), std::future_status::ready);
    const auto completed = result.get();
    ASSERT_EQ(completed.code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_EQ(completed.result->error_code, Action::Result::SUCCESSFUL);
    ASSERT_NO_FATAL_FAILURE(cycle(4));
    serial_->clear_tx();
  }

  void expect_hold(std::int32_t left_raw, std::int32_t right_raw, int cycles) {
    ASSERT_NO_FATAL_FAILURE(cycle(cycles));
    const auto bytes = serial_->take_tx();
    serial_->clear_tx();  // take_tx() snapshots without consuming FakeSerial
    ASSERT_EQ(bytes.size(), static_cast<std::size_t>(cycles) * 42U);
    std::array<unsigned, 2> seen{};
    for (std::size_t offset = 0; offset < bytes.size(); offset += 21U) {
      ASSERT_EQ(bytes[offset], 0xF7U);
      ASSERT_EQ(bytes[offset + 2U], 14U);
      ASSERT_EQ(bytes[offset + 8U], 0x06U);
      ASSERT_EQ(bytes[offset + 9U], 0U);
      ASSERT_EQ(bytes[offset + 10U], 0U);
      ASSERT_EQ(bytes[offset + 11U], 0x0CU);
      ASSERT_EQ(bytes[offset + 12U], 8U);
      const auto id = bytes[offset + 7U];
      ASSERT_TRUE(id == 104U || id == 105U);
      const auto index = id == 104U ? 0U : 1U;
      ++seen[index];
      const std::uint32_t bits =
          (static_cast<std::uint32_t>(bytes[offset + 13U]) << 24U) |
          (static_cast<std::uint32_t>(bytes[offset + 14U]) << 16U) |
          (static_cast<std::uint32_t>(bytes[offset + 15U]) << 8U) |
          bytes[offset + 16U];
      const std::int64_t raw = bits >= 0x80000000U
          ? static_cast<std::int64_t>(bits) - 0x100000000LL : bits;
      // Independent golden integers, not a call to the production codec.
      EXPECT_NEAR(raw, index == 0U ? left_raw : right_raw, 1);
      EXPECT_EQ(bytes[offset + 17U], 0U);
      EXPECT_EQ(bytes[offset + 18U], index == 0U ? 100U : 170U);
      EXPECT_EQ(bytes[offset + 19U], 0U);
      EXPECT_EQ(bytes[offset + 20U], index == 0U ? 50U : 90U);
    }
    EXPECT_EQ(seen[0], static_cast<unsigned>(cycles));
    EXPECT_EQ(seen[1], static_cast<unsigned>(cycles));
  }

  rclcpp_action::Client<Action>::SharedPtr action_client_;
};

TEST_P(ServoPairJtcMappingTest, DistinctTargetsSurviveUpdatesReorderingAndSixteenSecondHold) {
  ASSERT_NO_FATAL_FAILURE(cycle(2));
  ASSERT_EQ(drive_switch({kController}, {}), controller_interface::return_type::OK);
  ASSERT_NO_FATAL_FAILURE(cycle(4));
  serial_->clear_tx();
  ASSERT_NO_FATAL_FAILURE(expect_hold(-30000, -5000, 20));
  ASSERT_NO_FATAL_FAILURE(submit(1.10, 0.60, false));
  ASSERT_NO_FATAL_FAILURE(expect_hold(-28000, -8000, 40));
  ASSERT_NO_FATAL_FAILURE(submit(1.20, 0.60, true));
  ASSERT_NO_FATAL_FAILURE(expect_hold(-26000, -8000, 40));
  ASSERT_NO_FATAL_FAILURE(submit(1.20, 0.40, false));
  ASSERT_NO_FATAL_FAILURE(expect_hold(-26000, -2000, 40));
  ASSERT_NO_FATAL_FAILURE(submit(1.05, 0.55, true));
  // 8000 * 2 ms is at least 16 s of real wall time. Drain each chunk so
  // FakeSerial capacity cannot hide dropped/repeated late hold commands.
  const auto hold_start = std::chrono::steady_clock::now();
  for (int chunk = 0; chunk < 80; ++chunk)
    ASSERT_NO_FATAL_FAILURE(expect_hold(-29000, -6500, 100));
  EXPECT_GE(std::chrono::steady_clock::now() - hold_start, std::chrono::seconds(16));
  ASSERT_EQ(drive_switch({}, {kController}), controller_interface::return_type::OK);
  serial_->clear_tx();
  ASSERT_NO_FATAL_FAILURE(cycle(10));
  EXPECT_TRUE(serial_->take_tx().empty());
}

INSTANTIATE_TEST_SUITE_P(HardwareDeclarationOrder, ServoPairJtcMappingTest,
                        ::testing::Values(false, true));
