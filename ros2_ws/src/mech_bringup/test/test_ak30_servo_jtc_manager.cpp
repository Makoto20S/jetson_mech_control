#include <array>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_bringup/ak30_servo_system.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_simulation/fake_serial.hpp"
#include "rclcpp/rclcpp.hpp"
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

std::vector<std::uint8_t> feedback_wire() {
  using namespace mech::mech_control_core;
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};  // 0 deg -> 1 rad
  const auto frame = RawCanFrame::create(
      42U, *CanId::create(0x2968U, CanFrameFormat::Extended),
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

  void SetUp() override {
    serial_ = std::make_shared<FakeSerial>(65536U);
    auto plugin = std::make_unique<Ak30ServoSystem>();
    plugin->set_serial_port_factory_for_testing(
        [serial = serial_](const std::string&) { return serial; });
    auto resources = std::make_unique<hardware_interface::ResourceManager>();
    resources->import_component(std::move(plugin), servo_info());
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
      ASSERT_TRUE(serial_->inject_rx(feedback_wire()));
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

  std::shared_ptr<FakeSerial> serial_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  controller_interface::ControllerInterfaceBaseSharedPtr controller_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point next_cycle_;
};

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

  ASSERT_EQ(drive_switch({}, {kController}), controller_interface::return_type::OK);
  serial_->clear_tx();
  ASSERT_NO_FATAL_FAILURE(cycle(20));
  EXPECT_TRUE(serial_->take_tx().empty());
}
}  // namespace
