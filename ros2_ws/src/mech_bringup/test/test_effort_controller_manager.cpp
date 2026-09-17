// T9: production effort plugin + ControllerManager + runtime + FakeTransport.
// Follows the lifecycle/switch discipline in test_controller_manager_integration.cpp.
#include "mech_controllers/effort_command_controller.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mech_bringup/ak30_force_runtime.hpp"
#include "mech_control_core/frame.hpp"
#include "mech_control_core/time.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include "mech_protocol_cubemars/ak30_mapping.hpp"
#include "mech_simulation/fake_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mech::mech_bringup {
namespace {

using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::FrameDirection;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::TransportResult;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::ForceControlSubMode;
using mech::mech_simulation::FakeTransport;

constexpr std::uint16_t kDriveId = 104U;
constexpr std::uint32_t kLogicalBus = 1U;
constexpr char kHardwareName[] = "ak30_bench";
constexpr char kJointName[] = "motor1_joint";
constexpr char kControllerName[] = "motor1_effort_controller";
constexpr std::int64_t kPeriodNanoseconds = 2000000;
constexpr std::int64_t kFeedbackPeriodNs = 20000000;


[[nodiscard]] Ak30RuntimeConfig runtime_config() {
  Ak30RuntimeConfig config{};
  config.drive_id = kDriveId;
  config.logical_bus = kLogicalBus;
  config.sub_mode = ForceControlSubMode::Torque;
  config.mapping = Ak30Mapping{};
  config.gains.kp = 0.0;
  config.gains.kd = 0.0;
  config.control_period_nanoseconds = kPeriodNanoseconds;
  config.command_ttl_nanoseconds = 4000000;
  config.command_hard_ttl_nanoseconds = 6000000;
  config.feedback_period_nanoseconds = 20000000;
  config.feedback_ttl_nanoseconds = 60000000;
  return config;
}

[[nodiscard]] hardware_interface::HardwareInfo hardware_info() {
  hardware_interface::HardwareInfo info;
  info.name = kHardwareName;
  info.type = "system";
  info.hardware_class_type = "mech_hardware_ros2_control/CompositeSystem";
  const auto interface = [](const std::string& name) {
    hardware_interface::InterfaceInfo value;
    value.name = name;
    value.size = 1;
    return value;
  };
  hardware_interface::ComponentInfo joint;
  joint.name = kJointName;
  joint.type = "joint";
  joint.command_interfaces = {
      interface(hardware_interface::HW_IF_EFFORT),
      interface(
          mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
  joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                            interface(hardware_interface::HW_IF_VELOCITY),
                            interface(hardware_interface::HW_IF_EFFORT)};
  info.joints.push_back(joint);
  return info;
}

[[nodiscard]] RawCanFrame feedback_frame(std::int64_t arrival_ns) {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03;
  payload[1] = 0x84;
  payload[2] = 0x00;
  payload[3] = 0x00;
  payload[4] = 0x00;
  payload[5] = 0xC8;
  payload[6] = 0x28;
  payload[7] = 0x00;
  return RawCanFrame::create(
             kLogicalBus,
             CanId::create(
                 mech::mech_protocol_cubemars::feedback_can_id(kDriveId),
                 CanFrameFormat::Extended)
                 .value(),
             CanFrameType::Classic, FrameDirection::Rx, 8U, payload,
             MonotonicTime::from_nanoseconds(arrival_ns).value())
      .value();
}

class EffortControllerManagerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const char* args[] = {"effort_manager_test", "--ros-args", "--params-file",
                         MECH_BRINGUP_SOURCE_DIR "/config/motor1_effort_controllers.yaml"};
    rclcpp::init(4, args);
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  void SetUp() override {
    transport_ = std::make_unique<FakeTransport>(4096U);
    auto system = std::make_unique<
        mech::mech_hardware_ros2_control::CompositeSystem>();
    ASSERT_TRUE(system->set_runtime(std::make_unique<Ak30ForceControlRuntime>(
        *transport_, [this]() { return now(); }, runtime_config())));

    auto resources = std::make_unique<hardware_interface::ResourceManager>();
    resources_ = resources.get();
    resources->import_component(std::move(system), hardware_info());
    rclcpp_lifecycle::State active{
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE};
    ASSERT_EQ(resources->set_component_state(kHardwareName, active),
              hardware_interface::return_type::OK);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options =
        controller_manager::get_cm_node_options();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
        std::move(resources), executor_, "controller_manager", "", options);

    controller_ = std::dynamic_pointer_cast<mech::mech_controllers::EffortCommandController>(
        manager_->load_controller(kControllerName));
    ASSERT_NE(controller_, nullptr);
    controller_->set_clock_for_testing([this]() { return now_nanoseconds_; });
    ASSERT_EQ(manager_->configure_controller(kControllerName),
              controller_interface::return_type::OK);
  }

  void TearDown() override {
    if (controller_ && controller_->get_state().id() ==
                           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      (void)drive_switch({}, {kControllerName});
    }
    controller_.reset();
    manager_.reset();
    executor_.reset();
    transport_.reset();
  }

  [[nodiscard]] MonotonicTime now() const {
    return MonotonicTime::from_nanoseconds(now_nanoseconds_).value();
  }

  void establish_feedback() {
    ASSERT_EQ(transport_->inject_receive(feedback_frame(now_nanoseconds_)),
              TransportResult::Ok);
    cycle(1);
  }

  void activate_controller() { switch_controller({kControllerName}, {}); }

  void deactivate_controller() { switch_controller({}, {kControllerName}); }

  controller_interface::return_type drive_switch(
      const std::vector<std::string>& start,
      const std::vector<std::string>& stop) {
    auto result = std::async(std::launch::async, [this, start, stop]() {
      return manager_->switch_controller(
          start, stop,
          controller_manager_msgs::srv::SwitchController::Request::STRICT);
    });
    for (int guard = 0; guard < 2000; ++guard) {
      if (result.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready) {
        return result.get();
      }
      cycle(1);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    EXPECT_EQ(result.wait_for(std::chrono::seconds(2)),
              std::future_status::ready);
    return result.get();
  }

  void switch_controller(const std::vector<std::string>& start,
                         const std::vector<std::string>& stop) {
    ASSERT_EQ(drive_switch(start, stop),
              controller_interface::return_type::OK);
  }

  void cycle(int count) {
    for (int index = 0; index < count; ++index) {
      now_nanoseconds_ += kPeriodNanoseconds;
      if (now_nanoseconds_ - last_feedback_nanoseconds_ >= kFeedbackPeriodNs) {
        last_feedback_nanoseconds_ = now_nanoseconds_;
        EXPECT_EQ(transport_->inject_receive(feedback_frame(now_nanoseconds_)),
                  TransportResult::Ok);
      }
      const rclcpp::Time time(now_nanoseconds_, RCL_STEADY_TIME);
      const rclcpp::Duration period(0, kPeriodNanoseconds);
      if (refresh_target_) (void)controller_->set_target(target_);
      manager_->read(time, period);
      manager_->update(time, period);
      manager_->write(time, period);
    }
  }

  [[nodiscard]] std::size_t transmitted() const {
    return transport_->pending_transmit();
  }

  void follow(double target, int cycles) {
    target_ = target;
    refresh_target_ = true;
    cycle(cycles);
  }

  RawCanFrame last_frame() {
    RawCanFrame frame{};
    RawCanFrame last{};
    EXPECT_GT(transmitted(), 0U);
    while (transport_->take_transmit(frame)) last = frame;
    return last;
  }

  void expect_effort_frame(double effort) {
    const auto frame = last_frame();
    ASSERT_EQ(frame.id.value, 0x0868U);
    EXPECT_EQ(frame.id.format, CanFrameFormat::Extended);
    EXPECT_EQ(frame.type, CanFrameType::Classic);
    ASSERT_EQ(frame.payload_size, 8U);
    const auto& b = frame.payload;
    EXPECT_EQ((b[0] << 4U) | (b[1] >> 4U), 0);  // Kp = 0
    EXPECT_EQ(((b[1] & 15U) << 8U) | b[2], 0U);  // Kd = 0
    EXPECT_EQ((b[3] << 8U) | b[4], 32767);  // position = 0
    EXPECT_EQ((b[5] << 4U) | (b[6] >> 4U), 2047U);  // velocity = 0
    const auto raw_effort = ((b[6] & 15U) << 8U) | b[7];
    const double decoded_effort = raw_effort * 30.0 / 4095.0 - 15.0;
    EXPECT_NEAR(decoded_effort, effort, 30.0 / 4095.0);
  }

  hardware_interface::ResourceManager* resources_{nullptr};
  bool refresh_target_{false};
  double target_{0.0};
  std::unique_ptr<FakeTransport> transport_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<mech::mech_controllers::EffortCommandController> controller_;
  std::int64_t now_nanoseconds_{1000000000000};
  std::int64_t last_feedback_nanoseconds_{0};
};

TEST_F(EffortControllerManagerTest, LoadsDeploymentAndNeedsExplicitTarget) {
  EXPECT_EQ(manager_->get_parameter("update_rate").as_int(), 500);
  EXPECT_DOUBLE_EQ(controller_->get_node()->get_parameter("maximum").as_double(), 0.1);
  EXPECT_DOUBLE_EQ(controller_->get_node()->get_parameter("max_slew_per_second").as_double(), 0.2);
  EXPECT_EQ(controller_->command_interface_configuration().names,
            (std::vector<std::string>{"motor1_joint/effort", "motor1_joint/command_generation"}));
  EXPECT_TRUE(controller_->state_interface_configuration().names.empty());
  establish_feedback();
  activate_controller();
  cycle(100);  // Longer than upstream and hardware deadlines, no target yet.
  ASSERT_EQ(transmitted(), 0U);
  auto speed = resources_->claim_state_interface("motor1_joint/velocity");
  EXPECT_DOUBLE_EQ(speed.get_value(), 0.0);  // Unsupported in Torque, never used as rest evidence.
  auto effort = resources_->claim_state_interface("motor1_joint/effort");
  EXPECT_GT(effort.get_value(), 0.0);
  follow(0.0, 5);
  expect_effort_frame(0.0);
}

TEST_F(EffortControllerManagerTest, RampsClampsReversesAndStopsOnWire) {
  establish_feedback();
  activate_controller();
  follow(2.0, 101);
  expect_effort_frame(0.040);  // 0.2 N*m/s, read-before-write one-cycle lag.
  follow(2.0, 500);
  expect_effort_frame(0.1);  // YAML bounds, refreshed identical upstream values.
  follow(-2.0, 1100);
  expect_effort_frame(-0.1);
  follow(0.0, 600);
  expect_effort_frame(0.0);
  deactivate_controller();
  refresh_target_ = false;
  (void)last_frame();  // Discard frames produced before the switch completed.
  cycle(100);
  EXPECT_EQ(transmitted(), 0U);
}

TEST_F(EffortControllerManagerTest, StaleUpstreamStopsFramesWhileManagerKeepsCycling) {
  establish_feedback();
  activate_controller();
  follow(0.08, 400);
  expect_effort_frame(0.08);
  refresh_target_ = false;
  cycle(60);  // 120 ms > upstream hard deadline and hardware lease.
  expect_effort_frame(0.08);  // Last real frame is frozen, never synthetic zero.
  cycle(100);
  EXPECT_EQ(transmitted(), 0U);
}

TEST_F(EffortControllerManagerTest, ReclaimCannotReplayOldEffort) {
  establish_feedback();
  activate_controller();
  follow(0.08, 400);
  expect_effort_frame(0.08);
  deactivate_controller();
  refresh_target_ = false;
  (void)last_frame();
  EXPECT_FALSE(controller_->set_target(-0.08));
  cycle(100);
  EXPECT_EQ(transmitted(), 0U);
  activate_controller();
  cycle(100);
  EXPECT_EQ(transmitted(), 0U);
  follow(-0.08, 101);
  expect_effort_frame(-0.040);
}

TEST_F(EffortControllerManagerTest, RejectsConflictingControllerWithoutTransmitting) {
  auto competitor = manager_->load_controller(
      "competing_effort", "mech_controllers/EffortCommandController");
  ASSERT_NE(competitor, nullptr);
  competitor->get_node()->set_parameter({"joint", kJointName});
  ASSERT_EQ(manager_->configure_controller("competing_effort"),
            controller_interface::return_type::OK);
  establish_feedback();
  activate_controller();
  EXPECT_EQ(drive_switch({"competing_effort"}, {}),
            controller_interface::return_type::ERROR);
  EXPECT_EQ(competitor->get_state().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(controller_->get_state().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  cycle(100);
  EXPECT_EQ(transmitted(), 0U);
}

}  // namespace
}  // namespace mech::mech_bringup
