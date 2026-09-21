// T10: the installed upstream joint_trajectory_controller, loaded by its
// pluginlib name and configured only by the shipped deployment YAML, drives
// CompositeSystem through the ADR-017 weak tier.
#include <algorithm>
#include <array>
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

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
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
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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
constexpr char kPositionControllerName[] = "motor1_trajectory_controller";
constexpr char kPositionVelocityControllerName[] =
    "motor1_position_velocity_trajectory_controller";
constexpr std::int64_t kPeriodNanoseconds = 2000000;
constexpr std::int64_t kFeedbackPeriodNs = 20000000;
constexpr double kFixtureFeedbackPosition =
    1.5707963267948966 - 5.760604931781636;
constexpr double kPositionQuantum = 25.12 / 65535.0;

[[nodiscard]] Ak30RuntimeConfig runtime_config(bool position_velocity) {
  Ak30RuntimeConfig config{};
  config.drive_id = kDriveId;
  config.logical_bus = kLogicalBus;
  config.sub_mode = ForceControlSubMode::Position;
  config.mapping = Ak30Mapping{};
  config.gains.kp = 1.0;
  config.gains.kd = 1.0;
  config.control_period_nanoseconds = kPeriodNanoseconds;
  config.command_ttl_nanoseconds = 4000000;
  config.command_hard_ttl_nanoseconds = 6000000;
  config.feedback_period_nanoseconds = kFeedbackPeriodNs;
  config.feedback_ttl_nanoseconds = 60000000;
  config.position_min_rad = -12.0;
  config.position_max_rad = 6.0;
  config.position_max_error_rad = 0.5;
  if (position_velocity) config.position_max_abs_velocity_rad_s = 1.0;
  return config;
}

[[nodiscard]] hardware_interface::HardwareInfo hardware_info(
    bool position_velocity) {
  hardware_interface::HardwareInfo info;
  info.name = kHardwareName;
  info.type = "system";
  info.hardware_class_type =
      "mech_hardware_ros2_control/CompositeSystem";
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
      interface(hardware_interface::HW_IF_POSITION),
      interface(mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
  if (position_velocity) {
    joint.command_interfaces.insert(joint.command_interfaces.begin() + 1,
                                    interface(hardware_interface::HW_IF_VELOCITY));
  }
  joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                            interface(hardware_interface::HW_IF_VELOCITY),
                            interface(hardware_interface::HW_IF_EFFORT)};
  info.joints.push_back(joint);
  return info;
}

[[nodiscard]] RawCanFrame feedback_frame(MonotonicTime arrival, std::uint16_t position_decidegrees) {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = static_cast<std::uint8_t>(position_decidegrees >> 8U);
  payload[1] = static_cast<std::uint8_t>(position_decidegrees & 0xFFU);
  // The P+V JTC seeds its interpolation from measured velocity. Keep this
  // trajectory fixture at rest; the former 10,000 eRPM fixture made JTC
  // correctly preserve a large initial velocity that immediately exceeded
  // this deployment's explicit 1 rad/s command bound.
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
             CanFrameType::Classic, FrameDirection::Rx, 8U, payload, arrival)
      .value();
}

class RecordingTelemetry final : public FeedbackTelemetryCapture {
 public:
  bool try_push(const FeedbackTelemetryEvent& event) noexcept override {
    events.push_back(event);
    return true;
  }
  std::vector<FeedbackTelemetryEvent> events;
};

class JtcIntegrationTest : public ::testing::TestWithParam<bool> {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      const std::string position_params =
          std::string(MECH_BRINGUP_SOURCE_DIR) +
          "/config/motor1_trajectory_controllers.yaml";
      const std::string position_velocity_params =
          std::string(MECH_BRINGUP_SOURCE_DIR) +
          "/config/motor1_position_velocity_trajectory_controllers.yaml";
      std::vector<std::string> args{"jtc_manager_test", "--ros-args",
                                    "--params-file", position_params,
                                    "--params-file", position_velocity_params};
      std::vector<char*> argv;
      argv.reserve(args.size());
      for (auto& arg : args) {
        argv.push_back(arg.data());
      }
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    position_velocity_ = GetParam();
    controller_name_ = position_velocity_ ? kPositionVelocityControllerName
                                          : kPositionControllerName;
    transport_ = std::make_unique<FakeTransport>(64U);
    auto system =
        std::make_unique<mech::mech_hardware_ros2_control::CompositeSystem>();
    ASSERT_TRUE(system->set_runtime(std::make_unique<Ak30ForceControlRuntime>(
        *transport_, []() { return steady_now(); },
        runtime_config(position_velocity_),
        &telemetry_)));
    auto resources = std::make_unique<hardware_interface::ResourceManager>();
    resources->import_component(std::move(system),
                                hardware_info(position_velocity_));
    rclcpp_lifecycle::State active{
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE};
    ASSERT_EQ(resources->set_component_state(kHardwareName, active),
              hardware_interface::return_type::OK);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
        std::move(resources), executor_, "controller_manager", "", options);
    controller_ = manager_->load_controller(controller_name_);
    ASSERT_NE(controller_, nullptr)
        << "is ros-humble-joint-trajectory-controller installed?";
    ASSERT_EQ(manager_->configure_controller(controller_name_),
              controller_interface::return_type::OK);
    publisher_node_ = std::make_shared<rclcpp::Node>("trajectory_publisher");
    publisher_ = publisher_node_->create_publisher<
        trajectory_msgs::msg::JointTrajectory>(
        std::string("/") + controller_name_ + "/joint_trajectory", 1);
    executor_->add_node(publisher_node_);
    next_cycle_ = std::chrono::steady_clock::now();
    last_feedback_ = next_cycle_ - std::chrono::nanoseconds(kFeedbackPeriodNs);
  }

  void TearDown() override {
    if (controller_ && controller_->get_state().id() ==
                           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      (void)drive_switch({}, {controller_name_});
    }
    publisher_.reset();
    publisher_node_.reset();
    controller_.reset();
    manager_.reset();
    executor_.reset();
    transport_.reset();
  }

  [[nodiscard]] static MonotonicTime steady_now() {
    return MonotonicTime::from_nanoseconds(
               std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
                   .count())
        .value();
  }

  void establish_feedback() {
    ASSERT_EQ(transport_->inject_receive(feedback_frame(steady_now(), feedback_position_decidegrees_)),
              TransportResult::Ok);
    last_feedback_ = std::chrono::steady_clock::now();
    cycle(1);
  }

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
    }
    EXPECT_EQ(result.wait_for(std::chrono::seconds(2)),
              std::future_status::ready);
    return result.get();
  }

  void cycle(int count) {
    for (int index = 0; index < count; ++index) {
      const auto current = std::chrono::steady_clock::now();
      if (next_cycle_ < current) {
        next_cycle_ = current;
      }
      next_cycle_ += std::chrono::nanoseconds(kPeriodNanoseconds);
      std::this_thread::sleep_until(next_cycle_);
      const auto steady = std::chrono::steady_clock::now();
      if (steady - last_feedback_ >=
          std::chrono::nanoseconds(kFeedbackPeriodNs)) {
        last_feedback_ = steady;
        EXPECT_EQ(transport_->inject_receive(feedback_frame(steady_now(), feedback_position_decidegrees_)),
                  TransportResult::Ok);
      }
      const auto clock = controller_->get_node()->get_clock();
      const rclcpp::Time time = clock->now();
      const rclcpp::Duration period(0, kPeriodNanoseconds);
      manager_->read(time, period);
      manager_->update(time, period);
      manager_->write(time, period);
      executor_->spin_some();
      drain_transmit();
    }
  }

  void wait_for_trajectory_subscription() {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (publisher_->get_subscription_count() == 0U &&
           std::chrono::steady_clock::now() < deadline) {
      cycle(1);
    }
    ASSERT_GT(publisher_->get_subscription_count(), 0U);
  }

  void drain_transmit() {
    RawCanFrame frame{};
    while (transport_->take_transmit(frame)) {
      const std::uint32_t raw =
          (static_cast<std::uint32_t>(frame.payload[3]) << 8U) |
          static_cast<std::uint32_t>(frame.payload[4]);
      const double device = mech::mech_protocol_cubemars::dequantize(
          raw, -12.56, 12.56, 16U);
      positions_.push_back(device - Ak30Mapping{}.zero_offset_rad.value);
      const std::uint32_t raw_velocity =
          (static_cast<std::uint32_t>(frame.payload[5]) << 4U) |
          (static_cast<std::uint32_t>(frame.payload[6]) >> 4U);
      velocities_.push_back(mech::mech_protocol_cubemars::dequantize(
          raw_velocity, -40.0, 40.0, 12U));
      const std::uint32_t raw_effort =
          ((static_cast<std::uint32_t>(frame.payload[6]) & 0x0FU) << 8U) |
          static_cast<std::uint32_t>(frame.payload[7]);
      efforts_.push_back(mech::mech_protocol_cubemars::dequantize(
          raw_effort, -15.0, 15.0, 12U));
    }
  }

  [[nodiscard]] std::vector<double> take_positions() {
    std::vector<double> result;
    result.swap(positions_);
    return result;
  }

  [[nodiscard]] std::vector<double> take_velocities() {
    std::vector<double> result;
    result.swap(velocities_);
    return result;
  }

  [[nodiscard]] std::vector<double> take_efforts() {
    std::vector<double> result;
    result.swap(efforts_);
    return result;
  }

  [[nodiscard]] bool saw_position_envelope() const {
    return std::any_of(telemetry_.events.begin(), telemetry_.events.end(),
                       [](const FeedbackTelemetryEvent& event) {
                         return event.reason ==
                                FeedbackTelemetryReason::PositionEnvelope;
                       });
  }

  [[nodiscard]] bool saw_position_tuple_rejection() const {
    return std::any_of(telemetry_.events.begin(), telemetry_.events.end(),
                       [](const FeedbackTelemetryEvent& event) {
                         return event.reason ==
                                FeedbackTelemetryReason::PositionTuple;
                       });
  }

  std::uint16_t feedback_position_decidegrees_{900U};
  bool position_velocity_{false};
  std::string controller_name_;
  std::unique_ptr<FakeTransport> transport_;
  RecordingTelemetry telemetry_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  controller_interface::ControllerInterfaceBaseSharedPtr controller_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::chrono::steady_clock::time_point next_cycle_;
  std::chrono::steady_clock::time_point last_feedback_;
};

TEST_P(JtcIntegrationTest, ActivationAndReactivationHoldMeasuredPosition) {
  establish_feedback();
  ASSERT_EQ(drive_switch({controller_name_}, {}),
            controller_interface::return_type::OK);
  cycle(10);
  auto positions = take_positions();
  ASSERT_GE(positions.size(), 10U);
  for (const double position : positions) {
    EXPECT_NEAR(position, kFixtureFeedbackPosition, kPositionQuantum);
  }

  ASSERT_EQ(drive_switch({}, {controller_name_}),
            controller_interface::return_type::OK);
  (void)take_positions();
  feedback_position_decidegrees_ = 1200U;
  establish_feedback();
  EXPECT_TRUE(take_positions().empty());
  ASSERT_EQ(drive_switch({controller_name_}, {}),
            controller_interface::return_type::OK);
  cycle(10);
  positions = take_positions();
  ASSERT_GE(positions.size(), 10U);
  for (const double position : positions) {
    EXPECT_NEAR(position, kFixtureFeedbackPosition + 0.5235987755982988,
                kPositionQuantum);
  }
}

TEST_P(JtcIntegrationTest, TrajectoryProducesBoundedMonotoneLegs) {
  establish_feedback();
  ASSERT_EQ(drive_switch({controller_name_}, {}),
            controller_interface::return_type::OK);
  cycle(5);
  (void)take_positions();
  trajectory_msgs::msg::JointTrajectory message;
  message.joint_names = {kJointName};
  trajectory_msgs::msg::JointTrajectoryPoint up;
  up.positions = {kFixtureFeedbackPosition + 0.1745};
  up.time_from_start = rclcpp::Duration(4, 0);
  trajectory_msgs::msg::JointTrajectoryPoint back;
  back.positions = {kFixtureFeedbackPosition};
  back.time_from_start = rclcpp::Duration(8, 0);
  message.points = {up, back};
  ASSERT_NO_FATAL_FAILURE(wait_for_trajectory_subscription());
  (void)take_positions();
  publisher_->publish(message);
  cycle(4500);
  const auto positions = take_positions();
  ASSERT_EQ(positions.size(), 4500U);
  const auto peak = std::max_element(positions.begin(), positions.end());
  ASSERT_NE(peak, positions.end());
  const std::size_t peak_index =
      static_cast<std::size_t>(std::distance(positions.begin(), peak));
  for (std::size_t index = 1; index < positions.size(); ++index) {
    EXPECT_LE(std::abs(positions[index] - positions[index - 1]),
              0.001 + kPositionQuantum);
    if (index <= peak_index) {
      EXPECT_GE(positions[index] + kPositionQuantum, positions[index - 1]);
    } else {
      EXPECT_LE(positions[index], positions[index - 1] + kPositionQuantum);
    }
  }
  EXPECT_NEAR(*peak, kFixtureFeedbackPosition + 0.1745, kPositionQuantum);
  EXPECT_NEAR(positions.back(), kFixtureFeedbackPosition, kPositionQuantum);
}

TEST_P(JtcIntegrationTest, PositionVelocityTrajectoryReachesBothWireFields) {
  if (!position_velocity_) GTEST_SKIP() << "position-only compatibility case";
  establish_feedback();
  ASSERT_EQ(drive_switch({controller_name_}, {}),
            controller_interface::return_type::OK);
  cycle(5);
  (void)take_positions();
  (void)take_velocities();
  (void)take_efforts();

  trajectory_msgs::msg::JointTrajectory message;
  message.joint_names = {kJointName};
  trajectory_msgs::msg::JointTrajectoryPoint target;
  target.positions = {kFixtureFeedbackPosition + 0.1};
  target.velocities = {0.05};
  target.time_from_start = rclcpp::Duration(1, 0);
  trajectory_msgs::msg::JointTrajectoryPoint stop;
  stop.positions = {kFixtureFeedbackPosition + 0.15};
  stop.velocities = {0.0};
  stop.time_from_start = rclcpp::Duration(2, 0);
  message.points = {target, stop};
  ASSERT_NO_FATAL_FAILURE(wait_for_trajectory_subscription());
  publisher_->publish(message);
  cycle(1100);

  const auto positions = take_positions();
  const auto velocities = take_velocities();
  const auto efforts = take_efforts();
  ASSERT_EQ(positions.size(), velocities.size());
  ASSERT_EQ(positions.size(), efforts.size());
  EXPECT_TRUE(std::any_of(positions.begin(), positions.end(), [](double value) {
    return value > kFixtureFeedbackPosition + 0.02;
  }));
  EXPECT_TRUE(std::any_of(velocities.begin(), velocities.end(),
                          [](double value) {
                            return std::abs(value) > 0.02;
                          }));
  double maximum_abs_velocity = 0.0;
  for (const double velocity : velocities) {
    maximum_abs_velocity = std::max(maximum_abs_velocity, std::abs(velocity));
  }
  EXPECT_LT(maximum_abs_velocity, 1.0);
  for (const double effort : efforts) {
    EXPECT_NEAR(effort, 0.0, 30.0 / 4095.0);
  }
}

TEST_P(JtcIntegrationTest, DeactivationStopsFramesWhileHardwareKeepsCycling) {
  establish_feedback();
  ASSERT_EQ(drive_switch({controller_name_}, {}),
            controller_interface::return_type::OK);
  (void)take_positions();
  cycle(10);
  ASSERT_EQ(take_positions().size(), 10U);
  ASSERT_EQ(drive_switch({}, {controller_name_}),
            controller_interface::return_type::OK);
  (void)take_positions();
  cycle(20);
  EXPECT_TRUE(take_positions().empty());
}

TEST_P(JtcIntegrationTest, OutOfEnvelopeGoalLatchesAndStopsTransmission) {
  establish_feedback();
  ASSERT_EQ(drive_switch({controller_name_}, {}),
            controller_interface::return_type::OK);
  cycle(5);
  (void)take_positions();
  trajectory_msgs::msg::JointTrajectory message;
  message.joint_names = {kJointName};
  trajectory_msgs::msg::JointTrajectoryPoint far;
  far.positions = {kFixtureFeedbackPosition + 2.0};
  far.time_from_start = rclcpp::Duration(0, 20000000);
  message.points = {far};
  ASSERT_NO_FATAL_FAILURE(wait_for_trajectory_subscription());
  (void)take_positions();
  publisher_->publish(message);
  cycle(50);
  const auto positions = take_positions();
  ASSERT_FALSE(positions.empty());
  for (const double position : positions) {
    EXPECT_LE(std::abs(position - kFixtureFeedbackPosition),
              0.5 + kPositionQuantum);
  }
  // In the P+V deployment the steep 2 rad / 20 ms trajectory can trip the
  // desired-velocity tuple bound before its position reaches the envelope.
  // Both are whole-command fail-closed gates; position-only reaches the
  // envelope directly.
  if (position_velocity_) {
    EXPECT_TRUE(saw_position_tuple_rejection());
  } else {
    EXPECT_TRUE(saw_position_envelope());
  }
  cycle(20);
  EXPECT_TRUE(take_positions().empty());
  EXPECT_NE(drive_switch({}, {controller_name_}),
            controller_interface::return_type::OK);
}

INSTANTIATE_TEST_SUITE_P(PositionAndPositionVelocity, JtcIntegrationTest,
                         ::testing::Values(false, true));

}  // namespace
}  // namespace mech::mech_bringup
