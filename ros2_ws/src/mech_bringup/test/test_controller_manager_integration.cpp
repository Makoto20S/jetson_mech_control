// E1's last evidence layer (ADR-015): the authorization boundary driven by a
// REAL controller_manager, not by a test calling CompositeSystem::read/write
// directly.
//
// What this adds over test_composite_system.cpp: those tests invoke
// perform_command_mode_switch() by hand with arguments the test chose. Here
// the real ControllerManager decides when to call it and with which interface
// names, derived from a controller's command_interface_configuration(). If our
// authorization hooks were wired to a transition the manager does not actually
// make, or expected names in a different shape, only this test would catch it.
//
// The TX counter is a FakeTransport behind the production
// Ak30ForceControlRuntime, so "no transmission" means no AK3.0 force-control
// CAN frame was produced - not merely that a C++ method went uncalled.
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
constexpr char kJointName[] = "joint_1";
constexpr char kControllerName[] = "writer";
constexpr char kControllerType[] = "test/WriterController";
// The manager's control period; also the runtime's configured period.
constexpr std::int64_t kPeriodNanoseconds = 2000000;
// motor1's configured reporting period: send_can_status_rate_hz = 50.
constexpr std::int64_t kFeedbackPeriodNs = 20000000;

[[nodiscard]] Ak30RuntimeConfig runtime_config() {
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
  // motor1's real pair, read back from its configuration: 50 Hz reporting
  // (20 ms) and a 3x window (60 ms). Using the real ratio here matters - it is
  // what makes this test exercise ten control cycles per feedback frame, the
  // same 500 Hz-against-50 Hz relationship the bench has.
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
  // ADR-017 shape: one motion command interface plus the always-exported
  // command_generation interface.
  joint.command_interfaces = {
      interface(hardware_interface::HW_IF_POSITION),
      interface(
          mech::mech_hardware_ros2_control::kCommandGenerationInterface)};
  joint.state_interfaces = {interface(hardware_interface::HW_IF_POSITION),
                            interface(hardware_interface::HW_IF_VELOCITY),
                            interface(hardware_interface::HW_IF_EFFORT)};
  info.joints.push_back(joint);
  return info;
}

// Feedback payload: 90.0 deg, 10000 ERPM, 2.0 A, 40 C, no fault.
[[nodiscard]] RawCanFrame feedback_frame(std::int64_t arrival_ns) {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03;
  payload[1] = 0x84;
  payload[2] = 0x03;
  payload[3] = 0xE8;
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

// Claims exactly one position command interface and writes a target on each of
// its first `writes_allowed` update() calls, then stops writing while staying
// active. That is the "controller went quiet but was never deactivated" shape -
// the hardware keeps being cycled by the manager either way.
class WriterController final : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {std::string(kJointName) + "/" + hardware_interface::HW_IF_POSITION}};
  }

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::CallbackReturn on_init() override {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time&,
                                           const rclcpp::Duration&) override {
    ++updates_;
    if (updates_ <= writes_allowed_ && !command_interfaces_.empty()) {
      command_interfaces_[0].set_value(target_);
      ++writes_;
    }
    return controller_interface::return_type::OK;
  }

  void allow_writes(std::size_t count) noexcept { writes_allowed_ = count; }
  [[nodiscard]] std::size_t writes() const noexcept { return writes_; }

 private:
  double target_{0.25};
  std::size_t updates_{0U};
  std::size_t writes_{0U};
  std::size_t writes_allowed_{0U};
};

class ControllerManagerIntegrationTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }

  void SetUp() override {
    transport_ = std::make_unique<FakeTransport>(64U);
    auto system = std::make_unique<
        mech::mech_hardware_ros2_control::CompositeSystem>();
    // Inject before import_component: ResourceManager calls on_init() during
    // the import, and set_runtime() is rejected once initialized.
    ASSERT_TRUE(system->set_runtime(std::make_unique<Ak30ForceControlRuntime>(
        *transport_, [this]() { return now(); }, runtime_config())));

    auto resources = std::make_unique<hardware_interface::ResourceManager>();
    resources->import_component(std::move(system), hardware_info());
    rclcpp_lifecycle::State active{
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
        hardware_interface::lifecycle_state_names::ACTIVE};
    ASSERT_EQ(resources->set_component_state(kHardwareName, active),
              hardware_interface::return_type::OK);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // Match the manager's advertised rate to the control period this test
    // drives, so per-controller update-rate throttling cannot silently skip
    // update() calls and make a TX count look like a fix.
    rclcpp::NodeOptions options =
        controller_manager::get_cm_node_options();
    options.parameter_overrides(
        {rclcpp::Parameter("update_rate", 500)});
    manager_ = std::make_shared<controller_manager::ControllerManager>(
        std::move(resources), executor_, "test_controller_manager", "", options);

    controller_ = std::make_shared<WriterController>();
    ASSERT_NE(manager_->add_controller(controller_, kControllerName,
                                       kControllerType),
              nullptr);
    ASSERT_EQ(manager_->configure_controller(kControllerName),
              controller_interface::return_type::OK);
  }

  void TearDown() override {
    // A ControllerManager must not be destroyed with an active controller
    // still holding a claim; deactivate best-effort so teardown reflects a
    // normal shutdown rather than an abort path.
    if (controller_ && controller_->get_state().id() ==
                           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      deactivate_controller();
    }
    manager_.reset();
    executor_.reset();
    controller_.reset();
    transport_.reset();
  }

  // Monotonic test clock shared by the runtime and the manager loop, advanced
  // one control period per cycle. Not a frozen clock: every cycle really moves
  // time forward, so the staged watchdog runs for real.
  [[nodiscard]] MonotonicTime now() const {
    return MonotonicTime::from_nanoseconds(now_nanoseconds_).value();
  }

  // ADR-016 Decision 3: the joint is unclaimable until its state is known, so
  // a controller cannot be activated before one feedback frame has landed.
  // That is the real startup shape - motor1 reports at 50 Hz against this
  // 500 Hz loop - and it is why this has to happen before the switch.
  void establish_feedback() {
    ASSERT_EQ(transport_->inject_receive(feedback_frame(now_nanoseconds_)),
              TransportResult::Ok);
    cycle(1);
  }

  void activate_controller() { switch_controller({kControllerName}, {}); }

  void deactivate_controller() { switch_controller({}, {kControllerName}); }

  // ControllerManager::switch_controller() blocks until the switch is applied,
  // and the switch is applied inside update(). So the request has to be issued
  // from another thread while this one keeps driving the loop - otherwise it
  // just times out. Cycles spent here are counted like any other, which is why
  // the assertions below compare TX counts before and after rather than
  // asserting exact totals.
  void switch_controller(const std::vector<std::string>& start,
                         const std::vector<std::string>& stop) {
    auto result = std::async(std::launch::async, [this, start, stop]() {
      return manager_->switch_controller(
          start, stop,
          controller_manager_msgs::srv::SwitchController::Request::STRICT);
    });
    // Bounded so a genuine hang fails the test instead of spinning forever.
    for (int guard = 0; guard < 2000; ++guard) {
      if (result.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready) {
        break;
      }
      cycle(1);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    ASSERT_EQ(result.wait_for(std::chrono::seconds(2)),
              std::future_status::ready);
    ASSERT_EQ(result.get(), controller_interface::return_type::OK);
  }

  // One full manager cycle in the documented order: read -> update -> write,
  // with the device reporting on its own schedule underneath. motor1 reports
  // every 20 ms while the loop runs every 2 ms, so a frame lands on one cycle
  // in ten - without that, the sample ages past its window and ADR-016 quite
  // correctly faults the component a few cycles in.
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
      manager_->read(time, period);
      manager_->update(time, period);
      manager_->write(time, period);
    }
  }

  // AK3.0 force-control command frames queued on the fake transport. Counted
  // rather than drained so a leak shows up as a growing number.
  [[nodiscard]] std::size_t transmitted() const {
    return transport_->pending_transmit();
  }

  std::unique_ptr<FakeTransport> transport_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<WriterController> controller_;
  std::int64_t now_nanoseconds_{0};
  std::int64_t last_feedback_nanoseconds_{0};
};

// E1: the manager cycles the hardware at its control rate while no controller
// is active, so nothing ever claims the command interface. No AK3.0 command
// frame may be produced. Before ADR-015 the unclaimed joint's
// CanonicalCommand{0, 0, 0} went down every cycle, which on this position
// deployment is a commanded move to the zero position.
TEST_F(ControllerManagerIntegrationTest, InactiveControllerProducesNoCommandFrames) {
  cycle(20);
  EXPECT_EQ(transmitted(), 0U);
  EXPECT_EQ(controller_->writes(), 0U);
}

// E2, NOT E1 - deliberately disabled, and it is the lease work's definition of
// done. An ACTIVE controller that stops writing still holds the claim, so
// CompositeSystem keeps dispatching an authorized command every cycle and
// Ak30ForceControlRuntime marks each of those as a refresh. Measured today:
// 20 silent cycles produce 20 further frames.
//
// ADR-015 cannot close this. A ros2_control command interface is a raw double
// pointer, so the hardware layer cannot observe set_value() calls; authorization
// is bound to the claim, and *freshness* has to come from somewhere else - the
// controller's own target validity and TTL. Enabling this test is exactly the
// exit criterion for that work. It is left compiled and named rather than
// deleted so the gap stays visible in the suite output instead of living only
// in a planning document.
TEST_F(ControllerManagerIntegrationTest,
       DISABLED_SilentControllerDoesNotRefreshItsCommand) {
  establish_feedback();
  controller_->allow_writes(1U);
  activate_controller();
  cycle(1);
  ASSERT_EQ(controller_->writes(), 1U);

  // The written target is submitted by the read() of the following cycle.
  cycle(1);
  const auto after_first_command = transmitted();
  ASSERT_GE(after_first_command, 1U);

  // Twenty further cycles with a silent controller: the staged watchdog must
  // freeze and then fail, never re-send.
  cycle(20);
  EXPECT_EQ(transmitted(), after_first_command);
  EXPECT_EQ(controller_->writes(), 1U);
}

// E1: deactivating the controller makes the real manager release the claim via
// perform_command_mode_switch(stop). Authorization is revoked and the pending
// command cancelled, so the frames stop immediately rather than after the hard
// TTL lapses - the 2026-09-12 audit measured 11 frames over 22 ms here.
TEST_F(ControllerManagerIntegrationTest, DeactivatingControllerStopsCommandFrames) {
  establish_feedback();
  controller_->allow_writes(3U);
  activate_controller();
  cycle(4);
  deactivate_controller();
  const auto while_active = transmitted();
  ASSERT_GE(while_active, 1U);

  cycle(20);
  EXPECT_EQ(transmitted(), while_active);
}

// E2, NOT E1 - deliberately disabled, same root cause as the silent-controller
// case above. On revoke, CompositeSystem keeps the joint's last command value
// rather than zeroing it, because on a position interface a substituted 0.0 is
// a commanded move to the zero position (ADR-012 Decision 3). That is the safer
// of the two available choices, but it means re-claiming re-dispatches the
// pre-deactivation target, and the runtime treats that dispatch as a refresh.
// Closing it needs the same controller-side freshness the case above needs:
// authorization must not be sufficient to transmit, a fresh target must be
// required too.
TEST_F(ControllerManagerIntegrationTest,
       DISABLED_ReactivationDoesNotReplayTheOldTarget) {
  establish_feedback();
  controller_->allow_writes(1U);
  activate_controller();
  cycle(2);
  deactivate_controller();
  const auto before = transmitted();
  ASSERT_GE(before, 1U);

  activate_controller();
  cycle(20);
  EXPECT_EQ(transmitted(), before);
}

}  // namespace
}  // namespace mech::mech_bringup
