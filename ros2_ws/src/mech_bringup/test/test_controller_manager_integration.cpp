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
// A write budget large enough that only allow_writes(0) can end it. See
// WriterController::allow_writes() for why a small fixed budget is unsafe.
constexpr std::size_t kWritesUntilSilenced = 100000U;

// Records every telemetry event the runtime emits. try_push() runs inside
// read(), i.e. on the thread driving the manager loop, and every read of
// events happens after cycling stops - so the plain vector needs no guard.
class RecordingTelemetry final : public FeedbackTelemetryCapture {
 public:
  bool try_push(const FeedbackTelemetryEvent& event) noexcept override {
    events.push_back(event);
    return true;
  }
  std::vector<FeedbackTelemetryEvent> events;
};

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

// Writes a target on each of its first `writes_allowed` update() calls, then
// stops writing while staying active. That is the "controller went quiet but
// was never deactivated" shape - the hardware keeps being cycled by the manager
// either way.
//
// The tier is a constructor argument because ADR-017 makes the
// command_generation interface OPTIONAL, and both halves of that decision need
// driving. Strong tier claims it alongside the motion interface and bumps it
// on every real write, so the hardware can see a refresh. Weak tier claims only
// the motion interface, exactly like a stock joint_trajectory_controller that
// has never heard of this project - being able to run those is why the
// framework exists (02_architecture_and_interfaces.md).
class WriterController final : public controller_interface::ControllerInterface {
 public:
  explicit WriterController(bool strong_tier) noexcept
      : strong_tier_(strong_tier) {}

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override {
    std::vector<std::string> names{std::string(kJointName) + "/" +
                                   hardware_interface::HW_IF_POSITION};
    if (strong_tier_) {
      names.push_back(
          std::string(kJointName) + "/" +
          mech::mech_hardware_ros2_control::kCommandGenerationInterface);
    }
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            names};
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
    const std::size_t expected = strong_tier_ ? 2U : 1U;
    if (updates_ <= writes_allowed_ &&
        command_interfaces_.size() == expected) {
      command_interfaces_[0].set_value(target_);
      // ADR-017: the generation is what makes this write observable to the
      // hardware. It changes only when a real target is written, never on a
      // bare manager cycle. A weak-tier controller has nothing to bump, which
      // is precisely the gap it leaves open.
      if (strong_tier_) {
        command_interfaces_[1].set_value(static_cast<double>(++generation_));
      }
      ++writes_;
    }
    return controller_interface::return_type::OK;
  }

  // How many update() calls this controller still writes a target on, counted
  // from activation.
  //
  // CAREFUL: a small fixed budget makes a test host-speed dependent, and the
  // failure is silent on a fast machine. drive_switch() keeps calling cycle()
  // until the manager applies the switch, each cycle advances the fixture's
  // clock by one control period, and how many it needs depends on real thread
  // scheduling. So a switch burns an unpredictable slice of this budget AND an
  // unpredictable amount of simulated time. Once the budget runs out the
  // controller is silent, and for a strong-tier controller silence is a fault
  // within one hard TTL.
  //
  // That is not hypothetical: DeactivatingControllerStopsCommandFrames passed on
  // x86_64 and failed on the Jetson's aarch64 with a fixed budget of 3, because
  // the slower host burned the budget inside activate_controller() and the
  // watchdog faulted the component before the test reached its deactivation.
  //
  // Use kWritesUntilSilenced and then allow_writes(0) at the exact moment the
  // test wants silence. Only use a small budget when the test is ABOUT the
  // controller falling silent, and then only where extra burned cycles cannot
  // change the outcome.
  void allow_writes(std::size_t count) noexcept { writes_allowed_ = count; }
  [[nodiscard]] std::size_t writes() const noexcept { return writes_; }

 private:
  bool strong_tier_;
  double target_{0.25};
  std::size_t updates_{0U};
  std::size_t writes_{0U};
  std::size_t writes_allowed_{0U};
  std::size_t generation_{0U};
};

class ControllerManagerIntegrationTest : public ::testing::Test {
 protected:
  // Which ADR-017 tier the fixture's controller is in. Virtual so the weak-tier
  // cases below reuse this whole fixture by overriding one answer; SetUp() runs
  // after construction, so the override is in effect by the time it is read.
  [[nodiscard]] virtual bool strong_tier() const { return true; }

  // Lets a derived fixture tighten the runtime configuration before the
  // component is imported; the default is the shared runtime_config().
  [[nodiscard]] virtual Ak30RuntimeConfig fixture_runtime_config() const {
    return runtime_config();
  }

  // Telemetry sink handed to the runtime at construction. Null by default, so
  // the existing cases keep the production-shaped "no sink" wiring; a derived
  // fixture that needs the failure REASON out of the hardware returns its own
  // recorder. Such a recorder must outlive the runtime: TearDown drops
  // manager_ (and with it the ResourceManager that owns the runtime) before
  // any member of a derived fixture is destroyed.
  [[nodiscard]] virtual FeedbackTelemetryCapture* telemetry() { return nullptr; }

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
        *transport_, [this]() { return now(); }, fixture_runtime_config(),
        telemetry())));

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

    controller_ = std::make_shared<WriterController>(strong_tier());
    ASSERT_NE(manager_->add_controller(controller_, kControllerName,
                                       kControllerType),
              nullptr);
    ASSERT_EQ(manager_->configure_controller(kControllerName),
              controller_interface::return_type::OK);
  }

  void TearDown() override {
    // A ControllerManager must not be destroyed with an active controller still
    // holding a claim, so deactivate first - but best-effort, and without
    // asserting. A test that deliberately drives the staged watchdog (ADR-012)
    // leaves the hardware component in an error state, and the manager then
    // refuses this switch because the interfaces are no longer available. That
    // refusal is the product behaving correctly; reporting it as a failure
    // would make every such test red for the wrong reason.
    if (controller_ && controller_->get_state().id() ==
                           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      (void)drive_switch({}, {kControllerName});
    }
    // The order below is load-bearing, not stylistic. A controller that was
    // refused deactivation still holds its LoanedCommandInterfaces, and their
    // deleters call back into the ResourceManager that ControllerManager owns.
    // Dropping the manager first leaves this fixture holding the last
    // reference, so the controller outlives that ResourceManager and each
    // deleter locks a destroyed mutex: std::system_error thrown out of a
    // noexcept destructor, which terminates the process. Releasing this
    // reference first hands the controller to ~ControllerManager, which
    // destroys its controller list while resource_manager_ is still alive -
    // declared earlier in the class, so destroyed later.
    controller_.reset();
    manager_.reset();
    executor_.reset();
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

  // Issues the switch and drives the manager loop until it completes, handing
  // back the manager's own answer instead of asserting on it - teardown needs a
  // refusal to be a value, not a test failure.
  //
  // ControllerManager::switch_controller() blocks until the switch is applied
  // inside update(). So the request has to be issued from another thread while
  // this one keeps driving the loop - otherwise it just times out. Cycles spent
  // here are counted like any other, which is why the assertions below compare
  // TX counts before and after rather than asserting exact totals.
  controller_interface::return_type drive_switch(
      const std::vector<std::string>& start,
      const std::vector<std::string>& stop) {
    auto result = std::async(std::launch::async, [this, start, stop]() {
      return manager_->switch_controller(
          start, stop,
          controller_manager_msgs::srv::SwitchController::Request::STRICT);
    });
    // Bounded so a genuine hang surfaces as a failure instead of spinning
    // forever. Note that abandoning the future is not an option: std::async's
    // future joins in its destructor, and the switch cannot complete unless
    // this thread keeps calling update() - so bailing out early would deadlock
    // rather than fail.
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

// ADR-017's exit criterion, driven by a real ControllerManager. An ACTIVE
// controller that stops writing still holds the claim, so CompositeSystem keeps
// dispatching an authorized command every cycle. Before ADR-017 the runtime
// marked every one of those as a refresh and 20 silent cycles produced 20
// further frames; now the claimed command_generation interface lets the
// hardware see that nothing was refreshed, and the staged watchdog freezes and
// then faults instead of re-sending.
//
// READ THIS BEFORE TRUSTING A GREEN HERE. The criterion is weaker than the one
// this test was written with, deliberately and by owner decision. It was "the
// hardware cannot be fooled by an ARBITRARY controller", which is the same
// statement as "no arbitrary controller may drive this hardware" - and this
// project exists to be a general ros2_control target, so that is unachievable
// rather than merely unmet (ADR-017, alternative I). What is asserted now is
// "cannot be fooled by a controller that claimed the generation interface",
// i.e. the strong tier. WriterController claims it, which is what makes this
// test the strong-tier case. The weak tier's remaining gap is pinned by
// WeakTierControllerKeepsSendingWhileSilent below - not by a document.
TEST_F(ControllerManagerIntegrationTest,
       SilentControllerDoesNotRefreshItsCommand) {
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
  // freeze and then fail, never re-send. The component ends this test in an
  // error state, which is the point - see TearDown for why that is tolerated
  // rather than asserted away.
  cycle(20);
  EXPECT_EQ(transmitted(), after_first_command);
  EXPECT_EQ(controller_->writes(), 1U);
}

// E1: deactivating the controller makes the real manager release the claim via
// perform_command_mode_switch(stop). Authorization is revoked and the pending
// command cancelled, so the frames stop immediately rather than after the hard
// TTL lapses - the 2026-09-12 audit measured 11 frames over 22 ms here.
//
// The controller writes on every update right up to the deactivation, and that
// is load-bearing twice over. It makes the assertion stronger: the controller
// is actively commanding at the moment the claim is revoked, so "the frames
// stopped" can only be the revoke, never the controller having gone quiet by
// itself. And it makes the test host-independent - see allow_writes() for why a
// fixed write budget is not safe here.
TEST_F(ControllerManagerIntegrationTest, DeactivatingControllerStopsCommandFrames) {
  establish_feedback();
  controller_->allow_writes(kWritesUntilSilenced);
  activate_controller();
  cycle(4);
  deactivate_controller();
  const auto while_active = transmitted();
  ASSERT_GE(while_active, 1U);

  // Deactivated, so the manager no longer calls update() and the controller
  // cannot write even though its budget is not exhausted. Any further frame
  // would be the hardware acting on a revoked claim.
  cycle(20);
  EXPECT_EQ(transmitted(), while_active);
}

// ADR-017's other exit criterion. On revoke, CompositeSystem keeps the joint's
// last command value rather than zeroing it, because on a position interface a
// substituted 0.0 is a commanded move to the zero position (ADR-012 Decision
// 3). That is the safer of the two available choices, but before ADR-017 it
// meant re-claiming re-dispatched the pre-deactivation target and the runtime
// treated that dispatch as a refresh - a move the operator never asked for, on
// the cycle a controller was activated. The generation baseline is now re-seeded
// when a claim changes hands, so a re-claim alone commands nothing.
//
// The controller writes throughout its first activation on purpose. A strong-
// tier controller that fell silent while still active would be faulted by the
// staged watchdog - correctly, and that is the test above - and a faulted
// component cannot be re-claimed at all, so the replay this test is about could
// never be observed. Keeping the lease healthy until the deactivation is what
// makes the reactivation reachable; the silence that matters here starts after
// it.
TEST_F(ControllerManagerIntegrationTest, ReactivationDoesNotReplayTheOldTarget) {
  establish_feedback();
  controller_->allow_writes(kWritesUntilSilenced);
  activate_controller();
  cycle(2);
  deactivate_controller();
  const auto before = transmitted();
  ASSERT_GE(before, 1U);
  const auto writes_before = controller_->writes();
  ASSERT_GE(writes_before, 1U);

  // Silent from here on, so the only thing that could produce a frame is the
  // re-claim replaying the target the controller gave before it was stopped.
  controller_->allow_writes(0U);
  activate_controller();
  cycle(20);
  EXPECT_EQ(transmitted(), before);
  EXPECT_EQ(controller_->writes(), writes_before);
}

// The weak tier: a controller that claims only the motion interface, which is
// every stock ros2_control controller. ADR-017 Decision 4 keeps these working,
// and these two tests are what stop that decision from being only a sentence in
// a document - both turn red if the weak tier's behaviour changes.
class WeakTierIntegrationTest : public ControllerManagerIntegrationTest {
 protected:
  [[nodiscard]] bool strong_tier() const override { return false; }
};

// The founding requirement, as an executable check: being able to swap in any
// standard ros2_control controller is why this framework exists
// (02_architecture_and_interfaces.md line 255). A controller that has never
// heard of command_generation must still be able to claim the joint and drive
// the motor. If a future change makes the generation interface mandatory, this
// is the test that says so.
TEST_F(WeakTierIntegrationTest, WeakTierControllerCanStillCommandTheHardware) {
  establish_feedback();
  controller_->allow_writes(1U);
  activate_controller();
  cycle(1);
  ASSERT_EQ(controller_->writes(), 1U);

  cycle(1);
  EXPECT_GE(transmitted(), 1U);
}

// The accepted risk, pinned. A weak-tier controller that stays ACTIVE and stops
// writing keeps producing command frames, because the hardware genuinely cannot
// tell its silence apart from the manager simply cycling - there is no
// generation to compare (ADR-017 Decision 4, composite_system.cpp: fresh =
// authorized).
//
// What that means at the bench, and it is not the same on every interface: on
// position this is a hold, which is why T7 is acceptable with the owner
// present. On velocity it is the motor continuing to turn at its last commanded
// speed, and on effort it is continuing to push - so T8 and T9 must not be run
// with a controller this project does not own. The strong tier closes this, and
// SilentControllerDoesNotRefreshItsCommand is the same scenario with the
// generation interface claimed; comparing the two is the point.
TEST_F(WeakTierIntegrationTest, WeakTierControllerKeepsSendingWhileSilent) {
  establish_feedback();
  controller_->allow_writes(1U);
  activate_controller();
  cycle(2);
  ASSERT_EQ(controller_->writes(), 1U);
  const auto after_first_command = transmitted();
  ASSERT_GE(after_first_command, 1U);

  // Twenty silent cycles. The strong tier produces nothing further here; the
  // weak tier keeps going, and stays healthy while it does - no watchdog fires,
  // because as far as the hardware can see it is being commanded.
  cycle(20);
  EXPECT_GT(transmitted(), after_first_command);
  EXPECT_EQ(controller_->writes(), 1U);
}

// ADR-019 through a real ControllerManager. The envelope is the answer to the
// gap the test above pins: a weak-tier controller cannot be made trustworthy,
// so the bound has to live in the hardware, below anything a controller can
// reach. The tightened error bound here is what the fixture's own numbers
// demand - the feedback frame decodes to about -4.19 rad and WriterController
// targets 0.25, so 0.5 rad of allowed error puts the very first authorized
// write outside the envelope.
class WeakTierEnvelopeTest : public WeakTierIntegrationTest {
 protected:
  [[nodiscard]] Ak30RuntimeConfig fixture_runtime_config() const override {
    auto config = runtime_config();
    config.position_max_error_rad = 0.5;
    return config;
  }

  // Declared here rather than in the base so only this case pays for it; the
  // base's TearDown releases the manager before this member dies, which is
  // what keeps the runtime from writing into a destroyed recorder.
  [[nodiscard]] FeedbackTelemetryCapture* telemetry() override {
    return &telemetry_;
  }

  RecordingTelemetry telemetry_;
};

// The controller writes throughout, and asserting that it did is half the
// claim: without it, zero transmissions would equally describe a controller
// that never commanded anything. What this test says is that the target WAS
// written into the command interface and the hardware still put no frame on
// the wire, and that nothing was clamped down to a reachable value instead.
//
// The refused deactivation is the observable for "the component latched".
// Humble's ControllerManager exposes no resource-manager accessor, so the
// component's state is only reachable through behaviour: a latched component
// fails read(), its interfaces stop being available, and the manager therefore
// refuses the STRICT switch that would stop the controller. A manager that
// accepted this switch would mean the hardware was still healthy - i.e. the
// envelope never fired.
//
// On its own, though, a refusal only says the component failed, and every
// other fail-closed rule in this stack (ADR-012 expiry, ADR-016 feedback
// quality) produces the same refusal. The telemetry reason is what separates
// them, so it is asserted here. The controller staying ACTIVE after the
// refused switch is the second half - the manager did not partially apply the
// switch, so the claim is still held and the refusal is about the hardware,
// not the controller.
TEST_F(WeakTierEnvelopeTest, WeakTierControllerCannotEscapeTheEnvelope) {
  establish_feedback();
  controller_->allow_writes(kWritesUntilSilenced);
  activate_controller();
  cycle(20);
  EXPECT_EQ(transmitted(), 0U);
  EXPECT_GT(controller_->writes(), 0U);
  EXPECT_NE(drive_switch({}, {kControllerName}),
            controller_interface::return_type::OK);
  EXPECT_EQ(controller_->get_state().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  std::size_t envelope_errors = 0U;
  for (const auto& event : telemetry_.events) {
    if (event.kind == FeedbackTelemetryKind::RuntimeError &&
        event.reason == FeedbackTelemetryReason::PositionEnvelope) {
      ++envelope_errors;
    }
  }
  EXPECT_GT(envelope_errors, 0U);
}

}  // namespace
}  // namespace mech::mech_bringup
