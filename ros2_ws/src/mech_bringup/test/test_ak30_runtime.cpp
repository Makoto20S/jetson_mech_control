#include "mech_bringup/ak30_force_runtime.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include "mech_control_core/adapter_template.hpp"
#include "mech_control_core/frame.hpp"
#include "mech_control_core/time.hpp"
#include "mech_control_core/transport.hpp"
#include "mech_protocol_cubemars/ak30_force_session.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include "mech_protocol_cubemars/ak30_mapping.hpp"
#include "mech_simulation/fake_transport.hpp"

namespace mech::mech_bringup {
namespace {

using mech::mech_control_core::AdapterResult;
using mech::mech_control_core::CanonicalDeviceState;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::DeviceConfig;
using mech::mech_control_core::FrameDirection;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::ProtocolProfile;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::SampleQuality;
using mech::mech_control_core::TransportResult;
using mech::mech_protocol_cubemars::Ak30Mapping;
using mech::mech_protocol_cubemars::feedback_can_id;
using mech::mech_protocol_cubemars::force_control_can_id;
using mech::mech_protocol_cubemars::ForceControlSubMode;
using mech::mech_simulation::FakeTransport;

// motor1: drive id 104 decimal -> command 0x0868, feedback 0x2968.
constexpr std::uint16_t kDriveId = 104U;
constexpr std::uint32_t kLogicalBus = 1U;

[[nodiscard]] MonotonicTime at(std::int64_t nanoseconds) {
  return MonotonicTime::from_nanoseconds(nanoseconds).value();
}

// Writes one command with transmit authorization - the shape CompositeSystem
// produces for a joint whose command interface is claimed (ADR-015). Every
// test that models a live controller goes through here; the unauthorized
// dispatch has its own dedicated tests.
[[nodiscard]] bool write_authorized(
    Ak30ForceControlRuntime& runtime,
    const mech_hardware_ros2_control::CanonicalCommand& command) noexcept {
  // fresh = true: this models a controller that actually refreshed its target
  // this cycle (ADR-017). The stale-but-authorized shape - the manager cycling
  // while the controller says nothing - has its own helper below.
  const mech_hardware_ros2_control::CommandDispatch dispatch{command, true, true};
  return runtime.write(&dispatch, 1U);
}

// Writes one dispatch that is authorized but NOT fresh: the claim is held and
// the manager cycled, but the controller did not refresh its target
// (ADR-017 Decision 3.3). The runtime must treat it as "no new command".
[[nodiscard]] bool write_authorized_stale(
    Ak30ForceControlRuntime& runtime,
    const mech_hardware_ros2_control::CanonicalCommand& command) noexcept {
  const mech_hardware_ros2_control::CommandDispatch dispatch{command, true, false};
  return runtime.write(&dispatch, 1U);
}

// Writes one dispatch WITHOUT authorization: the joint is not claimed, so the
// runtime must treat it as "no command at all" (ADR-015 Decision 2).
[[nodiscard]] bool write_unauthorized(
    Ak30ForceControlRuntime& runtime,
    const mech_hardware_ros2_control::CanonicalCommand& command) noexcept {
  const mech_hardware_ros2_control::CommandDispatch dispatch{command, false, false};
  return runtime.write(&dispatch, 1U);
}

// A test clock the test advances by hand; the production runtime injects
// steady_clock. Backwards moves are rejected by MonotonicTime semantics and
// must be surfaced by the runtime, never silently accepted.
class TestClock final {
 public:
  explicit TestClock(std::int64_t now_nanoseconds = 0)
      : now_nanoseconds_(now_nanoseconds) {}
  [[nodiscard]] MonotonicTime now() const noexcept { return at(now_nanoseconds_); }
  void advance(std::int64_t delta_nanoseconds) noexcept {
    now_nanoseconds_ += delta_nanoseconds;
  }
  void set(std::int64_t nanoseconds) noexcept { now_nanoseconds_ = nanoseconds; }

 private:
  std::int64_t now_nanoseconds_;
};

[[nodiscard]] Ak30RuntimeConfig runtime_config() {
  Ak30RuntimeConfig config{};
  config.drive_id = kDriveId;
  config.logical_bus = kLogicalBus;
  config.sub_mode = ForceControlSubMode::Position;
  config.mapping = Ak30Mapping{};
  config.gains.kp = 1.0;
  config.gains.kd = 1.0;
  config.control_period_nanoseconds = 2000000;
  config.command_ttl_nanoseconds = 4000000;
  config.command_hard_ttl_nanoseconds = 6000000;
  // A 1 ms reporting period keeps these tests able to age a sample out within
  // a few control cycles while still satisfying ADR-016 Decision 4's
  // "window >= period". motor1's real pair is 20 ms / 60 ms and is exercised
  // by RejectsFeedbackTtlShorterThanReportingPeriod and the deployment files.
  config.feedback_period_nanoseconds = 1000000;
  config.feedback_ttl_nanoseconds = 6000000;
  return config;
}

// Feedback builder mirroring the session test fixtures: 90.0 deg position,
// 10000 ERPM, 2.0 A Iq, 40 C, no fault.
[[nodiscard]] RawCanFrame feedback_frame(TestClock& clock) {
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
             CanId::create(feedback_can_id(kDriveId),
                           CanFrameFormat::Extended)
                 .value(),
             CanFrameType::Classic, FrameDirection::Rx, 8U, payload,
             clock.now())
      .value();
}

class Ak30RuntimeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    transport_ = std::make_unique<FakeTransport>(16U);
    config_ = runtime_config();
    runtime_ = std::make_unique<Ak30ForceControlRuntime>(
        *transport_, [this]() { return clock_.now(); }, config_);
  }

  std::unique_ptr<FakeTransport> transport_;
  TestClock clock_{0};
  Ak30RuntimeConfig config_{};
  std::unique_ptr<Ak30ForceControlRuntime> runtime_;
};

TEST_F(Ak30RuntimeTest, ConfigureStartStopLifecycleSucceeds) {
  EXPECT_TRUE(runtime_->configure(1U));
  EXPECT_TRUE(runtime_->start());
  runtime_->stop();
  runtime_->stop();  // idempotent
}

TEST_F(Ak30RuntimeTest, RejectsZeroResourceCount) {
  EXPECT_FALSE(runtime_->configure(0U));
}

TEST_F(Ak30RuntimeTest, RejectsBadConfig) {
  Ak30RuntimeConfig bad = runtime_config();
  bad.drive_id = 300U;  // wire field is 8 bits
  Ak30ForceControlRuntime runtime(*transport_, [this]() { return clock_.now(); }, bad);
  EXPECT_FALSE(runtime.configure(1U));
}

TEST_F(Ak30RuntimeTest, RejectsHardTtlAboveBudget) {
  Ak30RuntimeConfig bad = runtime_config();
  bad.command_ttl_nanoseconds = 5000000;
  bad.command_hard_ttl_nanoseconds = 8000000;  // > kMaxHardTtlNanoseconds (6 ms)
  Ak30ForceControlRuntime runtime(*transport_, [this]() { return clock_.now(); }, bad);
  EXPECT_FALSE(runtime.configure(1U));
}

// The ros2_control cycle is read -> update -> write, so the command a
// controller writes this cycle is submitted by the NEXT read. The first read
// after activation must therefore send nothing: it has no valid command yet,
// and inventing 0.0 would be a commanded move to the zero position
// (ADR-012 Decision 3).
TEST_F(Ak30RuntimeTest, FirstReadSendsNothingAndDoesNotInventZero) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  mech_hardware_ros2_control::CanonicalState states[1] = {};
  clock_.set(2000000);
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 0U);
  // No feedback has arrived, so there is nothing to publish and nothing is
  // published (ADR-016); these stay at their constructed values rather than
  // being overwritten with a zero that would look like a measurement.
  EXPECT_FALSE(runtime_->has_valid_sample());
  EXPECT_EQ(states[0].position, 0.0);
  EXPECT_EQ(states[0].velocity, 0.0);
  EXPECT_EQ(states[0].effort, 0.0);
}

// ADR-017 Decision 3.3: authorization says the joint is claimed; freshness
// says the controller spoke. A cycle that carries the first but not the second
// is the controller_manager turning, and it must not become a motor frame.
// Asserted on the transport's frame count, not on an internal flag.
TEST_F(Ak30RuntimeTest, AuthorizedButStaleDispatchSendsNoFrame) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  for (int cycle = 0; cycle < 20; ++cycle) {
    EXPECT_TRUE(write_authorized_stale(*runtime_, command));
    clock_.set(2000000 * (cycle + 1));
    mech_hardware_ros2_control::CanonicalState states[1] = {};
    EXPECT_TRUE(runtime_->read(states, 1U));
  }
  EXPECT_EQ(transport_->pending_transmit(), 0U);
}

// The other half of the same contract: once the controller does speak, the
// command goes out. A freshness gate that sent nothing ever would pass the
// test above and break every deployment.
TEST_F(Ak30RuntimeTest, AFreshDispatchAfterStaleOnesIsStillSent) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized_stale(*runtime_, command));
  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  ASSERT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 0U);

  ASSERT_TRUE(write_authorized(*runtime_, command));
  clock_.set(4000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 1U);
}

TEST_F(Ak30RuntimeTest, FollowingSubmitsWrittenCommand) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  EXPECT_TRUE(write_authorized(*runtime_, command));

  clock_.set(2000000);  // next cycle: one control period after the write
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);

  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));
  EXPECT_EQ(sent.id.value, force_control_can_id(kDriveId));
  EXPECT_EQ(sent.direction, FrameDirection::Tx);
}

TEST_F(Ak30RuntimeTest, ReadDecodesFeedbackIntoStates) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  EXPECT_TRUE(write_authorized(*runtime_, command));

  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime_->read(states, 1U));

  // Position sub-mode evidences position/velocity/effort; the fixture payload
  // decodes to 90.0 deg -> pi/2 rad (minus the provisional zero offset),
  // 10000 ERPM -> ~9.35 rad/s at the output shaft, 2.0 A -> ~1.48 N*m.
  ASSERT_EQ(transport_->inject_receive(feedback_frame(clock_)),
            TransportResult::Ok);
  mech_hardware_ros2_control::CanonicalState next[1] = {};
  clock_.set(6000000);
  EXPECT_TRUE(runtime_->read(next, 1U));
  EXPECT_NEAR(next[0].position,
              1.5707963267948966 - 5.760604931781636, 1e-6);
  EXPECT_NEAR(next[0].velocity, 9.3499, 1e-3);
  EXPECT_NEAR(next[0].effort, 1.4764, 1e-3);
}

// The core of this slice: the staged watchdog. Upstream stops refreshing,
// the frozen command is NOT re-submitted (so the session's stage clock
// advances on real time), and Expired faults within the 3-cycle budget.
TEST_F(Ak30RuntimeTest, WatchdogFreezesThenFaultsWithinThreeCycles) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.5};
  EXPECT_TRUE(write_authorized(*runtime_, command));

  // t=2 ms: first read submits the freshly written command (age 0).
  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame first{};
  ASSERT_TRUE(transport_->take_transmit(first));

  // The upstream goes silent: no further write. t=4 ms: age 2 ms < ttl 4 ms,
  // but the command is not fresh - the runtime must NOT re-submit it, or the
  // watchdog would never leave Following and a dead controller would be
  // masked forever. The device keeps executing the last accepted command
  // (that is what "freeze the last valid command" means on the wire).
  clock_.set(4000000);
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  // t=6.000001 ms: age 4,000,001 ns -> Holding. The runtime reports the
  // frozen state and still sends nothing.
  clock_.set(6000001);
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_TRUE(runtime_->holding());
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  // t=8.000001 ms: age 6,000,001 ns -> Expired. read() fails, routing
  // through CompositeSystem's fault path; no new frame is on the wire and
  // nothing ever resolved the stale command to 0.0.
  clock_.set(8000001);
  EXPECT_FALSE(runtime_->read(states, 1U));
  EXPECT_TRUE(runtime_->expired());
  EXPECT_EQ(transport_->pending_transmit(), 0U);
}

// A live controller refreshes every cycle: the watchdog stays in Following
// and the runtime keeps submitting. This is the 500 Hz steady state.
TEST_F(Ak30RuntimeTest, RefreshingControllerKeepsWatchdogFollowing) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  mech_hardware_ros2_control::CanonicalState states[1] = {};
  constexpr std::int64_t period = 2000000;
  for (int cycle = 1; cycle <= 10; ++cycle) {
    const mech_hardware_ros2_control::CanonicalCommand command{0.25};
    ASSERT_TRUE(write_authorized(*runtime_, command));
    clock_.set(cycle * period);
    ASSERT_TRUE(runtime_->read(states, 1U));
    ASSERT_EQ(transport_->pending_transmit(), 1U);
    RawCanFrame sent{};
    ASSERT_TRUE(transport_->take_transmit(sent));
    EXPECT_FALSE(runtime_->holding());
    EXPECT_FALSE(runtime_->expired());
  }
}

TEST_F(Ak30RuntimeTest, WouldBlockRetriesAndDoesNotFault) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  EXPECT_TRUE(write_authorized(*runtime_, command));

  transport_->force_next_send_results({TransportResult::WouldBlock});
  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime_->read(states, 1U));  // not a fault
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  // Next cycle retries and succeeds. The retry has to land strictly inside the
  // command's own deadline - written at t=0, so it expires at t=4 ms (two
  // control periods). Before the deadline was made immutable this read could
  // sit anywhere, because every retry minted a fresh two-period window.
  clock_.set(3000000);
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 1U);
}

// ADR-012's budget is the whole staged watchdog inside <=3 control cycles, so
// a command's validity window cannot be re-derived from the current time on
// every retry - that turns backpressure into an unbounded extension and lets a
// target that is milliseconds stale reach the motor. The window is minted once,
// when the controller's command is received.
TEST_F(Ak30RuntimeTest, BackpressureDoesNotExtendTheCommandDeadline) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  // Written at t=0: valid until t=4 ms.
  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, command));

  mech_hardware_ros2_control::CanonicalState states[1] = {};
  transport_->force_next_send_results(
      {TransportResult::WouldBlock, TransportResult::WouldBlock,
       TransportResult::WouldBlock});
  for (const std::int64_t at_nanoseconds : {1000000, 2000000, 3000000}) {
    clock_.set(at_nanoseconds);
    EXPECT_TRUE(runtime_->read(states, 1U)) << "at " << at_nanoseconds;
    EXPECT_EQ(transport_->pending_transmit(), 0U) << "at " << at_nanoseconds;
  }

  // The transport is healthy again, but the command is past its deadline. The
  // opportunity to send must not be taken: this frame would carry a target the
  // controller produced 5 ms ago.
  clock_.set(5000000);
  EXPECT_FALSE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 0U);
}

// The deadline binds the first submission too. "Nothing has been sent yet"
// is not a reason to keep an aged command alive - that is the loophole
// submitted_once_ would open if the expiry check sat behind it.
TEST_F(Ak30RuntimeTest, FirstSubmissionIsAlsoBoundByTheCommandDeadline) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, command));

  // No transport failure at all - simply nobody read until after the window.
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  clock_.set(4000000);
  EXPECT_FALSE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 0U);
}

// ADR-012's Holding stage means "do not RE-send the frozen command", not "do
// not send anything". The two are easy to conflate because the session reports
// the stage from the age of its last ACCEPTED command - a property of the past,
// not of the command currently in hand. Conflating them drops genuinely new
// commands: a controller that resumes after a gap, or a joint re-claimed by a
// different controller, has its first command silently swallowed for the width
// of the Holding window.
TEST_F(Ak30RuntimeTest, FreshCommandDuringHoldingIsStillSubmitted) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());
  mech_hardware_ros2_control::CanonicalState states[1] = {};

  const mech_hardware_ros2_control::CanonicalCommand first{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, first));
  clock_.set(2000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));

  // Nobody refreshes: at t=6.000001 ms the last accepted command is past the
  // 4 ms soft TTL, so the session reports Holding.
  clock_.set(6000001);
  ASSERT_TRUE(runtime_->read(states, 1U));
  ASSERT_TRUE(runtime_->holding());
  ASSERT_EQ(transport_->pending_transmit(), 0U);

  // A brand-new command arrives while that Holding window is still open. It is
  // not the frozen command being replayed - it is a fresh target, and it must
  // go out.
  clock_.set(6500000);
  const mech_hardware_ros2_control::CanonicalCommand second{0.30};
  ASSERT_TRUE(write_authorized(*runtime_, second));
  clock_.set(7000000);
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 1U);
}

// ADR-014: a Torque-mode runtime maps only the effort member into the
// device command, so the transmitted frame's payload is exactly the
// AKE60-8 golden for torque 2.0 N*m with every other field zero -
// recomputed from the L07 ranges with the ((1<<bits)-1) quantization,
// matching the protocol layer's own golden (test_ak30_force_wire.cpp).
// Garbage in the ignored position member must not leak into the frame.
TEST_F(Ak30RuntimeTest, TorqueSubModeEmitsEffortOnlyGoldenFrame) {
  Ak30RuntimeConfig config = runtime_config();
  config.sub_mode = ForceControlSubMode::Torque;
  Ak30ForceControlRuntime runtime(*transport_,
                                  [this]() { return clock_.now(); }, config);
  ASSERT_TRUE(runtime.configure(1U));
  ASSERT_TRUE(runtime.start());

  mech_hardware_ros2_control::CanonicalCommand command{};
  command.effort = 2.0;
  command.position = 9.0;  // ignored by the Torque sub-mode, must not leak
  EXPECT_TRUE(write_authorized(runtime, command));

  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime.read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));
  ASSERT_GE(sent.payload_size, 8U);
  // 00 00 00 7F FF 7F F9 10: kp=0 kd=0 pos=0 vel=0 torque=2.0
  const std::array<std::uint8_t, 8U> expected{
      0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF9, 0x10};
  for (std::size_t index = 0; index < 8U; ++index) {
    EXPECT_EQ(sent.payload[index], expected[index]) << "byte " << index;
  }
}

// ADR-014: a Velocity-mode runtime maps only the velocity member and
// FORCES effort to zero - the wire's effort field rides along as
// feedforward t_ff in every sub-mode, and the bench-proven Kd bound only
// holds without feedforward. Even a non-zero effort written into the
// command must not reach the frame. kd=1, v=0.3 rad/s recomputes to
// 00 03 33 7F FF 80 E7 FF (kp=0 kd=1 pos=0 vel=0.3 torque=0).
TEST_F(Ak30RuntimeTest, VelocitySubModeEmitsVelocityAndForcesZeroEffort) {
  Ak30RuntimeConfig config = runtime_config();
  config.sub_mode = ForceControlSubMode::Velocity;
  config.gains.kp = 0.0;
  config.gains.kd = 1.0;
  Ak30ForceControlRuntime runtime(*transport_,
                                  [this]() { return clock_.now(); }, config);
  ASSERT_TRUE(runtime.configure(1U));
  ASSERT_TRUE(runtime.start());

  mech_hardware_ros2_control::CanonicalCommand command{};
  command.velocity = 0.3;
  command.effort = 1.0;  // must be forced to zero, never feedforwarded
  EXPECT_TRUE(write_authorized(runtime, command));

  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime.read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));
  ASSERT_GE(sent.payload_size, 8U);
  const std::array<std::uint8_t, 8U> expected{
      0x00, 0x03, 0x33, 0x7F, 0xFF, 0x80, 0xE7, 0xFF};
  for (std::size_t index = 0; index < 8U; ++index) {
    EXPECT_EQ(sent.payload[index], expected[index]) << "byte " << index;
  }
}

// The staged watchdog is sub-mode-independent (ADR-014 Decision 5): the
// same freeze-then-fault timeline the Position test pins must hold for a
// Torque-mode device too - including never resolving the stale command
// to 0.0, which on the effort interface would be a silently-invented
// zero-torque command.
TEST_F(Ak30RuntimeTest, TorqueSubModeWatchdogFreezesThenFaults) {
  Ak30RuntimeConfig config = runtime_config();
  config.sub_mode = ForceControlSubMode::Torque;
  Ak30ForceControlRuntime runtime(*transport_,
                                  [this]() { return clock_.now(); }, config);
  ASSERT_TRUE(runtime.configure(1U));
  ASSERT_TRUE(runtime.start());

  mech_hardware_ros2_control::CanonicalCommand command{};
  command.effort = 0.1;
  EXPECT_TRUE(write_authorized(runtime, command));

  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime.read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame first{};
  ASSERT_TRUE(transport_->take_transmit(first));

  clock_.set(4000000);
  EXPECT_TRUE(runtime.read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  clock_.set(6000001);
  EXPECT_TRUE(runtime.read(states, 1U));
  EXPECT_TRUE(runtime.holding());
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  clock_.set(8000001);
  EXPECT_FALSE(runtime.read(states, 1U));
  EXPECT_TRUE(runtime.expired());
  EXPECT_EQ(transport_->pending_transmit(), 0U);
}

TEST_F(Ak30RuntimeTest, WriteRejectsNonFiniteCommand) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand bad{std::nan("")};
  EXPECT_FALSE(write_authorized(*runtime_, bad));
  const mech_hardware_ros2_control::CanonicalCommand inf{
      std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(write_authorized(*runtime_, inf));
}

// ADR-014: write() validates the member the sub-mode consumes. In Torque
// mode the effort member is the command and a non-finite velocity (which
// the Torque device ignores) must not fault the write - but a non-finite
// effort must. Position/Velocity modes mirror the same rule for their own
// consumed member.
TEST_F(Ak30RuntimeTest, WriteValidatesConsumedFieldPerSubMode) {
  {
    Ak30RuntimeConfig config = runtime_config();
    config.sub_mode = ForceControlSubMode::Torque;
    Ak30ForceControlRuntime runtime(*transport_,
                                    [this]() { return clock_.now(); }, config);
    ASSERT_TRUE(runtime.configure(1U));
    ASSERT_TRUE(runtime.start());

    // Non-finite effort (the consumed member) rejects.
    mech_hardware_ros2_control::CanonicalCommand bad{};
    bad.effort = std::nan("");
    EXPECT_FALSE(write_authorized(runtime, bad));
    bad.effort = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(write_authorized(runtime, bad));

    // A finite effort accepts even with garbage in the ignored members: a
    // Torque device never sees position/velocity (to_device_command drops
    // them), so the write must not fault on them.
    mech_hardware_ros2_control::CanonicalCommand odd{};
    odd.effort = 0.2;
    odd.position = std::numeric_limits<double>::infinity();
    EXPECT_TRUE(write_authorized(runtime, odd));
  }
  {
    Ak30RuntimeConfig config = runtime_config();
    config.sub_mode = ForceControlSubMode::Velocity;
    Ak30ForceControlRuntime runtime(*transport_,
                                    [this]() { return clock_.now(); }, config);
    ASSERT_TRUE(runtime.configure(1U));
    ASSERT_TRUE(runtime.start());

    mech_hardware_ros2_control::CanonicalCommand bad{};
    bad.velocity = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(write_authorized(runtime, bad));
    mech_hardware_ros2_control::CanonicalCommand ok{};
    ok.velocity = 0.3;
    ok.position = std::nan("");
    EXPECT_TRUE(write_authorized(runtime, ok));
  }
}

// ADR-016 Decision 1/2: feedback that aged past its window is a device that
// stopped talking - a fault, not a reason to publish zeros. The old version of
// this test asserted the zeros as correct behaviour, which froze the defect in
// place: on a position interface 0.0 is a real position, so "we do not know
// where the joint is" became indistinguishable from "the joint is at zero",
// and the bench's hold-current procedure would have seeded a hold at the zero
// position.
TEST_F(Ak30RuntimeTest, StaleFeedbackFailsClosedWithoutInventingZeros) {
  // Feedback staleness must be isolated from the command watchdog, so this
  // test shortens both the reporting period and the feedback TTL (1 ms / 2 ms)
  // to age a sample out quickly. ADR-016 Decision 4 only requires the TTL to
  // be no shorter than the period; motor1's real values are 20 ms / 60 ms.
  Ak30RuntimeConfig config = runtime_config();
  config.feedback_period_nanoseconds = 1000000;
  config.feedback_ttl_nanoseconds = 2000000;
  Ak30ForceControlRuntime runtime(*transport_,
                                  [this]() { return clock_.now(); }, config);
  ASSERT_TRUE(runtime.configure(1U));
  ASSERT_TRUE(runtime.start());

  // One Following cycle: write -> read submits at t=10 ms and the feedback
  // injected just before is processed at t=10 ms (fresh). The write is placed
  // at t=9 ms so the submission lands inside the command's own two-period
  // window; the window is minted at write time and no longer re-derived per
  // attempt.
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  clock_.set(9000000);
  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(runtime, command));
  ASSERT_EQ(transport_->inject_receive(feedback_frame(clock_)),
            TransportResult::Ok);
  clock_.set(10000000);
  ASSERT_TRUE(runtime.read(states, 1U));
  const double fresh_position = 1.5707963267948966 - 5.760604931781636;
  EXPECT_NEAR(states[0].position, fresh_position, 1e-6);
  EXPECT_TRUE(runtime.has_valid_sample());

  // Feedback goes silent. The controller keeps refreshing, so the command
  // watchdog stays Following, but the last feedback was processed at t=10 ms;
  // at t=12.000001 ms its age exceeds the feedback TTL and the snapshot is
  // Stale.
  const mech_hardware_ros2_control::CanonicalCommand refresh{0.25};
  ASSERT_TRUE(write_authorized(runtime, refresh));
  clock_.set(12000001);
  // Seeded with a recognisable value: a fail-closed read must leave the
  // caller's buffer untouched rather than overwrite it with zeros.
  mech_hardware_ros2_control::CanonicalState stale[1] = {};
  stale[0].position = -7.5;
  stale[0].velocity = -7.5;
  stale[0].effort = -7.5;
  EXPECT_FALSE(runtime.read(stale, 1U));
  EXPECT_FALSE(runtime.has_valid_sample());
  EXPECT_EQ(stale[0].position, -7.5);
  EXPECT_EQ(stale[0].velocity, -7.5);
  EXPECT_EQ(stale[0].effort, -7.5);
  // The quality evidence stays legible instead of being flattened into zeros.
  EXPECT_EQ(runtime.last_status().quality, SampleQuality::Stale);
  EXPECT_TRUE(runtime.last_status().host_rx_time.has_value());
}

// ADR-016 Decision 3: before any feedback has arrived the quality is Unknown.
// At 50 Hz reporting against a 500 Hz control loop that is roughly ten normal
// cycles after activation, so it must not fault - but the joint must also not
// be claimable, which is what stops a controller from ever acting on a
// position nobody has measured.
TEST_F(Ak30RuntimeTest, UnknownFeedbackDoesNotFaultButBlocksClaiming) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());
  EXPECT_FALSE(runtime_->has_valid_sample());

  mech_hardware_ros2_control::CanonicalState states[1] = {};
  states[0].position = -7.5;
  for (const std::int64_t at_nanoseconds : {2000000, 4000000, 6000000}) {
    clock_.set(at_nanoseconds);
    EXPECT_TRUE(runtime_->read(states, 1U)) << "at " << at_nanoseconds;
    EXPECT_FALSE(runtime_->has_valid_sample()) << "at " << at_nanoseconds;
    EXPECT_EQ(states[0].position, -7.5) << "at " << at_nanoseconds;
  }
  EXPECT_EQ(runtime_->last_status().quality, SampleQuality::Unknown);

  // The first real sample flips it, and only then is the joint claimable.
  clock_.set(8000000);
  ASSERT_EQ(transport_->inject_receive(feedback_frame(clock_)),
            TransportResult::Ok);
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_TRUE(runtime_->has_valid_sample());
  EXPECT_NEAR(states[0].position, 1.5707963267948966 - 5.760604931781636,
              1e-6);
}

// ADR-016 Decision 4: a feedback window shorter than the device's own
// reporting period can never be satisfied. motor1 reports at a configured
// 50 Hz (20 ms), so the 6 ms default this code shipped with was 3.3x too
// short - the snapshot would have been Stale for about 70% of control cycles,
// which only went unnoticed because Stale used to publish zeros silently.
TEST_F(Ak30RuntimeTest, RejectsFeedbackTtlShorterThanReportingPeriod) {
  Ak30RuntimeConfig config = runtime_config();
  config.feedback_period_nanoseconds = 20000000;  // 50 Hz, motor1's config
  config.feedback_ttl_nanoseconds = 6000000;      // the old default
  Ak30ForceControlRuntime runtime(*transport_,
                                  [this]() { return clock_.now(); }, config);
  EXPECT_FALSE(runtime.configure(1U));

  // Equal is the boundary and is accepted; the decision's 3x margin is a
  // deployment choice, not a hard requirement of the runtime.
  config.feedback_ttl_nanoseconds = 20000000;
  Ak30ForceControlRuntime boundary(*transport_,
                                   [this]() { return clock_.now(); }, config);
  EXPECT_TRUE(boundary.configure(1U));

  // A non-positive reporting period is not a description of any device.
  config.feedback_period_nanoseconds = 0;
  Ak30ForceControlRuntime no_period(*transport_,
                                    [this]() { return clock_.now(); }, config);
  EXPECT_FALSE(no_period.configure(1U));
}

TEST_F(Ak30RuntimeTest, RepeatedLifecycleHundredTimes) {
  for (int i = 0; i < 100; ++i) {
    FakeTransport transport{16U};
    TestClock clock{0};
    Ak30ForceControlRuntime runtime(transport, [&clock]() { return clock.now(); },
                                    runtime_config());
    ASSERT_TRUE(runtime.configure(1U));
    ASSERT_TRUE(runtime.start());
    const mech_hardware_ros2_control::CanonicalCommand command{0.25};
    ASSERT_TRUE(write_authorized(runtime, command));
    mech_hardware_ros2_control::CanonicalState states[1] = {};
    clock.set(2000000);
    ASSERT_TRUE(runtime.read(states, 1U));
    runtime.stop();
  }
}

// ADR-015 Decision 2: a dispatch without authorization is not a command. It
// must not become pending, so the next read() transmits nothing. In a
// single-joint deployment CompositeSystem skips the write() call entirely,
// so this pins the RuntimePort contract rather than a reachable production
// path - a multi-joint runtime would receive exactly this shape.
TEST_F(Ak30RuntimeTest, UnauthorizedDispatchNeverTransmits) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  EXPECT_TRUE(write_unauthorized(*runtime_, command));

  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 0U);
  EXPECT_FALSE(runtime_->expired());
}

// ADR-015 Decision 3: cancel_pending() drops the stored command immediately,
// so a released claim stops transmission on the very next cycle instead of
// riding out the hard TTL. The audit reproduced the opposite behaviour - 11
// frames over 22 ms after the claim was released.
TEST_F(Ak30RuntimeTest, CancelPendingStopsSubmissionBeforeAnyTransmit) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, command));
  runtime_->cancel_pending(0U);

  mech_hardware_ros2_control::CanonicalState states[1] = {};
  // Ten cycles inside and beyond the command's hard TTL: the hardware loop
  // keeps running, nothing goes out.
  for (int cycle = 1; cycle <= 10; ++cycle) {
    clock_.set(static_cast<std::int64_t>(cycle) * 2000000);
    EXPECT_TRUE(runtime_->read(states, 1U));
    EXPECT_EQ(transport_->pending_transmit(), 0U) << "cycle " << cycle;
  }
  // Never submitted, so the session's Expired still means "no command yet",
  // not the ADR-012 explicit failure.
  EXPECT_FALSE(runtime_->expired());
}

// Cancelling after a command was already submitted must not fault, and a new
// authorized write must be able to resume transmission - a re-claim is a
// normal, repeatable transition. The re-claim here stays inside the previous
// command's Following window; the Holding-window case has its own test
// (FreshCommandDuringHoldingIsStillSubmitted).
TEST_F(Ak30RuntimeTest, CancelPendingAfterSubmitAllowsLaterReclaim) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());
  mech_hardware_ros2_control::CanonicalState states[1] = {};

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, command));
  clock_.set(2000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));

  runtime_->cancel_pending(0U);
  clock_.set(4000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  // Re-claimed inside the Following window: a fresh authorized write
  // transmits again.
  ASSERT_TRUE(write_authorized(*runtime_, command));
  clock_.set(5000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 1U);
}

// Revoking authorization ends the command lease, so what follows is an
// UNCLAIMED joint - the same state the runtime is in right after start(), and
// one that nobody is commanding. That is not a fault.
//
// The distinction this pins down is the one ADR-015 and ADR-012 draw between
// two different silences. A controller that holds its claim and stops
// refreshing IS a fault: the device is still leased and its commanded state is
// going stale (WatchdogFreezesThenFaultsWithinThreeCycles). A controller that
// was deactivated is not: the claim is gone, transmission has already stopped,
// and the drive falls back on its own loss-of-control backstop. Letting the
// session's stage keep ageing the pre-revoke command would fault the hardware
// component a few cycles after any normal controller swap - which is exactly
// what deactivating one controller to activate another does, i.e. T8 and T9.
TEST_F(Ak30RuntimeTest, CancelPendingEndsTheLeaseSoAnUnclaimedJointDoesNotFault) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());
  mech_hardware_ros2_control::CanonicalState states[1] = {};

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, command));
  clock_.set(2000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  RawCanFrame sent{};
  ASSERT_TRUE(transport_->take_transmit(sent));

  runtime_->cancel_pending(0U);

  // Well past the 6 ms hard TTL measured from that submitted command. No
  // feedback is injected here on purpose: with no sample ever taken, ADR-016's
  // feedback path cannot fault either, so a failure can only come from the
  // command stage - which is what this test is about.
  for (std::int64_t at = 4000000; at <= 20000000; at += 2000000) {
    clock_.set(at);
    ASSERT_TRUE(runtime_->read(states, 1U)) << "read() failed at " << at << " ns";
    EXPECT_FALSE(runtime_->expired()) << "expired at " << at << " ns";
  }
  EXPECT_EQ(transport_->pending_transmit(), 0U);
}

// cancel_pending() is noexcept and index-checked: an out-of-range resource
// index is a caller bug, not a reason to corrupt state or transmit.
TEST_F(Ak30RuntimeTest, CancelPendingIgnoresOutOfRangeIndex) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, command));
  runtime_->cancel_pending(1U);  // only resource 0 exists

  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  ASSERT_TRUE(runtime_->read(states, 1U));
  // The out-of-range cancel touched nothing, so resource 0's command stands.
  EXPECT_EQ(transport_->pending_transmit(), 1U);
}

// The count that turns "no motor command was transmitted" into a repeatable
// assertion instead of an external strace session. T6 had to prove TX = 0 on
// the Jetson by tracing write() syscalls, because nothing in the code counted.
// That works, but it has to be rebuilt from scratch every bench run - and T7
// sends real commands while a motor is live, which is the wrong moment to be
// assembling a measurement tool.
//
// It counts ACCEPTED device commands: a cycle that was not authorized, and a
// cycle that was authorized but carried no refresh, are both "no command".
TEST_F(Ak30RuntimeTest, MotorCommandFramesCountsOnlyAcceptedTransmissions) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_EQ(runtime_->motor_command_frames(), 0U);

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_unauthorized(*runtime_, command));
  clock_.set(2000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(runtime_->motor_command_frames(), 0U) << "unclaimed (ADR-015)";

  ASSERT_TRUE(write_authorized_stale(*runtime_, command));
  clock_.set(4000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(runtime_->motor_command_frames(), 0U) << "unrefreshed (ADR-017)";

  ASSERT_TRUE(write_authorized(*runtime_, command));
  clock_.set(6000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  EXPECT_EQ(runtime_->motor_command_frames(), 1U);
}

// Backpressure must not inflate the count. The retry re-sends the SAME command,
// so one command that eventually goes out is one frame however many cycles the
// transport made it wait - otherwise the number would measure transport luck
// rather than how often the device was commanded.
TEST_F(Ak30RuntimeTest, MotorCommandFramesDoesNotCountAWouldBlockRetry) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());
  mech_hardware_ros2_control::CanonicalState states[1] = {};

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(write_authorized(*runtime_, command));
  transport_->force_next_send_results({TransportResult::WouldBlock});
  clock_.set(2000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(runtime_->motor_command_frames(), 0U)
      << "a command that never left the host is not a transmitted frame";

  clock_.set(3000000);
  ASSERT_TRUE(runtime_->read(states, 1U));
  ASSERT_EQ(transport_->pending_transmit(), 1U);
  EXPECT_EQ(runtime_->motor_command_frames(), 1U);
}

}  // namespace
}  // namespace mech::mech_bringup
