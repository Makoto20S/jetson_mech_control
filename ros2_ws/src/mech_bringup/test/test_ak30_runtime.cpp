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
  EXPECT_EQ(states[0].position, 0.0);
  EXPECT_EQ(states[0].velocity, 0.0);
  EXPECT_EQ(states[0].effort, 0.0);
}

TEST_F(Ak30RuntimeTest, FollowingSubmitsWrittenCommand) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  EXPECT_TRUE(runtime_->write(&command, 1U));

  clock_.set(4000000);  // next cycle
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
  EXPECT_TRUE(runtime_->write(&command, 1U));

  clock_.set(4000000);
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
  EXPECT_TRUE(runtime_->write(&command, 1U));

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
    ASSERT_TRUE(runtime_->write(&command, 1U));
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
  EXPECT_TRUE(runtime_->write(&command, 1U));

  transport_->force_next_send_results({TransportResult::WouldBlock});
  clock_.set(2000000);
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  EXPECT_TRUE(runtime_->read(states, 1U));  // not a fault
  EXPECT_EQ(transport_->pending_transmit(), 0U);

  // Next cycle retries and succeeds.
  clock_.set(4000000);
  EXPECT_TRUE(runtime_->read(states, 1U));
  EXPECT_EQ(transport_->pending_transmit(), 1U);
}

TEST_F(Ak30RuntimeTest, WriteRejectsNonFiniteCommand) {
  ASSERT_TRUE(runtime_->configure(1U));
  ASSERT_TRUE(runtime_->start());

  const mech_hardware_ros2_control::CanonicalCommand bad{std::nan("")};
  EXPECT_FALSE(runtime_->write(&bad, 1U));
  const mech_hardware_ros2_control::CanonicalCommand inf{
      std::numeric_limits<double>::infinity()};
  EXPECT_FALSE(runtime_->write(&inf, 1U));
}

TEST_F(Ak30RuntimeTest, StaleFeedbackYieldsZeroStatesWithoutFault) {
  // Feedback staleness must be isolated from the command watchdog: with the
  // default TTLs (command 4/6 ms, feedback 6 ms) a feedback frame always
  // ages out inside the command's hard window, so this test shortens
  // feedback_ttl to 2 ms - a legal configuration, the runtime only forwards
  // it to the session.
  Ak30RuntimeConfig config = runtime_config();
  config.feedback_ttl_nanoseconds = 2000000;
  Ak30ForceControlRuntime runtime(*transport_,
                                  [this]() { return clock_.now(); }, config);
  ASSERT_TRUE(runtime.configure(1U));
  ASSERT_TRUE(runtime.start());

  // One Following cycle: write -> read submits at t=10 ms and the feedback
  // injected just before is processed at t=10 ms (fresh).
  mech_hardware_ros2_control::CanonicalState states[1] = {};
  const mech_hardware_ros2_control::CanonicalCommand command{0.25};
  ASSERT_TRUE(runtime.write(&command, 1U));
  ASSERT_EQ(transport_->inject_receive(feedback_frame(clock_)),
            TransportResult::Ok);
  clock_.set(10000000);
  ASSERT_TRUE(runtime.read(states, 1U));
  EXPECT_NEAR(states[0].position, 1.5707963267948966 - 5.760604931781636,
              1e-6);  // fresh feedback is published

  // Feedback goes silent. The controller keeps refreshing, so the command
  // watchdog stays Following, but the last feedback was processed at
  // t=10 ms; at t=12.000001 ms its age exceeds feedback_ttl (2 ms) and the
  // snapshot is Stale: values zeroed, no fault, no watchdog escalation.
  const mech_hardware_ros2_control::CanonicalCommand refresh{0.25};
  ASSERT_TRUE(runtime.write(&refresh, 1U));
  clock_.set(12000001);
  mech_hardware_ros2_control::CanonicalState stale[1] = {};
  EXPECT_TRUE(runtime.read(stale, 1U));
  EXPECT_FALSE(runtime.holding());
  EXPECT_FALSE(runtime.expired());
  EXPECT_EQ(stale[0].position, 0.0);
  EXPECT_EQ(stale[0].velocity, 0.0);
  EXPECT_EQ(stale[0].effort, 0.0);
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
    ASSERT_TRUE(runtime.write(&command, 1U));
    mech_hardware_ros2_control::CanonicalState states[1] = {};
    clock.set(2000000);
    ASSERT_TRUE(runtime.read(states, 1U));
    runtime.stop();
  }
}

}  // namespace
}  // namespace mech::mech_bringup
