#include "mech_bringup/ak30_servo_runtime.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "mech_simulation/fake_transport.hpp"

namespace {
using namespace mech::mech_bringup;
using namespace mech::mech_control_core;
using namespace mech::mech_hardware_ros2_control;
using mech::mech_simulation::FakeTransport;
using mech::mech_protocol_cubemars::ServoPositionSessionConfig;

MonotonicTime at(std::int64_t ns) { return *MonotonicTime::from_nanoseconds(ns); }

ServoPositionSessionConfig joint(std::uint16_t id) {
  ServoPositionSessionConfig c;
  c.drive_id = id;
  c.logical_bus = 2;
  c.target_rad_to_deg = {2.0, 10.0, true};
  c.feedback_deg_to_rad = {0.5, -5.0, true};
  c.speed_erpm = 1230.0;
  c.acceleration_raw = 450.0;
  c.position_min_rad = -4.0;
  c.position_max_rad = 4.0;
  c.max_target_error_rad = 2.0;
  c.command_ttl_ns = 4000000;
  c.command_hard_ttl_ns = 6000000;
  c.feedback_ttl_ns = 6000000;
  return c;
}

Ak30ServoRuntimeConfig config() {
  Ak30ServoRuntimeConfig c;
  c.logical_bus = 2;
  c.physical_bus = "fake-servo";
  c.control_period_ns = 2000000;
  c.joints = {joint(0), joint(105)};
  c.joints[1].target_rad_to_deg = {-3.0, -1.0, true};
  c.joints[1].feedback_deg_to_rad = {-0.25, 2.5, true};
  c.joints[1].speed_erpm = 640.0;
  c.joints[1].acceleration_raw = 120.0;
  return c;
}

RawCanFrame feedback(int id, std::int64_t ns, std::uint8_t status = 0) {
  std::array<std::uint8_t, kMaxCanPayloadBytes> bytes{};
  bytes[1] = 100;
  bytes[7] = status;
  return *RawCanFrame::create(2, *CanId::create(0x2900U | static_cast<unsigned>(id),
      CanFrameFormat::Extended), CanFrameType::Classic, FrameDirection::Rx,
      8, bytes, at(ns));
}

CommandDispatch fresh(double position) {
  CommandDispatch result{};
  result.command.position = position;
  result.authorized = true;
  result.fresh = true;
  return result;
}

struct Fixture final {
  FakeTransport transport{100U};
  BusOwnershipRegistry ownership;
  std::int64_t now{0};
  Ak30ServoRuntime runtime{transport, [this] { return at(now); }, config(), ownership};
  std::array<CanonicalState, 2> states{};

  void start() {
    ASSERT_TRUE(runtime.configure(2));
    EXPECT_FALSE(transport.is_open());
    ASSERT_TRUE(runtime.start());
    EXPECT_TRUE(transport.is_open());
  }
  void sample(std::int64_t ns) {
    now = ns;
    ASSERT_EQ(transport.inject_receive(feedback(0, ns)), TransportResult::Ok);
    ASSERT_EQ(transport.inject_receive(feedback(105, ns)), TransportResult::Ok);
    ASSERT_TRUE(runtime.read(states.data(), states.size()));
  }
};

struct ClockAdvancingTransport final : Transport {
  FakeTransport fake{16U};
  std::int64_t& now;
  std::int64_t advance_to;
  bool advanced{false};

  ClockAdvancingTransport(std::int64_t& current, std::int64_t target)
      : now(current), advance_to(target) {}
  TransportKind kind() const noexcept override { return fake.kind(); }
  const TransportCapabilities& capabilities() const noexcept override {
    return fake.capabilities();
  }
  bool is_open() const noexcept override { return fake.is_open(); }
  bool open() noexcept override { return fake.open(); }
  void close() noexcept override { fake.close(); }
  TransportResult try_receive(RawCanFrame& frame) noexcept override {
    return fake.try_receive(frame);
  }
  TransportResult try_send(const RawCanFrame& frame) noexcept override {
    const auto result = fake.try_send(frame);
    if (result == TransportResult::Ok && !advanced) {
      now = advance_to;
      advanced = true;
    }
    return result;
  }
  TransportStats stats() const noexcept override { return fake.stats(); }
};

TEST(Ak30ServoRuntime, DualMappingsAndNoClaimSilence) {
  Fixture f;
  f.start();
  f.states[0].position = 7.0;
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_FALSE(f.runtime.has_valid_sample());
  EXPECT_EQ(f.states[0].position, 7.0);
  f.sample(100);
  EXPECT_TRUE(f.runtime.has_valid_sample());
  ASSERT_TRUE(f.runtime.diagnostic_snapshot(0).has_value());
  EXPECT_EQ(f.runtime.diagnostic_snapshot(0)->sequence, 1U);
  EXPECT_FALSE(f.runtime.diagnostic_snapshot(2).has_value());
  EXPECT_EQ(f.states[0].position, 0.0);
  EXPECT_EQ(f.states[1].position, 0.0);
  EXPECT_TRUE(std::isnan(f.states[0].velocity));
  CommandDispatch silent[2]{};
  ASSERT_TRUE(f.runtime.write(silent, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
  CommandDispatch commands[2]{fresh(1.0), fresh(1.0)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x600U);
  EXPECT_EQ((std::array<std::uint8_t, 8>{sent.payload[0], sent.payload[1],
      sent.payload[2], sent.payload[3], sent.payload[4], sent.payload[5],
      sent.payload[6], sent.payload[7]}),
      (std::array<std::uint8_t, 8>{0, 1, 0xD4, 0xC0, 0, 123, 0, 45}));
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x669U);
  EXPECT_EQ((std::array<std::uint8_t, 8>{sent.payload[0], sent.payload[1],
      sent.payload[2], sent.payload[3], sent.payload[4], sent.payload[5],
      sent.payload[6], sent.payload[7]}),
      (std::array<std::uint8_t, 8>{0xFF, 0xFF, 0x63, 0xC0, 0, 64, 0, 12}));
  EXPECT_FALSE(f.transport.take_transmit(sent));
  EXPECT_EQ(f.runtime.bus_stats().tx_frames, 2U);
}

TEST(Ak30ServoRuntime, RevocationOnlyCancelsOneRoute) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.runtime.cancel_pending(0);
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  RawCanFrame sent{};
  ASSERT_TRUE(f.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x669U);
  EXPECT_FALSE(f.transport.take_transmit(sent));
}

TEST(Ak30ServoRuntime, InvalidThenNormalBatchLatchesGroupAndCancelsBoth) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.now = 200;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, 150, 0x42)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(0, 160)), TransportResult::Ok);
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
  EXPECT_FALSE(f.runtime.has_valid_sample());
}

TEST(Ak30ServoRuntime, Frame65MustBeValidatedBeforeAnySend) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.now = 200;
  for (int index = 0; index < 64; ++index)
    ASSERT_EQ(f.transport.inject_receive(feedback(0, 101 + index)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, 166, 0x42)), TransportResult::Ok);
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
}

TEST(Ak30ServoRuntime, FinalClockRecheckRejectsNewlyStaleGroupMember) {
  FakeTransport transport;
  BusOwnershipRegistry ownership;
  std::int64_t now = 0;
  int calls = 0;
  bool advance_on_send = false;
  Ak30ServoRuntime runtime(transport, [&] {
    ++calls;
    return at(now + (advance_on_send && calls == 3 ? 1000100 : 0));
  }, config(), ownership);
  ASSERT_TRUE(runtime.configure(2));
  ASSERT_TRUE(runtime.start());
  now = 100;
  ASSERT_EQ(transport.inject_receive(feedback(0, now)), TransportResult::Ok);
  ASSERT_EQ(transport.inject_receive(feedback(105, now)), TransportResult::Ok);
  CanonicalState states[2]{};
  ASSERT_TRUE(runtime.read(states, 2));
  now = 5000000;
  ASSERT_EQ(transport.inject_receive(feedback(0, now)), TransportResult::Ok);
  ASSERT_TRUE(runtime.read(states, 2));
  CommandDispatch commands[2]{fresh(1), {}};
  ASSERT_TRUE(runtime.write(commands, 2));
  calls = 0;
  advance_on_send = true;
  EXPECT_FALSE(runtime.read(states, 2));
  EXPECT_EQ(transport.pending_transmit(), 0U);
}

TEST(Ak30ServoRuntime, EachSendRechecksDeadlineAndPeerFeedbackAfterFirstSend) {
  for (const bool peer_feedback_expires : {false, true}) {
    std::int64_t now = 0;
    const std::int64_t advance_to = peer_feedback_expires ? 3000100 : 4000100;
    ClockAdvancingTransport transport(now, advance_to);
    BusOwnershipRegistry ownership;
    auto settings = config();
    if (peer_feedback_expires) settings.joints[1].feedback_ttl_ns = 3000000;
    Ak30ServoRuntime runtime(transport, [&] { return at(now); }, settings,
                             ownership);
    ASSERT_TRUE(runtime.configure(2));
    ASSERT_TRUE(runtime.start());
    now = 100;
    ASSERT_EQ(transport.fake.inject_receive(feedback(0, now)), TransportResult::Ok);
    ASSERT_EQ(transport.fake.inject_receive(feedback(105, now)), TransportResult::Ok);
    CanonicalState states[2]{};
    ASSERT_TRUE(runtime.read(states, 2));
    CommandDispatch commands[2]{fresh(1), fresh(1)};
    ASSERT_TRUE(runtime.write(commands, 2));
    EXPECT_FALSE(runtime.read(states, 2));
    EXPECT_EQ(transport.fake.pending_transmit(), 1U);
    RawCanFrame sent{};
    ASSERT_TRUE(transport.fake.take_transmit(sent));
    EXPECT_TRUE(sent.id.value == 0x600U || sent.id.value == 0x669U);
  }
}

TEST(Ak30ServoRuntime, SentPeerMayEnterHoldingDuringAnotherRoutesSendBatch) {
  std::int64_t now = 0;
  ClockAdvancingTransport transport(now, 4000100);
  transport.advanced = true;  // Arm the clock jump for the later send batch.
  BusOwnershipRegistry ownership;
  auto settings = config();
  settings.joints.push_back(joint(106));
  Ak30ServoRuntime runtime(transport, [&] { return at(now); }, settings,
                           ownership);
  ASSERT_TRUE(runtime.configure(3));
  ASSERT_TRUE(runtime.start());
  now = 100;
  for (const int id : {0, 105, 106})
    ASSERT_EQ(transport.fake.inject_receive(feedback(id, now)), TransportResult::Ok);
  CanonicalState states[3]{};
  ASSERT_TRUE(runtime.read(states, 3));
  CommandDispatch commands[3]{fresh(1), {}, {}};
  ASSERT_TRUE(runtime.write(commands, 3));
  ASSERT_TRUE(runtime.read(states, 3));
  EXPECT_EQ(transport.fake.pending_transmit(), 1U);
  now = 3000100;
  for (const int id : {0, 105, 106})
    ASSERT_EQ(transport.fake.inject_receive(feedback(id, now)), TransportResult::Ok);
  ASSERT_TRUE(runtime.read(states, 3));
  commands[0].fresh = false;
  commands[1] = fresh(1);
  commands[2] = fresh(1);
  ASSERT_TRUE(runtime.write(commands, 3));
  transport.advanced = false;
  EXPECT_TRUE(runtime.read(states, 3));
  EXPECT_EQ(transport.fake.pending_transmit(), 3U);
}

TEST(Ak30ServoRuntime, ReceiveUsesClockAfterTransportStampsArrival) {
  FakeTransport transport;
  BusOwnershipRegistry ownership;
  bool receiving = false;
  int calls = 0;
  Ak30ServoRuntime runtime(transport, [&] {
    return at(!receiving ? 0 : calls++ == 0 ? 100 : 102);
  }, config(), ownership);
  ASSERT_TRUE(runtime.configure(2));
  ASSERT_TRUE(runtime.start());
  ASSERT_EQ(transport.inject_receive(feedback(0, 101)), TransportResult::Ok);
  ASSERT_EQ(transport.inject_receive(feedback(105, 101)), TransportResult::Ok);
  receiving = true;
  CanonicalState states[2]{};
  ASSERT_TRUE(runtime.read(states, 2));
  EXPECT_TRUE(runtime.has_valid_sample());
  EXPECT_EQ(states[0].position, 0.0);
  EXPECT_EQ(states[1].position, 0.0);
}

TEST(Ak30ServoRuntime, BackpressureAndSentCommandCannotOutliveOriginalDeadline) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.transport.force_next_send_results({TransportResult::WouldBlock, TransportResult::WouldBlock});
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
  f.transport.force_next_send_results({TransportResult::WouldBlock,
                                       TransportResult::WouldBlock});
  f.now = 4000099;
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 4000100;
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);

  f.runtime.stop();
  ASSERT_TRUE(f.runtime.start());
  f.sample(7000000);
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
  f.now = 10999999;
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 11000000;
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
  f.now = 12000000;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, f.now)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, f.now)), TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 13000000;
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
}

TEST(Ak30ServoRuntime, HoldingAcceptsFreshGenerationAndCancellationErasesWatchdog) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
  f.now = 4000100;
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 5000100;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, f.now)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, f.now)), TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  commands[0] = fresh(1.5);
  commands[1] = fresh(1.5);
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 4U);
  f.now = 6000100;
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.runtime.cancel_pending(0);
  f.runtime.cancel_pending(1);
  f.now = 10000100;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, f.now)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, f.now)), TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 11000100;
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 4U);
}

TEST(Ak30ServoRuntime, ReceiveBudgetDoesNotPostponeHardWatchdog) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 4000100;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, f.now)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, f.now)), TransportResult::Ok);
  for (int i = 0; i < 62; ++i)
    ASSERT_EQ(f.transport.inject_receive(feedback(106, f.now)), TransportResult::Ok);
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
  f.now = 6000100;
  for (int i = 0; i < 64; ++i)
    ASSERT_EQ(f.transport.inject_receive(feedback(106, f.now)), TransportResult::Ok);
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
}

TEST(Ak30ServoRuntime, OneAcceptedPeerCannotCarryUnsentPeerIntoHolding) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.transport.force_next_send_results({TransportResult::WouldBlock,
                                       TransportResult::Ok});
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 1U);
  f.now = 4000100;
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 1U);
}

TEST(Ak30ServoRuntime, FreshWriteAtAcceptedHardDeadlineCannotEraseExpiry) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 5000100;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, f.now)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, f.now)), TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 6000100;
  commands[0] = fresh(1.5);
  commands[1] = fresh(1.5);
  EXPECT_FALSE(f.runtime.write(commands, 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
}

TEST(Ak30ServoRuntime, FreshWriteAtUnsentSoftDeadlineCannotEraseExpiry) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.transport.force_next_send_results({TransportResult::WouldBlock,
                                       TransportResult::WouldBlock});
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 4000100;
  commands[0] = fresh(1.5);
  commands[1] = fresh(1.5);
  EXPECT_FALSE(f.runtime.write(commands, 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
}

TEST(Ak30ServoRuntime, ReleaseAtOldHardDeadlinePreservesOtherFreshRoute) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), {}};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 5000100;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, f.now)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, f.now)), TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 6000100;
  commands[0] = {};
  commands[1] = fresh(1.5);
  ASSERT_TRUE(f.runtime.write(commands, 2));
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 2U);
}

TEST(Ak30ServoRuntime, ReleasedMemberDoesNotFaultGroupAtOldHardDeadline) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.runtime.cancel_pending(0);
  f.now = 5000100;
  ASSERT_EQ(f.transport.inject_receive(feedback(0, f.now)), TransportResult::Ok);
  ASSERT_EQ(f.transport.inject_receive(feedback(105, f.now)), TransportResult::Ok);
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  commands[0] = {};
  commands[1] = fresh(1.5);
  ASSERT_TRUE(f.runtime.write(commands, 2));
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  f.now = 6000100;
  EXPECT_TRUE(f.runtime.read(f.states.data(), 2));
}

TEST(Ak30ServoRuntime, RestartClearsPendingAndFeedback) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.runtime.stop();
  EXPECT_FALSE(f.transport.is_open());
  ASSERT_TRUE(f.runtime.start());
  EXPECT_FALSE(f.runtime.has_valid_sample());
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
}

TEST(Ak30ServoRuntime, CleanupAllowsConfigureAndActivateAgain) {
  Fixture f;
  f.start();
  f.sample(100);
  f.runtime.stop();
  EXPECT_TRUE(f.runtime.configure(2));
  EXPECT_FALSE(f.transport.is_open());
  ASSERT_TRUE(f.runtime.start());
  EXPECT_FALSE(f.runtime.has_valid_sample());
  ASSERT_TRUE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
}

TEST(Ak30ServoRuntime, StalePeerFaultsGroupEvenWithoutPendingCommands) {
  Fixture f;
  f.start();
  f.sample(100);
  f.now = 6000100;
  EXPECT_FALSE(f.runtime.has_valid_sample());
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_FALSE(f.runtime.has_valid_sample());
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
}

TEST(Ak30ServoRuntime, RejectsConfigurationAndSharedOwnership) {
  FakeTransport t1, t2;
  BusOwnershipRegistry ownership;
  auto invalid = config();
  invalid.joints[1].drive_id = invalid.joints[0].drive_id;
  Ak30ServoRuntime bad(t1, [] { return at(0); }, invalid, ownership);
  EXPECT_FALSE(bad.configure(2));
  EXPECT_FALSE(t1.is_open());
  auto good = config();
  Ak30ServoRuntime first(t1, [] { return at(0); }, good, ownership);
  Ak30ServoRuntime second(t2, [] { return at(0); }, good, ownership);
  ASSERT_TRUE(first.configure(2));
  ASSERT_TRUE(second.configure(2));
  ASSERT_TRUE(first.start());
  EXPECT_FALSE(second.start());
  first.stop();
  EXPECT_TRUE(second.start());
  second.stop();
  good.joints[0].command_ttl_ns = 6000001;
  Ak30ServoRuntime ttl(t1, [] { return at(0); }, good, ownership);
  EXPECT_FALSE(ttl.configure(2));
  good = config();
  good.joints[0].command_ttl_ns = good.joints[0].command_hard_ttl_ns;
  Ak30ServoRuntime equal_ttl(t1, [] { return at(0); }, good, ownership);
  EXPECT_FALSE(equal_ttl.configure(2));
  good = config();
  good.control_period_ns = 1000000;
  good.joints[0].command_hard_ttl_ns = 4000000;
  Ak30ServoRuntime cycles(t1, [] { return at(0); }, good, ownership);
  EXPECT_FALSE(cycles.configure(2));
  auto unsupported = FakeTransport::default_capabilities();
  unsupported.supports_extended_frames = false;
  FakeTransport standard_only(16U, unsupported);
  Ak30ServoRuntime incompatible(standard_only, [] { return at(0); },
                                config(), ownership);
  EXPECT_FALSE(incompatible.configure(2));
  EXPECT_FALSE(standard_only.is_open());
}

TEST(Ak30ServoRuntime, ReceiveFailureCancelsGroup) {
  Fixture f;
  f.start();
  f.sample(100);
  CommandDispatch commands[2]{fresh(1), fresh(1)};
  ASSERT_TRUE(f.runtime.write(commands, 2));
  f.transport.force_next_receive_results({TransportResult::Disconnected});
  EXPECT_FALSE(f.runtime.read(f.states.data(), 2));
  EXPECT_EQ(f.transport.pending_transmit(), 0U);
}
}  // namespace
