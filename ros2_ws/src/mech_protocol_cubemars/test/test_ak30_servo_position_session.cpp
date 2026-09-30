#include "mech_protocol_cubemars/ak30_servo_position_session.hpp"
#include "mech_control_core/runtime.hpp"
#include "mech_simulation/fake_transport.hpp"

#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace {
using namespace mech::mech_control_core;
using namespace mech::mech_protocol_cubemars;

MonotonicTime at(std::int64_t ns) { return *MonotonicTime::from_nanoseconds(ns); }

ServoPositionSessionConfig synthetic_config() {
  ServoPositionSessionConfig c;
  c.drive_id = 104;
  c.logical_bus = 2;
  c.target_rad_to_deg = {2.0, 10.0, true};
  c.feedback_deg_to_rad = {0.5, -5.0, true};
  c.speed_erpm = 1230;
  c.acceleration_raw = 450;
  c.position_min_rad = -4;
  c.position_max_rad = 4;
  c.max_target_error_rad = 2;
  c.command_ttl_ns = 4000000;
  c.command_hard_ttl_ns = 6000000;
  c.feedback_ttl_ns = 6000000;
  return c;
}

RawCanFrame feedback(std::uint16_t bus, int id, std::int64_t ns,
                     std::uint8_t status = 0, std::uint16_t position_tenths = 100) {
  std::array<std::uint8_t, kMaxCanPayloadBytes> p{};
  p[0] = static_cast<std::uint8_t>(position_tenths >> 8);
  p[1] = static_cast<std::uint8_t>(position_tenths);
  p[7] = status;
  return *RawCanFrame::create(bus, *CanId::create(0x2900U | static_cast<unsigned>(id), CanFrameFormat::Extended),
      CanFrameType::Classic, FrameDirection::Rx, 8, p, at(ns));
}

void expect_same_frame(const RawCanFrame& actual, const RawCanFrame& expected) {
  EXPECT_EQ(actual.logical_bus, expected.logical_bus);
  EXPECT_EQ(actual.id.value, expected.id.value);
  EXPECT_EQ(actual.id.format, expected.id.format);
  EXPECT_EQ(actual.type, expected.type);
  EXPECT_EQ(actual.direction, expected.direction);
  EXPECT_EQ(actual.payload_size, expected.payload_size);
  EXPECT_EQ(actual.payload, expected.payload);
  EXPECT_EQ(actual.host_arrival.nanoseconds(), expected.host_arrival.nanoseconds());
  EXPECT_EQ(actual.source_timestamp.has_value(), expected.source_timestamp.has_value());
  if (actual.source_timestamp && expected.source_timestamp) {
    EXPECT_EQ(actual.source_timestamp->domain, expected.source_timestamp->domain);
    EXPECT_EQ(actual.source_timestamp->ticks, expected.source_timestamp->ticks);
  }
  EXPECT_EQ(actual.error_frame, expected.error_frame);
  EXPECT_EQ(actual.bitrate_switch, expected.bitrate_switch);
  EXPECT_EQ(actual.remote_request, expected.remote_request);
}

CanonicalDeviceCommand command(double position, std::uint64_t generation,
                               std::int64_t deadline) {
  return {position, 0.0, 0.0, generation, at(deadline)};
}

TEST(Ak30ServoPositionSession, SyntheticEvidenceAndExactPositionFrame) {
  Ak30ServoPositionSession session;
  auto c = synthetic_config();
  EXPECT_EQ(session.configure(c), AdapterResult::Ok);
  EXPECT_EQ(session.activate(at(100)), AdapterResult::Ok);
  EXPECT_EQ(session.accept_feedback(feedback(2,104,101), at(102)), AdapterResult::Ok);
  RawCanFrame out{};
  EXPECT_EQ(session.prepare_position(command(1,1,1000102), at(102), out), AdapterResult::Ok);
  EXPECT_EQ(out.id.value, 0x668U);
  EXPECT_EQ(out.logical_bus, 2U);
  EXPECT_EQ((std::array<std::uint8_t,8>{out.payload[0],out.payload[1],out.payload[2],out.payload[3],out.payload[4],out.payload[5],out.payload[6],out.payload[7]}),
            (std::array<std::uint8_t,8>{0,1,0xD4,0xC0,0,123,0,45}));
  EXPECT_EQ(session.snapshot(at(102)).position_rad, 0.0);
}

TEST(Ak30ServoPositionSession, SyntheticConfigRejectsMissingEvidenceAndZeroScale) {
  Ak30ServoPositionSession session;
  auto c = synthetic_config();
  c.target_rad_to_deg.evidence_declared = false;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
  c.target_rad_to_deg.evidence_declared = true;
  c.feedback_deg_to_rad.scale = 0;
  EXPECT_EQ(session.configure(c), AdapterResult::InvalidConfiguration);
}

TEST(Ak30ServoPositionSession, ConfigurationFailuresPreservePreviousValidSetup) {
  Ak30ServoPositionSession s;
  auto c = synthetic_config();
  ASSERT_EQ(s.configure(c), AdapterResult::Ok);
  auto bad = c;
  bad.drive_id = 256;
  EXPECT_EQ(s.configure(bad), AdapterResult::InvalidConfiguration);
  bad = c; bad.feedback_deg_to_rad.scale = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(s.configure(bad), AdapterResult::InvalidConfiguration);
  bad = c; bad.command_hard_ttl_ns = 6000001;
  EXPECT_EQ(s.configure(bad), AdapterResult::InvalidConfiguration);
  bad = c; bad.command_hard_ttl_ns = bad.command_ttl_ns;
  EXPECT_EQ(s.configure(bad), AdapterResult::InvalidConfiguration);
  bad = c; bad.speed_erpm = 9.99;
  EXPECT_EQ(s.configure(bad), AdapterResult::InvalidConfiguration);
  ASSERT_EQ(s.activate(at(10)), AdapterResult::Ok);
  EXPECT_EQ(s.configure(c), AdapterResult::InvalidConfiguration);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,11), at(11)), AdapterResult::Ok);
  RawCanFrame out{};
  EXPECT_EQ(s.prepare_position(command(1,1,1011), at(11), out), AdapterResult::Ok);
  EXPECT_EQ(out.id.value, 0x668U);
}

TEST(Ak30ServoPositionSession, FeedbackEpochFreshnessAndFaultLatch) {
  Ak30ServoPositionSession s;
  ASSERT_EQ(s.configure(synthetic_config()), AdapterResult::Ok);
  ASSERT_EQ(s.activate(at(100)), AdapterResult::Ok);
  EXPECT_EQ(s.activate(at(200)), AdapterResult::InvalidConfiguration);
  EXPECT_EQ(s.accept_feedback(feedback(2,104,99), at(100)), AdapterResult::Stale);
  EXPECT_EQ(s.accept_feedback(feedback(2,104,100), at(100)), AdapterResult::Ok);
  EXPECT_EQ(s.accept_feedback(feedback(2,105,101), at(101)), AdapterResult::InvalidCommand);
  EXPECT_EQ(s.accept_feedback(feedback(3,104,101), at(101)), AdapterResult::InvalidCommand);
  EXPECT_EQ(s.snapshot(at(100)).sequence, 1U);
  EXPECT_EQ(s.accept_feedback(feedback(2,104,99,1), at(101)), AdapterResult::Fault);
  EXPECT_EQ(s.accept_feedback(feedback(2,104,101), at(101)), AdapterResult::Fault);
  EXPECT_TRUE(s.fault_latched());
  EXPECT_EQ(s.snapshot(at(101)).availability, ServoPositionAvailability::Fault);
  RawCanFrame out{};
  EXPECT_EQ(s.prepare_position(command(1,1,1001), at(101), out), AdapterResult::Fault);
  s.deactivate();
  ASSERT_EQ(s.activate(at(200)), AdapterResult::Ok);
  EXPECT_EQ(s.snapshot(at(200)).availability, ServoPositionAvailability::Unknown);
  EXPECT_EQ(s.accept_feedback(feedback(2,104,201), at(201)), AdapterResult::Ok);
  EXPECT_EQ(s.snapshot(at(201)).sequence, 1U);
}

TEST(Ak30ServoPositionSession, InvalidOrUnavailableFeedbackRevokesAuthorization) {
  for (std::uint8_t status : {std::uint8_t{0x77}, std::uint8_t{0x42}}) {
    Ak30ServoPositionSession s;
    ASSERT_EQ(s.configure(synthetic_config()), AdapterResult::Ok);
    ASSERT_EQ(s.activate(at(0)), AdapterResult::Ok);
    ASSERT_EQ(s.accept_feedback(feedback(2,104,1), at(1)), AdapterResult::Ok);
    EXPECT_EQ(s.accept_feedback(feedback(2,104,2,status), at(2)), AdapterResult::InvalidCommand);
    RawCanFrame out{};
    EXPECT_EQ(s.prepare_position(command(1,1,1002), at(2), out), AdapterResult::Stale);
  }
  Ak30ServoPositionSession s;
  ASSERT_EQ(s.configure(synthetic_config()), AdapterResult::Ok);
  ASSERT_EQ(s.activate(at(0)), AdapterResult::Ok);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,1), at(1)), AdapterResult::Ok);
  auto malformed = feedback(2,104,2);
  malformed.payload_size = 7;
  EXPECT_EQ(s.accept_feedback(malformed, at(2)), AdapterResult::InvalidCommand);
  RawCanFrame out{};
  EXPECT_EQ(s.prepare_position(command(1,1,1002), at(2), out), AdapterResult::Stale);
  EXPECT_EQ(s.accept_feedback(feedback(2,104,3), at(3)), AdapterResult::Ok);
  EXPECT_EQ(s.snapshot(at(6000003)).availability, ServoPositionAvailability::Stale);
  EXPECT_EQ(s.prepare_position(command(1,1,6001003), at(6000003), out), AdapterResult::Stale);
}

TEST(Ak30ServoPositionSession, OlderNormalCannotOvertakeNewerUnknownStatus) {
  Ak30ServoPositionSession s;
  ASSERT_EQ(s.configure(synthetic_config()), AdapterResult::Ok);
  ASSERT_EQ(s.activate(at(0)), AdapterResult::Ok);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,10), at(10)), AdapterResult::Ok);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,30,0x42), at(30)), AdapterResult::InvalidCommand);
  EXPECT_EQ(s.accept_feedback(feedback(2,104,20), at(30)), AdapterResult::Stale);
  EXPECT_NE(s.snapshot(at(30)).availability, ServoPositionAvailability::Fresh);
  EXPECT_EQ(s.snapshot(at(30)).sequence, 1U);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,31), at(31)), AdapterResult::Ok);
  EXPECT_EQ(s.snapshot(at(31)).sequence, 2U);
}

TEST(Ak30ServoPositionSession, RejectedCommandsLeaveOutputAndGenerationUntouched) {
  Ak30ServoPositionSession s;
  auto c = synthetic_config();
  c.target_rad_to_deg = {-2, 10, true};
  ASSERT_EQ(s.configure(c), AdapterResult::Ok);
  ASSERT_EQ(s.activate(at(0)), AdapterResult::Ok);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,1), at(1)), AdapterResult::Ok);
  RawCanFrame out = feedback(2,104,1);
  const auto sentinel = out;
  auto bad = command(1,1,1001);
  bad.velocity = 1;
  EXPECT_EQ(s.prepare_position(bad, at(1), out), AdapterResult::InvalidCommand);
  bad = command(std::numeric_limits<double>::infinity(),1,1001);
  EXPECT_EQ(s.prepare_position(bad, at(1), out), AdapterResult::InvalidCommand);
  bad = command(1,1,1);
  EXPECT_EQ(s.prepare_position(bad, at(1), out), AdapterResult::InvalidCommand);
  bad = command(1,1,6000002);
  EXPECT_EQ(s.prepare_position(bad, at(1), out), AdapterResult::InvalidCommand);
  expect_same_frame(out, sentinel);
  EXPECT_EQ(s.prepare_position(command(1,1,1001), at(1), out), AdapterResult::Ok);
  EXPECT_EQ(out.id.value, 0x668U);
  EXPECT_EQ((std::array<std::uint8_t,4>{out.payload[0],out.payload[1],out.payload[2],out.payload[3]}),
            (std::array<std::uint8_t,4>{0,1,0x38,0x80})); // 8 degrees
  EXPECT_EQ(s.prepare_position(command(1,1,1001), at(1), out), AdapterResult::InvalidCommand);
}

TEST(Ak30ServoPositionSession, NegativeScaleQuantizationCannotCrossAbsoluteBound) {
  Ak30ServoPositionSession s;
  auto c = synthetic_config();
  c.target_rad_to_deg = {-1, -0.00009, true};
  c.position_min_rad = 0;
  c.position_max_rad = 1;
  ASSERT_EQ(s.configure(c), AdapterResult::Ok);
  ASSERT_EQ(s.activate(at(0)), AdapterResult::Ok);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,1), at(1)), AdapterResult::Ok);
  RawCanFrame out = feedback(2,104,1);
  const auto before = out;
  EXPECT_EQ(s.prepare_position(command(0,1,1001), at(1), out), AdapterResult::InvalidCommand);
  expect_same_frame(out, before);
  EXPECT_EQ(s.prepare_position(command(0.5,1,1001), at(1), out), AdapterResult::Ok);
  EXPECT_EQ(out.id.value, 0x668U);
}

struct TwoDeviceHarness final {
  mech::mech_simulation::FakeTransport transport{};
  FrameRouter router{};
  BusOwnershipRegistry ownership{};
  BusRuntime runtime{2, "fake0", transport, router, ownership};
  Ak30ServoPositionSession s104{};
  Ak30ServoPositionSession s105{};
  bool claim104{true};
  bool claim105{true};
  MonotonicTime cycle_now{};
  std::optional<CommandLease> pending104;
  std::optional<CommandLease> pending105;
  bool bound104{false};
  bool bound105{false};

  explicit TwoDeviceHarness(std::int64_t feedback_ttl_ns = 6000000) {
    const auto& capabilities = transport.capabilities();
    EXPECT_TRUE(capabilities.supports_classic_can);
    EXPECT_TRUE(capabilities.supports_extended_frames);
    EXPECT_GE(capabilities.max_payload_bytes, 8U);
    auto c104 = synthetic_config();
    auto c105 = synthetic_config();
    c105.drive_id = 105;
    c105.target_rad_to_deg = {-3, -1, true};
    c105.feedback_deg_to_rad = {-0.25, 2.5, true};
    c105.speed_erpm = 640;
    c105.acceleration_raw = 120;
    c104.feedback_ttl_ns = feedback_ttl_ns;
    c105.feedback_ttl_ns = feedback_ttl_ns;
    EXPECT_EQ(s104.configure(c104), AdapterResult::Ok);
    EXPECT_EQ(s105.configure(c105), AdapterResult::Ok);
    for (std::uint16_t id : {104,105}) {
      FrameFilter filter{};
      filter.format = CanFrameFormat::Extended;
      filter.value = 0x2900U | id;
      filter.mask = 0x1FFFFFFFU;
      // Route device identity first. Shape validation belongs to the session,
      // including matching CAN-FD traffic that must revoke a pending target.
      filter.frame_type.reset();
      EXPECT_FALSE(router.add_route({id,filter,0}).has_value());
    }
    EXPECT_EQ(runtime.start(), RuntimeResult::Ok);
    EXPECT_EQ(s104.activate(at(0)), AdapterResult::Ok);
    EXPECT_EQ(s105.activate(at(0)), AdapterResult::Ok);
  }

  static bool observe(void* context, std::uint16_t route, const RawCanFrame& frame) noexcept {
    auto& self = *static_cast<TwoDeviceHarness*>(context);
    const auto now = self.cycle_now;
    if (route == 104) (void)self.s104.accept_feedback(frame, now);
    if (route == 105) (void)self.s105.accept_feedback(frame, now);
    return true;
  }

  RuntimeResult receive(std::int64_t ns) {
    cycle_now = at(ns);
    const auto result = runtime.receive(at(ns), {this, &observe});
    if (result != RuntimeResult::Ok) {
      if (runtime.state() == RuntimeState::Fault) {
        bound104 = bound105 = false;
        pending104.reset();
        pending105.reset();
      }
      return result;
    }
    if (!claim104 || s104.snapshot(at(ns)).availability != ServoPositionAvailability::Fresh) {
      cancel_route(104);
    }
    if (!claim105 || s105.snapshot(at(ns)).availability != ServoPositionAvailability::Fresh) {
      cancel_route(105);
    }
    return result;
  }

  RuntimeResult submit(std::uint16_t id, double position, std::uint64_t generation,
                       std::int64_t now, std::int64_t deadline) {
    auto& s = id == 104 ? s104 : s105;
    if (id == 104 ? !claim104 : !claim105) {
      cancel_route(id);
      return RuntimeResult::InvalidCommand;
    }
    RawCanFrame frame{};
    const auto prepared = s.prepare_position(command(position,generation,deadline), at(now), frame);
    if (prepared != AdapterResult::Ok) {
      cancel_route(id);
      return RuntimeResult::InvalidCommand;
    }
    auto lease = CommandLease::create(id, generation, frame, at(now), at(deadline));
    if (!lease || runtime.submit(*lease) != RuntimeResult::Ok) {
      cancel_route(id);
      return RuntimeResult::InvalidCommand;
    }
    (id == 104 ? pending104 : pending105) = *lease;
    (id == 104 ? bound104 : bound105) = true;
    return RuntimeResult::Ok;
  }

  void cancel_route(std::uint16_t id) {
    const auto result = runtime.cancel(id);
    const auto expected = runtime.state() != RuntimeState::Running
        ? RuntimeResult::NotRunning
        : (id == 104 ? bound104 : bound105)
            ? RuntimeResult::Ok : RuntimeResult::InvalidCommand;
    EXPECT_EQ(result, expected);
    (id == 104 ? pending104 : pending105).reset();
  }

  void stop() {
    runtime.stop();
    bound104 = bound105 = false;
    pending104.reset();
    pending105.reset();
  }

  RuntimeResult start() { return runtime.start(); }

  RuntimeResult recover() {
    const auto result = runtime.recover();
    bound104 = bound105 = false;
    pending104.reset();
    pending105.reset();
    return result;
  }

  RuntimeResult transmit(std::int64_t ns) {
    for (std::uint16_t id : {104,105}) {
      auto& pending = id == 104 ? pending104 : pending105;
      auto& session = id == 104 ? s104 : s105;
      const bool claimed = id == 104 ? claim104 : claim105;
      if (pending && (!claimed || !session.pending_target_still_authorized(
          pending->frame, pending->generation, pending->deadline, at(ns)))) {
        cancel_route(id);
      }
    }
    const auto result = runtime.transmit(at(ns));
    if (runtime.state() == RuntimeState::Fault) {
      bound104 = bound105 = false;
      pending104.reset();
      pending105.reset();
    }
    return result;
  }
};

TEST(Ak30ServoPositionSession, TwoSyntheticDevicesShareOneBusAndEmitIndependentModeSix) {
  TwoDeviceHarness h;
  mech::mech_simulation::FakeTransport second;
  BusRuntime rival{2,"fake0",second,h.router,h.ownership};
  EXPECT_EQ(rival.start(), RuntimeResult::AlreadyOwned);
  EXPECT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  EXPECT_EQ(h.transport.inject_receive(feedback(2,105,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  EXPECT_EQ(h.s104.snapshot(at(1)).sequence, 1U);
  EXPECT_EQ(h.s105.snapshot(at(1)).sequence, 1U);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(105,0,1,1,1001), RuntimeResult::Ok);
  ASSERT_EQ(h.transmit(2), RuntimeResult::Ok);
  RawCanFrame first{}, second_frame{};
  ASSERT_TRUE(h.transport.take_transmit(first));
  ASSERT_TRUE(h.transport.take_transmit(second_frame));
  EXPECT_FALSE(h.transport.take_transmit(first));
  EXPECT_EQ(first.id.value, 0x668U);
  EXPECT_EQ(second_frame.id.value, 0x669U);
  EXPECT_EQ((std::array<std::uint8_t,8>{second_frame.payload[0],second_frame.payload[1],second_frame.payload[2],second_frame.payload[3],second_frame.payload[4],second_frame.payload[5],second_frame.payload[6],second_frame.payload[7]}),
            (std::array<std::uint8_t,8>{0xFF,0xFF,0xD8,0xF0,0,64,0,12})); // -1 degree
}

TEST(Ak30ServoPositionSession, QueueFullThenInvalidReplacementCancelsOnlyAffectedRoute) {
  TwoDeviceHarness h;
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,105,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(105,0,1,1,1001), RuntimeResult::Ok);
  h.transport.force_next_send_results({TransportResult::QueueFull,TransportResult::WouldBlock});
  EXPECT_EQ(h.transmit(2), RuntimeResult::QueueFull);
  EXPECT_EQ(h.submit(104,10,2,3,1003), RuntimeResult::InvalidCommand);
  EXPECT_EQ(h.transmit(4), RuntimeResult::Ok);
  RawCanFrame frame{};
  ASSERT_TRUE(h.transport.take_transmit(frame));
  EXPECT_EQ(frame.id.value, 0x669U);
  EXPECT_FALSE(h.transport.take_transmit(frame));
}

TEST(Ak30ServoPositionSession, RevokedClaimCancelsPendingRouteWithoutConsumingDeniedGeneration) {
  TwoDeviceHarness h;
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,105,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(105,0,1,1,1001), RuntimeResult::Ok);
  h.transport.force_next_send_results({TransportResult::QueueFull,TransportResult::WouldBlock});
  ASSERT_EQ(h.transmit(2), RuntimeResult::QueueFull);
  h.claim104 = false;
  EXPECT_EQ(h.submit(104,1,2,3,1003), RuntimeResult::InvalidCommand);
  ASSERT_EQ(h.transmit(4), RuntimeResult::Ok);
  RawCanFrame sent{};
  ASSERT_TRUE(h.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x669U);
  EXPECT_FALSE(h.transport.take_transmit(sent));
  h.claim104 = true;
  EXPECT_EQ(h.transmit(5), RuntimeResult::Ok);
  EXPECT_FALSE(h.transport.take_transmit(sent));
  // Generation 2 was denied before prepare, so it remains available.
  ASSERT_EQ(h.submit(104,1,2,6,1006), RuntimeResult::Ok);
  ASSERT_EQ(h.transmit(7), RuntimeResult::Ok);
  ASSERT_TRUE(h.transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x668U);
}

TEST(Ak30ServoPositionSession, PendingTargetMustBeRecheckedAfterFreshPositionMoves) {
  TwoDeviceHarness h;
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  h.transport.force_next_send_results({TransportResult::QueueFull});
  ASSERT_EQ(h.transmit(2), RuntimeResult::QueueFull);
  // Synthetic feedback 6 degrees maps to -2 rad. The queued +1 rad target
  // now exceeds max_target_error=2 even though this sample is fresh/normal.
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,3,0,60)), TransportResult::Ok);
  ASSERT_EQ(h.receive(3), RuntimeResult::Ok);
  EXPECT_EQ(h.transmit(4), RuntimeResult::Ok);
  RawCanFrame frame{};
  EXPECT_FALSE(h.transport.take_transmit(frame));
}

TEST(Ak30ServoPositionSession, FeedbackCanExpireBetweenReceiveAndTransmit) {
  TwoDeviceHarness h(100);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  EXPECT_EQ(h.transmit(101), RuntimeResult::Ok); // feedback TTL equality expires
  RawCanFrame frame{};
  EXPECT_FALSE(h.transport.take_transmit(frame));
}

TEST(Ak30ServoPositionSession, ReadOnlyPendingCheckRejectsOldOrModifiedLease) {
  Ak30ServoPositionSession s;
  ASSERT_EQ(s.configure(synthetic_config()), AdapterResult::Ok);
  ASSERT_EQ(s.activate(at(0)), AdapterResult::Ok);
  ASSERT_EQ(s.accept_feedback(feedback(2,104,1), at(1)), AdapterResult::Ok);
  RawCanFrame first{}, second{};
  ASSERT_EQ(s.prepare_position(command(1,1,1001), at(1), first), AdapterResult::Ok);
  EXPECT_TRUE(s.pending_target_still_authorized(first,1,at(1001),at(2)));
  EXPECT_FALSE(s.pending_target_still_authorized(first,1,at(1001),at(1001)));
  auto modified = first;
  modified.payload[7] ^= 1U;
  EXPECT_FALSE(s.pending_target_still_authorized(modified,1,at(1001),at(2)));
  modified = first;
  modified.host_arrival = at(0);
  EXPECT_FALSE(s.pending_target_still_authorized(modified,1,at(1001),at(2)));
  modified = first;
  modified.source_timestamp = SourceTimestamp{SourceClockDomain::Device, 1};
  EXPECT_FALSE(s.pending_target_still_authorized(modified,1,at(1001),at(2)));
  EXPECT_FALSE(s.pending_target_still_authorized(first,1,at(1001),at(0)));
  ASSERT_EQ(s.prepare_position(command(1,2,1002), at(2), second), AdapterResult::Ok);
  EXPECT_FALSE(s.pending_target_still_authorized(first,1,at(1001),at(2)));
  EXPECT_TRUE(s.pending_target_still_authorized(second,2,at(1002),at(2)));
  s.deactivate();
  EXPECT_FALSE(s.pending_target_still_authorized(second,2,at(1002),at(2)));
  ASSERT_EQ(s.activate(at(3)), AdapterResult::Ok);
  EXPECT_FALSE(s.pending_target_still_authorized(second,2,at(1002),at(3)));
}

TEST(Ak30ServoPositionSession, FaultThenHealthyBeforeTransmitCannotUnlatch) {
  TwoDeviceHarness h;
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,105,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(105,0,1,1,1001), RuntimeResult::Ok);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,2,2)), TransportResult::Ok);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,3)), TransportResult::Ok);
  ASSERT_EQ(h.receive(3), RuntimeResult::Ok);
  EXPECT_TRUE(h.s104.fault_latched());
  EXPECT_EQ(h.transmit(3), RuntimeResult::Ok);
  RawCanFrame frame{};
  ASSERT_TRUE(h.transport.take_transmit(frame));
  EXPECT_EQ(frame.id.value, 0x669U);
  EXPECT_FALSE(h.transport.take_transmit(frame));
}

TEST(Ak30ServoPositionSession, SameBatchUnusableThenNormalRevokesOldPreparedLease) {
  for (int scenario = 0; scenario < 3; ++scenario) {
    TwoDeviceHarness h;
    ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
    ASSERT_EQ(h.transport.inject_receive(feedback(2,105,1)), TransportResult::Ok);
    ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
    ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
    ASSERT_EQ(h.submit(105,0,1,1,1001), RuntimeResult::Ok);
    h.transport.force_next_send_results({TransportResult::QueueFull,TransportResult::WouldBlock});
    ASSERT_EQ(h.transmit(2), RuntimeResult::QueueFull);
    auto unusable = feedback(2,104,3,scenario == 1 ? 0x42 : 0x77);
    if (scenario == 0) {
      unusable.payload[7] = 0;
      unusable.type = CanFrameType::FlexibleDataRate;
    }
    ASSERT_EQ(h.transport.inject_receive(unusable), TransportResult::Ok);
    ASSERT_EQ(h.transport.inject_receive(feedback(2,104,4)), TransportResult::Ok);
    ASSERT_EQ(h.receive(4), RuntimeResult::Ok);
    EXPECT_EQ(h.s104.snapshot(at(4)).availability, ServoPositionAvailability::Fresh);
    ASSERT_EQ(h.transmit(5), RuntimeResult::Ok);
    RawCanFrame sent{};
    ASSERT_TRUE(h.transport.take_transmit(sent));
    EXPECT_EQ(sent.id.value, 0x669U);
    EXPECT_FALSE(h.transport.take_transmit(sent));
    ASSERT_EQ(h.submit(104,1,2,6,1006), RuntimeResult::Ok);
    ASSERT_EQ(h.transmit(7), RuntimeResult::Ok);
    ASSERT_TRUE(h.transport.take_transmit(sent));
    EXPECT_EQ(sent.id.value, 0x668U);
  }
}

TEST(Ak30ServoPositionSession, StaleUnknownAndDisableFeedbackCancelOnlyAffectedRoute) {
  for (int scenario = 0; scenario < 3; ++scenario) {
    TwoDeviceHarness h;
    ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
    ASSERT_EQ(h.transport.inject_receive(feedback(2,105,1)), TransportResult::Ok);
    ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
    ASSERT_EQ(h.submit(104,1,1,1,4000001), RuntimeResult::Ok);
    ASSERT_EQ(h.submit(105,0,1,1,4000001), RuntimeResult::Ok);
    if (scenario == 0) {
      ASSERT_EQ(h.transport.inject_receive(feedback(2,104,2)), TransportResult::Ok);
      ASSERT_EQ(h.transport.inject_receive(feedback(2,105,6000002)), TransportResult::Ok);
      ASSERT_EQ(h.receive(6000002), RuntimeResult::Ok);
      // Both leases have expired by this time; test the stale session gate.
      EXPECT_EQ(h.submit(104,1,2,6000002,6001002), RuntimeResult::InvalidCommand);
      EXPECT_EQ(h.submit(105,0,2,6000002,6001002), RuntimeResult::Ok);
      ASSERT_EQ(h.transmit(6000003), RuntimeResult::Ok);
    } else {
      const std::uint8_t status = scenario == 1 ? 0x42 : 0x77;
      ASSERT_EQ(h.transport.inject_receive(feedback(2,104,2,status)), TransportResult::Ok);
      ASSERT_EQ(h.receive(2), RuntimeResult::Ok);
      ASSERT_EQ(h.transmit(2), RuntimeResult::Ok);
    }
    RawCanFrame frame{};
    ASSERT_TRUE(h.transport.take_transmit(frame));
    EXPECT_EQ(frame.id.value, 0x669U);
    EXPECT_FALSE(h.transport.take_transmit(frame));
  }
}

TEST(Ak30ServoPositionSession, DeadlineExpiryAndStopStartDoNotReplayQueuedTargets) {
  TwoDeviceHarness h;
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  EXPECT_EQ(h.transmit(1001), RuntimeResult::Ok);
  RawCanFrame frame{};
  EXPECT_FALSE(h.transport.take_transmit(frame));
  ASSERT_EQ(h.submit(104,1,2,1002,2002), RuntimeResult::Ok);
  h.transport.force_next_send_results({TransportResult::QueueFull});
  EXPECT_EQ(h.transmit(1003), RuntimeResult::QueueFull);
  h.stop();
  h.s104.deactivate();
  h.s105.deactivate();
  ASSERT_EQ(h.start(), RuntimeResult::Ok);
  ASSERT_EQ(h.s104.activate(at(1004)), AdapterResult::Ok);
  ASSERT_EQ(h.s105.activate(at(1004)), AdapterResult::Ok);
  EXPECT_EQ(h.receive(1004), RuntimeResult::Ok);
  EXPECT_EQ(h.transmit(1004), RuntimeResult::Ok);
  EXPECT_FALSE(h.transport.take_transmit(frame));
  EXPECT_EQ(h.submit(104,1,1,1004,2004), RuntimeResult::InvalidCommand);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1005)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1005), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1005,2005), RuntimeResult::Ok);
  ASSERT_EQ(h.transmit(1006), RuntimeResult::Ok);
  ASSERT_TRUE(h.transport.take_transmit(frame));
  EXPECT_EQ(frame.id.value, 0x668U);
}

TEST(Ak30ServoPositionSession, DisconnectRecoveryRequiresNewEpochFeedbackAndCommand) {
  TwoDeviceHarness h;
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,1)), TransportResult::Ok);
  ASSERT_EQ(h.receive(1), RuntimeResult::Ok);
  ASSERT_EQ(h.submit(104,1,1,1,1001), RuntimeResult::Ok);
  h.transport.force_next_receive_results({TransportResult::Disconnected});
  EXPECT_EQ(h.receive(2), RuntimeResult::Disconnected);
  EXPECT_EQ(h.runtime.state(), RuntimeState::Fault);
  h.s104.deactivate();
  h.s105.deactivate();
  ASSERT_EQ(h.recover(), RuntimeResult::Ok);
  ASSERT_EQ(h.s104.activate(at(3)), AdapterResult::Ok);
  ASSERT_EQ(h.s105.activate(at(3)), AdapterResult::Ok);
  EXPECT_EQ(h.transmit(3), RuntimeResult::Ok);
  RawCanFrame frame{};
  EXPECT_FALSE(h.transport.take_transmit(frame));
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,2)), TransportResult::Ok);
  EXPECT_EQ(h.receive(3), RuntimeResult::Ok);
  EXPECT_EQ(h.submit(104,1,1,3,1003), RuntimeResult::InvalidCommand);
  ASSERT_EQ(h.transport.inject_receive(feedback(2,104,4)), TransportResult::Ok);
  ASSERT_EQ(h.receive(4), RuntimeResult::Ok);
  EXPECT_EQ(h.submit(104,1,1,4,1004), RuntimeResult::Ok);
}
}  // namespace
