#include "mech_protocol_damiao/dm4340_mit_wire.hpp"
#include "mech_protocol_damiao/dm4340_mit_session.hpp"
#include "mech_control_core/runtime.hpp"
#include "mech_simulation/fake_transport.hpp"

#include <array>
#include <cmath>

int main() {
  using namespace mech::mech_protocol_damiao;
  using namespace mech::mech_control_core;
  std::array<std::uint8_t, 8> command{};
  if (!encode_mit({0.0, 0.0, 0.0, 0.0, 0.0}, kSdkDm4340Limits,
                  command) ||
      command != std::array<std::uint8_t, 8>{
                     0x7FU, 0xFFU, 0x7FU, 0xF0U,
                     0x00U, 0x00U, 0x07U, 0xFFU}) {
    return 1;
  }
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  payload[0] = 0x21U;
  payload[1] = 0x7FU;
  payload[2] = 0xFFU;
  payload[3] = 0x7FU;
  payload[4] = 0xF7U;
  payload[5] = 0xFFU;
  const auto frame = RawCanFrame::create(
      2U, *CanId::create(0x11U, CanFrameFormat::Standard),
      CanFrameType::Classic, FrameDirection::Rx, 8U, payload,
      MonotonicTime{});
  MitFeedback feedback{};
  if (!frame.has_value() ||
      !decode_mit_feedback(*frame, 2U, 0x11U, 1U,
                           kSdkDm4340Limits, feedback) ||
      feedback.raw_fault != 2U ||
      std::abs(feedback.position_rad) > 0.001 ||
      std::abs(feedback.velocity_rad_s) > 0.01 ||
      std::abs(feedback.torque_nm) > 0.02) {
    return 2;
  }
  Dm4340MitSession session;
  Dm4340MitSessionConfig config{};
  config.profile = ProtocolProfile::Dm4340MitStandard;
  config.logical_bus = 2U;
  config.command_can_id = 0x01U;
  config.feedback_can_id = 0x11U;
  config.motor_id = 1U;
  config.firmware_id = 1U;
  config.firmware_min = 1U;
  config.firmware_max = 1U;
  config.limits = kSdkDm4340Limits;
  config.joint = {1.0, 0.0, true};
  config.position_min_rad = -1.0;
  config.position_max_rad = 1.0;
  config.max_target_error_rad = 0.2;
  config.kp = 1.0;
  config.kd = 0.1;
  config.command_ttl_ns = 4000000;
  config.feedback_ttl_ns = 6000000;
  if (session.configure(config) != AdapterResult::InvalidConfiguration) {
    return 3;
  }
  config.firmware_evidence_declared = true;
  config.limits_evidence_declared = true;
  if (session.configure(config) != AdapterResult::Ok ||
      session.activate(MonotonicTime{}) != AdapterResult::Ok) {
    return 4;
  }
  RawCanFrame prepared{};
  const auto t1 = *MonotonicTime::from_nanoseconds(1);
  const auto deadline = *MonotonicTime::from_nanoseconds(4000001);
  const CanonicalDeviceCommand target{0.0, 0.0, 0.0, 1U, deadline};
  if (session.prepare_position(target, t1, prepared) != AdapterResult::Stale) {
    return 5;
  }
  auto healthy = *frame;
  healthy.payload[0] = 0x11U;  // Position commands require enabled status.
  healthy.host_arrival = t1;
  if (session.accept_feedback(healthy, t1) != AdapterResult::Ok ||
      session.prepare_position(target, t1, prepared) != AdapterResult::Ok ||
      !session.pending_target_still_authorized(prepared, 1U, deadline, t1)) {
    return 6;
  }
  auto altered = prepared;
  altered.payload[0] ^= 1U;
  if (session.pending_target_still_authorized(altered, 1U, deadline, t1) ||
      session.pending_target_still_authorized(
          prepared, 1U, deadline, deadline)) {
    return 7;
  }
  const auto stale = *MonotonicTime::from_nanoseconds(6000001);
  if (session.snapshot(stale).availability != DmFeedbackAvailability::Stale ||
      session.pending_target_still_authorized(prepared, 1U, deadline, stale)) {
    return 8;
  }
  auto moved = healthy;
  moved.host_arrival = *MonotonicTime::from_nanoseconds(2);
  moved.payload[1] = 0x83U;
  moved.payload[2] = 0xFFU;
  if (session.accept_feedback(moved, moved.host_arrival) !=
          AdapterResult::Ok ||
      session.pending_target_still_authorized(prepared, 1U, deadline,
                                               moved.host_arrival)) {
    return 11;
  }
  auto fault = healthy;
  fault.host_arrival = *MonotonicTime::from_nanoseconds(3);
  fault.payload[0] = 0x21U;
  if (session.accept_feedback(fault, fault.host_arrival) !=
          AdapterResult::Fault ||
      !session.fault_latched() ||
      session.pending_target_still_authorized(prepared, 1U, deadline,
                                               fault.host_arrival)) {
    return 9;
  }
  session.deactivate();
  if (session.activate(*MonotonicTime::from_nanoseconds(4)) !=
          AdapterResult::Ok ||
      session.snapshot(*MonotonicTime::from_nanoseconds(4)).availability !=
          DmFeedbackAvailability::Unknown) {
    return 10;
  }
  // A single fake bus owner can route both brands by format/ID. Only the bus
  // runtime performs TX; the Damiao session merely prepares a candidate.
  mech::mech_simulation::FakeTransport transport;
  FrameRouter router;
  if (router.add_route({1U, {CanFrameFormat::Standard, 0x11U, 0x7FFU,
                            CanFrameType::Classic}, 0U}) ||
      router.add_route({2U, {CanFrameFormat::Extended, 0x2968U,
                            kMaxExtendedCanId, CanFrameType::Classic}, 0U})) {
    return 12;
  }
  BusOwnershipRegistry ownership;
  BusRuntime bus{2U, "fake-damiao-cubemars", transport, router, ownership};
  if (bus.start() != RuntimeResult::Ok || transport.pending_transmit() != 0U) {
    return 13;
  }
  auto new_feedback = healthy;
  new_feedback.host_arrival = *MonotonicTime::from_nanoseconds(5);
  if (transport.inject_receive(new_feedback) != TransportResult::Ok) {
    return 14;
  }
  struct ObserverContext {
    Dm4340MitSession* session;
    MonotonicTime now;
    bool accepted{false};
  } observer_context{&session, new_feedback.host_arrival};
  const BusRuntime::RxObserver observer{
      &observer_context,
      [](void* raw, std::uint16_t route, const RawCanFrame& received)
          noexcept -> bool {
        auto& context = *static_cast<ObserverContext*>(raw);
        context.accepted = route == 1U &&
            context.session->accept_feedback(received, context.now) ==
                AdapterResult::Ok;
        return context.accepted;
      }};
  if (bus.receive(new_feedback.host_arrival, observer) != RuntimeResult::Ok ||
      !observer_context.accepted || transport.pending_transmit() != 0U) {
    return 15;
  }
  auto cubemars = new_feedback;
  cubemars.id = *CanId::create(0x2968U, CanFrameFormat::Extended);
  observer_context.accepted = false;
  if (transport.inject_receive(cubemars) != TransportResult::Ok) {
    return 20;
  }
  const BusRuntime::RxObserver mixed_observer{
      &observer_context,
      [](void* raw, std::uint16_t route, const RawCanFrame&)
          noexcept -> bool {
        auto& context = *static_cast<ObserverContext*>(raw);
        context.accepted = route == 2U;
        return context.accepted;
      }};
  if (bus.receive(new_feedback.host_arrival, mixed_observer) !=
          RuntimeResult::Ok || !observer_context.accepted ||
      session.snapshot(new_feedback.host_arrival).sequence != 1U) {
    return 21;
  }
  const auto deadline2 = *MonotonicTime::from_nanoseconds(4000005);
  const CanonicalDeviceCommand second{0.0, 0.0, 0.0, 1U, deadline2};
  if (session.prepare_position(second, new_feedback.host_arrival,
                               prepared) != AdapterResult::Ok) {
    return 16;
  }
  const auto lease = CommandLease::create(1U, 1U, prepared,
                                          new_feedback.host_arrival, deadline2);
  if (!lease || bus.submit(*lease) != RuntimeResult::Ok ||
      transport.pending_transmit() != 0U) {
    return 17;
  }
  struct GuardContext {
    Dm4340MitSession* session;
  } guard_context{&session};
  const BusRuntime::TxGuard guard{
      &guard_context,
      [](void* raw, const CommandLease& pending, MonotonicTime& now)
          noexcept -> bool {
        const auto& context = *static_cast<GuardContext*>(raw);
        return context.session->pending_target_still_authorized(
            pending.frame, pending.generation, pending.deadline, now);
      }};
  if (bus.transmit(new_feedback.host_arrival, guard) != RuntimeResult::Ok ||
      transport.pending_transmit() != 1U) {
    return 18;
  }
  RawCanFrame sent{};
  if (!transport.take_transmit(sent) ||
      sent.id.format != CanFrameFormat::Standard || sent.id.value != 1U ||
      sent.payload != prepared.payload) {
    return 19;
  }
  const CanonicalDeviceCommand rejected{2.0, 0.0, 0.0, 2U, deadline2};
  RawCanFrame rejected_frame{};
  if (session.prepare_position(rejected, new_feedback.host_arrival,
                               rejected_frame) != AdapterResult::InvalidCommand ||
      session.pending_target_still_authorized(
          prepared, 1U, deadline2, new_feedback.host_arrival)) {
    return 22;
  }
  bus.stop();
  return 0;
}
