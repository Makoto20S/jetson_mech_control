#include "mech_control_core/router.hpp"
#include "mech_control_core/runtime.hpp"
#include "mech_simulation/fake_transport.hpp"

#include <array>

#include <gtest/gtest.h>

namespace mech::mech_simulation {
namespace {

mech_control_core::RawCanFrame make_frame(
    mech_control_core::FrameDirection direction, std::uint32_t id_value,
    std::int64_t arrival) {
  const auto id = mech_control_core::CanId::create(
      id_value, mech_control_core::CanFrameFormat::Standard);
  const auto time = mech_control_core::MonotonicTime::from_nanoseconds(arrival);
  const std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{
      0xAAU, 0x55U};
  return *mech_control_core::RawCanFrame::create(
      1U, *id, mech_control_core::CanFrameType::Classic, direction, 2U,
      payload, *time);
}

TEST(FrameRouter, RoutesMatchingFramesToExplicitFanout) {
  mech_control_core::FrameRouter router;
  EXPECT_FALSE(router.add_route(mech_control_core::FrameRoute{
      1U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x180U, 0x7FFU,
                                     mech_control_core::CanFrameType::Classic},
      7U}));
  EXPECT_FALSE(router.add_route(mech_control_core::FrameRoute{
      2U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x180U, 0x7FFU,
                                     mech_control_core::CanFrameType::Classic},
      7U}));
  const auto destinations = router.route(
      make_frame(mech_control_core::FrameDirection::Rx, 0x180U, 10));
  ASSERT_EQ(destinations.size(), 2U);
  EXPECT_EQ(destinations[0], 1U);
  EXPECT_EQ(destinations[1], 2U);
}

TEST(BusRuntime, OwnsOneWriterAndExpiresLatestCommand) {
  mech_control_core::FrameRouter router;
  EXPECT_FALSE(router.add_route(mech_control_core::FrameRoute{
      1U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x180U, 0x7FFU,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  EXPECT_FALSE(router.add_route(mech_control_core::FrameRoute{
      2U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x100U, 0x7FFU,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  mech_control_core::BusRuntime runtime(1U, "fake0", transport, router,
                                        ownership);
  mech_control_core::BusRuntime second(1U, "fake0", transport, router,
                                       ownership);
  EXPECT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(second.start(), mech_control_core::RuntimeResult::AlreadyOwned);

  ASSERT_EQ(transport.inject_receive(
                make_frame(mech_control_core::FrameDirection::Rx, 0x180U, 10)),
            mech_control_core::TransportResult::Ok);
  const auto poll_time = mech_control_core::MonotonicTime::from_nanoseconds(20);
  ASSERT_TRUE(poll_time.has_value());
  EXPECT_EQ(runtime.poll(*poll_time), mech_control_core::RuntimeResult::Ok);
  const auto ttl = mech_control_core::MonotonicDuration::from_nanoseconds(20);
  ASSERT_TRUE(ttl.has_value());
  const auto snapshot = runtime.snapshots().read(1U, *poll_time, *ttl);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->sequence, 1U);
  EXPECT_EQ(snapshot->host_rx_time.nanoseconds(), 10);
  EXPECT_EQ(snapshot->age.nanoseconds(), 10);
  EXPECT_TRUE(snapshot->fresh);

  const auto deadline = mech_control_core::MonotonicTime::from_nanoseconds(30);
  ASSERT_TRUE(deadline.has_value());
  const auto lease = mech_control_core::CommandLease::create(
      2U, 1U, make_frame(mech_control_core::FrameDirection::Tx, 0x100U, 20),
      *poll_time, *deadline);
  ASSERT_TRUE(lease.has_value());
  EXPECT_EQ(runtime.submit(*lease), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.poll(*poll_time), mech_control_core::RuntimeResult::Ok);
  mech_control_core::RawCanFrame sent;
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x100U);
  EXPECT_FALSE(transport.take_transmit(sent));

  const auto second_deadline =
      mech_control_core::MonotonicTime::from_nanoseconds(25);
  ASSERT_TRUE(second_deadline.has_value());
  const auto second_lease = mech_control_core::CommandLease::create(
      2U, 2U, make_frame(mech_control_core::FrameDirection::Tx, 0x100U, 20),
      *poll_time, *second_deadline);
  ASSERT_TRUE(second_lease.has_value());
  EXPECT_EQ(runtime.submit(*second_lease), mech_control_core::RuntimeResult::Ok);
  const auto expired_time = mech_control_core::MonotonicTime::from_nanoseconds(26);
  ASSERT_TRUE(expired_time.has_value());
  EXPECT_EQ(runtime.poll(*expired_time), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.stats().expired_commands, 1U);
  EXPECT_FALSE(transport.take_transmit(sent));
  runtime.stop();
  EXPECT_EQ(second.start(), mech_control_core::RuntimeResult::Ok);
}

TEST(BusRuntime, DisconnectTransitionsToFault) {
  mech_control_core::FrameRouter router;
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  mech_control_core::BusRuntime runtime(1U, "fake1", transport, router,
                                        ownership);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);
  transport.close();
  const auto now = mech_control_core::MonotonicTime::from_nanoseconds(0);
  ASSERT_TRUE(now.has_value());
  EXPECT_EQ(runtime.poll(*now), mech_control_core::RuntimeResult::Disconnected);
  EXPECT_EQ(runtime.state(), mech_control_core::RuntimeState::Fault);
  EXPECT_EQ(runtime.stats().bus_faults, 1U);
}

// DEFECT 1 (TX side): a transient WouldBlock from try_send() -- exactly what
// a non-blocking write() returns on EAGAIN -- must be treated as
// backpressure like QueueFull, not as a fault. The lease must stay pending
// (mark_sent() must not have been called) so it is retried on a later
// cycle and eventually sent once the transport stops blocking.
//
// Verified to fail against the unfixed code: with the original
// `else { return fault_for(result); }` branch (no WouldBlock case in the TX
// loop), forcing try_send() to return WouldBlock drove state_ to Fault and
// poll() returned RuntimeResult::Fault instead of QueueFull, and the
// second poll() below returned NotRunning instead of Ok -- I confirmed this
// by temporarily reverting the TX loop in runtime.hpp to the pre-fix
// three-way branch (Ok / QueueFull / else-fault_for) while keeping this
// test, rebuilding mech_simulation_test, and observing exactly those two
// assertions fail before restoring the fix.
TEST(BusRuntime, TxWouldBlockDoesNotFaultAndRetriesUntilSent) {
  mech_control_core::FrameRouter router;
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      2U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x100U, 0x700U,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  mech_control_core::BusRuntime runtime(1U, "fake-wouldblock", transport,
                                        router, ownership);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);

  const auto submit_time = mech_control_core::MonotonicTime::from_nanoseconds(0);
  const auto deadline = mech_control_core::MonotonicTime::from_nanoseconds(1000);
  ASSERT_TRUE(submit_time.has_value());
  ASSERT_TRUE(deadline.has_value());
  const auto lease = mech_control_core::CommandLease::create(
      2U, 1U, make_frame(mech_control_core::FrameDirection::Tx, 0x100U, 0),
      *submit_time, *deadline);
  ASSERT_TRUE(lease.has_value());
  ASSERT_EQ(runtime.submit(*lease), mech_control_core::RuntimeResult::Ok);

  transport.force_next_send_results({mech_control_core::TransportResult::WouldBlock});
  const auto poll_time = mech_control_core::MonotonicTime::from_nanoseconds(10);
  ASSERT_TRUE(poll_time.has_value());
  EXPECT_EQ(runtime.poll(*poll_time), mech_control_core::RuntimeResult::QueueFull);
  EXPECT_EQ(runtime.state(), mech_control_core::RuntimeState::Running);
  EXPECT_EQ(runtime.stats().would_block, 1U);
  EXPECT_EQ(runtime.stats().bus_faults, 0U);
  mech_control_core::RawCanFrame sent;
  EXPECT_FALSE(transport.take_transmit(sent));

  // No forced result this time: the retried send should go through.
  const auto retry_time = mech_control_core::MonotonicTime::from_nanoseconds(20);
  ASSERT_TRUE(retry_time.has_value());
  EXPECT_EQ(runtime.poll(*retry_time), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.stats().tx_frames, 1U);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x100U);
}

// DEFECT 1 (RX side): an Invalid frame (truncated/malformed) from
// try_receive() is a normal bus event, not a runtime failure. It must be
// counted (rx_invalid) and the receive loop must continue so a later good
// frame in the same poll() cycle is still routed.
//
// Verified to fail against the unfixed code: the original RX loop had no
// TransportResult::Invalid case, so `if (result != TransportResult::Ok) {
// return fault_for(result); }` faulted the runtime on the very first
// (malformed) frame and never reached the following good frame. I
// confirmed this by temporarily reverting the RX loop's Invalid handling
// in runtime.hpp (removing the dedicated Invalid branch so Invalid fell
// into the generic fault_for() branch), rebuilding mech_simulation_test,
// and observing this test fail: poll() returned Fault instead of Ok, and
// the snapshot for route 1 was never published.
TEST(BusRuntime, RxInvalidIsCountedAndRoutingContinues) {
  mech_control_core::FrameRouter router;
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      1U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x180U, 0x7FFU,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  mech_control_core::BusRuntime runtime(1U, "fake-invalid", transport, router,
                                        ownership);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);

  // The forced Invalid result is consumed by the first try_receive() call;
  // the queued good frame is what the *second* try_receive() call (still
  // within the same poll() cycle's receive loop) will return.
  ASSERT_EQ(transport.inject_receive(
                make_frame(mech_control_core::FrameDirection::Rx, 0x180U, 10)),
            mech_control_core::TransportResult::Ok);
  transport.force_next_receive_results(
      {mech_control_core::TransportResult::Invalid});

  const auto poll_time = mech_control_core::MonotonicTime::from_nanoseconds(20);
  ASSERT_TRUE(poll_time.has_value());
  EXPECT_EQ(runtime.poll(*poll_time), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.state(), mech_control_core::RuntimeState::Running);
  EXPECT_EQ(runtime.stats().rx_invalid, 1U);
  EXPECT_EQ(runtime.stats().bus_faults, 0U);

  const auto ttl = mech_control_core::MonotonicDuration::from_nanoseconds(20);
  ASSERT_TRUE(ttl.has_value());
  const auto snapshot = runtime.snapshots().read(1U, *poll_time, *ttl);
  ASSERT_TRUE(snapshot.has_value());
  EXPECT_EQ(snapshot->host_rx_time.nanoseconds(), 10);
}

// DEFECT 1 (recovery path): once faulted (e.g. by a genuine Disconnected),
// the runtime must be recoverable via recover() rather than being wedged
// forever. recover() must not leak or double-release the
// BusOwnershipRegistry entry: after recovering, a second runtime bound to
// the same physical channel must still be correctly refused ownership.
//
// Verified to fail against the unfixed code: BusRuntime had no recover()
// member at all before this fix, so this test failed to compile against
// the unfixed runtime.hpp (confirmed by temporarily removing the
// recover() method added by this fix and re-running colcon build, which
// reproduced the exact compile error "no member named 'recover' in
// 'mech::mech_control_core::BusRuntime'").
TEST(BusRuntime, RecoverFromFaultRestoresRunning) {
  mech_control_core::FrameRouter router;
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  mech_control_core::BusRuntime runtime(1U, "fake-recover", transport, router,
                                        ownership);
  mech_control_core::BusRuntime other(1U, "fake-recover", transport, router,
                                      ownership);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);

  transport.close();
  const auto fault_time = mech_control_core::MonotonicTime::from_nanoseconds(0);
  ASSERT_TRUE(fault_time.has_value());
  ASSERT_EQ(runtime.poll(*fault_time),
            mech_control_core::RuntimeResult::Disconnected);
  ASSERT_EQ(runtime.state(), mech_control_core::RuntimeState::Fault);

  // While faulted, the ownership entry must still be held (recover() has
  // not run yet) so a second runtime cannot steal the channel out from
  // under it.
  EXPECT_EQ(other.start(), mech_control_core::RuntimeResult::AlreadyOwned);

  EXPECT_EQ(runtime.recover(), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.state(), mech_control_core::RuntimeState::Running);

  const auto poll_time = mech_control_core::MonotonicTime::from_nanoseconds(10);
  ASSERT_TRUE(poll_time.has_value());
  EXPECT_EQ(runtime.poll(*poll_time), mech_control_core::RuntimeResult::Ok);

  // recover() must have released and now hold exactly one ownership entry
  // (no leak, no double-release): stopping `runtime` frees the channel for
  // `other`.
  runtime.stop();
  EXPECT_EQ(other.start(), mech_control_core::RuntimeResult::Ok);
  other.stop();
}

TEST(BusRuntime, RecoverWhenNotFaultedReturnsInvalidState) {
  mech_control_core::FrameRouter router;
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  mech_control_core::BusRuntime runtime(1U, "fake-recover-noop", transport,
                                        router, ownership);
  EXPECT_EQ(runtime.recover(), mech_control_core::RuntimeResult::InvalidState);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.recover(), mech_control_core::RuntimeResult::InvalidState);
  EXPECT_EQ(runtime.state(), mech_control_core::RuntimeState::Running);
}

// DEFECT 4 (fairness): backpressure on one command slot must not prevent
// other slots from being attempted in the same poll() cycle.
//
// Verified to fail against the unfixed code: the original TX loop did
// `else if (result == QueueFull) { ...; return RuntimeResult::QueueFull; }`
// which returns out of poll() immediately, so the loop never reached the
// slot for route 3. I confirmed this by temporarily reverting the TX loop
// to `return RuntimeResult::QueueFull;` on the first QueueFull (removing
// the `backpressure = true; continue-the-loop` behavior added by this fix),
// rebuilding mech_simulation_test, and observing this test fail: tx_frames
// stayed 0 and take_transmit() returned false for the route-3 frame that
// should have gone out in the same cycle.
TEST(BusRuntime, BackpressureOnOneSlotDoesNotBlockOtherSlotsInSameCycle) {
  mech_control_core::FrameRouter router;
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      2U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x100U, 0x700U,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      3U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x200U, 0x700U,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  mech_control_core::BusRuntime runtime(1U, "fake-fairness", transport, router,
                                        ownership);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);

  const auto submit_time = mech_control_core::MonotonicTime::from_nanoseconds(0);
  const auto deadline = mech_control_core::MonotonicTime::from_nanoseconds(1000);
  ASSERT_TRUE(submit_time.has_value());
  ASSERT_TRUE(deadline.has_value());
  // Submitted first -> occupies command slot 0.
  const auto lease_a = mech_control_core::CommandLease::create(
      2U, 1U, make_frame(mech_control_core::FrameDirection::Tx, 0x100U, 0),
      *submit_time, *deadline);
  ASSERT_TRUE(lease_a.has_value());
  ASSERT_EQ(runtime.submit(*lease_a), mech_control_core::RuntimeResult::Ok);
  // Submitted second -> occupies command slot 1.
  const auto lease_b = mech_control_core::CommandLease::create(
      3U, 1U, make_frame(mech_control_core::FrameDirection::Tx, 0x200U, 0),
      *submit_time, *deadline);
  ASSERT_TRUE(lease_b.has_value());
  ASSERT_EQ(runtime.submit(*lease_b), mech_control_core::RuntimeResult::Ok);

  // Only the first try_send() call in this poll() cycle (slot 0, route 2)
  // is forced to report backpressure; slot 1 (route 3) is not forced and
  // must still be attempted in the same cycle.
  transport.force_next_send_results({mech_control_core::TransportResult::QueueFull});

  const auto poll_time = mech_control_core::MonotonicTime::from_nanoseconds(10);
  ASSERT_TRUE(poll_time.has_value());
  EXPECT_EQ(runtime.poll(*poll_time), mech_control_core::RuntimeResult::QueueFull);
  EXPECT_EQ(runtime.stats().queue_full, 1U);
  EXPECT_EQ(runtime.stats().tx_frames, 1U);

  mech_control_core::RawCanFrame sent;
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x200U);
  EXPECT_FALSE(transport.take_transmit(sent));

  // Slot 0's lease must still be pending (not marked sent) and gets
  // retried without re-submission.
  const auto retry_time = mech_control_core::MonotonicTime::from_nanoseconds(20);
  ASSERT_TRUE(retry_time.has_value());
  EXPECT_EQ(runtime.poll(*retry_time), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.stats().tx_frames, 2U);
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.id.value, 0x100U);
}

// DEFECT 2 (start()): ownership_.acquire() failing because the channel is
// already owned must be reported differently from failing because the
// registry is at capacity.
TEST(BusRuntime, StartDistinguishesAlreadyOwnedFromChannelCapacityExceeded) {
  mech_control_core::FrameRouter router;
  FakeTransport transport_a;
  FakeTransport transport_b;
  FakeTransport transport_c;
  mech_control_core::BusOwnershipRegistry ownership(1U);
  mech_control_core::BusRuntime runtime_a(1U, "channel-a", transport_a, router,
                                          ownership);
  mech_control_core::BusRuntime runtime_a_dup(1U, "channel-a", transport_a,
                                              router, ownership);
  mech_control_core::BusRuntime runtime_b(1U, "channel-b", transport_b, router,
                                          ownership);
  ASSERT_EQ(runtime_a.start(), mech_control_core::RuntimeResult::Ok);
  // Same channel, already owned by runtime_a.
  EXPECT_EQ(runtime_a_dup.start(), mech_control_core::RuntimeResult::AlreadyOwned);
  // Different channel, but the registry (capacity 1) is already full.
  EXPECT_EQ(runtime_b.start(),
            mech_control_core::RuntimeResult::ChannelCapacityExceeded);
  runtime_a.stop();
  // Now that the registry has room, a fresh channel succeeds.
  mech_control_core::BusRuntime runtime_c(1U, "channel-c", transport_c, router,
                                          ownership);
  EXPECT_EQ(runtime_c.start(), mech_control_core::RuntimeResult::Ok);
  runtime_c.stop();
}

// DEFECT 2 (submit()): no free CommandSlot ("capacity exhausted", an
// operational problem) must be reported differently from a malformed
// lease ("your command is invalid").
TEST(BusRuntime, SubmitDistinguishesCapacityExceededFromInvalidCommand) {
  mech_control_core::FrameRouter router;
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      2U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x100U, 0x700U,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      3U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x200U, 0x700U,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  // Exactly one command slot: the second distinct route_id cannot fit.
  mech_control_core::BusRuntime runtime(1U, "fake-submit", transport, router,
                                        ownership, 1U);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);

  const auto submit_time = mech_control_core::MonotonicTime::from_nanoseconds(0);
  const auto deadline = mech_control_core::MonotonicTime::from_nanoseconds(1000);
  ASSERT_TRUE(submit_time.has_value());
  ASSERT_TRUE(deadline.has_value());
  const auto lease_a = mech_control_core::CommandLease::create(
      2U, 1U, make_frame(mech_control_core::FrameDirection::Tx, 0x100U, 0),
      *submit_time, *deadline);
  ASSERT_TRUE(lease_a.has_value());
  ASSERT_EQ(runtime.submit(*lease_a), mech_control_core::RuntimeResult::Ok);

  // Second distinct route_id: no free slot remains (capacity exhausted).
  const auto lease_b = mech_control_core::CommandLease::create(
      3U, 1U, make_frame(mech_control_core::FrameDirection::Tx, 0x200U, 0),
      *submit_time, *deadline);
  ASSERT_TRUE(lease_b.has_value());
  EXPECT_EQ(runtime.submit(*lease_b),
            mech_control_core::RuntimeResult::CommandCapacityExceeded);

  // A malformed lease (route not known to the router) is a different
  // failure: InvalidCommand.
  const auto bad_lease = mech_control_core::CommandLease::create(
      99U, 1U, make_frame(mech_control_core::FrameDirection::Tx, 0x300U, 0),
      *submit_time, *deadline);
  ASSERT_TRUE(bad_lease.has_value());
  EXPECT_EQ(runtime.submit(*bad_lease),
            mech_control_core::RuntimeResult::InvalidCommand);
}

// DEFECT 3 (hidden capacity coupling): snapshot capacity is independently
// configurable, and an overflow (more distinct routes publishing than the
// snapshot store can hold) is reported as its own condition
// (snapshot_overflow), not folded into transport_errors.
TEST(BusRuntime, SnapshotCapacityOverflowIsCountedSeparatelyFromTransportErrors) {
  mech_control_core::FrameRouter router;
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      1U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x180U, 0x780U,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  ASSERT_FALSE(router.add_route(mech_control_core::FrameRoute{
      2U,
      mech_control_core::FrameFilter{mech_control_core::CanFrameFormat::Standard,
                                     0x200U, 0x780U,
                                     mech_control_core::CanFrameType::Classic},
      0U}));
  FakeTransport transport;
  mech_control_core::BusOwnershipRegistry ownership;
  // Two routes will publish distinct snapshots, but the snapshot store can
  // only hold one -- an explicit, independent capacity, not a hidden 32.
  mech_control_core::BusRuntime runtime(1U, "fake-snapshot", transport, router,
                                        ownership, 32U, 1U);
  ASSERT_EQ(runtime.start(), mech_control_core::RuntimeResult::Ok);

  ASSERT_EQ(transport.inject_receive(
                make_frame(mech_control_core::FrameDirection::Rx, 0x180U, 10)),
            mech_control_core::TransportResult::Ok);
  ASSERT_EQ(transport.inject_receive(
                make_frame(mech_control_core::FrameDirection::Rx, 0x200U, 20)),
            mech_control_core::TransportResult::Ok);

  const auto poll_time = mech_control_core::MonotonicTime::from_nanoseconds(30);
  ASSERT_TRUE(poll_time.has_value());
  EXPECT_EQ(runtime.poll(*poll_time), mech_control_core::RuntimeResult::Ok);
  EXPECT_EQ(runtime.state(), mech_control_core::RuntimeState::Running);
  EXPECT_EQ(runtime.stats().snapshot_overflow, 1U);
  EXPECT_EQ(runtime.stats().transport_errors, 0U);
}

}  // namespace
}  // namespace mech::mech_simulation
