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

}  // namespace
}  // namespace mech::mech_simulation
