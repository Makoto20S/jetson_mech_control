#include "mech_simulation/fake_clock.hpp"
#include "mech_simulation/fake_transport.hpp"

#include <array>
#include <limits>

#include <gtest/gtest.h>

namespace mech::mech_simulation {
namespace {

mech_control_core::RawCanFrame frame(mech_control_core::FrameDirection direction,
                                     std::int64_t arrival) {
  const auto id = mech_control_core::CanId::create(
      0x120U, mech_control_core::CanFrameFormat::Standard);
  const auto time = mech_control_core::MonotonicTime::from_nanoseconds(arrival);
  const std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{
      1U, 2U, 3U};
  return *mech_control_core::RawCanFrame::create(
      1U, *id, mech_control_core::CanFrameType::Classic, direction, 3U,
      payload, *time);
}

TEST(FakeClock, AdvancesWithoutSleepingAndRejectsBackwardTime) {
  const auto initial = mech_control_core::MonotonicTime::from_nanoseconds(10);
  const auto duration = mech_control_core::MonotonicDuration::from_nanoseconds(5);
  ASSERT_TRUE(initial.has_value());
  ASSERT_TRUE(duration.has_value());
  FakeClock clock(*initial);
  EXPECT_TRUE(clock.advance(*duration));
  EXPECT_EQ(clock.now().nanoseconds(), 15);
  EXPECT_FALSE(clock.set(*initial));
  const auto maximum = mech_control_core::MonotonicDuration::from_nanoseconds(
      std::numeric_limits<std::int64_t>::max());
  ASSERT_TRUE(maximum.has_value());
  EXPECT_FALSE(clock.advance(*maximum));
}

TEST(FakeTransport, PreservesQueueOrderAndBoundedFailure) {
  FakeTransport transport(1U);
  EXPECT_EQ(transport.open(), true);
  const auto rx = frame(mech_control_core::FrameDirection::Rx, 1);
  const auto tx = frame(mech_control_core::FrameDirection::Tx, 2);
  EXPECT_EQ(transport.inject_receive(rx),
            mech_control_core::TransportResult::Ok);
  EXPECT_EQ(transport.inject_receive(rx),
            mech_control_core::TransportResult::QueueFull);
  mech_control_core::RawCanFrame received;
  EXPECT_EQ(transport.try_receive(received),
            mech_control_core::TransportResult::Ok);
  EXPECT_EQ(received.host_arrival.nanoseconds(), 1);
  EXPECT_EQ(transport.try_receive(received),
            mech_control_core::TransportResult::WouldBlock);
  EXPECT_EQ(transport.try_send(tx), mech_control_core::TransportResult::Ok);
  EXPECT_EQ(transport.try_send(tx),
            mech_control_core::TransportResult::QueueFull);
  mech_control_core::RawCanFrame sent;
  ASSERT_TRUE(transport.take_transmit(sent));
  EXPECT_EQ(sent.direction, mech_control_core::FrameDirection::Tx);
}

TEST(FakeTransport, ClosedTransportFailsWithoutDeviceAccess) {
  FakeTransport transport;
  mech_control_core::RawCanFrame output;
  EXPECT_EQ(transport.try_receive(output),
            mech_control_core::TransportResult::Disconnected);
  EXPECT_EQ(transport.try_send(frame(mech_control_core::FrameDirection::Tx, 0)),
            mech_control_core::TransportResult::Disconnected);
}

}  // namespace
}  // namespace mech::mech_simulation
