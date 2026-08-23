#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_simulation/fake_serial.hpp"

#include <array>

#include <gtest/gtest.h>

namespace mech::mech_simulation {
namespace {

mech_control_core::RawCanFrame make_frame(
    mech_control_core::FrameDirection direction,
    mech_control_core::CanFrameType type = mech_control_core::CanFrameType::Classic) {
  const auto id = mech_control_core::CanId::create(
      0x1ABCDEU, mech_control_core::CanFrameFormat::Extended);
  const auto now = mech_control_core::MonotonicTime::from_nanoseconds(42);
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  payload[0] = 0xA5U;
  payload[1] = 0x5AU;
  return *mech_control_core::RawCanFrame::create(
      1U, *id, type, direction, type == mech_control_core::CanFrameType::Classic
                                  ? 2U
                                  : 9U,
      payload, *now);
}

TEST(UsbCdcTransport, InjectedSerialRoundTripHasNoDeviceDependency) {
  FakeSerial serial;
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  ASSERT_EQ(transport.try_send(make_frame(mech_control_core::FrameDirection::Tx)),
            mech_control_core::TransportResult::Ok);
  const auto wire = serial.take_tx();
  ASSERT_FALSE(wire.empty());
  serial.clear_tx();
  ASSERT_TRUE(serial.inject_rx(wire));
  mech_control_core::RawCanFrame received{};
  ASSERT_EQ(transport.try_receive(received), mech_control_core::TransportResult::Ok);
  EXPECT_EQ(received.id.value, 0x1ABCDEU);
  EXPECT_EQ(received.direction, mech_control_core::FrameDirection::Rx);
  EXPECT_EQ(received.payload[0], 0xA5U);
}

TEST(UsbCdcTransport, BatchesFdAndRejectsMalformedCrc) {
  FakeSerial serial;
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  ASSERT_EQ(transport.try_send(
                make_frame(mech_control_core::FrameDirection::Tx,
                           mech_control_core::CanFrameType::FlexibleDataRate)),
            mech_control_core::TransportResult::Ok);
  auto wire = serial.take_tx();
  ASSERT_FALSE(wire.empty());
  wire.back() ^= 0x01U;
  ASSERT_TRUE(serial.inject_rx(wire));
  mech_control_core::RawCanFrame received{};
  EXPECT_EQ(transport.try_receive(received), mech_control_core::TransportResult::WouldBlock);
  EXPECT_GE(transport.stats().rx_dropped, 1U);
}

TEST(UsbCdcTransport, QueueAndDisconnectFailuresAreExplicit) {
  FakeSerial serial(8U);
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 1U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  EXPECT_EQ(transport.try_send(make_frame(mech_control_core::FrameDirection::Tx)),
            mech_control_core::TransportResult::QueueFull);
  transport.close();
  mech_control_core::RawCanFrame received{};
  EXPECT_EQ(transport.try_receive(received), mech_control_core::TransportResult::Disconnected);
}

}  // namespace
}  // namespace mech::mech_simulation
