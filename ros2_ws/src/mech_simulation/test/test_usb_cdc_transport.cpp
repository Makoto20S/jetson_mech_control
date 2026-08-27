#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_simulation/fake_serial.hpp"

#include <array>
#include <optional>
#include <vector>

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

// Defect 1: a frame stamped for a different logical bus must never reach the
// wire. adapter_template.hpp exposes try_send() directly to device sessions,
// bypassing BusRuntime::submit()'s own logical_bus check, so the transport
// itself is the last line of defence -- exactly like SocketCanTransport.
TEST(UsbCdcTransport, RejectsCrossLogicalBusSendAndWritesNothing) {
  FakeSerial serial;
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  auto frame = make_frame(mech_control_core::FrameDirection::Tx);
  frame.logical_bus = 7U;
  EXPECT_EQ(transport.try_send(frame), mech_control_core::TransportResult::Invalid);
  EXPECT_TRUE(serial.take_tx().empty());
  EXPECT_GE(transport.stats().errors, 1U);
}

// Defect 1 (RTR half): the codec has no wire representation for a remote
// frame, so try_send() must reject one rather than silently encode it as a
// data frame carrying whatever bytes happen to be in the (zero-filled)
// payload.
TEST(UsbCdcTransport, RejectsRemoteRequestFrame) {
  FakeSerial serial;
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  const auto id = mech_control_core::CanId::create(
      0x123U, mech_control_core::CanFrameFormat::Standard);
  std::array<std::uint8_t, mech_control_core::kMaxCanPayloadBytes> payload{};
  const auto frame = mech_control_core::RawCanFrame::create(
      1U, *id, mech_control_core::CanFrameType::Classic,
      mech_control_core::FrameDirection::Tx, 4U, payload,
      *mech_control_core::MonotonicTime::from_nanoseconds(1), std::nullopt,
      false, true);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(transport.try_send(*frame), mech_control_core::TransportResult::Invalid);
  EXPECT_TRUE(serial.take_tx().empty());
}

// Defect 2: encode() used to compute a throwaway CRC over bytes it had not
// written yet, then unconditionally overwrite it. Removing that dead
// computation must not change the emitted bytes: round-trip through the
// codec (send -> wire -> receive) must still succeed byte-for-byte.
TEST(UsbCdcTransport, EncodeCrcIsUnchangedAfterRemovingDeadComputation) {
  FakeSerial serial;
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  ASSERT_EQ(transport.try_send(make_frame(mech_control_core::FrameDirection::Tx)),
            mech_control_core::TransportResult::Ok);
  const auto wire = serial.take_tx();
  ASSERT_GE(wire.size(), 7U);
  // Bytes [5,6] are the CRC-16 over the payload starting at byte 7. Recompute
  // it independently (mirroring the codec's own polynomial) and confirm it
  // matches what was actually written to the wire -- i.e. the surviving
  // (second) computation, not a stale first one.
  auto crc16 = [](const std::uint8_t* data, std::size_t size) noexcept {
    std::uint16_t crc = 0xFFFFU;
    while (size-- > 0U) {
      crc = static_cast<std::uint16_t>(crc ^ *data++);
      for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 1U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                               : static_cast<std::uint16_t>(crc >> 1U);
      }
    }
    return crc;
  };
  const auto payload_length =
      static_cast<std::size_t>(wire[2]) | (static_cast<std::size_t>(wire[3]) << 8U);
  ASSERT_EQ(wire.size(), payload_length + 7U);
  const auto expected_crc = crc16(wire.data() + 7U, payload_length);
  const auto actual_crc = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(wire[5]) | (static_cast<std::uint16_t>(wire[6]) << 8U));
  EXPECT_EQ(actual_crc, expected_crc);
  // And the round trip must still succeed end to end.
  serial.clear_tx();
  ASSERT_TRUE(serial.inject_rx(wire));
  mech_control_core::RawCanFrame received{};
  EXPECT_EQ(transport.try_receive(received), mech_control_core::TransportResult::Ok);
}

// Defect 3: a read that legitimately returns Ok with size == 0 ("no data
// right now") must surface as WouldBlock, not Invalid -- BusRuntime treats
// Invalid as a fault, and a benign empty read must never fault the bus.
TEST(UsbCdcTransport, OkZeroSizeReadIsWouldBlockNotInvalid) {
  FakeSerial serial;
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  serial.force_next_read(mech_control_core::TransportResult::Ok, 0U);
  mech_control_core::RawCanFrame received{};
  EXPECT_EQ(transport.try_receive(received), mech_control_core::TransportResult::WouldBlock);
  EXPECT_EQ(transport.stats().rx_dropped, 0U);
  EXPECT_EQ(transport.stats().errors, 0U);
}

// Defect 3 (overflow half): a genuine overflow is a real error (Invalid),
// but it must compact/reset pending_size_ so the transport recovers
// afterwards instead of getting permanently wedged. The old code's overflow
// branch incremented rx_dropped and returned Invalid but never touched
// pending_size_, so any bytes already buffered before the overflow stayed
// there -- silently corrupting/blocking every frame received afterwards.
// To exercise that, pending_ must hold a genuine partial frame (legitimate
// traffic caps a single frame at kMaxPayload==512 bytes, far below
// pending_'s 4096-byte capacity, so overflow can only be provoked here via a
// driver that over-reports its own read size -- exactly the case the guard
// defends against).
TEST(UsbCdcTransport, PendingOverflowRecoversInsteadOfWedging) {
  FakeSerial serial(8192U);
  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  // Fabricate a well-formed frame so a subsequent read can prove the
  // transport is genuinely alive again, not just returning WouldBlock.
  ASSERT_EQ(transport.try_send(make_frame(mech_control_core::FrameDirection::Tx)),
            mech_control_core::TransportResult::Ok);
  const auto good_wire = serial.take_tx();
  ASSERT_FALSE(good_wire.empty());
  serial.clear_tx();

  // Step 1: prime pending_ with a genuine partial frame: a valid header
  // announcing a 500-byte payload (total 507 bytes, under kMaxPayload+7==519
  // so it is accepted as "wait for more", not dropped outright), but only
  // 100 bytes actually delivered. pending_size_ becomes 100 and the
  // transport legitimately reports WouldBlock while it waits for the rest.
  std::vector<std::uint8_t> partial_frame(100U, 0U);
  partial_frame[0] = mech_control_core::UsbCdcCodec::kHeader;
  partial_frame[1] = mech_control_core::UsbCdcCodec::kPassCommand;
  partial_frame[2] = static_cast<std::uint8_t>(500U & 0xFFU);
  partial_frame[3] = static_cast<std::uint8_t>(500U >> 8U);
  ASSERT_TRUE(serial.inject_rx(partial_frame));
  mech_control_core::RawCanFrame received{};
  EXPECT_EQ(transport.try_receive(received), mech_control_core::TransportResult::WouldBlock);
  EXPECT_EQ(transport.stats().rx_dropped, 0U);

  // Step 2: a driver that over-reports its own read size (claims 2000 bytes
  // into a 1024-byte request) -- pending_size_ (100) + size (2000) overflows
  // pending_'s 4096-byte capacity is not required here; size (2000) alone
  // already exceeds read_capacity (1024), which is itself a real fault.
  ASSERT_TRUE(serial.inject_rx(std::vector<std::uint8_t>(2000U, 0xEEU)));
  serial.force_next_read(mech_control_core::TransportResult::Ok, 2000U,
                         /*report_size_verbatim=*/true);
  EXPECT_EQ(transport.try_receive(received), mech_control_core::TransportResult::Invalid);
  EXPECT_GE(transport.stats().rx_dropped, 1U);

  // Step 3: the transport must have recovered -- pending_size_ must not
  // still hold the stale 100-byte partial frame. Injecting a clean,
  // well-formed frame now (a normal, truthful read) must be received
  // successfully rather than the 100 stale bytes permanently misaligning
  // (or the overflow error repeating forever on) every frame after it.
  ASSERT_TRUE(serial.inject_rx(good_wire));
  bool recovered = false;
  for (int attempt = 0; attempt < 4 && !recovered; ++attempt) {
    const auto result = transport.try_receive(received);
    if (result == mech_control_core::TransportResult::Ok) {
      recovered = true;
    } else {
      ASSERT_NE(result, mech_control_core::TransportResult::Invalid)
          << "transport is wedged returning Invalid forever";
    }
  }
  EXPECT_TRUE(recovered);
  EXPECT_EQ(received.payload[0], 0xA5U);
}

// Defect 5: rx_ used to be a std::deque<RawCanFrame> (per-frame heap
// churn). The fixed-capacity ring buffer replacing it must preserve FIFO
// ordering and the exact same drop-on-full accounting: frames beyond
// receive_queue_capacity are dropped (rx_dropped, queue_full) while earlier
// frames already queued remain deliverable in order.
TEST(UsbCdcTransport, RingBufferOrderingAndDropOnFullMatchesOldDeque) {
  FakeSerial serial;
  // Two frames worth of wire bytes will be injected together, decoded into
  // one CdcFrameBatch, and queued into a receive_queue_capacity == 1 ring so
  // the second frame must be dropped as queue_full while the first is still
  // delivered.
  mech_control_core::UsbCdcTransport producer(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 4U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(producer.open());
  auto first = make_frame(mech_control_core::FrameDirection::Tx);
  first.payload[0] = 0x11U;
  ASSERT_EQ(producer.try_send(first), mech_control_core::TransportResult::Ok);
  auto second = make_frame(mech_control_core::FrameDirection::Tx);
  second.payload[0] = 0x22U;
  ASSERT_EQ(producer.try_send(second), mech_control_core::TransportResult::Ok);
  const auto wire = serial.take_tx();
  ASSERT_FALSE(wire.empty());

  mech_control_core::UsbCdcTransport transport(
      serial, mech_control_core::UsbCdcOptions{1U, 1000000U, 1U, 1024U,
                                               mech_control_core::CdcProtocolVersion{4U, 8U, 8U}});
  ASSERT_TRUE(transport.open());
  ASSERT_TRUE(serial.inject_rx(wire));

  mech_control_core::RawCanFrame received{};
  ASSERT_EQ(transport.try_receive(received), mech_control_core::TransportResult::Ok);
  EXPECT_EQ(received.payload[0], 0x11U);
  EXPECT_GE(transport.stats().queue_full, 1U);
  EXPECT_GE(transport.stats().rx_dropped, 1U);
  // The dropped second frame must not be deliverable later: the queue is
  // empty and no more bytes are pending.
  EXPECT_EQ(transport.try_receive(received), mech_control_core::TransportResult::WouldBlock);
}

}  // namespace
}  // namespace mech::mech_simulation
