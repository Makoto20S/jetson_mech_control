#include "mech_control_core/socketcan_transport.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"

#include <array>

#include <gtest/gtest.h>

namespace mech::mech_control_core {
namespace {

RawCanFrame frame(FrameDirection direction, CanFrameType type,
                  CanFrameFormat format, std::uint8_t size = 8U,
                  bool bitrate_switch = false) {
  const auto id = CanId::create(0x123U, format);
  const auto now = MonotonicTime::from_nanoseconds(10);
  std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  for (std::size_t index = 0U; index < size; ++index) {
    payload[index] = static_cast<std::uint8_t>(index + 1U);
  }
  return *RawCanFrame::create(1U, *id, type, direction, size, payload, *now,
                              std::nullopt, bitrate_switch);
}

TEST(UsbCdcCodec, RoundTripsClassicStandardFrame) {
  std::array<std::uint8_t, 528U> encoded{};
  std::size_t encoded_size = 0U;
  ASSERT_TRUE(UsbCdcCodec::encode(
      frame(FrameDirection::Tx, CanFrameType::Classic,
            CanFrameFormat::Standard, 8U),
      encoded, encoded_size));
  CdcFrameBatch decoded;
  ASSERT_TRUE(UsbCdcCodec::decode(
      encoded.data(), encoded_size, 1U,
      *MonotonicTime::from_nanoseconds(20), decoded));
  ASSERT_EQ(decoded.size, 1U);
  EXPECT_EQ(decoded.frames[0].id.value, 0x123U);
  EXPECT_EQ(decoded.frames[0].id.format, CanFrameFormat::Standard);
  EXPECT_EQ(decoded.frames[0].payload_size, 8U);
  EXPECT_EQ(decoded.frames[0].payload[7], 8U);
}

TEST(UsbCdcCodec, SupportsExtendedFdBatchedPayloadAndRejectsBadCrc) {
  std::array<std::uint8_t, 528U> encoded{};
  std::size_t encoded_size = 0U;
  ASSERT_TRUE(UsbCdcCodec::encode(
      frame(FrameDirection::Tx, CanFrameType::FlexibleDataRate,
            CanFrameFormat::Extended, 9U, true),
      encoded, encoded_size));
  EXPECT_EQ(encoded[0], UsbCdcCodec::kHeader);
  EXPECT_EQ(encoded[1], UsbCdcCodec::kPassCommand);
  EXPECT_EQ(encoded_size, 7U + 6U + 12U);
  encoded[encoded_size - 1U] ^= 0x01U;
  CdcFrameBatch decoded;
  EXPECT_FALSE(UsbCdcCodec::decode(
      encoded.data(), encoded_size, 1U,
      *MonotonicTime::from_nanoseconds(20), decoded));
}

TEST(UsbCdcCodec, RejectsUnknownAndTooOldBoardVersions) {
  EXPECT_FALSE(UsbCdcCodec::supports_version(CdcProtocolVersion{0U, 0U, 0U}));
  EXPECT_FALSE(UsbCdcCodec::supports_version(CdcProtocolVersion{4U, 8U, 7U}));
  EXPECT_TRUE(UsbCdcCodec::supports_version(CdcProtocolVersion{4U, 8U, 8U}));
  EXPECT_TRUE(UsbCdcCodec::supports_version(CdcProtocolVersion{5U, 0U, 0U}));
}

TEST(SocketCanTransport, InvalidInterfaceDoesNotOpenDevice) {
  SocketCanOptions options;
  options.interface_name = "interface-that-does-not-exist";
  options.logical_bus = 1U;
  options.nominal_bitrate_hz = 1000000U;
  SocketCanTransport transport(options);
  EXPECT_FALSE(transport.is_open());
  EXPECT_FALSE(transport.open());
  EXPECT_FALSE(transport.is_open());
  RawCanFrame received{};
  EXPECT_EQ(transport.try_receive(received), TransportResult::Disconnected);
}

TEST(SocketCanTransport, RejectsInvalidCapabilityBeforeSocketCall) {
  SocketCanOptions options;
  options.interface_name = "vcan0";
  options.logical_bus = 1U;
  SocketCanTransport transport(options);
  EXPECT_FALSE(transport.capabilities().is_valid());
  EXPECT_FALSE(transport.open());
}

}  // namespace
}  // namespace mech::mech_control_core
