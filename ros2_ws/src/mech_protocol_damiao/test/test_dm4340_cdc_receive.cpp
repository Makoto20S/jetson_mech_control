#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

namespace mech::mech_protocol_damiao {
namespace {

std::uint8_t crc8(const std::uint8_t* data, std::size_t size) {
  std::uint8_t crc = 0xFFU;
  for (std::size_t i = 0U; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? static_cast<std::uint8_t>((crc >> 1U) ^ 0x8CU)
                            : static_cast<std::uint8_t>(crc >> 1U);
    }
  }
  return crc;
}

std::uint16_t crc16(const std::uint8_t* data, std::size_t size) {
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0U; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

TEST(Dm4340CdcReceive, GatewayStandardFrameReachesMitDecoder) {
  using namespace mech_control_core;
  // Firmware 4.8.8 receive layout: 3-byte ID prefix, then the 14-byte
  // Classic-CAN record. Values are synthetic; this is not a motor capture.
  std::array<std::uint8_t, 24U> packet{
      0xF7U, 0x12U, 0x11U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x11U, 0x00U, 0x00U, 0x11U, 0x00U, 0x00U, 0x00U,
      0x08U, 0x08U, 0x01U, 0x7FU, 0xFFU, 0x7FU, 0xF7U,
      0xFFU, 0x00U, 0x00U};
  packet[4] = crc8(packet.data() + 1U, 3U);
  const auto checksum = crc16(packet.data() + 7U, packet.size() - 7U);
  packet[5] = static_cast<std::uint8_t>(checksum & 0xFFU);
  packet[6] = static_cast<std::uint8_t>(checksum >> 8U);

  CdcFrameBatch batch{};
  ASSERT_TRUE(UsbCdcCodec::decode(
      packet.data(), packet.size(), 1U,
      *MonotonicTime::from_nanoseconds(20), batch));
  ASSERT_EQ(batch.size, 1U);
  EXPECT_EQ(batch.frames[0].id.format, CanFrameFormat::Standard);
  EXPECT_EQ(batch.frames[0].id.value, 0x11U);
  EXPECT_EQ(batch.frames[0].direction, FrameDirection::Rx);

  MitFeedback feedback{};
  ASSERT_TRUE(decode_mit_feedback(batch.frames[0], 1U, 0x11U, 1U,
                                  kSdkDm4340Limits, feedback));
  EXPECT_NEAR(feedback.position_rad, 0.0, 0.001);
  EXPECT_EQ(feedback.raw_fault, 0U);
  EXPECT_FALSE(decode_mit_feedback(batch.frames[0], 1U, 0x12U, 1U,
                                   kSdkDm4340Limits, feedback));
}

}  // namespace
}  // namespace mech::mech_protocol_damiao
