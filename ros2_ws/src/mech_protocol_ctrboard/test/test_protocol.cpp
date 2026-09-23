#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "mech_protocol_ctrboard/protocol.hpp"

namespace {

using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::FrameDirection;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::RawCanFrame;
using namespace mech::mech_protocol_ctrboard;

MonotonicTime timestamp() {
  return *MonotonicTime::from_nanoseconds(1'000'000);
}

std::vector<std::uint8_t> make_packet(const SensorTelemetryWire& telemetry) {
  const auto* payload = reinterpret_cast<const std::uint8_t*>(&telemetry);
  std::vector<std::uint8_t> bytes{
      kHeader1, kHeader2, static_cast<std::uint8_t>(MessageId::SensorState),
      static_cast<std::uint8_t>(sizeof(telemetry))};
  bytes.insert(bytes.end(), payload, payload + sizeof(telemetry));
  const auto crc = crc16_ccitt(bytes.data(), bytes.size());
  bytes.push_back(static_cast<std::uint8_t>(crc & 0xFFU));
  bytes.push_back(static_cast<std::uint8_t>(crc >> 8U));
  return bytes;
}

std::vector<RawCanFrame> make_receive_frames(
    const std::vector<std::uint8_t>& bytes,
    std::uint32_t can_id = kMcuToHostCanId) {
  std::vector<RawCanFrame> frames;
  const auto id = *CanId::create(can_id, CanFrameFormat::Standard);
  for (std::size_t offset = 0U; offset < bytes.size(); offset += 8U) {
    std::array<std::uint8_t, mech::mech_control_core::kMaxCanPayloadBytes>
        payload{};
    const auto size = std::min<std::size_t>(8U, bytes.size() - offset);
    std::copy_n(bytes.begin() + static_cast<std::ptrdiff_t>(offset), size,
                payload.begin());
    frames.push_back(*RawCanFrame::create(
        1U, id, CanFrameType::Classic, FrameDirection::Rx,
        static_cast<std::uint8_t>(size), payload, timestamp()));
  }
  return frames;
}

std::vector<Packet> feed_all(StreamDecoder& decoder,
                             const std::vector<RawCanFrame>& frames) {
  std::vector<Packet> packets;
  for (const auto& frame : frames) {
    auto decoded = decoder.feed(frame);
    packets.insert(packets.end(), decoded.begin(), decoded.end());
  }
  return packets;
}

TEST(CtrBoardProtocol, SensorStateRoundTripsAcrossTwentySixFrames) {
  SensorTelemetryWire telemetry{};
  telemetry.timestamp_ms = 1234U;
  telemetry.imu_1.acceleration_g[2] = 1.0F;
  telemetry.imu_2.euler_deg[1] = 42.5F;
  telemetry.imu_2.quaternion_wxyz[0] = 1.0F;
  telemetry.fsr_left_points = 20U;
  telemetry.fsr_right_total = 2048U;
  telemetry.fsr_right_raw[19] = 255U;

  const auto frames = make_receive_frames(make_packet(telemetry));
  ASSERT_EQ(frames.size(), 26U);

  StreamDecoder decoder;
  const auto packets = feed_all(decoder, frames);
  ASSERT_EQ(packets.size(), 1U);
  SensorTelemetryWire recovered{};
  ASSERT_TRUE(decode_sensor_state(packets.front(), recovered));
  EXPECT_EQ(recovered.timestamp_ms, telemetry.timestamp_ms);
  EXPECT_FLOAT_EQ(recovered.imu_2.euler_deg[1],
                  telemetry.imu_2.euler_deg[1]);
  EXPECT_EQ(recovered.fsr_left_points, telemetry.fsr_left_points);
  EXPECT_EQ(recovered.fsr_right_raw[19], telemetry.fsr_right_raw[19]);
}

TEST(CtrBoardProtocol, CorruptCrcIsRejectedAndNextPacketResynchronizes) {
  SensorTelemetryWire telemetry{};
  telemetry.timestamp_ms = 7U;
  auto corrupted = make_packet(telemetry);
  corrupted.back() ^= 0x5AU;
  const auto good = make_packet(telemetry);
  corrupted.insert(corrupted.end(), good.begin(), good.end());

  StreamDecoder decoder;
  const auto packets = feed_all(decoder, make_receive_frames(corrupted));
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_EQ(decoder.crc_errors(), 1U);
  SensorTelemetryWire recovered{};
  ASSERT_TRUE(decode_sensor_state(packets.front(), recovered));
  EXPECT_EQ(recovered.timestamp_ms, telemetry.timestamp_ms);
}

TEST(CtrBoardProtocol, UnrelatedCanIdIsIgnored) {
  SensorTelemetryWire telemetry{};
  StreamDecoder decoder;
  EXPECT_TRUE(
      feed_all(decoder, make_receive_frames(make_packet(telemetry), 0x123U))
          .empty());
}

TEST(CtrBoardProtocol, WrongMessageIdDoesNotDecodeAsSensorState) {
  SensorTelemetryWire telemetry{};
  auto bytes = make_packet(telemetry);
  bytes[2] = 0x7FU;
  const auto crc = crc16_ccitt(bytes.data(), bytes.size() - 2U);
  bytes[bytes.size() - 2U] = static_cast<std::uint8_t>(crc & 0xFFU);
  bytes.back() = static_cast<std::uint8_t>(crc >> 8U);

  StreamDecoder decoder;
  const auto packets = feed_all(decoder, make_receive_frames(bytes));
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_FALSE(decode_sensor_state(packets.front(), telemetry));
}

}  // namespace
