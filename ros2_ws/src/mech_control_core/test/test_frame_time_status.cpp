#include "mech_control_core/frame.hpp"
#include "mech_control_core/status.hpp"

#include <array>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace mech::mech_control_core {
namespace {

TEST(CanId, EnforcesStandardAndExtendedBounds) {
  EXPECT_TRUE(CanId::create(kMaxStandardCanId, CanFrameFormat::Standard));
  EXPECT_FALSE(CanId::create(kMaxStandardCanId + 1U, CanFrameFormat::Standard));
  EXPECT_TRUE(CanId::create(kMaxExtendedCanId, CanFrameFormat::Extended));
  EXPECT_FALSE(CanId::create(kMaxExtendedCanId + 1U, CanFrameFormat::Extended));
}

TEST(RawCanFrame, EnforcesClassicAndFlexiblePayloadBounds) {
  const auto id = CanId::create(0x123U, CanFrameFormat::Standard);
  ASSERT_TRUE(id.has_value());
  const std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
  const auto arrival = MonotonicTime::from_nanoseconds(0);
  ASSERT_TRUE(arrival.has_value());

  EXPECT_TRUE(RawCanFrame::create(1U, *id, CanFrameType::Classic,
                                  FrameDirection::Rx, 8U, payload, *arrival));
  EXPECT_FALSE(RawCanFrame::create(1U, *id, CanFrameType::Classic,
                                   FrameDirection::Rx, 9U, payload, *arrival));
  EXPECT_TRUE(RawCanFrame::create(1U, *id, CanFrameType::FlexibleDataRate,
                                  FrameDirection::Rx, 64U, payload, *arrival));
  EXPECT_FALSE(RawCanFrame::create(1U, *id, CanFrameType::FlexibleDataRate,
                                   FrameDirection::Rx, 65U, payload, *arrival));
}

TEST(MonotonicTime, RejectsNegativeAndComputesElapsedWithoutWallClock) {
  EXPECT_FALSE(MonotonicTime::from_nanoseconds(-1).has_value());
  const auto start = MonotonicTime::from_nanoseconds(10);
  const auto end = MonotonicTime::from_nanoseconds(25);
  ASSERT_TRUE(start.has_value());
  ASSERT_TRUE(end.has_value());
  ASSERT_TRUE(elapsed_since(*start, *end).has_value());
  EXPECT_EQ(elapsed_since(*start, *end)->nanoseconds(), 15);
  EXPECT_FALSE(elapsed_since(*end, *start).has_value());
}

TEST(MonotonicTime, PreservesMaximumRepresentableValue) {
  const auto maximum = MonotonicTime::from_nanoseconds(
      std::numeric_limits<std::int64_t>::max());
  ASSERT_TRUE(maximum.has_value());
  EXPECT_EQ(maximum->nanoseconds(), std::numeric_limits<std::int64_t>::max());
}

TEST(StatusSnapshot, RequiresHostArrivalForKnownQuality) {
  EXPECT_TRUE(StatusSnapshot::create(SampleQuality::Unknown,
                                     DeviceState::Unknown, 0U, 0U,
                                     std::nullopt));
  EXPECT_FALSE(StatusSnapshot::create(SampleQuality::Valid, DeviceState::Ready,
                                      0U, 1U, std::nullopt));
  const auto arrival = MonotonicTime::from_nanoseconds(100);
  ASSERT_TRUE(arrival.has_value());
  const auto status = StatusSnapshot::create(
      SampleQuality::Valid, DeviceState::Ready, 0U, 1U, arrival);
  ASSERT_TRUE(status.has_value());
  EXPECT_TRUE(status->has_sample());
}

TEST(StatusSnapshot, PreservesFaultStateAndRawCode) {
  const auto arrival = MonotonicTime::from_nanoseconds(100);
  ASSERT_TRUE(arrival.has_value());
  const auto status = StatusSnapshot::create(
      SampleQuality::Invalid, DeviceState::Fault, 7U, 2U, arrival);
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->raw_fault_code, 7U);
}

TEST(StatusSnapshot, KeepsSourceTimestampSeparate) {
  const auto arrival = MonotonicTime::from_nanoseconds(100);
  ASSERT_TRUE(arrival.has_value());
  const SourceTimestamp source{SourceClockDomain::Device, 42U};
  const auto status = StatusSnapshot::create(
      SampleQuality::Valid, DeviceState::Active, 0U, 3U, arrival, source);
  ASSERT_TRUE(status.has_value());
  ASSERT_TRUE(status->source_timestamp.has_value());
  EXPECT_EQ(status->source_timestamp->ticks, 42U);
  EXPECT_EQ(status->host_rx_time->nanoseconds(), 100);
}

}  // namespace
}  // namespace mech::mech_control_core
