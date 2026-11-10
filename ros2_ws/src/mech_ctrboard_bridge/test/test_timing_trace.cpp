#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "gtest/gtest.h"
#include "mech_ctrboard_bridge/timing_trace.hpp"

namespace {
using mech::mech_ctrboard_bridge::PacketTimingRow;
using mech::mech_ctrboard_bridge::TimingTrace;

std::int64_t fake_time = 100;
std::int64_t clock_value() noexcept { return fake_time; }

std::string text(const std::filesystem::path& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

class TimingTraceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    directory = std::filesystem::temp_directory_path() /
                ("mech_ctrboard_timing_test_" + std::to_string(suffix));
    ASSERT_FALSE(std::filesystem::exists(directory));
    fake_time = 100;
  }
  void TearDown() override {
    for (const auto* name : {"bridge_packets.csv", "bridge_polls.csv", "bridge_meta.json"}) {
      std::filesystem::remove(directory / name);
    }
    std::filesystem::remove(directory);
  }
  std::filesystem::path directory;
};

TEST_F(TimingTraceTest, BoundedPacketsAndDeferredOutput) {
  TimingTrace trace(directory.string(), 2, 1, 1000000, clock_value);
  PacketTimingRow row;
  row.device_ms = 10;
  row.ros_stamp_ns = 123;
  trace.record_packet(row);
  row.device_ms = 20;
  trace.record_packet(row);
  row.device_ms = 30;
  trace.record_packet(row);
  EXPECT_EQ(trace.packet_observed(), 3U);
  EXPECT_TRUE(std::filesystem::is_empty(directory));
  trace.finish();
  const auto csv = text(directory / "bridge_packets.csv");
  EXPECT_NE(csv.find("\n1,10,123,"), std::string::npos);
  EXPECT_NE(csv.find("\n2,20,123,"), std::string::npos);
  EXPECT_EQ(csv.find("\n3,"), std::string::npos);
  const auto meta = text(directory / "bridge_meta.json");
  EXPECT_NE(meta.find("\"packet_observed\": 3"), std::string::npos);
  EXPECT_NE(meta.find("\"packet_stored\": 2"), std::string::npos);
  EXPECT_NE(meta.find("\"packet_overflow\": 1"), std::string::npos);
  trace.finish();
  EXPECT_EQ(csv, text(directory / "bridge_packets.csv"));
}

TEST_F(TimingTraceTest, PollAnomaliesAreBoundedAndTotalCountsAreSeparate) {
  TimingTrace trace(directory.string(), 1, 1, 1000000, clock_value);
  trace.record_poll(1000000, 1010000, 0, 0, 1);
  trace.record_poll(2000000, 2020000, 26, 1, 1);
  trace.record_poll(5000000, 5010000, 0, 0, 1);
  trace.record_poll(6000000, 8000001, 64, 2, 0);
  trace.finish();
  const auto csv = text(directory / "bridge_polls.csv");
  EXPECT_NE(csv.find("\n3,5000000,5010000,2000000,0,0,1"), std::string::npos);
  EXPECT_EQ(csv.find("\n4,"), std::string::npos);
  const auto meta = text(directory / "bridge_meta.json");
  EXPECT_NE(meta.find("\"poll_observed\": 4"), std::string::npos);
  EXPECT_NE(meta.find("\"poll_anomalies\": 2"), std::string::npos);
  EXPECT_NE(meta.find("\"poll_overflow\": 1"), std::string::npos);
  EXPECT_NE(meta.find("\"max_poll_gap_ns\": 3000000"), std::string::npos);
  EXPECT_NE(meta.find("\"max_poll_duration_ns\": 2000001"), std::string::npos);
}

TEST_F(TimingTraceTest, InvalidClockIsExplicitAndExistingEvidenceIsRefused) {
  TimingTrace trace(directory.string(), 1, 1, 1000000, clock_value);
  fake_time = 0;
  EXPECT_EQ(trace.now_ns(), 0);
  fake_time = 123;
  EXPECT_EQ(trace.now_ns(), 123);
  EXPECT_THROW(TimingTrace(directory.string(), 1, 1, 1000000, clock_value),
               std::invalid_argument);
  trace.finish(2);
  const auto meta = text(directory / "bridge_meta.json");
  EXPECT_NE(meta.find("\"clock_errors\": 1"), std::string::npos);
  EXPECT_NE(meta.find("\"serial_read_clock_errors\": 2"), std::string::npos);
}

TEST_F(TimingTraceTest, InvalidConfigurationCreatesNothing) {
  EXPECT_THROW(TimingTrace(directory.string(), 0, 1, 1000000, clock_value),
               std::invalid_argument);
  EXPECT_THROW(TimingTrace(directory.string(), 1000001, 1, 1000000, clock_value),
               std::invalid_argument);
  EXPECT_THROW(TimingTrace("relative_trace", 1, 1, 1000000, clock_value),
               std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(directory));
}
}  // namespace
