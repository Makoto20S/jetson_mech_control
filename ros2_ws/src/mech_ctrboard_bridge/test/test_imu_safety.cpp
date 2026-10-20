#include "mech_ctrboard_bridge/imu_safety.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

namespace mech::mech_ctrboard_bridge {
namespace {

using mech_protocol_ctrboard::ImuSampleWire;
using namespace std::chrono_literals;

ImuSampleWire valid_stationary_sample() {
  ImuSampleWire sample{};
  sample.acceleration_g[2] = 1.0F;
  sample.quaternion_wxyz[0] = 1.0F;
  return sample;
}

TEST(ImuSafety, MarksAnAllZeroPlaceholderUnavailable) {
  const auto prepared = prepare_imu_messages(
      ImuSampleWire{}, "imu", builtin_interfaces::msg::Time());
  EXPECT_FALSE(prepared.validity.sample_present);
  EXPECT_FALSE(prepared.euler.has_value());
  EXPECT_EQ(prepared.imu.orientation_covariance[0], -1.0);
  EXPECT_EQ(prepared.imu.angular_velocity_covariance[0], -1.0);
  EXPECT_EQ(prepared.imu.linear_acceleration_covariance[0], -1.0);
}

TEST(ImuSafety, KeepsIndependentMeasurementsWhenOrientationIsInvalid) {
  ImuSampleWire sample{};
  sample.acceleration_g[2] = 1.0F;
  sample.angular_velocity_dps[1] = 12.0F;
  sample.euler_deg[1] = 23.0F;
  const auto prepared = prepare_imu_messages(
      sample, "imu", builtin_interfaces::msg::Time());
  EXPECT_TRUE(prepared.validity.sample_present);
  EXPECT_FALSE(prepared.validity.orientation_valid);
  EXPECT_TRUE(prepared.validity.angular_velocity_valid);
  EXPECT_TRUE(prepared.validity.linear_acceleration_valid);
  ASSERT_TRUE(prepared.euler.has_value());
  EXPECT_EQ(prepared.imu.orientation_covariance[0], -1.0);
  EXPECT_EQ(prepared.imu.angular_velocity_covariance[0], 0.0);
  EXPECT_EQ(prepared.imu.linear_acceleration_covariance[0], 0.0);
}

TEST(ImuSafety, AcceptsZeroAngularVelocityFromAStationaryImu) {
  const auto prepared =
      prepare_imu_messages(valid_stationary_sample(), "imu",
                           builtin_interfaces::msg::Time());
  EXPECT_TRUE(prepared.validity.fully_valid());
  EXPECT_EQ(prepared.imu.angular_velocity.x, 0.0);
  EXPECT_EQ(prepared.imu.angular_velocity.y, 0.0);
  EXPECT_EQ(prepared.imu.angular_velocity.z, 0.0);
  EXPECT_EQ(prepared.imu.angular_velocity_covariance[0], 0.0);
  EXPECT_TRUE(prepared.euler.has_value());
}

TEST(ImuSafety, HandlesOneMissingAndOneValidImuIndependently) {
  const auto missing = prepare_imu_messages(
      ImuSampleWire{}, "imu_1", builtin_interfaces::msg::Time());
  const auto valid =
      prepare_imu_messages(valid_stationary_sample(), "imu_2",
                           builtin_interfaces::msg::Time());
  EXPECT_FALSE(missing.validity.sample_present);
  EXPECT_FALSE(missing.euler.has_value());
  EXPECT_TRUE(valid.validity.fully_valid());
  EXPECT_TRUE(valid.euler.has_value());
}

TEST(ImuSafety, DoesNotPublishNonFiniteAngularVelocity) {
  auto sample = valid_stationary_sample();
  sample.angular_velocity_dps[0] = std::numeric_limits<float>::quiet_NaN();
  const auto prepared = prepare_imu_messages(
      sample, "imu", builtin_interfaces::msg::Time());
  EXPECT_FALSE(prepared.validity.angular_velocity_valid);
  EXPECT_EQ(prepared.imu.angular_velocity_covariance[0], -1.0);
  EXPECT_TRUE(std::isfinite(prepared.imu.angular_velocity.x));
  EXPECT_TRUE(prepared.validity.orientation_valid);
  EXPECT_TRUE(prepared.validity.linear_acceleration_valid);
}

TEST(SensorSafetyMonitor, ReportsWaitingTimeoutDegradedFaultAndRecovery) {
  const auto start = SensorSafetyMonitor::TimePoint{};
  SensorSafetyMonitor monitor(500ms, start);
  EXPECT_EQ(monitor.snapshot(start + 100ms).state,
            SensorSafetyState::WaitingForData);
  EXPECT_EQ(monitor.snapshot(start + 500ms).state,
            SensorSafetyState::TimedOut);

  const ImuValidity valid{true, true, true, true, true};
  const ImuValidity missing{};
  monitor.note_packet(valid, missing, start + 600ms);
  EXPECT_EQ(monitor.snapshot(start + 700ms).state,
            SensorSafetyState::Degraded);
  monitor.note_transport_fault();
  EXPECT_EQ(monitor.snapshot(start + 701ms).state,
            SensorSafetyState::TransportFault);
  monitor.note_packet(valid, valid, start + 800ms);
  EXPECT_EQ(monitor.snapshot(start + 900ms).state,
            SensorSafetyState::Healthy);
  EXPECT_EQ(monitor.snapshot(start + 1301ms).state,
            SensorSafetyState::TimedOut);
}

}  // namespace
}  // namespace mech::mech_ctrboard_bridge
