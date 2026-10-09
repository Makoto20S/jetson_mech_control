#include "mech_ctrboard_bridge/imu_safety.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace mech::mech_ctrboard_bridge {
namespace {

constexpr double kGravityMetersPerSecondSquared = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kMinimumQuaternionNorm = 1.0e-6;

template <std::size_t Size>
bool array_is_finite(const float (&values)[Size]) noexcept {
  return std::all_of(std::begin(values), std::end(values),
                     [](float value) { return std::isfinite(value); });
}

template <std::size_t Size>
bool array_has_nonzero_finite_value(const float (&values)[Size]) noexcept {
  return std::any_of(std::begin(values), std::end(values), [](float value) {
    return std::isfinite(value) && value != 0.0F;
  });
}

bool sample_has_data(
    const mech_protocol_ctrboard::ImuSampleWire& sample) noexcept {
  return array_has_nonzero_finite_value(sample.acceleration_g) ||
         array_has_nonzero_finite_value(sample.angular_velocity_dps) ||
         array_has_nonzero_finite_value(sample.euler_deg) ||
         array_has_nonzero_finite_value(sample.quaternion_wxyz);
}

std::uint64_t nonnegative_age_ms(SensorSafetyMonitor::TimePoint newer,
                                 SensorSafetyMonitor::TimePoint older) noexcept {
  if (newer <= older) {
    return 0U;
  }
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(newer - older)
          .count());
}

}  // namespace

bool ImuValidity::fully_valid() const noexcept {
  return sample_present && orientation_valid && angular_velocity_valid &&
         linear_acceleration_valid && euler_valid;
}

PreparedImuMessages prepare_imu_messages(
    const mech_protocol_ctrboard::ImuSampleWire& sample,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp) {
  PreparedImuMessages prepared;
  prepared.imu.header.stamp = stamp;
  prepared.imu.header.frame_id = frame_id;
  prepared.validity.sample_present = sample_has_data(sample);

  const bool quaternion_finite = array_is_finite(sample.quaternion_wxyz);
  double quaternion_norm = 0.0;
  if (quaternion_finite) {
    for (const float value : sample.quaternion_wxyz) {
      quaternion_norm += static_cast<double>(value) * value;
    }
    quaternion_norm = std::sqrt(quaternion_norm);
  }
  prepared.validity.orientation_valid =
      prepared.validity.sample_present && quaternion_finite &&
      quaternion_norm > kMinimumQuaternionNorm;
  if (prepared.validity.orientation_valid) {
    prepared.imu.orientation.w = sample.quaternion_wxyz[0] / quaternion_norm;
    prepared.imu.orientation.x = sample.quaternion_wxyz[1] / quaternion_norm;
    prepared.imu.orientation.y = sample.quaternion_wxyz[2] / quaternion_norm;
    prepared.imu.orientation.z = sample.quaternion_wxyz[3] / quaternion_norm;
  } else {
    prepared.imu.orientation.w = 1.0;
    prepared.imu.orientation_covariance[0] = -1.0;
  }

  prepared.validity.angular_velocity_valid =
      prepared.validity.sample_present &&
      array_is_finite(sample.angular_velocity_dps);
  if (prepared.validity.angular_velocity_valid) {
    prepared.imu.angular_velocity.x =
        sample.angular_velocity_dps[0] * kDegreesToRadians;
    prepared.imu.angular_velocity.y =
        sample.angular_velocity_dps[1] * kDegreesToRadians;
    prepared.imu.angular_velocity.z =
        sample.angular_velocity_dps[2] * kDegreesToRadians;
  } else {
    prepared.imu.angular_velocity_covariance[0] = -1.0;
  }

  prepared.validity.linear_acceleration_valid =
      prepared.validity.sample_present && array_is_finite(sample.acceleration_g);
  if (prepared.validity.linear_acceleration_valid) {
    prepared.imu.linear_acceleration.x =
        sample.acceleration_g[0] * kGravityMetersPerSecondSquared;
    prepared.imu.linear_acceleration.y =
        sample.acceleration_g[1] * kGravityMetersPerSecondSquared;
    prepared.imu.linear_acceleration.z =
        sample.acceleration_g[2] * kGravityMetersPerSecondSquared;
  } else {
    prepared.imu.linear_acceleration_covariance[0] = -1.0;
  }

  prepared.validity.euler_valid =
      prepared.validity.sample_present && array_is_finite(sample.euler_deg) &&
      (prepared.validity.orientation_valid ||
       array_has_nonzero_finite_value(sample.euler_deg));
  if (prepared.validity.euler_valid) {
    geometry_msgs::msg::Vector3Stamped euler;
    euler.header = prepared.imu.header;
    euler.vector.x = sample.euler_deg[0];
    euler.vector.y = sample.euler_deg[1];
    euler.vector.z = sample.euler_deg[2];
    prepared.euler = euler;
  }
  return prepared;
}

SensorSafetyMonitor::SensorSafetyMonitor(std::chrono::milliseconds timeout,
                                         TimePoint start_time)
    : timeout_(timeout),
      start_time_(start_time),
      last_packet_time_(start_time) {
  if (timeout_.count() <= 0) {
    throw std::invalid_argument("sensor timeout must be positive");
  }
}

void SensorSafetyMonitor::note_packet(const ImuValidity& imu_1,
                                      const ImuValidity& imu_2,
                                      TimePoint time) noexcept {
  imu_1_ = imu_1;
  imu_2_ = imu_2;
  last_packet_time_ = time;
  received_packet_ = true;
  transport_fault_ = false;
}

void SensorSafetyMonitor::note_transport_fault() noexcept {
  transport_fault_ = true;
}

void SensorSafetyMonitor::set_decoder_counters(
    std::uint64_t crc_errors, std::uint64_t discarded_bytes) noexcept {
  crc_errors_ = crc_errors;
  discarded_bytes_ = discarded_bytes;
}

SensorSafetySnapshot SensorSafetyMonitor::snapshot(TimePoint time) const noexcept {
  SensorSafetySnapshot result;
  result.imu_1 = imu_1_;
  result.imu_2 = imu_2_;
  result.crc_errors = crc_errors_;
  result.discarded_bytes = discarded_bytes_;
  const TimePoint reference = received_packet_ ? last_packet_time_ : start_time_;
  result.packet_age_ms = nonnegative_age_ms(time, reference);
  if (transport_fault_) {
    result.state = SensorSafetyState::TransportFault;
  } else if (result.packet_age_ms >=
             static_cast<std::uint64_t>(timeout_.count())) {
    result.state = SensorSafetyState::TimedOut;
  } else if (!received_packet_) {
    result.state = SensorSafetyState::WaitingForData;
  } else if (imu_1_.fully_valid() && imu_2_.fully_valid()) {
    result.state = SensorSafetyState::Healthy;
  } else {
    result.state = SensorSafetyState::Degraded;
  }
  return result;
}

const char* to_string(SensorSafetyState state) noexcept {
  switch (state) {
    case SensorSafetyState::WaitingForData:
      return "waiting_for_data";
    case SensorSafetyState::Healthy:
      return "healthy";
    case SensorSafetyState::Degraded:
      return "degraded";
    case SensorSafetyState::TransportFault:
      return "transport_fault";
    case SensorSafetyState::TimedOut:
      return "timed_out";
  }
  return "unknown";
}

}  // namespace mech::mech_ctrboard_bridge
