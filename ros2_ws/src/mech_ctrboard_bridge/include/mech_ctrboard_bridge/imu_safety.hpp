#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mech_protocol_ctrboard/protocol.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mech::mech_ctrboard_bridge {

struct ImuValidity final {
  bool sample_present{false};
  bool orientation_valid{false};
  bool angular_velocity_valid{false};
  bool linear_acceleration_valid{false};
  bool euler_valid{false};

  [[nodiscard]] bool fully_valid() const noexcept;
};

struct PreparedImuMessages final {
  sensor_msgs::msg::Imu imu;
  std::optional<geometry_msgs::msg::Vector3Stamped> euler;
  ImuValidity validity;
};

[[nodiscard]] PreparedImuMessages prepare_imu_messages(
    const mech_protocol_ctrboard::ImuSampleWire& sample,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp);

enum class SensorSafetyState : std::uint8_t {
  WaitingForData,
  Healthy,
  Degraded,
  TransportFault,
  TimedOut,
};

struct SensorSafetySnapshot final {
  SensorSafetyState state{SensorSafetyState::WaitingForData};
  ImuValidity imu_1;
  ImuValidity imu_2;
  std::uint64_t packet_age_ms{0U};
  std::uint64_t crc_errors{0U};
  std::uint64_t discarded_bytes{0U};
};

class SensorSafetyMonitor final {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit SensorSafetyMonitor(
      std::chrono::milliseconds timeout,
      TimePoint start_time = Clock::now());

  void note_packet(const ImuValidity& imu_1, const ImuValidity& imu_2,
                   TimePoint time = Clock::now()) noexcept;
  void note_transport_fault() noexcept;
  void set_decoder_counters(std::uint64_t crc_errors,
                            std::uint64_t discarded_bytes) noexcept;
  [[nodiscard]] SensorSafetySnapshot snapshot(
      TimePoint time = Clock::now()) const noexcept;

 private:
  std::chrono::milliseconds timeout_;
  TimePoint start_time_;
  TimePoint last_packet_time_;
  bool received_packet_{false};
  bool transport_fault_{false};
  ImuValidity imu_1_;
  ImuValidity imu_2_;
  std::uint64_t crc_errors_{0U};
  std::uint64_t discarded_bytes_{0U};
};

[[nodiscard]] const char* to_string(SensorSafetyState state) noexcept;

}  // namespace mech::mech_ctrboard_bridge
