#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "mech_control_core/frame.hpp"

namespace mech::mech_protocol_ctrboard {

constexpr std::uint32_t kMcuToHostCanId = 0x621U;
constexpr std::uint8_t kHeader1 = 0xEBU;
constexpr std::uint8_t kHeader2 = 0x90U;
constexpr std::size_t kMaxPayloadSize = 255U;

enum class MessageId : std::uint8_t {
  SensorState = 0x05U,
};

#pragma pack(push, 1)
struct ImuSampleWire final {
  float acceleration_g[3];
  float angular_velocity_dps[3];
  float euler_deg[3];
  float quaternion_wxyz[4];
};

struct SensorTelemetryWire final {
  std::uint32_t timestamp_ms;
  ImuSampleWire imu_1;
  ImuSampleWire imu_2;
  std::uint8_t fsr_left_phase;
  std::uint8_t fsr_right_phase;
  std::uint8_t fsr_left_points;
  std::uint8_t fsr_right_points;
  std::uint16_t fsr_left_total;
  std::uint16_t fsr_right_total;
  std::uint16_t fsr_left_raw[20];
  std::uint16_t fsr_right_raw[20];
};
#pragma pack(pop)

static_assert(sizeof(float) == 4U);
static_assert(std::numeric_limits<float>::is_iec559);
static_assert(sizeof(ImuSampleWire) == 52U);
static_assert(sizeof(SensorTelemetryWire) == 196U);

struct Packet final {
  MessageId message_id;
  std::vector<std::uint8_t> payload;
};

[[nodiscard]] std::uint16_t crc16_ccitt(const std::uint8_t* data,
                                        std::size_t size) noexcept;

class StreamDecoder final {
 public:
  [[nodiscard]] std::vector<Packet> feed(
      const mech_control_core::RawCanFrame& frame);
  [[nodiscard]] std::uint64_t crc_errors() const noexcept;
  [[nodiscard]] std::uint64_t discarded_bytes() const noexcept;
  void reset() noexcept;

 private:
  std::vector<std::uint8_t> buffer_;
  std::uint64_t crc_errors_{0U};
  std::uint64_t discarded_bytes_{0U};
};

[[nodiscard]] bool decode_sensor_state(
    const Packet& packet, SensorTelemetryWire& telemetry) noexcept;

}  // namespace mech::mech_protocol_ctrboard
