#pragma once

#include <cstdint>
#include <optional>
#include <limits>

#include "mech_control_core/adapter_template.hpp"
#include "mech_protocol_cubemars/ak30_servo_wire.hpp"

namespace mech::mech_protocol_cubemars {

// Offline fixture only. Neither affine transform implies a verified shaft,
// origin, gear ratio, or relationship to the other transform.
struct ServoPositionAffine final {
  double scale{0.0};
  double offset{0.0};
  bool evidence_declared{false};
};

struct ServoPositionSessionConfig final {
  std::uint16_t drive_id{0U};
  std::uint16_t logical_bus{0U};
  mech_control_core::ProtocolProfile profile{
      mech_control_core::ProtocolProfile::Ak30ServoExtended};
  ServoPositionAffine target_rad_to_deg{};
  ServoPositionAffine feedback_deg_to_rad{};
  double speed_erpm{0.0};
  double acceleration_raw{0.0};
  double position_min_rad{0.0};
  double position_max_rad{0.0};
  double max_target_error_rad{0.0};
  std::int64_t command_ttl_ns{0};
  std::int64_t command_hard_ttl_ns{0};
  std::int64_t feedback_ttl_ns{0};
};

enum class ServoPositionAvailability : std::uint8_t {
  Unknown, Fresh, Stale, Invalid, DisableAcknowledged, Fault
};

struct ServoPositionSnapshot final {
  double position_rad{std::numeric_limits<double>::quiet_NaN()};
  double electrical_speed_erpm{std::numeric_limits<double>::quiet_NaN()};
  double current_iq_a{std::numeric_limits<double>::quiet_NaN()};
  ServoPositionAvailability availability{ServoPositionAvailability::Unknown};
  std::uint8_t raw_status{0U};
  std::uint64_t sequence{0U};
  std::optional<mech_control_core::MonotonicTime> host_rx_time;
};

// Offline position building block: owns no Transport, never opens or writes a
// bus. A caller must cancel this route on every preparation/admission failure.
class Ak30ServoPositionSession final {
 public:
  [[nodiscard]] mech_control_core::AdapterResult configure(
      const ServoPositionSessionConfig& config) noexcept;
  [[nodiscard]] mech_control_core::AdapterResult activate(
      mech_control_core::MonotonicTime epoch_start) noexcept;
  void deactivate() noexcept;
  [[nodiscard]] mech_control_core::AdapterResult accept_feedback(
      const mech_control_core::RawCanFrame& frame,
      mech_control_core::MonotonicTime now) noexcept;
  [[nodiscard]] ServoPositionSnapshot snapshot(
      mech_control_core::MonotonicTime now) const noexcept;
  [[nodiscard]] mech_control_core::AdapterResult prepare_position(
      const mech_control_core::CanonicalDeviceCommand& command,
      mech_control_core::MonotonicTime now,
      mech_control_core::RawCanFrame& output) noexcept;
  // Recheck the last prepared frame immediately before bus transmit, including
  // freshness and target error against the newest accepted feedback. Read-only;
  // does not consume a generation, send, or cancel a bus lease.
  [[nodiscard]] bool pending_target_still_authorized(
      const mech_control_core::RawCanFrame& frame,
      std::uint64_t generation,
      mech_control_core::MonotonicTime deadline,
      mech_control_core::MonotonicTime now) const noexcept;
  [[nodiscard]] bool fault_latched() const noexcept { return fault_latched_; }

 private:
  void invalidate_prepared() noexcept {
    last_prepared_.reset();
    last_prepared_deadline_.reset();
  }

  ServoPositionSessionConfig config_{};
  bool configured_{false};
  bool active_{false};
  bool fault_latched_{false};
  mech_control_core::MonotonicTime epoch_start_{};
  std::optional<mech_control_core::MonotonicTime> last_arrival_;
  ServoPositionSnapshot sample_{};
  std::uint64_t last_generation_{0U};
  std::optional<mech_control_core::RawCanFrame> last_prepared_;
  std::optional<mech_control_core::MonotonicTime> last_prepared_deadline_;
};

}  // namespace mech::mech_protocol_cubemars
