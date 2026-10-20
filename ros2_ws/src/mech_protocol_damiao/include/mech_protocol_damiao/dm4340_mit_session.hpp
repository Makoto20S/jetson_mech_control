#pragma once

#include <cstdint>
#include <limits>
#include <optional>

#include "mech_control_core/adapter_template.hpp"
#include "mech_protocol_damiao/dm4340_mit_wire.hpp"

namespace mech::mech_protocol_damiao {

struct JointPositionMap final {
  // Joint radians = motor radians * scale + offset. The same mapping is
  // inverted for targets; it must be established on the attached mechanism.
  double scale{0.0};
  double offset_rad{0.0};
  bool evidence_declared{false};
};

struct Dm4340MitSessionConfig final {
  mech_control_core::ProtocolProfile profile{
      mech_control_core::ProtocolProfile::Unknown};
  std::uint16_t logical_bus{0U};
  std::uint32_t command_can_id{0U};
  std::uint32_t feedback_can_id{0U};
  std::uint8_t motor_id{0U};
  std::uint32_t firmware_id{0U};
  std::uint32_t firmware_min{0U};
  std::uint32_t firmware_max{0U};
  bool firmware_evidence_declared{false};
  MitLimits limits{};
  bool limits_evidence_declared{false};
  JointPositionMap joint{};
  double position_min_rad{0.0};
  double position_max_rad{0.0};
  double max_target_error_rad{0.0};
  double kp{0.0};
  double kd{0.0};
  // Optional bench-calibrated MIT torque feedforward; sign follows motor velocity.
  double velocity_feedforward_nm{0.0};
  // Zero disables velocity-mode preparation; position mode remains unchanged.
  double max_command_velocity_rad_s{0.0};
  double max_command_effort_nm{0.0};
  double max_feedback_velocity_rad_s{0.0};
  double max_feedback_torque_nm{0.0};
  std::int64_t command_ttl_ns{0};
  std::int64_t feedback_ttl_ns{0};
};

enum class DmFeedbackAvailability : std::uint8_t {
  Unknown, Fresh, Stale, Invalid, Fault
};

struct Dm4340MitSnapshot final {
  double joint_position_rad{std::numeric_limits<double>::quiet_NaN()};
  double raw_motor_position_rad{std::numeric_limits<double>::quiet_NaN()};
  double raw_motor_velocity_rad_s{std::numeric_limits<double>::quiet_NaN()};
  double raw_motor_torque_nm{std::numeric_limits<double>::quiet_NaN()};
  DmFeedbackAvailability availability{DmFeedbackAvailability::Unknown};
  std::uint8_t raw_fault{0U};
  std::uint64_t sequence{0U};
  std::optional<mech_control_core::MonotonicTime> host_rx_time;
};

// Offline, single-device position session. No Transport and no enable command.
// The bus owner must cancel pending leases whenever feedback or authorization
// is rejected; preparation alone never causes transmission.
class Dm4340MitSession final {
 public:
  [[nodiscard]] mech_control_core::AdapterResult configure(
      const Dm4340MitSessionConfig& config) noexcept;
  [[nodiscard]] mech_control_core::AdapterResult activate(
      mech_control_core::MonotonicTime epoch_start) noexcept;
  void deactivate() noexcept;
  [[nodiscard]] mech_control_core::AdapterResult accept_feedback(
      const mech_control_core::RawCanFrame& frame,
      mech_control_core::MonotonicTime now) noexcept;
  [[nodiscard]] Dm4340MitSnapshot snapshot(
      mech_control_core::MonotonicTime now) const noexcept;
  [[nodiscard]] mech_control_core::AdapterResult prepare_position(
      const mech_control_core::CanonicalDeviceCommand& command,
      mech_control_core::MonotonicTime now,
      mech_control_core::RawCanFrame& output) noexcept;
  [[nodiscard]] mech_control_core::AdapterResult prepare_velocity(
      const mech_control_core::CanonicalDeviceCommand& command,
      mech_control_core::MonotonicTime now,
      mech_control_core::RawCanFrame& output) noexcept;
  [[nodiscard]] mech_control_core::AdapterResult prepare_effort(
      const mech_control_core::CanonicalDeviceCommand& command,
      mech_control_core::MonotonicTime now,
      mech_control_core::RawCanFrame& output) noexcept;
  [[nodiscard]] bool pending_target_still_authorized(
      const mech_control_core::RawCanFrame& frame,
      std::uint64_t generation,
      mech_control_core::MonotonicTime deadline,
      mech_control_core::MonotonicTime now) const noexcept;
  [[nodiscard]] bool fault_latched() const noexcept { return fault_latched_; }

 private:
  void clear_prepared() noexcept;

  Dm4340MitSessionConfig config_{};
  bool configured_{false};
  bool active_{false};
  bool fault_latched_{false};
  mech_control_core::MonotonicTime epoch_start_{};
  std::optional<mech_control_core::MonotonicTime> last_arrival_;
  Dm4340MitSnapshot sample_{};
  std::uint64_t last_generation_{0U};
  std::optional<mech_control_core::RawCanFrame> last_prepared_;
  std::optional<mech_control_core::MonotonicTime> last_deadline_;
  double last_prepared_joint_position_rad_{
      std::numeric_limits<double>::quiet_NaN()};
  bool last_prepared_requires_enabled_{false};
};

}  // namespace mech::mech_protocol_damiao
