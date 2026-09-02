#include "mech_protocol_cubemars/ak30_mapping.hpp"

#include <cmath>

namespace mech::mech_protocol_cubemars {
namespace {

[[nodiscard]] bool positive_and_verified(const EvidencedValue& value) noexcept {
  return value.verified && std::isfinite(value.value) && value.value > 0.0;
}

// Shaft angle in radians on whichever side the encoder reports, converted to
// the output shaft.
[[nodiscard]] double feedback_position_rad(const Ak30Mapping& mapping,
                                           double position_deg) noexcept {
  const double raw_rad = position_deg * kPi / 180.0;
  return mapping.position_is_output_shaft ? raw_rad
                                          : raw_rad / mapping.gear_ratio.value;
}

}  // namespace

bool mapping_is_sufficient(const Ak30Mapping& mapping,
                           ForceControlSubMode sub_mode) noexcept {
  if (!mapping.ranges.is_valid()) {
    return false;
  }
  // direction_sign and torque_constant are consumed by every sub-mode: the
  // former in both directions, the latter because effort is always exported.
  if (!mapping.direction_sign.verified ||
      (mapping.direction_sign.value != 1.0 &&
       mapping.direction_sign.value != -1.0)) {
    return false;
  }
  if (!positive_and_verified(mapping.torque_constant_nm_per_a)) {
    return false;
  }
  if (sub_mode == ForceControlSubMode::Torque) {
    return true;
  }
  // Velocity decode is ERPM / pole_pairs / gear_ratio.
  if (!positive_and_verified(mapping.pole_pairs) ||
      !positive_and_verified(mapping.gear_ratio)) {
    return false;
  }
  if (sub_mode == ForceControlSubMode::Velocity) {
    return true;
  }
  // Position additionally needs the zero reference and the encoder's shaft.
  return mapping.position_source_known && mapping.zero_offset_rad.verified &&
         std::isfinite(mapping.zero_offset_rad.value);
}

void to_device_command(const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
                       const ForceControlGains& gains,
                       const mech_control_core::CanonicalDeviceCommand& command,
                       ForceControlCommand& output) noexcept {
  const double sign = mapping.direction_sign.value;
  output = ForceControlCommand{};
  // The torque field is a feedforward term in every sub-mode - the vendor's own
  // pack_cmd() names it t_ff - so effort rides along rather than being dropped.
  output.torque_nm = sign * command.effort;

  if (sub_mode == ForceControlSubMode::Torque) {
    return;
  }

  // Assumed output-side rad/s, matching the torque field. L07 does not state
  // which side the command velocity refers to; vendor question B15.
  output.kd = gains.kd;
  output.velocity_rad_s = sign * command.velocity;

  if (sub_mode == ForceControlSubMode::Velocity) {
    return;
  }

  output.kp = gains.kp;
  const double shaft_rad = sign * (command.position + mapping.zero_offset_rad.value);
  output.position_rad = mapping.position_is_output_shaft
                            ? shaft_rad
                            : shaft_rad * mapping.gear_ratio.value;
}

void to_canonical_state(const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
                        const ForceControlFeedback& feedback,
                        mech_control_core::CanonicalDeviceState& output) noexcept {
  const double sign = mapping.direction_sign.value;
  const auto fields = evidenced_state_fields(sub_mode);

  if (fields.effort) {
    // T = Kt * Iq, and L07 states T is output-shaft torque.
    output.effort = sign * mapping.torque_constant_nm_per_a.value *
                    feedback.current_iq_a;
  }
  if (fields.velocity) {
    // 输出端转速 = ERPM / 极对数 / 减速比, then rpm to rad/s.
    const double output_rpm = feedback.electrical_speed_erpm /
                              mapping.pole_pairs.value /
                              mapping.gear_ratio.value;
    output.velocity = sign * output_rpm * (2.0 * kPi / 60.0);
  }
  if (fields.position) {
    output.position = sign * feedback_position_rad(mapping, feedback.position_deg) -
                      mapping.zero_offset_rad.value;
  }
}

}  // namespace mech::mech_protocol_cubemars
