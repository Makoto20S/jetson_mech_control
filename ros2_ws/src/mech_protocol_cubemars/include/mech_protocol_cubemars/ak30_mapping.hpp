#pragma once

#include <cstdint>

#include "mech_control_core/adapter_template.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"

namespace mech::mech_protocol_cubemars {

inline constexpr double kPi = 3.14159265358979323846;

// Mirrors the *_verified tri-state that ADR-012 established for
// TransportCapabilities: verified means an evidence source was read,
// unverified with a value means somebody asserted it, unverified with zero
// means genuinely unknown and deliberately not claimed.
struct EvidencedValue final {
  double value{0.0};
  bool verified{false};
};

// The three sub-modes share control mode ID 8 and are distinguished by payload
// content, so this is explicit configuration and is never inferred from a
// received frame.
enum class ForceControlSubMode : std::uint8_t { Position, Velocity, Torque };

// Which CanonicalDeviceState fields the configured sub-mode can populate from
// evidence. Anything false is left at 0.0 and must not be exported as a
// ros2_control interface by a later slice.
struct EvidencedStateFields final {
  bool position{false};
  bool velocity{false};
  bool effort{false};
};

// Gains are configuration, not canonical command fields: CanonicalDeviceCommand
// has no place for them and adding one would be a contract change needing an
// ADR.
struct ForceControlGains final {
  double kp{0.0};
  double kd{0.0};
};

// Defaults are motor1's evidence state as the repository records it today:
// pole_pairs, gear_ratio, torque_constant and direction_sign are verified, so
// the torque and velocity sub-modes configure; position still waits on B4
// (position_source_shaft) and the zero-offset chain.
struct Ak30Mapping final {
  EvidencedValue pole_pairs{14.0, true};
  // Verified 2026-09-02 from three independent sources agreeing on 8:1 — the
  // host tool's dedicated 减速器参数设置 field reading `Ratio: 8`, the model
  // naming convention in L07's own force-control parameter table (AK80-9 is
  // 9:1, AK60-39 is 39:1, AKH70-48 is 48:1, so AKE60-8 is 8:1), and the
  // displayed reduction ratio. The export's `si_gear_ratio = 0` is a different,
  // unset VESC-lineage SI-display field, not a counter-source; an earlier
  // summary mistook the two for one field and recorded a conflict that does not
  // exist.
  EvidencedValue gear_ratio{8.0, true};
  EvidencedValue zero_offset_rad{0.0, false};
  // Verified 2026-09-03 by direct bench measurement: commanding +0.8 and
  // +1.0 rad/s (velocity sub-mode, Kd=1, Kp=0) turned the output shaft
  // clockwise (owner-observed) while feedback position increased and Iq stayed
  // positive-dominant (+0.42 A peak). The canonical positive direction is
  // therefore the direction in which feedback position increases: +1.0. Vendor
  // question B9's remaining value is confirming the
  // foc_encoder_inverted/m_invert_direction layer chain, not this sign.
  EvidencedValue direction_sign{1.0, true};
  // L07 p.37 for AKE60-8, owner-guaranteed for this variant (ADR-013 section 4).
  EvidencedValue torque_constant_nm_per_a{0.7382, true};
  // Vendor question B4: L07 writes 输出端 explicitly for torque and speed but
  // not for position, so the source is undetermined.
  bool position_source_known{false};
  bool position_is_output_shaft{false};
  ForceControlRanges ranges{ake60_8_ranges()};
};

[[nodiscard]] constexpr EvidencedStateFields evidenced_state_fields(
    ForceControlSubMode sub_mode) noexcept {
  switch (sub_mode) {
    case ForceControlSubMode::Torque:
      return EvidencedStateFields{false, false, true};
    case ForceControlSubMode::Velocity:
      return EvidencedStateFields{false, true, true};
    case ForceControlSubMode::Position:
      return EvidencedStateFields{true, true, true};
  }
  return EvidencedStateFields{};
}

// Fails closed. The same mapping converts both directions, so suppressing only
// decode would still let a session emit a command computed from an unverified
// mapping - worse, because it moves the motor. ADR-009 Decision 2 prescribes
// rejecting at configure rather than downgrading.
[[nodiscard]] bool mapping_is_sufficient(const Ak30Mapping& mapping,
                                         ForceControlSubMode sub_mode) noexcept;

// Precondition: mapping_is_sufficient(mapping, sub_mode). Range and finiteness
// checking belongs to encode_force_control, which runs on the result.
void to_device_command(
    const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
    const ForceControlGains& gains,
    const mech_control_core::CanonicalDeviceCommand& command,
    ForceControlCommand& output) noexcept;

// Populates only the fields evidenced_state_fields() reports for the sub-mode;
// the rest are left as the caller set them. Does not touch output.status.
void to_canonical_state(
    const Ak30Mapping& mapping, ForceControlSubMode sub_mode,
    const ForceControlFeedback& feedback,
    mech_control_core::CanonicalDeviceState& output) noexcept;

}  // namespace mech::mech_protocol_cubemars
