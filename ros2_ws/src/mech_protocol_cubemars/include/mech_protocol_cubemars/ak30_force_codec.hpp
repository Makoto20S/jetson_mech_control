#pragma once

#include <cstdint>

#include "mech_control_core/adapter_template.hpp"
#include "mech_protocol_cubemars/ak30_mapping.hpp"

namespace mech::mech_protocol_cubemars {

// Pure codec: no I/O, no session state, no allocation, no device probing. The
// sub-mode is fixed at construction because force control's three sub-modes
// share control mode ID 8 and inferring one from payload content would be a
// silent-failure path.
class Ak30ForceControlCodec final : public mech_control_core::DeviceCodec {
 public:
  Ak30ForceControlCodec(std::uint8_t drive_id, ForceControlSubMode sub_mode,
                        Ak30Mapping mapping, ForceControlGains gains) noexcept;

  [[nodiscard]] mech_control_core::ProtocolProfile profile() const noexcept override;

  [[nodiscard]] mech_control_core::AdapterResult encode(
      const mech_control_core::CanonicalDeviceCommand& command,
      std::uint16_t logical_bus, mech_control_core::MonotonicTime now,
      mech_control_core::RawCanFrame& output) const noexcept override;

  [[nodiscard]] mech_control_core::AdapterResult decode(
      const mech_control_core::RawCanFrame& frame,
      mech_control_core::CanonicalDeviceState& output) const noexcept override;

  [[nodiscard]] ForceControlSubMode sub_mode() const noexcept { return sub_mode_; }
  [[nodiscard]] const Ak30Mapping& mapping() const noexcept { return mapping_; }

 private:
  std::uint8_t drive_id_;
  ForceControlSubMode sub_mode_;
  Ak30Mapping mapping_;
  ForceControlGains gains_;
};

}  // namespace mech::mech_protocol_cubemars
