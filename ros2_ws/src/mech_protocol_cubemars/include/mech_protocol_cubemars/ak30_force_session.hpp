#pragma once

#include <cstdint>
#include <optional>

#include "mech_control_core/adapter_template.hpp"
#include "mech_control_core/transport.hpp"
#include "mech_protocol_cubemars/ak30_force_codec.hpp"

namespace mech::mech_protocol_cubemars {

// ADR-012 requires the whole watchdog to fit <=3 control cycles, which is
// <=6 ms at the documented 500 Hz (03_mvp_delivery_plan.md:215).
inline constexpr std::int64_t kMaxHardTtlNanoseconds = 6000000;

// Restates the staged watchdog that mech_controllers::WatchdogStage already
// implements. This package must not depend on mech_controllers - that would
// invert the dependency direction and break the EXPECTED_PACKAGES boundary -
// so the semantics are duplicated deliberately. ADR-012 already records
// unifying these into core's CommandSlot as an open review trigger.
enum class CommandStage : std::uint8_t { Following, Holding, Expired };

struct Ak30SessionConfig final {
  // Wider than the 8-bit wire field so an out-of-range value is representable
  // and therefore rejectable at configure rather than silently truncated.
  std::uint16_t drive_id{0U};
  ForceControlSubMode sub_mode{ForceControlSubMode::Torque};
  Ak30Mapping mapping{};
  ForceControlGains gains{};
  // 02:160 requires on_configure to validate a firmware range. DeviceConfig has
  // no firmware field, and adding one would be a canonical contract change
  // requiring an ADR first (adapter_contract_v1.md item 7), so it lives here.
  // AK3.0 has no firmware query on the wire, so this is operator-asserted.
  std::uint32_t firmware_id{0U};
  std::uint32_t firmware_id_min{0U};
  std::uint32_t firmware_id_max{0U};
  std::int64_t command_ttl_nanoseconds{4000000};
  std::int64_t command_hard_ttl_nanoseconds{6000000};
  std::int64_t feedback_ttl_nanoseconds{6000000};
};

// Owns device semantics; borrows the transport and never opens a channel.
class Ak30ForceControlSession final : public mech_control_core::DeviceSession {
 public:
  Ak30ForceControlSession(mech_control_core::Transport& transport,
                          Ak30SessionConfig config) noexcept;

  [[nodiscard]] mech_control_core::AdapterResult configure(
      const mech_control_core::DeviceConfig& config,
      const mech_control_core::TransportCapabilities& capabilities) noexcept override;
  [[nodiscard]] mech_control_core::AdapterResult activate() noexcept override;
  void deactivate() noexcept override;
  [[nodiscard]] mech_control_core::AdapterResult submit(
      const mech_control_core::CanonicalDeviceCommand& command,
      mech_control_core::MonotonicTime now) noexcept override;
  [[nodiscard]] mech_control_core::AdapterResult process(
      const mech_control_core::RawCanFrame& frame,
      mech_control_core::MonotonicTime now) noexcept override;
  [[nodiscard]] mech_control_core::CanonicalDeviceState snapshot(
      mech_control_core::MonotonicTime now) const noexcept override;

  [[nodiscard]] CommandStage command_stage(
      mech_control_core::MonotonicTime now) const noexcept;
  [[nodiscard]] bool fault_latched() const noexcept { return fault_latched_; }

 private:
  enum class Lifecycle : std::uint8_t { Unconfigured, Ready, Active };

  Ak30SessionConfig config_;
  std::optional<Ak30ForceControlCodec> codec_;
  Lifecycle lifecycle_{Lifecycle::Unconfigured};
  std::uint16_t logical_bus_{0U};
  bool fault_latched_{false};
  std::uint64_t sequence_{0U};
  mech_control_core::CanonicalDeviceState last_state_{};
  std::optional<mech_control_core::MonotonicTime> last_feedback_time_;
  std::optional<mech_control_core::MonotonicTime> last_command_time_;
};

}  // namespace mech::mech_protocol_cubemars
