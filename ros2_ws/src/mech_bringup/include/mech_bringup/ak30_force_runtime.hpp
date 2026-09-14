#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "mech_control_core/transport.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_protocol_cubemars/ak30_force_session.hpp"
#include "mech_protocol_cubemars/ak30_mapping.hpp"

namespace mech::mech_bringup {

// Configuration for one force-control joint wired through the composite
// SystemInterface. Mirrors Ak30SessionConfig's fields that a deployment may
// legitimately set; the sub-mode is explicit configuration (never inferred)
// and the gains ride the Position sub-mode's Kp/Kd impedance path.
struct Ak30RuntimeConfig final {
  std::uint16_t drive_id{104U};
  std::uint32_t logical_bus{1U};
  mech::mech_protocol_cubemars::ForceControlSubMode sub_mode{
      mech::mech_protocol_cubemars::ForceControlSubMode::Position};
  mech::mech_protocol_cubemars::Ak30Mapping mapping{};
  mech::mech_protocol_cubemars::ForceControlGains gains{};
  std::uint32_t device_id{1U};
  // Operator-asserted firmware identity: AK3.0 has no firmware query on the
  // wire, so 02:160's range check runs on these values.
  std::uint32_t firmware_id{0x0304U};
  std::uint32_t firmware_id_min{0x0300U};
  std::uint32_t firmware_id_max{0x03FFU};
  // Watchdog budget: ADR-012 requires the whole staged watchdog to fit
  // <=3 control cycles (<=6 ms at 500 Hz); defaults match the documented
  // ttl 4 ms / hard 6 ms.
  std::int64_t control_period_nanoseconds{2000000};
  std::int64_t command_ttl_nanoseconds{4000000};
  std::int64_t command_hard_ttl_nanoseconds{6000000};
  std::int64_t feedback_ttl_nanoseconds{6000000};
};

// The first production consumer of Ak30ForceControlSession::command_stage()
// (ADR-012's staged watchdog): this runtime is the RuntimePort that wires the
// force-control adapter into CompositeSystem. It borrows an injected
// Transport (UsbCdcTransport in production, FakeTransport in tests) and an
// injected clock, and follows the per-cycle shape the device probes proved:
// submit the stored command, drain received frames into the session, publish
// the snapshot. It never opens a channel and never synthesizes a command -
// in particular it never resolves "no fresh command" to 0.0, which on the
// position interface would be a commanded move to the zero position.
class Ak30ForceControlRuntime final
    : public mech_hardware_ros2_control::RuntimePort {
 public:
  using Clock = std::function<mech::mech_control_core::MonotonicTime()>;

  // The transport must outlive this runtime; the clock is called once per
  // read()/write() cycle and must be monotonic.
  Ak30ForceControlRuntime(mech::mech_control_core::Transport& transport,
                          Clock clock, Ak30RuntimeConfig config) noexcept;

  [[nodiscard]] bool configure(std::size_t resource_count) noexcept override;
  [[nodiscard]] bool start() noexcept override;
  void stop() noexcept override;
  // Submit the stored command per the staged watchdog, drain feedback, and
  // publish the snapshot. Returns false on watchdog expiry or a latched
  // fault, which routes through CompositeSystem's ERROR path.
  [[nodiscard]] bool read(mech_hardware_ros2_control::CanonicalState* states,
                          std::size_t count) noexcept override;
  // Stores the latest finite commands; they are submitted by the NEXT read,
  // matching ros2_control's read -> update -> write ordering.
  [[nodiscard]] bool write(
      const mech_hardware_ros2_control::CanonicalCommand* commands,
      std::size_t count) noexcept override;

  [[nodiscard]] bool holding() const noexcept { return holding_; }
  [[nodiscard]] bool expired() const noexcept { return expired_; }

 private:
  // Submits the stored command if one exists; returns false on a hard
  // failure. Implements the staged-watchdog submission policy.
  [[nodiscard]] bool submit_stored(
      mech::mech_control_core::MonotonicTime now) noexcept;
  void publish_states(
      mech_hardware_ros2_control::CanonicalState* states,
      std::size_t count,
      mech::mech_control_core::MonotonicTime now) const noexcept;

  mech::mech_control_core::Transport& transport_;
  Clock clock_;
  Ak30RuntimeConfig config_;
  mech::mech_protocol_cubemars::Ak30ForceControlSession session_;
  std::vector<mech_hardware_ros2_control::CanonicalCommand> pending_;
  std::size_t resource_count_{0U};
  bool configured_{false};
  bool started_{false};
  // Watchdog bookkeeping: have_pending_ = a controller command was written;
  // fresh_write_ = it has not been submitted yet (a refresh); submitted_once_
  // = the session accepted at least one command, so its Expired means stale
  // rather than never-commanded.
  bool have_pending_{false};
  bool fresh_write_{false};
  bool submitted_once_{false};
  bool holding_{false};
  bool expired_{false};
};

}  // namespace mech::mech_bringup
