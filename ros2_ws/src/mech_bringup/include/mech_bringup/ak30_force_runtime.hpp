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
  // The device's own reporting period, read back from its configuration:
  // motor1 has send_can_status_rate_hz = 50, so 20 ms. ADR-016 Decision 4
  // requires the feedback window to be at least this long - a window shorter
  // than the period it measures against can never be satisfied.
  std::int64_t feedback_period_nanoseconds{20000000};
  // 3x the reporting period: one lost frame must not stop the bench, and
  // stopping the motor is the drive's own job anyway (motor1's loss-of-control
  // protection is 1000 ms with zero brake current). This window is about
  // knowing the device went quiet, not about braking.
  std::int64_t feedback_ttl_nanoseconds{60000000};
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
  // Stores the latest authorized commands; they are submitted by the NEXT
  // read, matching ros2_control's read -> update -> write ordering. A
  // dispatch whose authorized flag is false contributes nothing: no pending
  // command, and no refresh of an earlier one (ADR-015).
  [[nodiscard]] bool write(
      const mech_hardware_ros2_control::CommandDispatch* commands,
      std::size_t count) noexcept override;
  // Drops the stored command and its freshness for one resource, so a
  // released claim or a lifecycle exit stops submission immediately instead
  // of waiting for the hard TTL (ADR-015 Decision 3).
  void cancel_pending(std::size_t index) noexcept override;
  // ADR-016: true only while the last read() saw Valid or Degraded feedback.
  [[nodiscard]] bool has_valid_sample() const noexcept override {
    return has_valid_sample_;
  }

  [[nodiscard]] bool holding() const noexcept { return holding_; }
  [[nodiscard]] bool expired() const noexcept { return expired_; }
  // ADR-016 Decision 5: the quality evidence must survive the ROS boundary.
  // The exported state interfaces are three bare doubles with nowhere to put
  // quality, sequence or arrival time, so they are kept here for tests and
  // diagnostics rather than discarded.
  [[nodiscard]] const mech::mech_control_core::StatusSnapshot& last_status()
      const noexcept {
    return last_status_;
  }

 private:
  // Submits the stored command if one exists; returns false on a hard
  // failure. Implements the staged-watchdog submission policy.
  [[nodiscard]] bool submit_stored(
      mech::mech_control_core::MonotonicTime now) noexcept;
  // Publishes the snapshot only when it is usable, and reports whether the
  // caller may keep running (ADR-016). Unusable-but-never-sampled is survivable
  // (the startup transient); unusable-after-sampling is a fault.
  [[nodiscard]] bool publish_states(
      mech_hardware_ros2_control::CanonicalState* states,
      std::size_t count,
      mech::mech_control_core::MonotonicTime now) noexcept;

  mech::mech_control_core::Transport& transport_;
  Clock clock_;
  Ak30RuntimeConfig config_;
  mech::mech_protocol_cubemars::Ak30ForceControlSession session_;
  std::vector<mech_hardware_ros2_control::CanonicalCommand> pending_;
  // The pending command's validity window, minted once in write() from the
  // time the command was received - never re-derived on a retry. ADR-012
  // budgets the whole staged watchdog at <=3 control cycles, and recomputing
  // this per attempt turns transport backpressure into an unbounded extension
  // that lets a milliseconds-stale target reach the motor. Only meaningful
  // while have_pending_ is true.
  mech::mech_control_core::MonotonicTime pending_deadline_{};
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
  // ADR-016: quality of the most recent snapshot, and whether it was usable.
  mech::mech_control_core::StatusSnapshot last_status_{};
  bool has_valid_sample_{false};
};

}  // namespace mech::mech_bringup
