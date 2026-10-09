#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "mech_control_core/runtime.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_protocol_cubemars/ak30_servo_position_session.hpp"

namespace mech::mech_bringup {
class CommandTrace;

enum class Ak30ServoFaultReason : std::uint8_t {
  ClockRegression, FeedbackRejected, FeedbackUnavailable, HardDeadline,
  UnsentSoftDeadline, PositionPreparation, LeaseSubmission,
  TargetAuthorization, InvalidDispatch, ReceiveFailure, TransmitFailure,
  SelectedLeaseMissing
};
enum class Ak30ServoFaultPhase : std::uint8_t {
  Read, Write, ReceiveObserver, SendGuard, Watchdog, Pending
};

// Fixed-size first-cause evidence. Capturing this record never logs, allocates,
// or mutates the bus, including when called from a bus observer/send guard.
struct Ak30ServoFault final {
  Ak30ServoFaultReason reason{};
  Ak30ServoFaultPhase phase{};
  std::size_t index{static_cast<std::size_t>(-1)};
  std::uint16_t drive_id{0U};
  mech_control_core::MonotonicTime now{};
  std::int64_t previous_time_ns{0};
  std::int64_t soft_deadline_ns{0};
  std::int64_t hard_deadline_ns{0};
  double target_position_rad{std::numeric_limits<double>::quiet_NaN()};
  double max_target_error_rad{0.0};
  mech_protocol_cubemars::ServoPositionSnapshot feedback{};
  mech_control_core::AdapterResult adapter_result{mech_control_core::AdapterResult::Ok};
  mech_control_core::RuntimeResult runtime_result{mech_control_core::RuntimeResult::Ok};
};

[[nodiscard]] const char* fault_reason_name(Ak30ServoFaultReason reason) noexcept;
[[nodiscard]] const char* fault_phase_name(Ak30ServoFaultPhase phase) noexcept;

struct Ak30ServoRuntimeConfig final {
  std::uint16_t logical_bus{0U};
  std::string physical_bus;
  std::int64_t control_period_ns{0};
  std::vector<mech_protocol_cubemars::ServoPositionSessionConfig> joints;
};

// One bus and one existing position session per configured resource. The
// transport, clock, and ownership registry must outlive this runtime.
class Ak30ServoRuntime final : public mech_hardware_ros2_control::RuntimePort {
 public:
  using Clock = std::function<mech_control_core::MonotonicTime()>;

  Ak30ServoRuntime(mech_control_core::Transport& transport, Clock clock,
                   Ak30ServoRuntimeConfig config,
                   mech_control_core::BusOwnershipRegistry& ownership,
                   CommandTrace* trace = nullptr);

  [[nodiscard]] bool configure(std::size_t resource_count) noexcept override;
  [[nodiscard]] bool start() noexcept override;
  void stop() noexcept override;
  [[nodiscard]] bool read(mech_hardware_ros2_control::CanonicalState* states,
                          std::size_t count) noexcept override;
  [[nodiscard]] bool write(
      const mech_hardware_ros2_control::CommandDispatch* commands,
      std::size_t count) noexcept override;
  void cancel_pending(std::size_t index) noexcept override;
  [[nodiscard]] bool has_valid_sample() const noexcept override;
  [[nodiscard]] std::optional<mech_protocol_cubemars::ServoPositionSnapshot>
  diagnostic_snapshot(std::size_t index) const noexcept;
  [[nodiscard]] const mech_control_core::RuntimeStats& bus_stats() const noexcept {
    return bus_.stats();
  }

  [[nodiscard]] const std::optional<Ak30ServoFault>& first_fault() const noexcept {
    return first_fault_;
  }

 private:
  void record_fault(Ak30ServoFaultReason reason, Ak30ServoFaultPhase phase,
                    mech_control_core::MonotonicTime now,
                    std::size_t index = static_cast<std::size_t>(-1),
                    mech_control_core::AdapterResult adapter = mech_control_core::AdapterResult::Ok,
                    mech_control_core::RuntimeResult runtime = mech_control_core::RuntimeResult::Ok) noexcept;
  struct Pending final {
    std::optional<mech_control_core::CanonicalDeviceCommand> command;
    std::optional<mech_control_core::CommandLease> lease;
    mech_control_core::MonotonicTime hard_deadline{};
    std::uint64_t generation{0U};
  };

  static bool observe_frame(void* context, std::uint16_t route,
                            const mech_control_core::RawCanFrame& frame) noexcept;
  static bool guard_send(void* context,
                         const mech_control_core::CommandLease& lease,
                         mech_control_core::MonotonicTime& attempt_now) noexcept;
  [[nodiscard]] bool fail_group() noexcept;
  [[nodiscard]] bool check_time(mech_control_core::MonotonicTime now) noexcept;
  [[nodiscard]] bool check_watchdog(
      mech_control_core::MonotonicTime now) noexcept;
  [[nodiscard]] bool check_pending(mech_control_core::MonotonicTime now) noexcept;
  [[nodiscard]] std::uint16_t route(std::size_t index) const noexcept;

  CommandTrace* trace_{nullptr};
  mech_control_core::Transport& transport_;
  Clock clock_;
  Ak30ServoRuntimeConfig config_;
  mech_control_core::FrameRouter router_;
  mech_control_core::BusRuntime bus_;
  std::vector<mech_protocol_cubemars::Ak30ServoPositionSession> sessions_;
  std::vector<Pending> pending_;
  std::optional<Ak30ServoFault> first_fault_;
  Ak30ServoFaultPhase phase_{Ak30ServoFaultPhase::Read};
  bool configured_{false};
  bool started_{false};
  bool has_valid_sample_{false};
  bool ever_full_sample_{false};
  bool group_fault_{false};
  std::optional<mech_control_core::MonotonicTime> last_time_;
};

}  // namespace mech::mech_bringup
