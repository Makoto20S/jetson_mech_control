#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "mech_control_core/runtime.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_protocol_damiao/dm4340_mit_session.hpp"

namespace mech::mech_bringup {

enum class Dm4340CommandMode { Velocity, Effort };

struct Dm4340RuntimeConfig final {
  std::string physical_bus;
  std::uint16_t logical_bus{0U};
  std::vector<mech_protocol_damiao::Dm4340MitSessionConfig> joints;
  std::int64_t control_period_ns{0};
  std::int64_t command_hard_ttl_ns{0};
  std::int64_t query_period_ns{0};
  bool motor_side_timeout_verified{false};
  bool supervised_effort_bench{false};
  Dm4340CommandMode command_mode{Dm4340CommandMode::Velocity};
};

enum class Dm4340FaultReason {
  None,
  StatusQuerySend,
  Receive,
  FeedbackRejected,
  FeedbackExpired,
  MotorNotEnabled,
  CommandDeadline,
  CommandRejected,
  CommandPreparation,
  CommandQueue,
  CommandTransmit,
  StopFailed,
};

[[nodiscard]] const char* dm4340_fault_name(Dm4340FaultReason reason) noexcept;

// One bus owner and one MIT session per joint. Enable is an explicit
// hardware-lifecycle operation, never a side effect of a velocity target.
class Dm4340Runtime final : public mech_hardware_ros2_control::RuntimePort {
 public:
  using Clock = std::function<mech_control_core::MonotonicTime()>;

  Dm4340Runtime(mech_control_core::Transport& transport, Clock clock,
                        Dm4340RuntimeConfig config,
                        mech_control_core::BusOwnershipRegistry& ownership);
  ~Dm4340Runtime() override { stop(); }

  bool configure(std::size_t resource_count) noexcept override;
  bool start() noexcept override;
  void stop() noexcept override;
  bool read(mech_hardware_ros2_control::CanonicalState* states,
            std::size_t count) noexcept override;
  bool write(const mech_hardware_ros2_control::CommandDispatch* commands,
             std::size_t count) noexcept override;
  void cancel_pending(std::size_t index) noexcept override;
  bool has_valid_sample() const noexcept override;
  bool has_fresh_ready_feedback() const noexcept;
  bool adopt_or_enable_from_feedback() noexcept;
  [[nodiscard]] Dm4340CommandMode command_mode() const noexcept {
    return config_.command_mode;
  }
  [[nodiscard]] Dm4340FaultReason fault_reason() const noexcept {
    return fault_reason_;
  }
  [[nodiscard]] std::uint8_t rejected_status() const noexcept {
    return rejected_status_;
  }

 private:
  struct Device final {
    mech_protocol_damiao::Dm4340MitSession session;
    std::optional<mech_control_core::CanonicalDeviceCommand> pending;
    std::optional<mech_control_core::CommandLease> lease;
    std::optional<mech_control_core::MonotonicTime> hard_deadline;
    std::optional<mech_control_core::MonotonicTime> last_query;
    std::uint64_t generation{0U};
    bool ever_fresh{false};
    bool command_sent{false};
    bool claimed{false};
    bool disarmed{false};
    bool disable_pending{false};
  };

  static bool observe(void* context, std::uint16_t route,
                      const mech_control_core::RawCanFrame& frame) noexcept;
  static bool authorize_send(void* context,
                             const mech_control_core::CommandLease& lease,
                             mech_control_core::MonotonicTime& now) noexcept;
  bool send_special(std::uint32_t id,
                    const std::array<std::uint8_t, 8>& payload) noexcept;
  bool halt_device(std::size_t index) noexcept;
  bool halt_all(bool fault) noexcept;
  bool fail(Dm4340FaultReason reason) noexcept;
  bool prepare_and_transmit(mech_control_core::MonotonicTime now) noexcept;
  std::uint16_t route(std::size_t index) const noexcept;
  bool valid_sample(std::size_t index) const noexcept;

  mech_control_core::Transport& transport_;
  Clock clock_;
  Dm4340RuntimeConfig config_;
  mech_control_core::FrameRouter router_;
  mech_control_core::BusRuntime bus_;
  std::vector<Device> devices_;
  bool configured_{false};
  bool running_{false};
  bool faulted_{false};
  Dm4340FaultReason fault_reason_{Dm4340FaultReason::None};
  std::uint8_t rejected_status_{0U};
};

}  // namespace mech::mech_bringup
