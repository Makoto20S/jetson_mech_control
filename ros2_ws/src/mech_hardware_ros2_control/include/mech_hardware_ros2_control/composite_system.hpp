#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"

namespace mech::mech_hardware_ros2_control {

struct CanonicalCommand final {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct CanonicalState final {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

// A command paired with the authorization to transmit it (ADR-015).
//
// The two travel together because they are only ever meaningful together: a
// CanonicalCommand on its own cannot distinguish "the controller commanded
// zero" from "nobody claimed this joint, so this is a default-constructed
// placeholder". On a position interface that ambiguity is a commanded move to
// the zero position (ADR-012 Decision 3), so the boundary carries the
// authorization explicitly instead of encoding it in a sentinel value.
struct CommandDispatch final {
  CanonicalCommand command{};
  // True only while the joint's command interface is claimed through
  // perform_command_mode_switch(); revoked by stop/deactivate/cleanup/error.
  bool authorized{false};
};

// RuntimePort is the narrow, non-blocking boundary between ros2_control and
// device sessions/BusRuntime. It deliberately contains no CAN or vendor fields.
class RuntimePort {
 public:
  virtual ~RuntimePort() = default;
  [[nodiscard]] virtual bool configure(std::size_t resource_count) noexcept = 0;
  [[nodiscard]] virtual bool start() noexcept = 0;
  virtual void stop() noexcept = 0;
  [[nodiscard]] virtual bool read(CanonicalState* states,
                                  std::size_t count) noexcept = 0;
  // Hands down one dispatch per resource. An implementation must treat
  // authorized == false as "this joint has no command at all" - not as a
  // zero command, and not as a reason to keep an earlier command alive.
  [[nodiscard]] virtual bool write(const CommandDispatch* commands,
                                   std::size_t count) noexcept = 0;
  // Drops any pending-but-unsent command for one resource and clears its
  // freshness (ADR-015 Decision 3). Called when a claim is released or the
  // component leaves ACTIVE, so revocation does not wait for the hard TTL to
  // lapse on its own. Per-resource: releasing one joint must not cancel a
  // live command on another.
  virtual void cancel_pending(std::size_t index) noexcept = 0;
};

// Subclassable since the composition-point slice (PR #13): mech_bringup's
// Ak30System inherits the tested lifecycle/claim/switch/watchdog machinery
// and injects a production runtime via set_runtime before on_init. This is a
// composition point, not a brand branch - the brand stays confined to
// mech_protocol_* and mech_bringup (AdapterContract item 1; see
// docs/development/architecture_package_map.md section 6 for the checks).
class CompositeSystem : public hardware_interface::SystemInterface {
 public:
  CompositeSystem();

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  // Test/adapter injection must happen before on_init. Plugin construction uses
  // a deterministic loopback runtime until a concrete adapter is configured.
  [[nodiscard]] bool set_runtime(std::unique_ptr<RuntimePort> runtime) noexcept;
  [[nodiscard]] bool fault_latched() const noexcept { return fault_latched_; }
  [[nodiscard]] bool active() const noexcept { return active_; }
  // ADR-015: whether this joint currently has transmit authorization. Exposed
  // for tests and diagnostics; the authorization itself is driven only by the
  // lifecycle and command-mode-switch paths.
  [[nodiscard]] bool authorized(std::size_t index) const noexcept {
    return index < authorized_.size() && authorized_[index] != 0U;
  }

 private:
  [[nodiscard]] bool validate_info(
      const hardware_interface::HardwareInfo& info) const noexcept;
  [[nodiscard]] bool validate_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) const noexcept;
  [[nodiscard]] bool known_command_interface(const std::string& name) const noexcept;
  // Revokes transmit authorization for every joint and tells the runtime to
  // drop each pending command (ADR-015 Decision 3). Used by the lifecycle
  // transitions that leave ACTIVE.
  void revoke_all_authorization() noexcept;
  // Resolves "<joint>/<interface>" to an index into joint_names_. Joint names
  // may themselves contain '/', so the interface suffix is split off from the
  // right; returns std::nullopt if no known joint matches the resolved prefix.
  [[nodiscard]] std::optional<std::size_t> resolve_joint_index(
      const std::string& name) const noexcept;
  // The command-interface name each joint's URDF declared, one of
  // position/velocity/effort (ADR-014). Decides which CanonicalCommand member
  // the exported CommandInterface writes into.
  [[nodiscard]] const std::string& joint_command_interface_name(
      std::size_t index) const noexcept;

  std::unique_ptr<RuntimePort> runtime_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> joint_command_interface_names_;
  std::vector<CanonicalCommand> commands_;
  std::vector<CanonicalState> states_;
  // Per-joint transmit authorization (ADR-015), and the single source of
  // truth for what perform_command_mode_switch() has granted. Deliberately
  // not std::vector<bool>: an out-of-range index into that specialization
  // lands on padding bits inside the same word, so it neither crashes nor
  // trips AddressSanitizer.
  std::vector<unsigned char> authorized_;
  // Pre-allocated per-cycle dispatch buffer, derived from commands_ and
  // authorized_. Held as a member so write() allocates nothing.
  std::vector<CommandDispatch> dispatch_;
  bool initialized_{false};
  bool configured_{false};
  bool active_{false};
  bool fault_latched_{false};
};

}  // namespace mech::mech_hardware_ros2_control
