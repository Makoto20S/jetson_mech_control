#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "mech_control_core/command_contract.hpp"

namespace mech::mech_hardware_ros2_control {

// The canonical name lives in mech_control_core because controllers need it
// too and must not depend on a hardware plugin to spell it. Bound by reference
// rather than copied, so there is exactly one definition of the string: the two
// names cannot drift, and existing call sites keep working unchanged.
inline constexpr auto& kCommandGenerationInterface =
    mech::mech_control_core::kCommandGenerationInterface;

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
  // ADR-017: whether this cycle carries a command the controller actually
  // refreshed, as opposed to the controller_manager simply having cycled.
  //
  // Separate from `authorized` because the two answer different questions and
  // have different remedies: losing authorization means drop the pending
  // command (cancel_pending), while merely not being fresh means send nothing
  // new and leave any pending retry alone. In the weak tier this is always
  // true whenever `authorized` is - which is exactly the pre-ADR-017
  // behaviour, stated rather than implied.
  //
  // Defaults to false so a dispatch built by something that does not know
  // about freshness is treated as "no new command" rather than as a refresh.
  bool fresh{false};
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
  // It must treat fresh == false the same way for the purpose of accepting a
  // NEW command (ADR-017), but must NOT cancel an already-pending one: a
  // command under backpressure is still bounded by its own deadline, and a
  // controller falling quiet is not a reason to abandon a command it already
  // gave.
  [[nodiscard]] virtual bool write(const CommandDispatch* commands,
                                   std::size_t count) noexcept = 0;
  // Drops any pending-but-unsent command for one resource and clears its
  // freshness (ADR-015 Decision 3). Called when a claim is released or the
  // component leaves ACTIVE, so revocation does not wait for the hard TTL to
  // lapse on its own. Per-resource: releasing one joint must not cancel a
  // live command on another.
  virtual void cancel_pending(std::size_t index) noexcept = 0;
  // Whether the most recent read() observed a usable feedback sample
  // (ADR-016 Decision 3). False means the device's state is not known - either
  // nothing has arrived yet, or what arrived aged out - and a joint whose
  // state is unknown must not be claimable, because a controller cannot seed a
  // hold from a position nobody has measured.
  [[nodiscard]] virtual bool has_valid_sample() const noexcept = 0;
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
  [[nodiscard]] unsigned char joint_command_interface_mask(
      std::size_t index) const noexcept;

  std::unique_ptr<RuntimePort> runtime_;
  std::vector<std::string> joint_names_;
  // Immutable configured motion bundle. Bits identify position, velocity and
  // effort; the legal bundles are the three singletons and position combined
  // with either or both auxiliary fields.
  std::vector<unsigned char> joint_command_interface_masks_;
  std::vector<CanonicalCommand> commands_;
  std::vector<CanonicalState> states_;
  // ADR-017: the storage behind each joint's exported command_generation
  // interface. One cell per joint - sharing a cell would make one controller's
  // refresh look like a refresh of every joint.
  std::vector<double> generations_;
  // Per-joint transmit authorization (ADR-015), and the single source of
  // truth for what perform_command_mode_switch() has granted. Deliberately
  // not std::vector<bool>: an out-of-range index into that specialization
  // lands on padding bits inside the same word, so it neither crashes nor
  // trips AddressSanitizer.
  std::vector<unsigned char> authorized_;
  // The motion interfaces currently claimed for each joint. Authorization is
  // granted only when this equals the joint's complete configured bundle.
  std::vector<unsigned char> motion_claimed_;
  // ADR-017 Decision 2: whether each joint's controller also claimed the
  // generation interface, i.e. which protection tier that joint is in. Set
  // from the start_interfaces list the manager passes to
  // perform_command_mode_switch(), so the tier is OBSERVED rather than
  // declared - nothing a deployment or a controller says can move a joint into
  // the strong tier without actually holding the interface.
  std::vector<unsigned char> generation_claimed_;
  // ADR-017 Decision 3: the generation value the hardware has already acted
  // on. A command counts as new only while the exported interface differs
  // from this. Re-seeded from the buffer whenever a claim is taken or dropped
  // (Decision 3.4), which is what stops a re-claimed joint from replaying the
  // previous controller's target: the value is already the baseline, so it is
  // not a change until someone writes a different one.
  std::vector<double> generation_baseline_;
  // Pre-allocated per-cycle dispatch buffer, derived from commands_ and
  // authorized_. Held as a member so write() allocates nothing.
  std::vector<CommandDispatch> dispatch_;
  bool initialized_{false};
  bool configured_{false};
  bool active_{false};
  bool fault_latched_{false};
};

}  // namespace mech::mech_hardware_ros2_control
