#include "mech_hardware_ros2_control/composite_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mech::mech_hardware_ros2_control {
namespace {

// A joint's single command interface may be any of the three canonical
// command kinds (ADR-014); the name decides which CanonicalCommand member
// the exported CommandInterface writes into. This is the complete accepted
// set - an URDF declaring anything else (or more than one) fails on_init.
constexpr std::array<const char*, 3U> kCommandInterfaceNames{
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT};

[[nodiscard]] bool is_command_interface_name(const std::string& name) noexcept {
  for (const char* candidate : kCommandInterfaceNames) {
    if (name == candidate) return true;
  }
  return false;
}

// Whether an interface name's suffix is the ADR-017 generation interface.
// Matched on the suffix rather than by rebuilding "<joint>/<name>" so it works
// before the joint index is known, and so joint names containing '/' are not a
// special case.
[[nodiscard]] bool is_generation_interface(const std::string& name) noexcept {
  const auto slash = name.rfind('/');
  if (slash == std::string::npos) return false;
  return name.compare(slash + 1U, std::string::npos,
                      kCommandGenerationInterface) == 0;
}

[[nodiscard]] double* command_member(CanonicalCommand& command,
                                     const std::string& name) noexcept {
  if (name == hardware_interface::HW_IF_POSITION) return &command.position;
  if (name == hardware_interface::HW_IF_VELOCITY) return &command.velocity;
  return &command.effort;
}

class LoopbackRuntime final : public RuntimePort {
 public:
  bool configure(std::size_t resource_count) noexcept override {
    try {
      states_.assign(resource_count, CanonicalState{});
      commands_.assign(resource_count, CanonicalCommand{});
    } catch (...) {
      return false;
    }
    return resource_count > 0U;
  }

  bool start() noexcept override {
    running_ = !states_.empty();
    return running_;
  }

  void stop() noexcept override { running_ = false; }

  bool read(CanonicalState* states, std::size_t count) noexcept override {
    if (!running_ || states == nullptr || count != states_.size()) return false;
    for (std::size_t index = 0U; index < count; ++index) {
      // Loopback semantics (ADR-014): the position command ramps toward its
      // target as before, and velocity/effort commands feed straight through
      // into the matching state field. A joint exports exactly one command
      // interface, so at most one of these members is ever non-zero - the
      // additive form is deterministic and value-independent without the
      // runtime needing to know which kind each joint declared.
      const auto delta = commands_[index].position - states_[index].position;
      const auto step = std::clamp(delta, -0.01, 0.01);
      states_[index].position += step;
      states_[index].velocity = step + commands_[index].velocity;
      states_[index].effort = commands_[index].effort;
      states[index] = states_[index];
    }
    return true;
  }

  bool write(const CommandDispatch* commands, std::size_t count) noexcept override {
    if (!running_ || commands == nullptr || count != commands_.size()) return false;
    for (std::size_t index = 0U; index < count; ++index) {
      // ADR-015: an unauthorized dispatch carries no command. Keeping the
      // previous value (rather than zeroing it) is the fail-safe choice here:
      // on a position interface, substituting 0.0 would be a commanded move
      // to the zero position.
      if (commands[index].authorized) commands_[index] = commands[index].command;
    }
    return true;
  }

  void cancel_pending(std::size_t index) noexcept override {
    // The loopback holds no unsent command; the command it mirrors is the
    // state it already published, so there is nothing to drop.
    (void)index;
  }

  bool has_valid_sample() const noexcept override {
    // The loopback synthesizes its state from the commands it was given, so
    // once running it always has one. It models no device and therefore no
    // device silence.
    return running_;
  }

 private:
  bool running_{false};
  std::vector<CanonicalState> states_;
  std::vector<CanonicalCommand> commands_;
};

bool exactly_interfaces(const hardware_interface::ComponentInfo& joint,
                        const std::vector<std::string>& expected_state) {
  if (joint.state_interfaces.size() != expected_state.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < expected_state.size(); ++index) {
    if (joint.state_interfaces[index].name != expected_state[index]) return false;
  }
  // ADR-017 (revising ADR-014 Decision 2): exactly one MOTION command
  // interface whose name is one of the three canonical kinds, plus exactly one
  // command_generation interface. The motion name is per-joint configuration,
  // so the order-sensitive exact list only applies to state interfaces here,
  // and the generation interface may appear in either position.
  //
  // The URDF must declare both even though CLAIMING the generation interface
  // is the controller's choice: the hardware exports it unconditionally, and a
  // deployment that did not declare it would disagree with what is exported.
  std::size_t motion = 0U;
  std::size_t generation = 0U;
  for (const auto& command : joint.command_interfaces) {
    if (command.name == kCommandGenerationInterface) {
      ++generation;
    } else if (is_command_interface_name(command.name)) {
      ++motion;
    } else {
      return false;
    }
  }
  return motion == 1U && generation == 1U;
}

}  // namespace

CompositeSystem::CompositeSystem() : runtime_(std::make_unique<LoopbackRuntime>()) {}

bool CompositeSystem::set_runtime(std::unique_ptr<RuntimePort> runtime) noexcept {
  if (initialized_ || runtime == nullptr) return false;
  runtime_ = std::move(runtime);
  return true;
}

bool CompositeSystem::validate_info(
    const hardware_interface::HardwareInfo& info) const noexcept {
  if (info.name.empty() || info.joints.empty() || !info.sensors.empty() ||
      !info.gpios.empty()) {
    return false;
  }
  const std::vector<std::string> state_interfaces{
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT};
  for (std::size_t index = 0U; index < info.joints.size(); ++index) {
    const auto& joint = info.joints[index];
    if (joint.name.empty() ||
        !exactly_interfaces(joint, state_interfaces)) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      if (info.joints[previous].name == joint.name) return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn CompositeSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (!validate_info(info) || runtime_ == nullptr ||
      hardware_interface::SystemInterface::on_init(info) !=
          hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  joint_names_.clear();
  joint_names_.reserve(info.joints.size());
  joint_command_interface_names_.clear();
  joint_command_interface_names_.reserve(info.joints.size());
  for (const auto& joint : info.joints) {
    joint_names_.push_back(joint.name);
    // The motion interface, not command_interfaces[0]: ADR-017 lets the
    // generation interface appear in either position, and validate_info has
    // already established that exactly one of each is present.
    for (const auto& command : joint.command_interfaces) {
      if (command.name != kCommandGenerationInterface) {
        joint_command_interface_names_.push_back(command.name);
        break;
      }
    }
  }
  commands_.assign(info.joints.size(), CanonicalCommand{});
  states_.assign(info.joints.size(), CanonicalState{});
  generations_.assign(info.joints.size(), 0.0);
  authorized_.assign(info.joints.size(), 0U);
  generation_claimed_.assign(info.joints.size(), 0U);
  generation_baseline_.assign(info.joints.size(), 0.0);
  dispatch_.assign(info.joints.size(), CommandDispatch{});
  initialized_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CompositeSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  result.reserve(joint_names_.size() * 3U);
  for (std::size_t index = 0U; index < joint_names_.size(); ++index) {
    result.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION,
                        &states_[index].position);
    result.emplace_back(joint_names_[index], hardware_interface::HW_IF_VELOCITY,
                        &states_[index].velocity);
    result.emplace_back(joint_names_[index], hardware_interface::HW_IF_EFFORT,
                        &states_[index].effort);
  }
  return result;
}

std::vector<hardware_interface::CommandInterface>
CompositeSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  result.reserve(joint_names_.size() * 2U);
  for (std::size_t index = 0U; index < joint_names_.size(); ++index) {
    const std::string& interface_name = joint_command_interface_names_[index];
    result.emplace_back(
        joint_names_[index], interface_name,
        command_member(commands_[index], interface_name));
    // ADR-017 Decision 1: always exported. Whether a controller claims it
    // decides the joint's protection tier, but that decision belongs to the
    // controller, not to the deployment - so the offer is unconditional.
    result.emplace_back(joint_names_[index], kCommandGenerationInterface,
                        &generations_[index]);
  }
  return result;
}

void CompositeSystem::revoke_all_authorization() noexcept {
  // ADR-015 Decision 3: revocation is immediate. Telling the runtime to drop
  // each pending command is what separates "the controller stopped" from
  // "the last command still has a few milliseconds of TTL left" - waiting for
  // the hard TTL to lapse leaves a window in which frames still go out.
  std::fill(authorized_.begin(), authorized_.end(), 0U);
  // ADR-017: the tier is a property of the claim, so it dies with the claim.
  // A joint that comes back in the weak tier must not inherit the strong
  // tier's bookkeeping from whoever held it last.
  std::fill(generation_claimed_.begin(), generation_claimed_.end(), 0U);
  // Decision 3.4: re-seed from the buffer rather than zeroing. Zero is a value
  // a controller could legitimately be sitting on, and treating it as "no
  // generation yet" would make that controller's next identical write look
  // like a refresh.
  generation_baseline_ = generations_;
  if (runtime_ == nullptr) return;
  for (std::size_t index = 0U; index < authorized_.size(); ++index) {
    runtime_->cancel_pending(index);
  }
}

hardware_interface::CallbackReturn CompositeSystem::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!initialized_ || active_ || !runtime_->configure(joint_names_.size())) {
    fault_latched_ = true;
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::fill(commands_.begin(), commands_.end(), CanonicalCommand{});
  std::fill(states_.begin(), states_.end(), CanonicalState{});
  std::fill(authorized_.begin(), authorized_.end(), 0U);
  configured_ = true;
  fault_latched_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CompositeSystem::on_cleanup(
    const rclcpp_lifecycle::State&) {
  if (active_) return hardware_interface::CallbackReturn::ERROR;
  revoke_all_authorization();
  runtime_->stop();
  configured_ = false;
  fault_latched_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CompositeSystem::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!configured_ || active_ || fault_latched_ || !runtime_->start()) {
    fault_latched_ = true;
    return hardware_interface::CallbackReturn::ERROR;
  }
  active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CompositeSystem::on_deactivate(
    const rclcpp_lifecycle::State&) {
  if (!configured_) return hardware_interface::CallbackReturn::ERROR;
  revoke_all_authorization();
  runtime_->stop();
  active_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CompositeSystem::on_error(
    const rclcpp_lifecycle::State&) {
  revoke_all_authorization();
  runtime_->stop();
  active_ = false;
  configured_ = false;
  fault_latched_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::optional<std::size_t> CompositeSystem::resolve_joint_index(
    const std::string& name) const noexcept {
  // Joint names may contain '/', so split the interface suffix off from the
  // right rather than the left (e.g. "arm/j1/position" -> joint "arm/j1").
  const auto slash = name.rfind('/');
  if (slash == std::string::npos) return std::nullopt;
  const auto joint = name.substr(0U, slash);
  const auto found = std::find(joint_names_.begin(), joint_names_.end(), joint);
  if (found == joint_names_.end()) return std::nullopt;
  return static_cast<std::size_t>(found - joint_names_.begin());
}

const std::string& CompositeSystem::joint_command_interface_name(
    std::size_t index) const noexcept {
  return joint_command_interface_names_[index];
}

bool CompositeSystem::known_command_interface(const std::string& name) const noexcept {
  const auto index = resolve_joint_index(name);
  if (!index.has_value()) return false;
  const std::string prefix = joint_names_[*index] + "/";
  // ADR-017: a joint now owns two command interfaces. Both are known names;
  // whether claiming them together is legal is validate_switch's business.
  return name == prefix + joint_command_interface_name(*index) ||
         name == prefix + kCommandGenerationInterface;
}

bool CompositeSystem::validate_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) const noexcept {
  // ADR-016 Decision 3: a joint whose state is not known must not be
  // claimable. Releasing a claim stays allowed - a switch that only stops
  // interfaces is always safe, and refusing it would strand a controller on a
  // device that has gone silent.
  if (!start_interfaces.empty() &&
      (runtime_ == nullptr || !runtime_->has_valid_sample())) {
    return false;
  }
  for (std::size_t index = 0U; index < start_interfaces.size(); ++index) {
    if (!known_command_interface(start_interfaces[index]) ||
        std::find(stop_interfaces.begin(), stop_interfaces.end(),
                  start_interfaces[index]) != stop_interfaces.end() ||
        std::find(start_interfaces.begin(), start_interfaces.begin() + index,
                  start_interfaces[index]) != start_interfaces.begin() + index) {
      return false;
    }
  }
  for (std::size_t index = 0U; index < stop_interfaces.size(); ++index) {
    if (!known_command_interface(stop_interfaces[index]) ||
        std::find(stop_interfaces.begin(), stop_interfaces.begin() + index,
                  stop_interfaces[index]) != stop_interfaces.begin() + index) {
      return false;
    }
  }
  // ADR-017 Decision 5: the generation interface modifies a motion command and
  // means nothing on its own, so it may only be claimed alongside the motion
  // interface of the same joint in the same switch. Holding it alone would
  // occupy a joint without ever commanding it.
  for (const auto& name : start_interfaces) {
    if (!is_generation_interface(name)) continue;
    const auto index = resolve_joint_index(name);
    if (!index.has_value()) return false;
    const std::string motion =
        joint_names_[*index] + "/" + joint_command_interface_name(*index);
    if (std::find(start_interfaces.begin(), start_interfaces.end(), motion) ==
        start_interfaces.end()) {
      return false;
    }
  }
  return true;
}

hardware_interface::return_type CompositeSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  return validate_switch(start_interfaces, stop_interfaces)
             ? hardware_interface::return_type::OK
             : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type CompositeSystem::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  if (!active_ || !validate_switch(start_interfaces, stop_interfaces)) {
    return hardware_interface::return_type::ERROR;
  }
  // Motion and generation interfaces resolve to the SAME joint index, so they
  // need separate ledgers: booking both into authorized_ would make a
  // strong-tier controller look like it claimed one joint twice.
  auto next = authorized_;
  auto next_generation = generation_claimed_;
  for (const auto& name : stop_interfaces) {
    const auto index = resolve_joint_index(name);
    // validate_switch() already checked known_command_interface(), but never
    // index using a value derived from a failed lookup -- defence in depth.
    if (!index.has_value()) return hardware_interface::return_type::ERROR;
    auto& ledger = is_generation_interface(name) ? next_generation : next;
    if (ledger[*index] == 0U) return hardware_interface::return_type::ERROR;
    ledger[*index] = 0U;
  }
  for (const auto& name : start_interfaces) {
    const auto index = resolve_joint_index(name);
    if (!index.has_value()) return hardware_interface::return_type::ERROR;
    auto& ledger = is_generation_interface(name) ? next_generation : next;
    if (ledger[*index] != 0U) return hardware_interface::return_type::ERROR;
    ledger[*index] = 1U;
  }
  authorized_ = std::move(next);
  generation_claimed_ = std::move(next_generation);
  // ADR-017 Decision 3.4: every joint whose claim just changed hands starts
  // from the buffer's current value. Without this, a re-claimed joint would
  // treat the previous controller's leftover generation as a fresh command and
  // replay its target - the same symptom ADR-015 could not close, because
  // revocation deliberately keeps the last command value.
  for (const auto& names : {std::cref(stop_interfaces), std::cref(start_interfaces)}) {
    for (const auto& name : names.get()) {
      const auto index = resolve_joint_index(name);
      if (index.has_value()) generation_baseline_[*index] = generations_[*index];
    }
  }
  // ADR-015 Decision 3: releasing a claim drops that joint's pending command
  // now. Resolved a second time rather than collected above, so a rejected
  // switch cannot cancel a live command. A joint appearing in both lists ends
  // up authorized again but still loses its pre-switch command - re-claiming
  // does not inherit the previous controller's target.
  for (const auto& name : stop_interfaces) {
    const auto index = resolve_joint_index(name);
    if (index.has_value()) runtime_->cancel_pending(*index);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CompositeSystem::read(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!active_ || fault_latched_ || !runtime_->read(states_.data(), states_.size())) {
    fault_latched_ = true;
    return hardware_interface::return_type::ERROR;
  }
  // A device adapter decoding a corrupt frame must not be able to push
  // NaN/Inf into exported ros2_control state interfaces unnoticed.
  for (const auto& state : states_) {
    if (!std::isfinite(state.position) || !std::isfinite(state.velocity) ||
        !std::isfinite(state.effort)) {
      fault_latched_ = true;
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CompositeSystem::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
  if (!active_ || fault_latched_) return hardware_interface::return_type::ERROR;
  // Every element of commands_ is validated, including unauthorized ones: a
  // controller that wrote a non-finite value into an interface it does not
  // hold is still a defect worth latching, and all three members matter
  // because the runtime maps them per sub-mode.
  for (const auto& command : commands_) {
    if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
        !std::isfinite(command.effort)) {
      fault_latched_ = true;
      return hardware_interface::return_type::ERROR;
    }
  }
  // ADR-015: a joint without transmit authorization contributes no command.
  // Passing it down as CanonicalCommand{0, 0, 0} would make an unclaimed
  // position joint look like a commanded move to the zero position, which is
  // what the 2026-09-12 audit reproduced on the real code path.
  bool any_authorized = false;
  for (std::size_t index = 0U; index < commands_.size(); ++index) {
    const bool authorized = authorized_[index] != 0U;
    dispatch_[index].command = authorized ? commands_[index] : CanonicalCommand{};
    dispatch_[index].authorized = authorized;
    // ADR-017 Decision 4: in the weak tier every authorized cycle counts as a
    // refresh. That is the pre-ADR-017 behaviour and the accepted risk - the
    // hardware genuinely cannot tell, and refusing to send would break every
    // standard ros2_control controller.
    bool fresh = authorized;
    if (authorized && generation_claimed_[index] != 0U) {
      // Decision 3.2: inequality, not ordering - a controller that restarts
      // its counter from any value is still saying something new, and
      // requiring global monotonicity would demand bookkeeping across
      // controller lifetimes that nothing guarantees.
      //
      // Decision 3.5: a non-finite generation is "cannot tell", which is not a
      // change. It is deliberately not a latched fault either: unlike a
      // non-finite motion command it commands nothing, so the fail-closed
      // answer is to send nothing rather than to take the device down. The
      // baseline is left alone so a later finite value still reads as new.
      const double generation = generations_[index];
      fresh = std::isfinite(generation) &&
              generation != generation_baseline_[index];
      if (fresh) generation_baseline_[index] = generation;
    }
    dispatch_[index].fresh = fresh;
    any_authorized = any_authorized || authorized;
  }
  // Nothing is authorized: there is no command to hand down at all. Skipping
  // the call (rather than passing an all-unauthorized array) keeps "the
  // controller_manager cycled" from reaching the device layer as an event.
  if (!any_authorized) return hardware_interface::return_type::OK;
  if (!runtime_->write(dispatch_.data(), dispatch_.size())) {
    fault_latched_ = true;
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mech::mech_hardware_ros2_control

PLUGINLIB_EXPORT_CLASS(mech::mech_hardware_ros2_control::CompositeSystem,
                       hardware_interface::SystemInterface)
