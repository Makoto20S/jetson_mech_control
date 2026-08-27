#include "mech_hardware_ros2_control/composite_system.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mech::mech_hardware_ros2_control {
namespace {

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
      const auto delta = commands_[index].position - states_[index].position;
      const auto step = std::clamp(delta, -0.01, 0.01);
      states_[index].position += step;
      states_[index].velocity = step;
      states_[index].effort = 0.0;
      states[index] = states_[index];
    }
    return true;
  }

  bool write(const CanonicalCommand* commands, std::size_t count) noexcept override {
    if (!running_ || commands == nullptr || count != commands_.size()) return false;
    std::copy_n(commands, count, commands_.begin());
    return true;
  }

 private:
  bool running_{false};
  std::vector<CanonicalState> states_;
  std::vector<CanonicalCommand> commands_;
};

bool exactly_interfaces(const hardware_interface::ComponentInfo& joint,
                        const std::vector<std::string>& expected_state,
                        const std::vector<std::string>& expected_command) {
  if (joint.state_interfaces.size() != expected_state.size() ||
      joint.command_interfaces.size() != expected_command.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < expected_state.size(); ++index) {
    if (joint.state_interfaces[index].name != expected_state[index]) return false;
  }
  for (std::size_t index = 0U; index < expected_command.size(); ++index) {
    if (joint.command_interfaces[index].name != expected_command[index]) return false;
  }
  return true;
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
  const std::vector<std::string> command_interfaces{
      hardware_interface::HW_IF_POSITION};
  for (std::size_t index = 0U; index < info.joints.size(); ++index) {
    const auto& joint = info.joints[index];
    if (joint.name.empty() ||
        !exactly_interfaces(joint, state_interfaces, command_interfaces)) {
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
  for (const auto& joint : info.joints) joint_names_.push_back(joint.name);
  commands_.assign(info.joints.size(), CanonicalCommand{});
  states_.assign(info.joints.size(), CanonicalState{});
  claimed_.assign(info.joints.size(), false);
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
  result.reserve(joint_names_.size());
  for (std::size_t index = 0U; index < joint_names_.size(); ++index) {
    result.emplace_back(joint_names_[index], hardware_interface::HW_IF_POSITION,
                        &commands_[index].position);
  }
  return result;
}

hardware_interface::CallbackReturn CompositeSystem::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!initialized_ || active_ || !runtime_->configure(joint_names_.size())) {
    fault_latched_ = true;
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::fill(commands_.begin(), commands_.end(), CanonicalCommand{});
  std::fill(states_.begin(), states_.end(), CanonicalState{});
  std::fill(claimed_.begin(), claimed_.end(), false);
  configured_ = true;
  fault_latched_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CompositeSystem::on_cleanup(
    const rclcpp_lifecycle::State&) {
  if (active_) return hardware_interface::CallbackReturn::ERROR;
  runtime_->stop();
  configured_ = false;
  fault_latched_ = false;
  std::fill(claimed_.begin(), claimed_.end(), false);
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
  runtime_->stop();
  active_ = false;
  std::fill(claimed_.begin(), claimed_.end(), false);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CompositeSystem::on_error(
    const rclcpp_lifecycle::State&) {
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

bool CompositeSystem::known_command_interface(const std::string& name) const noexcept {
  const auto index = resolve_joint_index(name);
  if (!index.has_value()) return false;
  return name == joint_names_[*index] + "/" + hardware_interface::HW_IF_POSITION;
}

bool CompositeSystem::validate_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) const noexcept {
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
  auto next = claimed_;
  for (const auto& name : stop_interfaces) {
    const auto index = resolve_joint_index(name);
    // validate_switch() already checked known_command_interface(), but never
    // index using a value derived from a failed lookup -- defence in depth.
    if (!index.has_value()) return hardware_interface::return_type::ERROR;
    if (!next[*index]) return hardware_interface::return_type::ERROR;
    next[*index] = false;
  }
  for (const auto& name : start_interfaces) {
    const auto index = resolve_joint_index(name);
    if (!index.has_value()) return hardware_interface::return_type::ERROR;
    if (next[*index]) return hardware_interface::return_type::ERROR;
    next[*index] = true;
  }
  claimed_ = std::move(next);
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
  // Every element of commands_ is handed to runtime_->write() below,
  // including unclaimed interfaces, so every element must be validated.
  for (const auto& command : commands_) {
    if (!std::isfinite(command.position)) {
      fault_latched_ = true;
      return hardware_interface::return_type::ERROR;
    }
  }
  if (!runtime_->write(commands_.data(), commands_.size())) {
    fault_latched_ = true;
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace mech::mech_hardware_ros2_control

PLUGINLIB_EXPORT_CLASS(mech::mech_hardware_ros2_control::CompositeSystem,
                       hardware_interface::SystemInterface)
