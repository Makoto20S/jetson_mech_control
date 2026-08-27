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
};

struct CanonicalState final {
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
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
  [[nodiscard]] virtual bool write(const CanonicalCommand* commands,
                                   std::size_t count) noexcept = 0;
};

class CompositeSystem final : public hardware_interface::SystemInterface {
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

 private:
  [[nodiscard]] bool validate_info(
      const hardware_interface::HardwareInfo& info) const noexcept;
  [[nodiscard]] bool validate_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) const noexcept;
  [[nodiscard]] bool known_command_interface(const std::string& name) const noexcept;
  // Resolves "<joint>/<interface>" to an index into joint_names_. Joint names
  // may themselves contain '/', so the interface suffix is split off from the
  // right; returns std::nullopt if no known joint matches the resolved prefix.
  [[nodiscard]] std::optional<std::size_t> resolve_joint_index(
      const std::string& name) const noexcept;

  std::unique_ptr<RuntimePort> runtime_;
  std::vector<std::string> joint_names_;
  std::vector<CanonicalCommand> commands_;
  std::vector<CanonicalState> states_;
  std::vector<bool> claimed_;
  bool initialized_{false};
  bool configured_{false};
  bool active_{false};
  bool fault_latched_{false};
};

}  // namespace mech::mech_hardware_ros2_control
