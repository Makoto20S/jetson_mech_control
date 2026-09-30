#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "mech_bringup/ak30_servo_runtime.hpp"
#include "mech_control_core/runtime.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"

namespace mech::mech_bringup {

// Production ROS composition for the position-only servo profile. All motor
// submissions remain in CompositeSystem -> Ak30ServoRuntime -> BusRuntime.
class Ak30ServoSystem final : public mech_hardware_ros2_control::CompositeSystem {
 public:
  using SerialPortFactory =
      std::function<std::shared_ptr<mech_control_core::CdcSerialPort>(
          const std::string&)>;

  Ak30ServoSystem() noexcept = default;
  ~Ak30ServoSystem() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  [[nodiscard]] std::uint64_t pass_through_frames() const noexcept {
    return pass_through_frames_;
  }
  [[nodiscard]] std::uint64_t motor_command_frames() const noexcept;

  [[nodiscard]] std::optional<mech_protocol_cubemars::ServoPositionSnapshot>
  diagnostic_snapshot(std::size_t index) const noexcept;

  // Test seams must be set before initialization. Production uses the
  // PosixCdcSerialPort, steady clock, and a process-wide bus registry.
  void set_serial_port_factory_for_testing(SerialPortFactory factory) noexcept;
  void set_clock_for_testing(Ak30ServoRuntime::Clock clock) noexcept;
  void set_ownership_registry_for_testing(
      mech_control_core::BusOwnershipRegistry& registry) noexcept;

 private:
  SerialPortFactory serial_factory_;
  Ak30ServoRuntime::Clock clock_;
  mech_control_core::BusOwnershipRegistry* registry_{nullptr};
  std::shared_ptr<mech_control_core::CdcSerialPort> serial_;
  std::unique_ptr<mech_control_core::UsbCdcTransport> transport_;
  // CompositeSystem owns this runtime. Stop it in the derived destructor
  // while the borrowed transport and registry are still alive.
  Ak30ServoRuntime* runtime_view_{nullptr};
  std::uint64_t pass_through_frames_{0U};
  bool init_attempted_{false};
};

}  // namespace mech::mech_bringup
