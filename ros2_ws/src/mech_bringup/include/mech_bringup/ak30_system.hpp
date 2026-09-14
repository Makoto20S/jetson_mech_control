#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "mech_bringup/ak30_force_runtime.hpp"
#include "mech_bringup/ak30_runtime_params.hpp"
#include "mech_control_core/status.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"

namespace mech::mech_bringup {

// The production composition point between the launch path and real AK3.0
// hardware (the gap recorded in architecture_package_map.md section 5.1).
// A pluginlib-constructed CompositeSystem necessarily runs its built-in
// loopback runtime; this subclass replaces that with the bench-proven chain
// PosixCdcSerialPort -> UsbCdcTransport -> Ak30ForceControlRuntime, injected
// via set_runtime() before the base on_init() runs (set_runtime is refused
// once initialized - the same seam the integration tests use).
//
// Object construction vs device I/O (the runtime borrows a Transport&, so
// the transport object must live at one stable address from on_init until
// destruction):
//   - on_init:      parse parameters (fail-closed), construct the serial
//                   port and transport objects (pure construction - a
//                   PosixCdcSerialPort constructor only stores the path and
//                   UsbCdcTransport's only stores references; no device is
//                   touched), build the runtime over *transport_, inject,
//                   delegate to the base.
//   - on_configure: transport_->open() and the 0x12 pass-through init are
//                   sent here - the first moment any device I/O happens,
//                   matching ros2_control's "INACTIVE = communication
//                   started" and the probe's proven order
//                   (open -> init -> session configure -> activate).
//   - on_cleanup:   delegate (the base stops the runtime), then close the
//                   channel without destroying the objects, so a later
//                   on_configure reopens the same transport the runtime
//                   references. Closing never happens in on_deactivate -
//                   deactivate->activate re-entry is a legal lifecycle.
//
// Destruction order is safe: derived members (transport_, then serial_) are
// destroyed before the base's runtime_, and the runtime guards every
// transport access behind its started_ flag with stop() already run, so it
// never touches the transport after teardown - the invariant the probes
// relied on, re-verified under ASan by the plugin tests.
//
// Subclassing (rather than delegating) inherits the tested lifecycle, claim,
// switch, and watchdog machinery from CompositeSystem; this is a composition
// point, not a brand branch. The brand stays confined to mech_bringup and
// mech_protocol_cubemars (AdapterContract item 1).
class Ak30System final : public mech_hardware_ros2_control::CompositeSystem {
 public:
  // Builds a serial port for a device path. Injectable so offline tests can
  // substitute mech_simulation::FakeSerial and assert the 0x12 init frame
  // byte-for-byte without ever opening a real device; production always uses
  // the PosixCdcSerialPort default. Must be set before on_init, the same
  // discipline as set_runtime().
  //
  // shared_ptr (not unique_ptr) because std::function requires a
  // copy-constructible target: a lambda that moves out a unique_ptr cannot
  // be stored. The plugin still owns exactly one serial port per on_init;
  // the factory itself may be copied freely.
  using SerialPortFactory =
      std::function<std::shared_ptr<mech::mech_control_core::CdcSerialPort>(
          const std::string&)>;

  // ADR-016 Decision 5: exactly the four fields that must survive the ROS
  // boundary - quality, sequence, host_rx_time, raw_fault_code. Age is not a
  // field because it is derivable from consecutive host arrival times, and the
  // ADR names four things, not five.
  struct FeedbackTelemetry final {
    std::uint64_t sequence{0U};
    std::int64_t host_rx_nanoseconds{0};
    mech::mech_control_core::SampleQuality quality{};
    mech::mech_control_core::DeviceState device_state{};
    std::uint32_t raw_fault_code{0U};
  };

  // Where a telemetry record goes. The default writes one structured line to
  // the ROS logger; tests substitute a recorder so "emitted once per new frame"
  // is an assertion about records rather than about log text.
  using TelemetrySink = std::function<void(const FeedbackTelemetry&)>;

  Ak30System() noexcept;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  // Vendor pass-through init frames this plugin has sent. Separate from the
  // runtime's motor command count on purpose: the 0x12 frame configures the USB
  // box and is not a motor command, and T6 requires the two to be counted apart
  // rather than summed into one "TX" number.
  [[nodiscard]] std::uint64_t pass_through_frames() const noexcept {
    return pass_through_frames_;
  }

  // Tests only; call before on_init (mirrors set_runtime's precondition).
  void set_serial_port_factory_for_testing(SerialPortFactory factory) noexcept;

  // Tests only; call before on_init.
  void set_telemetry_sink_for_testing(TelemetrySink sink) noexcept;

 private:
  void log_frame_summary(const char* reason) const noexcept;

  SerialPortFactory serial_factory_;
  std::string device_path_;
  // Declaration order matters: transport_ must be destroyed before serial_
  // (it borrows the port), and both before the base's runtime_.
  std::shared_ptr<mech::mech_control_core::CdcSerialPort> serial_;
  std::unique_ptr<mech::mech_control_core::UsbCdcTransport> transport_;
  // Non-owning; the base owns the runtime. Used only to read counters and the
  // status snapshot for diagnostics, never to drive the device.
  Ak30ForceControlRuntime* runtime_view_{nullptr};
  TelemetrySink telemetry_sink_;
  bool telemetry_enabled_{false};
  std::uint64_t last_emitted_sequence_{0U};
  bool has_emitted_{false};
  std::uint64_t pass_through_frames_{0U};
};

}  // namespace mech::mech_bringup
