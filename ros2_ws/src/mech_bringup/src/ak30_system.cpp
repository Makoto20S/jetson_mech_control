// The production composition point: pluginlib loads this class so the
// launch path constructs the real PosixCdcSerialPort -> UsbCdcTransport ->
// Ak30ForceControlRuntime chain instead of CompositeSystem's built-in
// loopback runtime. Lifecycle and ordering details are documented in
// ak30_system.hpp.

#include "mech_bringup/ak30_system.hpp"

#include <chrono>
#include <utility>

#include "mech_bringup/ak30_force_runtime.hpp"
#include "mech_bringup/pass_through_init.hpp"
#include "mech_bringup/posix_cdc_serial_port.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mech::mech_bringup {
namespace {

// Probe-proven transport options (ak30_position_probe.cpp): logical_bus 1,
// nominal_bitrate_hz 0 = the board's firmware-fixed bitrate is genuinely
// unknown and must not be claimed (ADR-012 tri-state), receive queue 128,
// board firmware 4.8.8 = the vendor's minimum, verified read-back material.
mech::mech_control_core::UsbCdcOptions probe_options() noexcept {
  mech::mech_control_core::UsbCdcOptions options{};
  options.logical_bus = 1U;
  options.nominal_bitrate_hz = 0U;
  options.receive_queue_capacity = 128U;
  options.verified_board_version = {4U, 8U, 8U};
  return options;
}

mech::mech_control_core::MonotonicTime steady_now() noexcept {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch();
  return *mech::mech_control_core::MonotonicTime::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ticks).count());
}

}  // namespace

Ak30System::Ak30System() noexcept = default;

void Ak30System::set_serial_port_factory_for_testing(
    SerialPortFactory factory) noexcept {
  serial_factory_ = std::move(factory);
}

hardware_interface::CallbackReturn Ak30System::on_init(
    const hardware_interface::HardwareInfo& info) {
  // The URDF's <param> entries arrive as an unordered_map; Ak30RuntimeParams
  // parses an ordered one. on_init is not real-time, so the conversion is a
  // plain copy. Fail-closed: an invalid parameter set must reject the whole
  // hardware, never run on guessed values.
  std::map<std::string, std::string> params;
  for (const auto& entry : info.hardware_parameters) {
    if (!params.emplace(entry.first, entry.second).second) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  const auto parsed = Ak30RuntimeParams::parse(params);
  if (!parsed.has_value()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  device_path_ = parsed->device_path;

  if (!serial_factory_) {
    serial_factory_ = [](const std::string& device_path) {
      return std::shared_ptr<mech::mech_control_core::CdcSerialPort>(
          std::make_shared<PosixCdcSerialPort>(device_path));
    };
  }
  // Construct the transport chain here, at one stable address for the whole
  // hardware lifetime (the runtime keeps a Transport& and the transport
  // borrows the port). Pure object construction: a PosixCdcSerialPort
  // constructor only stores the path, and UsbCdcTransport's constructor only
  // stores references and computes capabilities - no device is touched by
  // merely loading a URDF. All device I/O starts in on_configure.
  serial_ = serial_factory_(device_path_);
  if (serial_ == nullptr) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  transport_ =
      std::make_unique<mech::mech_control_core::UsbCdcTransport>(*serial_,
                                                                  probe_options());

  // set_runtime() is refused once the base is initialized
  // (composite_system.cpp), so the runtime must be injected here, before the
  // delegation below - the same ordering the integration tests pin.
  if (!set_runtime(std::make_unique<Ak30ForceControlRuntime>(
          *transport_, steady_now, parsed->config))) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return CompositeSystem::on_init(info);
}

hardware_interface::CallbackReturn Ak30System::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  // The probe's proven order: open the port, then arm pass-through mode, then
  // the base's runtime->configure() (session configure against
  // transport.capabilities()). This is the first moment any device I/O
  // happens - "INACTIVE = communication started".
  if (!transport_->open() || !send_pass_through_init(*serial_)) {
    serial_->close();
    return hardware_interface::CallbackReturn::ERROR;
  }
  return CompositeSystem::on_configure(previous_state);
}

hardware_interface::CallbackReturn Ak30System::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  const auto result = CompositeSystem::on_cleanup(previous_state);
  // The base's runtime->stop() has run, so the runtime no longer touches the
  // transport; close the channel but keep the objects alive at their stable
  // addresses (the runtime still references transport_), so a later
  // on_configure reopens the same chain. on_deactivate deliberately does not
  // close: deactivate -> activate re-entry must keep the channel alive.
  serial_->close();
  return result;
}

}  // namespace mech::mech_bringup

PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::Ak30System,
                       hardware_interface::SystemInterface)
