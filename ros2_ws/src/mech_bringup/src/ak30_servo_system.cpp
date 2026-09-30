#include "mech_bringup/ak30_servo_system.hpp"

#include <chrono>
#include <utility>

#include "mech_bringup/ak30_servo_runtime_params.hpp"
#include "mech_bringup/pass_through_init.hpp"
#include "mech_bringup/posix_cdc_serial_port.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mech::mech_bringup {
namespace {

mech_control_core::BusOwnershipRegistry& shared_registry() {
  static mech_control_core::BusOwnershipRegistry registry;
  return registry;
}

mech_control_core::MonotonicTime steady_now() noexcept {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch();
  return *mech_control_core::MonotonicTime::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ticks).count());
}

mech_control_core::UsbCdcOptions transport_options(std::uint16_t logical_bus) {
  mech_control_core::UsbCdcOptions options{};
  options.logical_bus = logical_bus;
  options.nominal_bitrate_hz = 0U;
  options.receive_queue_capacity = 128U;
  // The existing USB gateway transport profile, verified for firmware 4.8.8.
  // Construction does not query a device.
  options.verified_board_version = {4U, 8U, 8U};
  return options;
}

}  // namespace

Ak30ServoSystem::~Ak30ServoSystem() {
  if (runtime_view_ != nullptr) runtime_view_->stop();
}

void Ak30ServoSystem::set_serial_port_factory_for_testing(
    SerialPortFactory factory) noexcept {
  if (!init_attempted_) serial_factory_ = std::move(factory);
}

void Ak30ServoSystem::set_clock_for_testing(
    Ak30ServoRuntime::Clock clock) noexcept {
  if (!init_attempted_) clock_ = std::move(clock);
}

void Ak30ServoSystem::set_ownership_registry_for_testing(
    mech_control_core::BusOwnershipRegistry& registry) noexcept {
  if (!init_attempted_) registry_ = &registry;
}

hardware_interface::CallbackReturn Ak30ServoSystem::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (init_attempted_) return hardware_interface::CallbackReturn::ERROR;
  init_attempted_ = true;
  const auto parsed = Ak30ServoRuntimeParams::parse(info);
  if (!parsed) return hardware_interface::CallbackReturn::ERROR;

  if (!serial_factory_) {
    serial_factory_ = [](const std::string& path) {
      return std::make_shared<PosixCdcSerialPort>(path);
    };
  }
  serial_ = serial_factory_(parsed->device_path);
  if (!serial_) return hardware_interface::CallbackReturn::ERROR;
  transport_ = std::make_unique<mech_control_core::UsbCdcTransport>(
      *serial_, transport_options(parsed->config.logical_bus));
  if (!clock_) clock_ = steady_now;
  if (registry_ == nullptr) registry_ = &shared_registry();
  auto runtime = std::make_unique<Ak30ServoRuntime>(
      *transport_, clock_, parsed->config, *registry_);
  runtime_view_ = runtime.get();
  if (!set_runtime(std::move(runtime))) {
    runtime_view_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }
  return CompositeSystem::on_init(info);
}

hardware_interface::CallbackReturn Ak30ServoSystem::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  const auto result = CompositeSystem::on_activate(previous_state);
  if (result != hardware_interface::CallbackReturn::SUCCESS) return result;
  if (!send_pass_through_init(*serial_)) {
    (void)CompositeSystem::on_error(previous_state);
    return hardware_interface::CallbackReturn::ERROR;
  }
  ++pass_through_frames_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::optional<mech_protocol_cubemars::ServoPositionSnapshot>
Ak30ServoSystem::diagnostic_snapshot(std::size_t index) const noexcept {
  return runtime_view_ == nullptr ? std::nullopt : runtime_view_->diagnostic_snapshot(index);
}

std::uint64_t Ak30ServoSystem::motor_command_frames() const noexcept {
  return runtime_view_ == nullptr ? 0U : runtime_view_->bus_stats().tx_frames;
}

}  // namespace mech::mech_bringup

PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::Ak30ServoSystem,
                       hardware_interface::SystemInterface)
