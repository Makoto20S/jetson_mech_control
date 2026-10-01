#include "mech_bringup/ak30_servo_system.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <fstream>
#include "mech_bringup/serial_trace.hpp"

#include "rclcpp/logging.hpp"
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
  if (command_trace_) {
    try {
      std::ofstream output(command_trace_path_);
      command_trace_->dump(output);
      output.flush();
      if (!output) RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.ak30_servo"), "Failed to save command trace");
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.ak30_servo"), "Failed to save command trace");
    }
  }
  // Shutdown only: never format records or touch the file in the control loop.
  if (const auto trace = std::dynamic_pointer_cast<SerialTrace>(serial_); trace && !serial_trace_path_.empty()) {
    try {
      std::ofstream output(serial_trace_path_);
      trace->dump(output);
      output.flush();
      if (!output) RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.ak30_servo"),
                               "Failed to save serial trace");
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.ak30_servo"), "Failed to save serial trace");
    }
  }
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
  if (const auto path = std::getenv("MECH_SERVO_CHAIN_PATH"); path && *path) {
    command_trace_path_ = path;
    command_trace_ = std::make_unique<CommandTrace>();
    if (auto port = std::dynamic_pointer_cast<PosixCdcSerialPort>(serial_))
      port->set_command_trace(command_trace_.get());
    for (std::size_t i = 0; i < parsed->config.joints.size(); ++i) {
      const auto& j = parsed->config.joints[i];
      command_trace_->record("config_target", i, 0, j.target_rad_to_deg.scale,
          j.target_rad_to_deg.offset, j.drive_id, j.command_ttl_ns);
      command_trace_->record("config_limits", i, 0, j.speed_erpm,
          j.acceleration_raw, 0, j.command_hard_ttl_ns);
      command_trace_->record("config_feedback", i, 0, j.feedback_deg_to_rad.scale,
          j.feedback_deg_to_rad.offset, 0, j.feedback_ttl_ns);
    }
  }
  if (const auto path = std::getenv("MECH_SERVO_TRACE_PATH"); path && *path) {
    serial_trace_path_ = path;
    serial_ = std::make_shared<SerialTrace>(serial_, command_trace_ ? 131072U : 32768U,
                                            command_trace_.get());
  }
  // Chain capture also works without the separate raw RX/TX file.
  if (command_trace_ && !std::dynamic_pointer_cast<SerialTrace>(serial_))
    serial_ = std::make_shared<SerialTrace>(serial_, 1U, command_trace_.get());
  // Serial wrapping must precede construction of the transport borrowing it.
  transport_ = std::make_unique<mech_control_core::UsbCdcTransport>(
      *serial_, transport_options(parsed->config.logical_bus));
  mech_control_core::Transport* runtime_transport = transport_.get();
  if (command_trace_) {
    traced_transport_ = std::make_unique<CommandTraceTransport>(*transport_, *command_trace_);
    runtime_transport = traced_transport_.get();
  }
  if (!clock_) clock_ = steady_now;
  if (registry_ == nullptr) registry_ = &shared_registry();
  auto runtime = std::make_unique<Ak30ServoRuntime>(
      *runtime_transport, clock_, parsed->config, *registry_, command_trace_.get());
  runtime_view_ = runtime.get();
  if (!set_runtime(std::move(runtime))) {
    runtime_view_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto result = CompositeSystem::on_init(info);
  if (command_trace_ && result == hardware_interface::CallbackReturn::SUCCESS) {
    trace_commands_ = CompositeSystem::export_command_interfaces();
    trace_states_ = CompositeSystem::export_state_interfaces();
  }
  return result;
}

hardware_interface::CallbackReturn Ak30ServoSystem::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  const auto result = CompositeSystem::on_activate(previous_state);
  if (result != hardware_interface::CallbackReturn::SUCCESS) return result;
  first_fault_reported_ = false;
  if (!send_pass_through_init(*serial_)) {
    (void)CompositeSystem::on_error(previous_state);
    return hardware_interface::CallbackReturn::ERROR;
  }
  ++pass_through_frames_;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Ak30ServoSystem::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  if (command_trace_) ++command_trace_->cycle;
  const auto result = CompositeSystem::read(time, period);
  if (result == hardware_interface::return_type::ERROR) report_first_fault();
  return result;
}

hardware_interface::return_type Ak30ServoSystem::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  capture_interfaces("interface");
  const auto result = CompositeSystem::write(time, period);
  if (result == hardware_interface::return_type::ERROR) report_first_fault();
  return result;
}

void Ak30ServoSystem::capture_interfaces(const char* stage) {
  if (!command_trace_) return;
  std::size_t joint = 0;
  for (const auto& command : trace_commands_) {
    if (command.get_interface_name() != "position") continue;
    const double measured = joint < trace_states_.size() ? trace_states_[joint].get_value() : 0;
    command_trace_->record(stage, joint, 0, command.get_value(), measured,
                           authorized(joint) ? 1 : 0);
    ++joint;
  }
}

hardware_interface::return_type Ak30ServoSystem::perform_command_mode_switch(
    const std::vector<std::string>& start, const std::vector<std::string>& stop) {
  capture_interfaces("claim_before");
  const auto result = CompositeSystem::perform_command_mode_switch(start, stop);
  capture_interfaces(result == hardware_interface::return_type::OK ? "claim_after" : "claim_rejected");
  return result;
}

void Ak30ServoSystem::report_first_fault() {
  if (first_fault_reported_ || runtime_view_ == nullptr || !runtime_view_->first_fault()) return;
  first_fault_reported_ = true;
  const auto& fault = *runtime_view_->first_fault();
  // Runtime callbacks have returned and revoked/cancelled the group before
  // this one-time error report. Normal read/write cycles perform no logging.
  RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.ak30_servo"),
      "AK30 servo first fault reason=%s phase=%s index=%zu drive_id=%u "
      "monotonic_ns=%" PRId64 " previous_ns=%" PRId64
      " soft_deadline_ns=%" PRId64 " hard_deadline_ns=%" PRId64
      " target_rad=%.17g feedback_rad=%.17g feedback_deg=%.17g "
      "max_target_error_rad=%.17g feedback_rx_ns=%" PRId64
      " availability=%u raw_status=%u adapter_result=%u runtime_result=%u",
      fault_reason_name(fault.reason), fault_phase_name(fault.phase), fault.index,
      static_cast<unsigned>(fault.drive_id), fault.now.nanoseconds(),
      fault.previous_time_ns, fault.soft_deadline_ns, fault.hard_deadline_ns,
      fault.target_position_rad, fault.feedback.position_rad,
      fault.feedback.feedback_position_deg, fault.max_target_error_rad,
      fault.feedback.host_rx_time ? fault.feedback.host_rx_time->nanoseconds() : 0,
      static_cast<unsigned>(fault.feedback.availability),
      static_cast<unsigned>(fault.feedback.raw_status),
      static_cast<unsigned>(fault.adapter_result), static_cast<unsigned>(fault.runtime_result));
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
