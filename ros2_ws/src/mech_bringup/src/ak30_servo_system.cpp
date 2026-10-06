#include "mech_bringup/ak30_servo_system.hpp"

#include <chrono>
#include <charconv>
#include <cinttypes>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
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

std::size_t trace_capacity(const char* value) {
  if (!value || value[0] < '1' || value[0] > '9')
    throw std::invalid_argument("trace capacities must be positive decimal integers");
  const auto end = value + std::char_traits<char>::length(value);
  std::size_t capacity = 0;
  const auto parsed = std::from_chars(value, end, capacity);
  if (parsed.ec != std::errc{} || parsed.ptr != end)
    throw std::invalid_argument("invalid trace capacity");
  return capacity;
}

}  // namespace

Ak30ServoSystem::~Ak30ServoSystem() {
  if (runtime_view_ != nullptr) runtime_view_->stop();
  if (snapshot_mode_) {
    // No formatting, compression or disk writes before the operator's mode15
    // helper. Shared tmpfs storage outlives this process for deferred export.
    seal_snapshots();
    return;
  }
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

void Ak30ServoSystem::seal_snapshots() noexcept {
  if (!snapshot_mode_ || !trace_initialized_) return;
  if (command_trace_) command_trace_->seal_snapshot();
  if (const auto trace = std::dynamic_pointer_cast<SerialTrace>(serial_))
    trace->seal_snapshot();
}

hardware_interface::CallbackReturn Ak30ServoSystem::on_shutdown(
    const rclcpp_lifecycle::State&) {
  // Humble can finish the hardware lifecycle and exit on SIGINT without
  // destroying this plugin. The manager serializes this terminal callback
  // with read/write; FINALIZED prevents later updates. Ordinary deactivate
  // must not seal because a component can be activated again.
  if (runtime_view_) runtime_view_->stop();
  seal_snapshots();
  return hardware_interface::CallbackReturn::SUCCESS;
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

  std::size_t chain_capacity = 524288;
  std::size_t raw_capacity = 32768;
  std::shared_ptr<SerialTrace> mapped_serial;
  try {
    const auto snapshot_dir = std::getenv("MECH_SERVO_SNAPSHOT_DIR");
    if (snapshot_dir) {
      if (!*snapshot_dir) throw std::invalid_argument("empty snapshot directory");
      snapshot_mode_ = true;
      command_trace_path_ = std::string(snapshot_dir) + "/chain.snapshot";
      serial_trace_path_ = std::string(snapshot_dir) + "/serial.snapshot";
      chain_capacity = kDefaultSnapshotChainCapacity;
      raw_capacity = kDefaultSnapshotRawCapacity;
    } else {
      if (const auto path = std::getenv("MECH_SERVO_CHAIN_PATH"); path && *path)
        command_trace_path_ = path;
      if (const auto path = std::getenv("MECH_SERVO_TRACE_PATH"); path && *path)
        serial_trace_path_ = path;
      if (!command_trace_path_.empty()) raw_capacity = 131072;
    }
    const auto chain_env = std::getenv("MECH_SERVO_CHAIN_CAPACITY");
    const auto raw_env = std::getenv("MECH_SERVO_RAW_CAPACITY");
    if (chain_env || raw_env) {
      if (!chain_env || !raw_env)
        throw std::invalid_argument("both trace capacity variables are required together");
      chain_capacity = trace_capacity(chain_env);
      raw_capacity = trace_capacity(raw_env);
    }
    if (chain_capacity > kTraceSnapshotBudget / CommandTrace::record_bytes ||
        raw_capacity > kTraceSnapshotBudget / SerialTrace::record_bytes)
      throw std::invalid_argument("trace capacity exceeds memory budget");
    const auto bytes = chain_capacity * CommandTrace::record_bytes +
        raw_capacity * SerialTrace::record_bytes + 2 * sizeof(TraceSnapshotHeader);
    if (bytes > kTraceSnapshotBudget)
      throw std::invalid_argument("combined trace memory exceeds 1536 MiB");
    if (snapshot_mode_) admit_trace_snapshots(snapshot_dir, bytes);
    if (!command_trace_path_.empty())
      command_trace_ = std::make_unique<CommandTrace>(chain_capacity,
          snapshot_mode_ ? command_trace_path_ : std::string{});
    if (snapshot_mode_) {
      mapped_serial = std::make_shared<SerialTrace>(nullptr, raw_capacity,
          command_trace_.get(), serial_trace_path_);
      serial_ = mapped_serial;
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.ak30_servo"),
                 "Trace admission failed before serial construction: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!serial_factory_) {
    serial_factory_ = [](const std::string& path) {
      return std::make_shared<PosixCdcSerialPort>(path);
    };
  }
  auto port = serial_factory_(parsed->device_path);
  if (!port) return hardware_interface::CallbackReturn::ERROR;
  if (command_trace_) {
    if (auto posix = std::dynamic_pointer_cast<PosixCdcSerialPort>(port))
      posix->set_command_trace(command_trace_.get());
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
  if (snapshot_mode_) {
    if (!mapped_serial->attach_port(std::move(port)))
      return hardware_interface::CallbackReturn::ERROR;
  } else {
    serial_ = std::move(port);
    if (!serial_trace_path_.empty())
      serial_ = std::make_shared<SerialTrace>(serial_, raw_capacity, command_trace_.get());
  }
  // Chain capture also works without the separate raw RX/TX file.
  if (command_trace_ && !std::dynamic_pointer_cast<SerialTrace>(serial_))
    serial_ = std::make_shared<SerialTrace>(serial_, 1U, command_trace_.get());
  // Serial wrapping must precede construction of the transport borrowing it.
  transport_ = std::make_unique<mech_control_core::UsbCdcTransport>(
      *serial_, transport_options(parsed->config.logical_bus), clock_);
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
  trace_initialized_ = result == hardware_interface::CallbackReturn::SUCCESS;
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
