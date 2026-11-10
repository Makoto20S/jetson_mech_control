// The production composition point: pluginlib loads this class so the
// launch path constructs the real PosixCdcSerialPort -> UsbCdcTransport ->
// Ak30ForceControlRuntime chain instead of CompositeSystem's built-in
// loopback runtime. Lifecycle and ordering details are documented in
// ak30_system.hpp.

#include "mech_bringup/ak30_system.hpp"
#include <algorithm>

#include <chrono>
#include <atomic>
#include <exception>
#include <fstream>
#include <thread>
#include <utility>

#include "mech_bringup/ak30_force_runtime.hpp"
#include "mech_bringup/pass_through_init.hpp"
#include "mech_bringup/seven_channel_gateway.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mech_bringup/posix_cdc_serial_port.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "realtime_tools/lock_free_queue.hpp"

namespace mech::mech_bringup {
namespace {

// Probe-proven transport options (ak30_position_probe.cpp): logical_bus 1,
// nominal_bitrate_hz 0 = the board's firmware-fixed bitrate is genuinely
// unknown and must not be claimed (ADR-012 tri-state), receive queue 128,
// board firmware 4.8.8 = the vendor's minimum, verified read-back material.
mech::mech_control_core::UsbCdcOptions probe_options(std::uint32_t bus) noexcept {
  return seven_channel_gateway_options(bus, 0U);
}

mech::mech_control_core::MonotonicTime steady_now() noexcept {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch();
  return *mech::mech_control_core::MonotonicTime::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ticks).count());
}

[[nodiscard]] const char* quality_name(
    mech::mech_control_core::SampleQuality quality) noexcept;

}  // namespace

class Ak30System::TelemetryWorker final : public FeedbackTelemetryCapture {
 public:
  static constexpr std::size_t kCapacity = 256U;

  explicit TelemetryWorker(TelemetrySink sink)
      : sink_(std::move(sink)), thread_([this]() { run(); }) {}

  ~TelemetryWorker() override {
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();
  }

  bool try_push(const FeedbackTelemetryEvent& event) noexcept override {
    if (queue_.push(event)) return true;
    dropped_.fetch_add(1U, std::memory_order_relaxed);
    pending_dropped_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }

  [[nodiscard]] std::uint64_t dropped() const noexcept {
    return dropped_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] std::uint64_t output_errors() const noexcept {
    return output_errors_.load(std::memory_order_relaxed);
  }

 private:
  void output(const FeedbackTelemetryEvent& event) noexcept {
    try {
      if (sink_) {
        sink_(event);
      } else {
        RCLCPP_INFO(
            rclcpp::get_logger("ak30_system"),
            "telemetry kind=%u reason=%u host_receive_seq=%lu host_rx_ns=%ld "
            "host_rx_available=%u observed_at_ns=%ld age_ns=%ld age_available=%u "
            "quality=%s device_state=%u fault=%u raw_erpm_available=%u raw_erpm=%.17g "
            "motor_command_frames=%lu pass_through_frames=%lu diagnostic_loss=%lu",
            static_cast<unsigned>(event.kind),
            static_cast<unsigned>(event.reason),
            static_cast<unsigned long>(event.host_receive_sequence),
            static_cast<long>(event.host_rx_nanoseconds),
            event.host_rx_available ? 1U : 0U,
            static_cast<long>(event.observed_at_nanoseconds),
            static_cast<long>(event.age_nanoseconds),
            event.age_available ? 1U : 0U, quality_name(event.quality),
            static_cast<unsigned>(event.device_state),
            static_cast<unsigned>(event.raw_fault_code),
            event.raw_erpm_available ? 1U : 0U, event.raw_erpm,
            static_cast<unsigned long>(event.motor_command_frames),
            static_cast<unsigned long>(event.pass_through_frames),
            static_cast<unsigned long>(event.diagnostic_loss_count));
      }
      emitted_.fetch_add(1U, std::memory_order_relaxed);
    } catch (const std::exception&) {
      output_errors_.fetch_add(1U, std::memory_order_relaxed);
    } catch (...) {
      output_errors_.fetch_add(1U, std::memory_order_relaxed);
    }
  }

  void report_drops() noexcept {
    const auto count = pending_dropped_.exchange(0U, std::memory_order_acq_rel);
    if (count == 0U) return;
    FeedbackTelemetryEvent event{};
    event.kind = FeedbackTelemetryKind::RuntimeError;
    event.reason = FeedbackTelemetryReason::QueueOverflow;
    event.diagnostic_loss_count = count;
    output(event);
  }

  void run() noexcept {
    while (!stop_.load(std::memory_order_acquire)) {
      FeedbackTelemetryEvent event{};
      if (queue_.pop(event)) {
        output(event);
        report_drops();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    FeedbackTelemetryEvent event{};
    while (queue_.pop(event)) output(event);
    report_drops();
  }

  // Lifecycle callbacks and the read loop can enqueue from different threads.
  realtime_tools::LockFreeMPMCQueue<FeedbackTelemetryEvent, kCapacity> queue_;
  TelemetrySink sink_;
  std::atomic<bool> stop_{false};
  std::atomic<std::uint64_t> dropped_{0U};
  std::atomic<std::uint64_t> pending_dropped_{0U};
  std::atomic<std::uint64_t> emitted_{0U};
  std::atomic<std::uint64_t> output_errors_{0U};
  std::thread thread_;
};

Ak30System::Ak30System() noexcept = default;

Ak30System::~Ak30System() {
  if (runtime_view_ != nullptr) runtime_view_->stop();
  telemetry_worker_.reset();
  flush_trace();
}

void Ak30System::flush_trace() noexcept {
  if (!command_trace_) return;
  try {
    std::ofstream output(command_trace_path_);
    command_trace_->dump(output);
    output.flush();
    if (!output) throw std::runtime_error("trace output failed");
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("ak30_system"), "MIT trace: %s", error.what());
  }
}

void Ak30System::set_serial_port_factory_for_testing(
    SerialPortFactory factory) noexcept {
  serial_factory_ = std::move(factory);
}

void Ak30System::set_telemetry_sink_for_testing(TelemetrySink sink) noexcept {
  telemetry_sink_ = std::move(sink);
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

  // ADR-014 Decision 3: the sub-mode and the URDF command interface name are
  // two spellings of the same choice, so disagreeing spellings are a
  // deployment error rather than a preference to reconcile. The base
  // CompositeSystem checks the shape but is deliberately vendor-neutral - it
  // accepts any one of the three canonical names and knows nothing about
  // sub_mode - so the correspondence can only be enforced here, where the
  // parameter is known.
  //
  // This sits ahead of the transport chain below because everything that
  // reaches the device hangs off the serial port: refusing before the port is
  // acquired is the earliest point at which a mismatched deployment can be
  // rejected with nothing transmitted, and it keeps the rejection independent
  // of whether the base happens to catch the same shape later.
  const char* const expected_command_interface =
      expected_command_interface_name(parsed->config.sub_mode);
  for (const auto& joint : info.joints) {
    // CompositeSystem validates the generic bundle shape. This AK30 boundary
    // adds the vendor sub-mode rule: Velocity and Torque remain singleton
    // modes, while Position requires position and may export velocity and/or
    // effort only with explicit positive runtime limits.
    bool has_position = false;
    bool has_velocity = false;
    bool has_effort = false;
    for (const auto& command : joint.command_interfaces) {
      if (command.name ==
          mech::mech_hardware_ros2_control::kCommandGenerationInterface) {
        continue;
      }
      if (command.name == hardware_interface::HW_IF_POSITION) {
        has_position = true;
      } else if (command.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity = true;
      } else if (command.name == hardware_interface::HW_IF_EFFORT) {
        has_effort = true;
      } else {
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    const std::size_t motion_interfaces =
        (has_position ? 1U : 0U) + (has_velocity ? 1U : 0U) +
        (has_effort ? 1U : 0U);
    if (parsed->config.sub_mode !=
        mech::mech_protocol_cubemars::ForceControlSubMode::Position) {
      if (motion_interfaces != 1U ||
          (has_position && expected_command_interface !=
              std::string(hardware_interface::HW_IF_POSITION)) ||
          (has_velocity && expected_command_interface !=
              std::string(hardware_interface::HW_IF_VELOCITY)) ||
          (has_effort && expected_command_interface !=
              std::string(hardware_interface::HW_IF_EFFORT))) {
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else if (!has_position ||
               (has_velocity &&
                parsed->config.position_max_abs_velocity_rad_s <= 0.0) ||
               (has_effort &&
                parsed->config.position_max_abs_feedforward_nm <= 0.0)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  auto canonical_info = info;
  for (auto& joint : canonical_info.joints) {
    std::array<unsigned, raw_names_.size()> found{};
    auto& interfaces = joint.state_interfaces;
    for (auto it = interfaces.begin(); it != interfaces.end();) {
      const auto name = std::find(raw_names_.begin(), raw_names_.end(), it->name);
      if (name == raw_names_.end()) {
        ++it;
      } else {
        ++found[name - raw_names_.begin()];
        it = interfaces.erase(it);
      }
    }
    if (std::any_of(found.begin(), found.end(), [](unsigned n) { return n != 0; })) {
      if (info.joints.size() != 1 ||
          parsed->config.sub_mode != mech::mech_protocol_cubemars::ForceControlSubMode::Torque ||
          !std::all_of(found.begin(), found.end(), [](unsigned n) { return n == 1; }))
        return hardware_interface::CallbackReturn::ERROR;
      raw_joint_ = joint.name;
      raw_drive_id_ = parsed->config.drive_id;
      raw_values_.fill(std::numeric_limits<double>::quiet_NaN());
    }
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
  command_trace_path_ = parsed->command_trace_path;
  if (!command_trace_path_.empty()) {
    command_trace_ = std::make_unique<CommandTrace>(131072U);
    if (auto posix = std::dynamic_pointer_cast<PosixCdcSerialPort>(serial_))
      posix->set_write_trace_sink(command_trace_.get());
  }
  transport_ =
      std::make_unique<mech::mech_control_core::UsbCdcTransport>(*serial_,
                                      probe_options(parsed->config.logical_bus));
  mech::mech_control_core::Transport* runtime_transport = transport_.get();
  if (command_trace_) {
    traced_transport_ = std::make_unique<CommandTraceTransport>(
        *transport_, *command_trace_, true);
    runtime_transport = traced_transport_.get();
  }

  // set_runtime() is refused once the base is initialized
  // (composite_system.cpp), so the runtime must be injected here, before the
  // delegation below - the same ordering the integration tests pin.
  if (parsed->feedback_telemetry_log) {
    telemetry_worker_ = std::make_unique<TelemetryWorker>(telemetry_sink_);
  }
  auto runtime = std::make_unique<Ak30ForceControlRuntime>(
      *runtime_transport, steady_now, parsed->config, telemetry_worker_.get());
  runtime_view_ = runtime.get();
  if (!set_runtime(std::move(runtime))) {
    runtime_view_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }
  return CompositeSystem::on_init(canonical_info);
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
  // Counted here because this is the only place the plugin emits it, and T6
  // requires it to stay distinguishable from motor command frames.
  ++pass_through_frames_;
  return CompositeSystem::on_configure(previous_state);
}

hardware_interface::CallbackReturn Ak30System::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  raw_values_[3] = 0.0;
  const auto result = CompositeSystem::on_cleanup(previous_state);
  // The base's runtime->stop() has run, so the runtime no longer touches the
  // transport; close the channel but keep the objects alive at their stable
  // addresses (the runtime still references transport_), so a later
  // on_configure reopens the same chain. on_deactivate deliberately does not
  // close: deactivate -> activate re-entry must keep the channel alive.
  serial_->close();
  enqueue_summary(FeedbackTelemetryReason::Cleanup);
  flush_trace();
  return result;
}

namespace {

[[nodiscard]] const char* quality_name(
    mech::mech_control_core::SampleQuality quality) noexcept {
  switch (quality) {
    case mech::mech_control_core::SampleQuality::Valid:    return "Valid";
    case mech::mech_control_core::SampleQuality::Degraded: return "Degraded";
    case mech::mech_control_core::SampleQuality::Stale:    return "Stale";
    case mech::mech_control_core::SampleQuality::Unknown:  return "Unknown";
    case mech::mech_control_core::SampleQuality::Invalid:  return "Invalid";
  }
  return "Unhandled";
}

}  // namespace

hardware_interface::return_type Ak30System::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto result = CompositeSystem::read(time, period);
  if (!raw_joint_.empty()) {
    if (runtime_view_->raw_feedback_available()) {
      const auto& f = runtime_view_->raw_feedback();
      const auto& status = runtime_view_->raw_feedback_status();
      const bool ok = result == hardware_interface::return_type::OK &&
                      runtime_view_->has_valid_sample() && f.raw_status == 0 &&
                      runtime_view_->last_status().device_state != mech::mech_control_core::DeviceState::Fault;
      raw_values_ = {{static_cast<double>(raw_drive_id_), static_cast<double>(status.sequence),
          static_cast<double>(status.host_rx_time->nanoseconds() / 1000), ok ? 1. : 0.,
          f.position_deg, f.electrical_speed_erpm, f.current_iq_a, f.board_temperature_c,
          static_cast<double>(f.raw_status)}};
    } else {
      raw_values_[3] = 0.0;
    }
  }
  return result;
}

// One summary line on the way out, on both the normal and the failing path.
// T6 had to reconstruct these numbers from an external trace; a run that ends -
// cleanly or in error - should say how many motor commands it sent without
// anyone having to prepare a measurement first.
hardware_interface::CallbackReturn Ak30System::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  raw_values_[3] = 0.0;
  const auto result = CompositeSystem::on_deactivate(previous_state);
  enqueue_summary(FeedbackTelemetryReason::Deactivate);
  return result;
}

hardware_interface::CallbackReturn Ak30System::on_error(
    const rclcpp_lifecycle::State& previous_state) {
  raw_values_[3] = 0.0;
  const auto result = CompositeSystem::on_error(previous_state);
  enqueue_summary(FeedbackTelemetryReason::Error);
  flush_trace();
  return result;
}

hardware_interface::CallbackReturn Ak30System::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  raw_values_[3] = 0.0;
  if (runtime_view_ != nullptr) runtime_view_->stop();
  const auto result = CompositeSystem::on_shutdown(previous_state);
  if (serial_) serial_->close();
  enqueue_summary(FeedbackTelemetryReason::Cleanup);
  flush_trace();
  return result;
}

std::vector<hardware_interface::StateInterface> Ak30System::export_state_interfaces() {
  auto states = CompositeSystem::export_state_interfaces();
  if (!raw_joint_.empty()) {
    for (std::size_t i = 0; i < raw_names_.size(); ++i)
      states.emplace_back(raw_joint_, raw_names_[i], &raw_values_[i]);
  }
  return states;
}

void Ak30System::enqueue_summary(FeedbackTelemetryReason reason) noexcept {
  if (telemetry_worker_ == nullptr) return;
  FeedbackTelemetryEvent event{};
  event.kind = FeedbackTelemetryKind::LifecycleSummary;
  event.reason = reason;
  event.motor_command_frames =
      runtime_view_ == nullptr ? 0U : runtime_view_->motor_command_frames();
  event.pass_through_frames = pass_through_frames_;
  (void)telemetry_worker_->try_push(event);
}

std::uint64_t Ak30System::telemetry_dropped_events() const noexcept {
  return telemetry_worker_ == nullptr ? 0U : telemetry_worker_->dropped();
}

std::uint64_t Ak30System::telemetry_output_errors() const noexcept {
  return telemetry_worker_ == nullptr ? 0U : telemetry_worker_->output_errors();
}

}  // namespace mech::mech_bringup

PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::Ak30System,
                       hardware_interface::SystemInterface)
