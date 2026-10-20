#include "mech_bringup/dm4340_system.hpp"

#include <charconv>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <set>
#include <thread>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mech_control_core/posix_cdc_serial_port.hpp"
#include "mech_bringup/seven_channel_gateway.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mech::mech_bringup {
namespace {
using Parameters = std::unordered_map<std::string, std::string>;

mech_control_core::BusOwnershipRegistry& shared_registry() {
  static mech_control_core::BusOwnershipRegistry registry;
  return registry;
}

mech_control_core::MonotonicTime steady_now() noexcept {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch();
  return *mech_control_core::MonotonicTime::from_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(ticks).count());
}

bool keys_are(const Parameters& params, const std::set<std::string>& keys) {
  if (params.size() != keys.size()) return false;
  for (const auto& key : keys)
    if (params.count(key) != 1U) return false;
  return true;
}

bool integer(const Parameters& params, const char* key,
             std::uint64_t& value) {
  const auto& text = params.at(key);
  if (text.empty()) return false;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

bool positive_ns(const Parameters& params, const char* key,
                 std::int64_t& value) {
  std::uint64_t parsed = 0U;
  if (!integer(params, key, parsed) || parsed == 0U ||
      parsed > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
    return false;
  value = static_cast<std::int64_t>(parsed);
  return true;
}

bool real(const Parameters& params, const char* key, double& value) {
  const auto& text = params.at(key);
  if (text.empty() ||
      text.find_first_not_of("0123456789+-.eE") != std::string::npos)
    return false;
  char* end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return end == text.c_str() + text.size() && std::isfinite(value);
}

bool declared(const Parameters& params, const char* key) {
  return params.at(key) == "true";
}

bool complete_claim_switch(const std::string& joint,
                           const std::vector<std::string>& start,
                           const std::vector<std::string>& stop,
                           Dm4340CommandMode mode) {
  const auto motion = joint + "/" +
      (mode == Dm4340CommandMode::Velocity
           ? hardware_interface::HW_IF_VELOCITY
           : hardware_interface::HW_IF_EFFORT);
  const auto generation = joint + "/" +
      mech_hardware_ros2_control::kCommandGenerationInterface;
  const auto contains = [](const std::vector<std::string>& names,
                           const std::string& name) {
    return std::find(names.begin(), names.end(), name) != names.end();
  };
  return contains(start, motion) == contains(start, generation) &&
         contains(stop, motion) == contains(stop, generation);
}

bool parse(const hardware_interface::HardwareInfo& info,
           Dm4340RuntimeConfig& config,
           mech_control_core::CdcProtocolVersion& board,
           bool& enable_on_activate) {
  static const std::set<std::string> hardware_keys{
      "profile", "device_path", "logical_bus", "control_period_ns",
      "command_ttl_ns", "command_hard_ttl_ns", "feedback_ttl_ns",
      "query_period_ns", "motor_side_timeout_verified",
      "enable_on_activate",
      "board_version_major", "board_version_minor", "board_version_patch",
      "board_version_verified", "can_bitrate_verified"};
  static const std::set<std::string> velocity_joint_keys{
      "command_can_id", "feedback_can_id", "motor_id", "firmware_id",
      "firmware_min", "firmware_max", "firmware_evidence_declared",
      "limit_position_rad", "limit_velocity_rad_s", "limit_torque_nm",
      "limits_evidence_declared", "joint_scale", "joint_offset_rad",
      "joint_mapping_verified", "position_min_rad", "position_max_rad",
      "max_target_error_rad", "kp", "kd", "velocity_feedforward_nm",
      "max_command_velocity_rad_s",
      "max_feedback_velocity_rad_s", "max_feedback_torque_nm"};
  if (info.name.empty() || info.type != "system" ||
      info.joints.empty() || info.joints.size() > 6U ||
      !info.sensors.empty() || !info.gpios.empty() ||
      !keys_are(info.hardware_parameters, hardware_keys)) return false;
  const auto& first_interfaces = info.joints.front().command_interfaces;
  if (first_interfaces.size() != 2U) return false;
  if (first_interfaces[0].name == hardware_interface::HW_IF_VELOCITY)
    config.command_mode = Dm4340CommandMode::Velocity;
  else if (first_interfaces[0].name == hardware_interface::HW_IF_EFFORT)
    config.command_mode = Dm4340CommandMode::Effort;
  else return false;
  auto joint_keys = velocity_joint_keys;
  if (config.command_mode == Dm4340CommandMode::Effort)
    joint_keys.insert("max_command_effort_nm");
  const auto& h = info.hardware_parameters;
  const bool bench = h.at("profile") == "dm4340_mit_supervised_effort_bench";
  if ((h.at("profile") != "dm4340_mit_standard" && !bench) ||
      (bench && config.command_mode != Dm4340CommandMode::Effort) ||
      (bench && info.joints.size() != 1U) ||
      (!bench && !declared(h, "motor_side_timeout_verified")) ||
      !declared(h, "board_version_verified") ||
      !declared(h, "can_bitrate_verified") ||
      (h.at("enable_on_activate") != "true" &&
       h.at("enable_on_activate") != "false") ||
      h.at("device_path").rfind("/dev/ttyACM", 0U) != 0U)
    return false;
  enable_on_activate = declared(h, "enable_on_activate");

  std::uint64_t bus = 0;
  std::uint64_t major = 0, minor = 0, patch = 0;
  if (!integer(h, "logical_bus", bus) || bus == 0U || bus > 65535U ||
      !integer(h, "board_version_major", major) || major > 255U ||
      !integer(h, "board_version_minor", minor) || minor > 255U ||
      !integer(h, "board_version_patch", patch) || patch > 255U)
    return false;
  board = {static_cast<std::uint8_t>(major), static_cast<std::uint8_t>(minor),
           static_cast<std::uint8_t>(patch)};
  if (!mech_control_core::UsbCdcCodec::supports_version(board)) return false;
  config.physical_bus = h.at("device_path");
  config.logical_bus = static_cast<std::uint16_t>(bus);
  config.motor_side_timeout_verified = declared(h, "motor_side_timeout_verified");
  config.supervised_effort_bench = bench;
  if (!positive_ns(h, "control_period_ns", config.control_period_ns) ||
      !positive_ns(h, "command_hard_ttl_ns", config.command_hard_ttl_ns) ||
      !positive_ns(h, "query_period_ns", config.query_period_ns)) return false;
  std::int64_t command_ttl = 0, feedback_ttl = 0;
  if (!positive_ns(h, "command_ttl_ns", command_ttl) ||
      !positive_ns(h, "feedback_ttl_ns", feedback_ttl)) return false;
  for (const auto& joint : info.joints) {
    if (joint.name.empty() || joint.type != "joint" ||
        !keys_are(joint.parameters, joint_keys) ||
        joint.command_interfaces.size() != 2U ||
        joint.command_interfaces[0].name !=
            (config.command_mode == Dm4340CommandMode::Velocity
                 ? hardware_interface::HW_IF_VELOCITY
                 : hardware_interface::HW_IF_EFFORT) ||
        joint.command_interfaces[1].name !=
            mech_hardware_ros2_control::kCommandGenerationInterface ||
        joint.state_interfaces.size() != 3U ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
      return false;
    const auto& p = joint.parameters;
    std::uint64_t command_id = 0, feedback_id = 0, motor_id = 0;
    std::uint64_t firmware = 0, firmware_min = 0, firmware_max = 0;
    if (!integer(p, "command_can_id", command_id) || command_id == 0U ||
        command_id > 0x7FFU ||
        !integer(p, "feedback_can_id", feedback_id) || feedback_id > 0x7FFU ||
        !integer(p, "motor_id", motor_id) || motor_id == 0U || motor_id > 15U ||
        !integer(p, "firmware_id", firmware) || firmware > UINT32_MAX ||
        !integer(p, "firmware_min", firmware_min) || firmware_min > UINT32_MAX ||
        !integer(p, "firmware_max", firmware_max) || firmware_max > UINT32_MAX)
      return false;
    mech_protocol_damiao::Dm4340MitSessionConfig c{};
    c.profile = mech_control_core::ProtocolProfile::Dm4340MitStandard;
    c.logical_bus = config.logical_bus;
    c.command_can_id = static_cast<std::uint32_t>(command_id);
    c.feedback_can_id = static_cast<std::uint32_t>(feedback_id);
    c.motor_id = static_cast<std::uint8_t>(motor_id);
    c.firmware_id = static_cast<std::uint32_t>(firmware);
    c.firmware_min = static_cast<std::uint32_t>(firmware_min);
    c.firmware_max = static_cast<std::uint32_t>(firmware_max);
    c.firmware_evidence_declared = declared(p, "firmware_evidence_declared");
    c.limits_evidence_declared = declared(p, "limits_evidence_declared");
    c.joint.evidence_declared = bench || declared(p, "joint_mapping_verified");
    c.command_ttl_ns = command_ttl;
    c.feedback_ttl_ns = feedback_ttl;
    if (!real(p, "limit_position_rad", c.limits.position_rad) ||
        !real(p, "limit_velocity_rad_s", c.limits.velocity_rad_s) ||
        !real(p, "limit_torque_nm", c.limits.torque_nm) ||
        !real(p, "joint_scale", c.joint.scale) ||
        !real(p, "joint_offset_rad", c.joint.offset_rad) ||
        !real(p, "position_min_rad", c.position_min_rad) ||
        !real(p, "position_max_rad", c.position_max_rad) ||
        !real(p, "max_target_error_rad", c.max_target_error_rad) ||
        !real(p, "kp", c.kp) || !real(p, "kd", c.kd) ||
        !real(p, "velocity_feedforward_nm", c.velocity_feedforward_nm) ||
        !real(p, "max_command_velocity_rad_s", c.max_command_velocity_rad_s) ||
        !real(p, "max_feedback_velocity_rad_s", c.max_feedback_velocity_rad_s) ||
        !real(p, "max_feedback_torque_nm", c.max_feedback_torque_nm) ||
        (config.command_mode == Dm4340CommandMode::Effort &&
         !real(p, "max_command_effort_nm", c.max_command_effort_nm)) ||
        (bench && (declared(p, "joint_mapping_verified") ||
                   c.joint.scale != 1.0 || c.joint.offset_rad != 0.0 ||
                   c.max_command_effort_nm > 0.1))) return false;
    mech_protocol_damiao::Dm4340MitSession probe;
    if (probe.configure(c) != mech_control_core::AdapterResult::Ok) return false;
    for (const auto& prior : config.joints)
      if (prior.command_can_id == c.command_can_id || prior.motor_id == c.motor_id)
        return false;
    config.joints.push_back(c);
  }
  return true;
}
}  // namespace

Dm4340System::~Dm4340System() {
  if (runtime_view_) runtime_view_->stop();
}

void Dm4340System::set_serial_factory_for_testing(SerialFactory factory) noexcept {
  if (!init_attempted_) serial_factory_ = std::move(factory);
}

void Dm4340System::set_clock_for_testing(
    Dm4340Runtime::Clock clock) noexcept {
  if (!init_attempted_) clock_ = std::move(clock);
}

void Dm4340System::set_registry_for_testing(
    mech_control_core::BusOwnershipRegistry& registry) noexcept {
  if (!init_attempted_) registry_ = &registry;
}

hardware_interface::CallbackReturn Dm4340System::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (init_attempted_) return hardware_interface::CallbackReturn::ERROR;
  init_attempted_ = true;
  Dm4340RuntimeConfig config{};
  mech_control_core::CdcProtocolVersion board{};
  if (!parse(info, config, board, enable_on_activate_))
    return hardware_interface::CallbackReturn::ERROR;
  joint_names_.clear();
  for (std::size_t index = 0; index < info.joints.size(); ++index) {
    joint_names_.push_back(info.joints[index].name);
  }
  if (!clock_) clock_ = steady_now;
  if (!serial_factory_) {
    serial_factory_ = [](const std::string& path) {
      return std::make_shared<mech_control_core::PosixCdcSerialPort>(path);
    };
  }
  serial_ = serial_factory_(config.physical_bus);
  if (!serial_) return hardware_interface::CallbackReturn::ERROR;
  auto options = seven_channel_gateway_options(config.logical_bus, 1000000U);
  options.verified_board_version = board;
  transport_ = std::make_unique<mech_control_core::UsbCdcTransport>(
      *serial_, options, clock_);
  if (!registry_) registry_ = &shared_registry();
  auto runtime = std::make_unique<Dm4340Runtime>(
      *transport_, clock_, config, *registry_);
  runtime_view_ = runtime.get();
  if (!set_runtime(std::move(runtime))) {
    runtime_view_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }
  return CompositeSystem::on_init(info);
}

hardware_interface::CallbackReturn Dm4340System::on_activate(
    const rclcpp_lifecycle::State& state) {
  fault_logged_ = false;
  const auto result = CompositeSystem::on_activate(state);
  if (result != hardware_interface::CallbackReturn::SUCCESS) return result;
  if (!mech_control_core::initialize_usb_cdc_pass_through(*serial_)) {
    (void)CompositeSystem::on_error(state);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!enable_on_activate_) return hardware_interface::CallbackReturn::SUCCESS;

  const auto fail = [this, &state]() {
    (void)CompositeSystem::on_error(state);
    return hardware_interface::CallbackReturn::ERROR;
  };
  std::array<mech_hardware_ros2_control::CanonicalState, 6U> states{};
  const auto wait_for = [this, &states](bool enabled) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(1200);
    while (std::chrono::steady_clock::now() < deadline) {
      if (!runtime_view_->read(states.data(), joint_names_.size())) return false;
      if (enabled ? runtime_view_->has_valid_sample()
                  : runtime_view_->has_fresh_ready_feedback()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  };
  if (!wait_for(false)) return fail();
  if (!runtime_view_->adopt_or_enable_from_feedback() ||
      !wait_for(true)) return fail();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Dm4340System::read(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto result = CompositeSystem::read(time, period);
  if (result != hardware_interface::return_type::OK && !fault_logged_) {
    fault_logged_ = true;
    RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.dm4340"),
                 "DM4340 first read failure: %s (feedback status=%u)",
                 runtime_view_ ? dm4340_fault_name(runtime_view_->fault_reason())
                               : "runtime_unavailable",
                 runtime_view_ ? runtime_view_->rejected_status() : 0U);
  }
  return result;
}

hardware_interface::return_type Dm4340System::write(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto result = CompositeSystem::write(time, period);
  if (result != hardware_interface::return_type::OK && !fault_logged_) {
    fault_logged_ = true;
    RCLCPP_ERROR(rclcpp::get_logger("mech_bringup.dm4340"),
                 "DM4340 first write failure: %s (feedback status=%u)",
                 runtime_view_ ? dm4340_fault_name(runtime_view_->fault_reason())
                               : "runtime_unavailable",
                 runtime_view_ ? runtime_view_->rejected_status() : 0U);
  }
  return result;
}

hardware_interface::CallbackReturn Dm4340System::on_shutdown(
    const rclcpp_lifecycle::State&) {
  if (runtime_view_) runtime_view_->stop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Dm4340System::prepare_command_mode_switch(
    const std::vector<std::string>& start,
    const std::vector<std::string>& stop) {
  for (const auto& joint : joint_names_)
    if (!complete_claim_switch(joint, start, stop,
                               runtime_view_->command_mode()))
      return hardware_interface::return_type::ERROR;
  return CompositeSystem::prepare_command_mode_switch(start, stop);
}

hardware_interface::return_type Dm4340System::perform_command_mode_switch(
    const std::vector<std::string>& start,
    const std::vector<std::string>& stop) {
  for (const auto& joint : joint_names_)
    if (!complete_claim_switch(joint, start, stop,
                               runtime_view_->command_mode()))
      return hardware_interface::return_type::ERROR;
  return CompositeSystem::perform_command_mode_switch(start, stop);
}

}  // namespace mech::mech_bringup

PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::Dm4340System,
                       hardware_interface::SystemInterface)
