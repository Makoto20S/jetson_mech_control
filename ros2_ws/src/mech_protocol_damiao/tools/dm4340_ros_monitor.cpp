#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "mech_control_core/posix_cdc_serial_port.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_protocol_damiao/dm4340_mit_wire.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace {

using mech::mech_control_core::CdcProtocolVersion;
using mech::mech_control_core::CanFrameFormat;
using mech::mech_control_core::CanFrameType;
using mech::mech_control_core::CanId;
using mech::mech_control_core::FrameDirection;
using mech::mech_control_core::MonotonicTime;
using mech::mech_control_core::PosixCdcSerialPort;
using mech::mech_control_core::RawCanFrame;
using mech::mech_control_core::TransportResult;
using mech::mech_control_core::UsbCdcCodec;
using mech::mech_control_core::UsbCdcOptions;
using mech::mech_control_core::UsbCdcTransport;
using mech::mech_control_core::kMaxCanPayloadBytes;
using mech::mech_protocol_damiao::MitFeedback;

void add_value(diagnostic_msgs::msg::DiagnosticStatus& status,
               const std::string& key, const std::string& value) {
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = value;
  status.values.push_back(std::move(entry));
}

std::string payload_hex(const RawCanFrame& frame) {
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setfill('0');
  for (std::uint8_t i = 0U; i < frame.payload_size; ++i) {
    stream << std::setw(2) << static_cast<unsigned>(frame.payload[i]);
  }
  return stream.str();
}

class Dm4340RosMonitor final : public rclcpp::Node {
 public:
  Dm4340RosMonitor()
      : Node("dm4340_ros_monitor"),
        device_path_(declare_parameter<std::string>("device_path", "")),
        logical_bus_(declare_parameter<int>("logical_bus", 1)),
        feedback_can_id_(declare_parameter<int>("feedback_can_id", -1)),
        motor_id_(declare_parameter<int>("motor_id", -1)),
        duration_seconds_(declare_parameter<int>("duration_seconds", 10)),
        status_query_period_ms_(declare_parameter<int>("status_query_period_ms", 0)),
        board_version_(read_version()),
        serial_(device_path_),
        transport_(serial_, make_options()) {
    if (device_path_.rfind("/dev/ttyACM", 0U) != 0U ||
        device_path_.size() <= 11U || logical_bus_ < 1 ||
        logical_bus_ > 65535 || duration_seconds_ < 1 ||
        duration_seconds_ > 30 ||
        ((feedback_can_id_ == -1) != (motor_id_ == -1)) ||
        feedback_can_id_ > 0x7FF || feedback_can_id_ < -1 ||
        motor_id_ > 15 || motor_id_ < -1 ||
        (status_query_period_ms_ != 0 &&
         (status_query_period_ms_ < 100 || status_query_period_ms_ > 1000 ||
          motor_id_ < 0 || feedback_can_id_ < 0)) ||
        !UsbCdcCodec::supports_version(board_version_)) {
      throw std::invalid_argument("invalid monitor parameters; device_path, "
                                  "verified board_version, and 1..30 s are required");
    }
    if (!transport_.open()) {
      throw std::runtime_error("cannot open " + device_path_);
    }
    if (!serial_.send_pass_through_init()) {
      transport_.close();
      throw std::runtime_error("gateway receive-pass init failed");
    }
    diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "damiao/monitor/diagnostics", 10);
    motor_axis_ = create_publisher<sensor_msgs::msg::JointState>(
        "damiao/monitor/motor_axis", 10);
    deadline_ = std::chrono::steady_clock::now() +
                std::chrono::seconds(duration_seconds_);
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(2),
                                    [this]() { poll(); });
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200),
                                      [this]() { publish_status(); });
    if (status_query_period_ms_ != 0) {
      query_timer_ = create_wall_timer(
          std::chrono::milliseconds(status_query_period_ms_),
          [this]() { query_status(); });
    }
    RCLCPP_INFO(get_logger(), "Monitor on %s for %d s; status query %s; enable/MIT TX disabled",
                device_path_.c_str(), duration_seconds_,
                status_query_period_ms_ == 0 ? "off" : "on");
  }

  ~Dm4340RosMonitor() override { transport_.close(); }

 private:
  CdcProtocolVersion read_version() {
    const int major = declare_parameter<int>("board_version_major", 0);
    const int minor = declare_parameter<int>("board_version_minor", 0);
    const int patch = declare_parameter<int>("board_version_patch", 0);
    if (major < 0 || major > 255 || minor < 0 || minor > 255 ||
        patch < 0 || patch > 255) {
      throw std::invalid_argument("board_version components must be 0..255");
    }
    return {static_cast<std::uint8_t>(major),
            static_cast<std::uint8_t>(minor),
            static_cast<std::uint8_t>(patch)};
  }

  UsbCdcOptions make_options() const {
    UsbCdcOptions options;
    options.logical_bus = static_cast<std::uint16_t>(
        std::max(0, std::min(logical_bus_, 65535)));
    options.receive_queue_capacity = 64U;
    options.serial_read_capacity = 1024U;
    options.verified_board_version = board_version_;
    return options;
  }

  void publish(diagnostic_msgs::msg::DiagnosticStatus status) {
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(std::move(status));
    diagnostics_->publish(std::move(message));
  }

  void publish_frame(const RawCanFrame& frame) {
    last_raw_frame_ = std::chrono::steady_clock::now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "damiao/received_can_frame";
    status.hardware_id = device_path_;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "raw CAN frame; motor identity unverified";
    add_value(status, "logical_bus", std::to_string(frame.logical_bus));
    add_value(status, "can_id", std::to_string(frame.id.value));
    add_value(status, "format", frame.id.format ==
              mech::mech_control_core::CanFrameFormat::Standard ? "standard" : "extended");
    add_value(status, "direction", frame.direction ==
              mech::mech_control_core::FrameDirection::Rx ? "rx" : "tx");
    add_value(status, "frame_type", frame.type ==
              mech::mech_control_core::CanFrameType::Classic ? "classic" : "fd");
    add_value(status, "dlc", std::to_string(frame.payload_size));
    add_value(status, "data_hex", payload_hex(frame));
    if (feedback_can_id_ >= 0) {
      MitFeedback feedback{};
      if (mech::mech_protocol_damiao::decode_mit_feedback(
              frame, static_cast<std::uint16_t>(logical_bus_),
              static_cast<std::uint32_t>(feedback_can_id_),
              static_cast<std::uint8_t>(motor_id_),
              mech::mech_protocol_damiao::kSdkDm4340Limits, feedback)) {
        ++decoded_count_;
        last_feedback_ = last_raw_frame_;
        last_feedback_fault_ = feedback.raw_fault;
        status.message = "MIT feedback decoded with SDK reference ranges";
        add_value(status, "position_motor_rad", std::to_string(feedback.position_rad));
        add_value(status, "velocity_motor_rad_s", std::to_string(feedback.velocity_rad_s));
        add_value(status, "torque_sdk_nm", std::to_string(feedback.torque_nm));
        add_value(status, "raw_fault", std::to_string(feedback.raw_fault));
        if (!mech::mech_protocol_damiao::feedback_status_ok(feedback.raw_fault)) {
          status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        } else {
          sensor_msgs::msg::JointState sample;
          sample.header.stamp = now();
          sample.name = {"dm4340_motor_" + std::to_string(motor_id_)};
          sample.position = {feedback.position_rad};
          sample.velocity = {feedback.velocity_rad_s};
          sample.effort = {feedback.torque_nm};
          motor_axis_->publish(std::move(sample));
        }
      }
    }
    publish(std::move(status));
  }

  void publish_status() {
    const auto now_steady = std::chrono::steady_clock::now();
    const bool raw_fresh = raw_count_ != 0U &&
        now_steady - last_raw_frame_ <= std::chrono::milliseconds(500);
    const bool feedback_fresh = feedback_can_id_ < 0 ||
        (decoded_count_ != 0U &&
         now_steady - last_feedback_ <= std::chrono::milliseconds(500));
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "damiao/receive_monitor";
    status.hardware_id = device_path_;
    status.level = feedback_fresh &&
                   !mech::mech_protocol_damiao::feedback_status_ok(last_feedback_fault_)
        ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
        : (!raw_fresh || !feedback_fresh
               ? diagnostic_msgs::msg::DiagnosticStatus::WARN
               : diagnostic_msgs::msg::DiagnosticStatus::OK);
    status.message = raw_count_ == 0U
        ? "no frames received"
        : (!raw_fresh ? "raw frames stale"
           : (!feedback_fresh ? "configured MIT feedback absent or stale"
              : (!mech::mech_protocol_damiao::feedback_status_ok(last_feedback_fault_)
                     ? "motor feedback fault"
                                            : "receiving fresh frames")));
    add_value(status, "raw_frames", std::to_string(raw_count_));
    add_value(status, "decoded_feedback", std::to_string(decoded_count_));
    add_value(status, "status_queries", std::to_string(status_queries_));
    if (feedback_can_id_ >= 0 && decoded_count_ != 0U) {
      add_value(status, "last_raw_fault", std::to_string(last_feedback_fault_));
    }
    const auto stats = transport_.stats();
    add_value(status, "dropped_frames", std::to_string(stats.rx_dropped));
    add_value(status, "motor_tx_frames", std::to_string(stats.tx_frames));
    publish(std::move(status));
  }

  void query_status() {
    if (std::chrono::steady_clock::now() >= deadline_) return;
    const auto id = CanId::create(0x7FFU, CanFrameFormat::Standard);
    const auto stamp = MonotonicTime::from_nanoseconds(0);
    if (!id || !stamp) throw std::runtime_error("invalid status query frame");
    std::array<std::uint8_t, kMaxCanPayloadBytes> payload{};
    payload[0] = static_cast<std::uint8_t>(motor_id_);
    payload[2] = 0xCCU;
    const auto frame = RawCanFrame::create(
        static_cast<std::uint16_t>(logical_bus_), *id,
        CanFrameType::Classic, FrameDirection::Tx, 8U, payload, *stamp);
    if (!frame || transport_.try_send(*frame) != TransportResult::Ok) {
      throw std::runtime_error("DM4340 status query send failed");
    }
    ++status_queries_;
  }

  void poll() {
    if (std::chrono::steady_clock::now() >= deadline_) {
      publish_status();
      RCLCPP_INFO(get_logger(), "Capture complete: raw=%llu decoded=%llu motor_tx=%llu",
                  static_cast<unsigned long long>(raw_count_),
                  static_cast<unsigned long long>(decoded_count_),
                  static_cast<unsigned long long>(transport_.stats().tx_frames));
      rclcpp::shutdown();
      return;
    }
    for (int i = 0; i < 64; ++i) {
      RawCanFrame frame{};
      const auto result = transport_.try_receive(frame);
      if (result == TransportResult::WouldBlock) return;
      if (result == TransportResult::Invalid) continue;
      if (result != TransportResult::Ok) {
        RCLCPP_ERROR(get_logger(), "gateway receive failed: %u",
                     static_cast<unsigned>(result));
        throw std::runtime_error("gateway receive failure");
      }
      ++raw_count_;
      publish_frame(frame);
    }
  }

  std::string device_path_;
  int logical_bus_;
  int feedback_can_id_;
  int motor_id_;
  int duration_seconds_;
  int status_query_period_ms_;
  CdcProtocolVersion board_version_;
  PosixCdcSerialPort serial_;
  UsbCdcTransport transport_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr motor_axis_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr query_timer_;
  std::chrono::steady_clock::time_point deadline_;
  std::uint64_t raw_count_{0U};
  std::uint64_t decoded_count_{0U};
  std::uint64_t status_queries_{0U};
  std::uint8_t last_feedback_fault_{0U};
  std::chrono::steady_clock::time_point last_raw_frame_{};
  std::chrono::steady_clock::time_point last_feedback_{};
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Dm4340RosMonitor>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("dm4340_ros_monitor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
