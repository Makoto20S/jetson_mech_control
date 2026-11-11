#include <chrono>
#include <cstdint>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <time.h>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mech_ctrboard_bridge/posix_cdc_serial_port.hpp"
#include "mech_control_core/socketcan_transport.hpp"
#include "mech_control_core/transport.hpp"
#include "mech_control_core/usb_cdc_transport.hpp"
#include "mech_ctrboard_bridge/imu_safety.hpp"
#include "mech_ctrboard_bridge/timing_trace.hpp"
#include "mech_ctrboard_bridge/topic_names.hpp"
#include "mech_protocol_ctrboard/protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace mech::mech_ctrboard_bridge {
namespace {

using mech_control_core::CanFrameFormat;
using mech_control_core::CanFrameType;
using mech_control_core::CdcProtocolVersion;
using mech_control_core::FrameFilter;
using mech_control_core::RawCanFrame;
using mech_control_core::SocketCanOptions;
using mech_control_core::SocketCanTransport;
using mech_control_core::Transport;
using mech_control_core::TransportResult;
using mech_control_core::UsbCdcOptions;
using mech_control_core::UsbCdcTransport;
using mech_protocol_ctrboard::Packet;
using mech_protocol_ctrboard::SensorTelemetryWire;

constexpr std::size_t kMaximumReceiveFramesPerPoll = 64U;
constexpr auto kDiagnosticPeriod = std::chrono::milliseconds(200);

std::int64_t clock_monotonic_ns() noexcept {
  timespec stamp{};
  if (::clock_gettime(CLOCK_MONOTONIC, &stamp) != 0) return 0;
  return static_cast<std::int64_t>(stamp.tv_sec) * 1000000000LL + stamp.tv_nsec;
}

std::uint16_t checked_logical_bus(int value) {
  if (value < 0 || value > 65535) {
    throw std::invalid_argument("logical_bus must be in [0, 65535]");
  }
  return static_cast<std::uint16_t>(value);
}

SocketCanOptions make_transport_options(const std::string& interface_name,
                                        std::uint16_t logical_bus) {
  if (interface_name.empty()) {
    throw std::invalid_argument("interface must not be empty");
  }
  SocketCanOptions options;
  options.interface_name = interface_name;
  options.logical_bus = logical_bus;
  options.receive_queue_capacity = 256U;
  options.nominal_bitrate_hz = 1'000'000U;
  options.enable_can_fd = false;
  options.enable_error_frames = false;
  options.filters.push_back(FrameFilter{
      CanFrameFormat::Standard, mech_protocol_ctrboard::kMcuToHostCanId,
      0x7FFU, CanFrameType::Classic});
  return options;
}

std::uint8_t checked_version_component(int value, const char* parameter) {
  if (value < 0 || value > 255) {
    throw std::invalid_argument(std::string(parameter) + " must be in [0, 255]");
  }
  return static_cast<std::uint8_t>(value);
}

std::chrono::milliseconds checked_sensor_timeout(int value) {
  if (value < 20 || value > 10000) {
    throw std::invalid_argument("sensor_timeout_ms must be in [20, 10000]");
  }
  return std::chrono::milliseconds(value);
}

UsbCdcOptions make_usb_cdc_options(std::uint16_t logical_bus,
                                   CdcProtocolVersion board_version) {
  UsbCdcOptions options;
  options.logical_bus = logical_bus;
  options.nominal_bitrate_hz = 1'000'000U;
  options.receive_queue_capacity = 256U;
  options.serial_read_capacity = 1024U;
  options.verified_board_version = board_version;
  return options;
}

const char* bool_text(bool value) noexcept {
  return value ? "true" : "false";
}

}  // namespace

class CtrBoardBridgeNode final : public rclcpp::Node {
 public:
  CtrBoardBridgeNode()
      : Node("ctrboard_bridge"),
        transport_backend_(
            declare_parameter<std::string>("transport", "socketcan")),
        interface_name_(declare_parameter<std::string>("interface", "can0")),
        device_path_(
            declare_parameter<std::string>("device_path", "")),
        logical_bus_(checked_logical_bus(
            declare_parameter<int>("logical_bus", 1))),
        board_version_{checked_version_component(
                           declare_parameter<int>("board_version_major", 4),
                           "board_version_major"),
                       checked_version_component(
                           declare_parameter<int>("board_version_minor", 8),
                           "board_version_minor"),
                       checked_version_component(
                           declare_parameter<int>("board_version_patch", 8),
                           "board_version_patch")},
        poll_period_ms_(declare_parameter<int>("poll_period_ms", 1)),
        imu_1_frame_id_(
            declare_parameter<std::string>("imu_1_frame_id", "imu_1_link")),
        imu_2_frame_id_(
            declare_parameter<std::string>("imu_2_frame_id", "imu_2_link")),
        sensor_timeout_(checked_sensor_timeout(
            declare_parameter<int>("sensor_timeout_ms", 500))),
        safety_monitor_(sensor_timeout_) {
    if (poll_period_ms_ < 1 || poll_period_ms_ > 1000) {
      throw std::invalid_argument("poll_period_ms must be in [1, 1000]");
    }
    if (imu_1_frame_id_.empty() || imu_2_frame_id_.empty()) {
      throw std::invalid_argument("IMU frame IDs must not be empty");
    }
    rcl_interfaces::msg::ParameterDescriptor trace_descriptor;
    trace_descriptor.read_only = true;
    const auto trace_path = declare_parameter<std::string>("timing_trace_path", "", trace_descriptor);
    const auto trace_capacity = declare_parameter<int>("timing_trace_capacity", 400000, trace_descriptor);
    const auto trace_poll_capacity = declare_parameter<int>("timing_trace_poll_capacity", 100000, trace_descriptor);
    if (!trace_path.empty()) {
      if (transport_backend_ != "usb_cdc") {
        throw std::invalid_argument("timing trace currently supports only usb_cdc");
      }
      timing_trace_ = std::make_unique<TimingTrace>(
          trace_path, trace_capacity, trace_poll_capacity,
          static_cast<std::int64_t>(poll_period_ms_) * 1000000LL, clock_monotonic_ns);
    }
    if (transport_backend_ == "socketcan") {
      transport_ = std::make_unique<SocketCanTransport>(
          make_transport_options(interface_name_, logical_bus_));
      if (!transport_->open()) {
        throw std::runtime_error("failed to open SocketCAN interface " +
                                 interface_name_);
      }
    } else if (transport_backend_ == "usb_cdc") {
      if (device_path_.empty()) {
        throw std::invalid_argument("device_path must not be empty");
      }
      cdc_serial_ = std::make_unique<PosixCdcSerialPort>(device_path_);
      cdc_serial_->set_read_timing_enabled(timing_trace_ != nullptr);
      UsbCdcTransport::Clock arrival_clock;
      if (timing_trace_) {
        arrival_clock = [this]() noexcept {
          return *mech_control_core::MonotonicTime::from_nanoseconds(timing_trace_->now_ns());
        };
      }
      transport_ = std::make_unique<UsbCdcTransport>(
          *cdc_serial_, make_usb_cdc_options(logical_bus_, board_version_), arrival_clock);
      if (!transport_->open()) {
        throw std::runtime_error(
            "failed to open USB-CDC device " + device_path_ +
            "; check that it exists and the user has dialout access");
      }
      if (!cdc_serial_->send_pass_through_init()) {
        transport_->close();
        throw std::runtime_error("failed to initialize USB-CDC pass-through on " +
                                 device_path_);
      }
    } else {
      throw std::invalid_argument(
          "transport must be either 'socketcan' or 'usb_cdc'");
    }

    imu_1_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        kImu1DataTopic, rclcpp::SensorDataQoS());
    imu_2_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        kImu2DataTopic, rclcpp::SensorDataQoS());
    imu_1_euler_publisher_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>(
            kImu1EulerTopic, rclcpp::SensorDataQoS());
    imu_2_euler_publisher_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>(
            kImu2EulerTopic, rclcpp::SensorDataQoS());
    fsr_left_publisher_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
        "fsr/left/raw", rclcpp::SensorDataQoS());
    fsr_right_publisher_ = create_publisher<std_msgs::msg::UInt16MultiArray>(
        "fsr/right/raw", rclcpp::SensorDataQoS());
    fsr_left_total_publisher_ = create_publisher<std_msgs::msg::UInt32>(
        "fsr/left/total", rclcpp::SensorDataQoS());
    fsr_right_total_publisher_ = create_publisher<std_msgs::msg::UInt32>(
        "fsr/right/total", rclcpp::SensorDataQoS());
    sensor_status_publisher_ =
        create_publisher<std_msgs::msg::UInt8MultiArray>(
            "ctrboard/sensor_status", rclcpp::SensorDataQoS());
    device_timestamp_publisher_ = create_publisher<std_msgs::msg::UInt32>(
        "ctrboard/timestamp_ms", rclcpp::SensorDataQoS());
    diagnostic_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            kDiagnosticTopic, rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(std::chrono::milliseconds(poll_period_ms_),
                               [this]() { poll_transport(); });
    diagnostic_timer_ = create_wall_timer(
        kDiagnosticPeriod, [this]() { publish_diagnostics(); });
    endpoint_ = transport_backend_ == "socketcan" ? interface_name_
                                                   : device_path_;
    RCLCPP_INFO(get_logger(),
                "CtrBoard receive-only bridge opened %s via %s: RX 0x%03X, "
                "Classic CAN",
                endpoint_.c_str(), transport_backend_.c_str(),
                mech_protocol_ctrboard::kMcuToHostCanId);
    RCLCPP_WARN(
        get_logger(),
        "The legacy 196-byte telemetry payload has no per-IMU freshness "
        "field; startup absence and packet timeout are monitored, but a "
        "single IMU freezing after its first sample cannot be proven");
  }

  ~CtrBoardBridgeNode() override {
    if (transport_) {
      transport_->close();
    }
    if (timing_trace_) {
      try {
        timing_trace_->finish(cdc_serial_ ? cdc_serial_->read_clock_errors() : 0);
      } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Timing trace incomplete: %s", error.what());
      }
    }
  }

 private:
  void poll_transport() {
    const auto poll_begin = timing_trace_ ? timing_trace_->now_ns() : 0;
    const auto packets_before = timing_trace_ ? timing_trace_->packet_observed() : 0;
    std::uint64_t frames = 0;
    auto terminal_result = TransportResult::Ok;
    for (std::size_t count = 0U; count < kMaximumReceiveFramesPerPoll;
         ++count) {
      RawCanFrame frame{};
      const auto result = transport_->try_receive(frame);
      terminal_result = result;
      if (result == TransportResult::WouldBlock) {
        break;
      }
      if (result != TransportResult::Ok) {
        safety_monitor_.note_transport_fault();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "CtrBoard transport receive error: %u",
                              static_cast<unsigned int>(result));
        break;
      }
      ++frames;
      const auto decode_begin = timing_trace_ ? timing_trace_->now_ns() : 0;
      const auto serial_read_end = timing_trace_ && cdc_serial_ ? cdc_serial_->last_read_end_ns() : 0;
      for (const auto& packet : decoder_.feed(frame)) {
        handle_packet(packet, frame.host_arrival.nanoseconds(), serial_read_end, decode_begin);
      }
    }

    safety_monitor_.set_decoder_counters(decoder_.crc_errors(),
                                         decoder_.discarded_bytes());

    if (decoder_.crc_errors() != reported_crc_errors_) {
      reported_crc_errors_ = decoder_.crc_errors();
      RCLCPP_WARN(get_logger(), "CtrBoard CRC errors: %llu",
                  static_cast<unsigned long long>(reported_crc_errors_));
    }
    if (timing_trace_) {
      timing_trace_->record_poll(poll_begin, timing_trace_->now_ns(), frames,
                                timing_trace_->packet_observed() - packets_before,
                                static_cast<std::uint32_t>(terminal_result));
    }
  }

  void handle_packet(const Packet& packet, std::int64_t frame_arrival,
                     std::int64_t serial_read_end, std::int64_t decode_begin) {
    SensorTelemetryWire telemetry{};
    if (!mech_protocol_ctrboard::decode_sensor_state(packet, telemetry)) {
      return;
    }
    PacketTimingRow trace;
    if (timing_trace_) {
      trace.device_ms = telemetry.timestamp_ms;
      trace.frame_host_arrival_ns = frame_arrival;
      trace.last_serial_read_end_ns = serial_read_end;
      trace.decode_begin_ns = decode_begin;
      trace.decode_end_ns = timing_trace_->now_ns();
    }
    const auto stamp = now();
    if (timing_trace_) trace.ros_stamp_ns = stamp.nanoseconds();
    const auto message_stamp =
        static_cast<builtin_interfaces::msg::Time>(stamp);
    const auto imu_1 = prepare_imu_messages(
        telemetry.imu_1, imu_1_frame_id_, message_stamp);
    const auto imu_2 = prepare_imu_messages(
        telemetry.imu_2, imu_2_frame_id_, message_stamp);
    if (timing_trace_) trace.imu1_publish_begin_ns = timing_trace_->now_ns();
    imu_1_publisher_->publish(imu_1.imu);
    if (timing_trace_) {
      trace.imu1_publish_end_ns = timing_trace_->now_ns();
      trace.imu2_publish_begin_ns = timing_trace_->now_ns();
    }
    imu_2_publisher_->publish(imu_2.imu);
    if (timing_trace_) trace.imu2_publish_end_ns = timing_trace_->now_ns();
    if (imu_1.euler.has_value()) {
      imu_1_euler_publisher_->publish(*imu_1.euler);
    }
    if (imu_2.euler.has_value()) {
      imu_2_euler_publisher_->publish(*imu_2.euler);
    }
    safety_monitor_.note_packet(imu_1.validity, imu_2.validity);

    std_msgs::msg::UInt16MultiArray fsr_left;
    fsr_left.data.assign(std::begin(telemetry.fsr_left_raw),
                         std::end(telemetry.fsr_left_raw));
    fsr_left_publisher_->publish(fsr_left);
    std_msgs::msg::UInt16MultiArray fsr_right;
    fsr_right.data.assign(std::begin(telemetry.fsr_right_raw),
                          std::end(telemetry.fsr_right_raw));
    fsr_right_publisher_->publish(fsr_right);

    std_msgs::msg::UInt32 left_total;
    left_total.data = telemetry.fsr_left_total;
    fsr_left_total_publisher_->publish(left_total);
    std_msgs::msg::UInt32 right_total;
    right_total.data = telemetry.fsr_right_total;
    fsr_right_total_publisher_->publish(right_total);

    std_msgs::msg::UInt8MultiArray status;
    status.data = {telemetry.fsr_left_phase, telemetry.fsr_right_phase,
                   telemetry.fsr_left_points, telemetry.fsr_right_points};
    sensor_status_publisher_->publish(status);

    std_msgs::msg::UInt32 device_timestamp;
    device_timestamp.data = telemetry.timestamp_ms;
    device_timestamp_publisher_->publish(device_timestamp);
    if (timing_trace_) {
      trace.packet_done_ns = timing_trace_->now_ns();
      timing_trace_->record_packet(trace);
    }
  }

  void publish_diagnostics() {
    const auto snapshot = safety_monitor_.snapshot();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "ctrboard_sensor_safety";
    status.hardware_id = endpoint_;
    switch (snapshot.state) {
      case SensorSafetyState::Healthy:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "sensor packet and both IMUs are valid";
        break;
      case SensorSafetyState::WaitingForData:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "waiting for first sensor packet";
        break;
      case SensorSafetyState::Degraded:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "one or more IMU fields are unavailable";
        break;
      case SensorSafetyState::TransportFault:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "transport receive fault";
        break;
      case SensorSafetyState::TimedOut:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "sensor packet timeout";
        break;
    }
    const auto add = [&status](const std::string& key,
                               const std::string& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
    add("state", to_string(snapshot.state));
    add("transport", transport_backend_);
    add("endpoint", endpoint_);
    add("packet_age_ms", std::to_string(snapshot.packet_age_ms));
    add("timeout_ms", std::to_string(sensor_timeout_.count()));
    add("crc_errors", std::to_string(snapshot.crc_errors));
    add("discarded_bytes", std::to_string(snapshot.discarded_bytes));
    add("imu_1_sample_present", bool_text(snapshot.imu_1.sample_present));
    add("imu_1_orientation_valid",
        bool_text(snapshot.imu_1.orientation_valid));
    add("imu_1_angular_velocity_valid",
        bool_text(snapshot.imu_1.angular_velocity_valid));
    add("imu_1_linear_acceleration_valid",
        bool_text(snapshot.imu_1.linear_acceleration_valid));
    add("imu_2_sample_present", bool_text(snapshot.imu_2.sample_present));
    add("imu_2_orientation_valid",
        bool_text(snapshot.imu_2.orientation_valid));
    add("imu_2_angular_velocity_valid",
        bool_text(snapshot.imu_2.angular_velocity_valid));
    add("imu_2_linear_acceleration_valid",
        bool_text(snapshot.imu_2.linear_acceleration_valid));
    add("per_imu_freshness",
        "unavailable in legacy 196-byte telemetry payload");
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  std::unique_ptr<TimingTrace> timing_trace_;
  std::string transport_backend_;
  std::string interface_name_;
  std::string device_path_;
  std::uint16_t logical_bus_;
  CdcProtocolVersion board_version_;
  int poll_period_ms_;
  std::string imu_1_frame_id_;
  std::string imu_2_frame_id_;
  std::chrono::milliseconds sensor_timeout_;
  SensorSafetyMonitor safety_monitor_;
  std::string endpoint_;
  std::unique_ptr<PosixCdcSerialPort> cdc_serial_;
  std::unique_ptr<Transport> transport_;
  mech_protocol_ctrboard::StreamDecoder decoder_;
  std::uint64_t reported_crc_errors_{0U};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_1_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_2_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      imu_1_euler_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
      imu_2_euler_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
      fsr_left_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr
      fsr_right_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr
      fsr_left_total_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr
      fsr_right_total_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr
      sensor_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr
      device_timestamp_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace mech::mech_ctrboard_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<mech::mech_ctrboard_bridge::CtrBoardBridgeNode>());
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("ctrboard_bridge"), "%s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
