#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "mech_control_core/socketcan_transport.hpp"
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
using mech_control_core::FrameFilter;
using mech_control_core::RawCanFrame;
using mech_control_core::SocketCanOptions;
using mech_control_core::SocketCanTransport;
using mech_control_core::TransportResult;
using mech_protocol_ctrboard::ImuSampleWire;
using mech_protocol_ctrboard::Packet;
using mech_protocol_ctrboard::SensorTelemetryWire;

constexpr std::size_t kMaximumReceiveFramesPerPoll = 64U;
constexpr double kGravityMetersPerSecondSquared = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;

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

bool imu_is_finite(const ImuSampleWire& imu) {
  const auto array_is_finite = [](const auto& values) {
    return std::all_of(std::begin(values), std::end(values),
                       [](float value) { return std::isfinite(value); });
  };
  return array_is_finite(imu.acceleration_g) &&
         array_is_finite(imu.angular_velocity_dps) &&
         array_is_finite(imu.euler_deg) &&
         array_is_finite(imu.quaternion_wxyz);
}

}  // namespace

class CtrBoardBridgeNode final : public rclcpp::Node {
 public:
  CtrBoardBridgeNode()
      : Node("ctrboard_bridge"),
        interface_name_(declare_parameter<std::string>("interface", "can0")),
        logical_bus_(checked_logical_bus(
            declare_parameter<int>("logical_bus", 1))),
        poll_period_ms_(declare_parameter<int>("poll_period_ms", 1)),
        imu_1_frame_id_(
            declare_parameter<std::string>("imu_1_frame_id", "imu_1_link")),
        imu_2_frame_id_(
            declare_parameter<std::string>("imu_2_frame_id", "imu_2_link")),
        transport_(make_transport_options(interface_name_, logical_bus_)) {
    if (poll_period_ms_ < 1 || poll_period_ms_ > 1000) {
      throw std::invalid_argument("poll_period_ms must be in [1, 1000]");
    }
    if (imu_1_frame_id_.empty() || imu_2_frame_id_.empty()) {
      throw std::invalid_argument("IMU frame IDs must not be empty");
    }
    if (!transport_.open()) {
      throw std::runtime_error("failed to open SocketCAN interface " +
                               interface_name_);
    }

    imu_1_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "imu/1/data", rclcpp::SensorDataQoS());
    imu_2_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "imu/2/data", rclcpp::SensorDataQoS());
    imu_1_euler_publisher_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "imu/1/euler_deg", rclcpp::SensorDataQoS());
    imu_2_euler_publisher_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "imu/2/euler_deg", rclcpp::SensorDataQoS());
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

    timer_ = create_wall_timer(std::chrono::milliseconds(poll_period_ms_),
                               [this]() { poll_transport(); });
    RCLCPP_INFO(
        get_logger(),
        "CtrBoard receive-only bridge opened %s: RX 0x%03X, Classic CAN",
        interface_name_.c_str(), mech_protocol_ctrboard::kMcuToHostCanId);
  }

  ~CtrBoardBridgeNode() override { transport_.close(); }

 private:
  void poll_transport() {
    for (std::size_t count = 0U; count < kMaximumReceiveFramesPerPoll;
         ++count) {
      RawCanFrame frame{};
      const auto result = transport_.try_receive(frame);
      if (result == TransportResult::WouldBlock) {
        break;
      }
      if (result != TransportResult::Ok) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "SocketCAN receive error: %u",
                              static_cast<unsigned int>(result));
        break;
      }
      for (const auto& packet : decoder_.feed(frame)) {
        handle_packet(packet);
      }
    }

    if (decoder_.crc_errors() != reported_crc_errors_) {
      reported_crc_errors_ = decoder_.crc_errors();
      RCLCPP_WARN(get_logger(), "CtrBoard CRC errors: %llu",
                  static_cast<unsigned long long>(reported_crc_errors_));
    }
  }

  void handle_packet(const Packet& packet) {
    SensorTelemetryWire telemetry{};
    if (!mech_protocol_ctrboard::decode_sensor_state(packet, telemetry)) {
      return;
    }
    if (!imu_is_finite(telemetry.imu_1) ||
        !imu_is_finite(telemetry.imu_2)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Dropped CtrBoard sample containing NaN or Inf");
      return;
    }

    const auto stamp = now();
    publish_imu(telemetry.imu_1, imu_1_frame_id_, stamp, imu_1_publisher_,
                imu_1_euler_publisher_);
    publish_imu(telemetry.imu_2, imu_2_frame_id_, stamp, imu_2_publisher_,
                imu_2_euler_publisher_);

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
  }

  static void publish_imu(
      const ImuSampleWire& sample, const std::string& frame_id,
      const rclcpp::Time& stamp,
      const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr& publisher,
      const rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr&
          euler_publisher) {
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id;

    const double norm = std::sqrt(
        static_cast<double>(sample.quaternion_wxyz[0]) *
            sample.quaternion_wxyz[0] +
        static_cast<double>(sample.quaternion_wxyz[1]) *
            sample.quaternion_wxyz[1] +
        static_cast<double>(sample.quaternion_wxyz[2]) *
            sample.quaternion_wxyz[2] +
        static_cast<double>(sample.quaternion_wxyz[3]) *
            sample.quaternion_wxyz[3]);
    if (norm > 1.0e-6) {
      message.orientation.w = sample.quaternion_wxyz[0] / norm;
      message.orientation.x = sample.quaternion_wxyz[1] / norm;
      message.orientation.y = sample.quaternion_wxyz[2] / norm;
      message.orientation.z = sample.quaternion_wxyz[3] / norm;
    } else {
      message.orientation.w = 1.0;
      message.orientation_covariance[0] = -1.0;
    }

    message.angular_velocity.x =
        sample.angular_velocity_dps[0] * kDegreesToRadians;
    message.angular_velocity.y =
        sample.angular_velocity_dps[1] * kDegreesToRadians;
    message.angular_velocity.z =
        sample.angular_velocity_dps[2] * kDegreesToRadians;
    message.linear_acceleration.x =
        sample.acceleration_g[0] * kGravityMetersPerSecondSquared;
    message.linear_acceleration.y =
        sample.acceleration_g[1] * kGravityMetersPerSecondSquared;
    message.linear_acceleration.z =
        sample.acceleration_g[2] * kGravityMetersPerSecondSquared;
    publisher->publish(message);

    geometry_msgs::msg::Vector3Stamped euler;
    euler.header = message.header;
    euler.vector.x = sample.euler_deg[0];
    euler.vector.y = sample.euler_deg[1];
    euler.vector.z = sample.euler_deg[2];
    euler_publisher->publish(euler);
  }

  std::string interface_name_;
  std::uint16_t logical_bus_;
  int poll_period_ms_;
  std::string imu_1_frame_id_;
  std::string imu_2_frame_id_;
  SocketCanTransport transport_;
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
  rclcpp::TimerBase::SharedPtr timer_;
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
