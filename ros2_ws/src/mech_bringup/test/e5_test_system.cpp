#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mech_bringup/ak30_force_runtime.hpp"
#include "mech_control_core/frame.hpp"
#include "mech_control_core/time.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include "mech_simulation/fake_transport.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mech::mech_bringup::test {
namespace {

constexpr std::uint16_t kDriveId = 104U;
constexpr std::uint32_t kLogicalBus = 1U;

mech_control_core::MonotonicTime steady_now() {
  const auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return mech_control_core::MonotonicTime::from_nanoseconds(value).value();
}

mech_control_core::RawCanFrame feedback_frame(
    mech_control_core::MonotonicTime arrival) {
  std::array<std::uint8_t, 64U> payload{};
  payload[0] = 0x03;
  payload[1] = 0x84;
  payload[2] = 0x03;
  payload[3] = 0xE8;
  payload[4] = 0x00;
  payload[5] = 0xC8;
  payload[6] = 0x28;
  return mech_control_core::RawCanFrame::create(
             kLogicalBus,
             mech_control_core::CanId::create(
                 mech_protocol_cubemars::feedback_can_id(kDriveId),
                 mech_control_core::CanFrameFormat::Extended)
                 .value(),
             mech_control_core::CanFrameType::Classic,
             mech_control_core::FrameDirection::Rx, 8U, payload, arrival)
      .value();
}

Ak30RuntimeConfig runtime_config() {
  Ak30RuntimeConfig config{};
  config.drive_id = kDriveId;
  config.logical_bus = kLogicalBus;
  config.sub_mode = mech_protocol_cubemars::ForceControlSubMode::Position;
  config.gains.kp = 1.0;
  config.gains.kd = 1.0;
  config.control_period_nanoseconds = 2000000;
  config.command_ttl_nanoseconds = 4000000;
  config.command_hard_ttl_nanoseconds = 6000000;
  config.feedback_period_nanoseconds = 20000000;
  config.feedback_ttl_nanoseconds = 60000000;
  return config;
}

}  // namespace

class E5TestSystem final
    : public mech_hardware_ros2_control::CompositeSystem,
      public FeedbackTelemetryCapture {
 public:
  bool try_push(const FeedbackTelemetryEvent& event) noexcept override {
    if (event.kind == FeedbackTelemetryKind::RuntimeError) {
      error_reason_.store(static_cast<std::uint64_t>(event.reason));
      error_quality_.store(static_cast<std::uint64_t>(event.quality));
    }
    return true;
  }

  ~E5TestSystem() override {
    executor_.cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override {
    runtime_ = std::make_unique<Ak30ForceControlRuntime>(
        transport_, []() { return steady_now(); }, runtime_config(), this);
    if (!set_runtime(std::move(runtime_))) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto result = CompositeSystem::on_init(info);
    if (result != hardware_interface::CallbackReturn::SUCCESS) return result;

    node_ = std::make_shared<rclcpp::Node>("e5_test_system");
    feedback_service_ = node_->create_service<std_srvs::srv::SetBool>(
        "~/set_feedback",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
          feedback_enabled_.store(request->data, std::memory_order_release);
          response->success = true;
          response->message = request->data ? "feedback enabled" : "feedback disabled";
        });
    reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/reset_trace",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const auto requested =
              reset_requested_.fetch_add(1U, std::memory_order_acq_rel) + 1U;
          const auto deadline = std::chrono::steady_clock::now() +
                                std::chrono::milliseconds(200);
          while (reset_applied_.load(std::memory_order_acquire) != requested &&
                 std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          response->success =
              reset_applied_.load(std::memory_order_acquire) == requested;
        });
    trace_publisher_ = node_->create_publisher<std_msgs::msg::UInt64MultiArray>(
        "~/trace", rclcpp::QoS(1).reliable().transient_local());
    trace_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10), [this]() { publish_trace(); });
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
    return result;
  }

  hardware_interface::return_type read(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override {
    trace_version_.fetch_add(1U);
    const auto requested = reset_requested_.load(std::memory_order_acquire);
    if (reset_applied_.load(std::memory_order_relaxed) != requested) {
      trace_count_.store(0U);
      trace_id_.store(0U);
      for (auto& byte : trace_payload_) byte.store(0U);
      reset_applied_.store(requested);
    }
    const auto now = steady_now();
    if (feedback_enabled_.load(std::memory_order_acquire) &&
        now.nanoseconds() - last_feedback_ns_ >= 20000000) {
      (void)transport_.inject_receive(feedback_frame(now));
      last_feedback_ns_ = now.nanoseconds();
    }

    const auto result = CompositeSystem::read(time, period);
    mech_control_core::RawCanFrame frame;
    while (transport_.take_transmit(frame)) {
      trace_id_.store(frame.id.value);
      for (std::size_t index = 0; index < 8U; ++index) {
        trace_payload_[index].store(frame.payload[index]);
      }
      trace_count_.fetch_add(1U);
    }
    observed_at_.store(steady_now().nanoseconds());
    read_cycles_.fetch_add(1U);
    trace_version_.fetch_add(1U);
    return result;
  }

 private:
  void publish_trace() {
    if (!trace_publisher_) return;
    const auto version = trace_version_.load();
    if ((version & 1U) != 0U) return;
    std_msgs::msg::UInt64MultiArray message;
    message.data.reserve(15U);
    message.data.push_back(trace_count_.load());
    message.data.push_back(trace_id_.load());
    for (std::size_t index = 0; index < 8U; ++index) {
      message.data.push_back(trace_payload_[index].load());
    }
    message.data.push_back(reset_applied_.load());
    message.data.push_back(observed_at_.load());
    message.data.push_back(read_cycles_.load());
    message.data.push_back(error_reason_.load());
    message.data.push_back(error_quality_.load());
    if (trace_version_.load() != version) return;
    trace_publisher_->publish(message);
  }

  mech_simulation::FakeTransport transport_{64U};
  std::unique_ptr<Ak30ForceControlRuntime> runtime_;
  std::atomic<bool> feedback_enabled_{false};
  std::atomic<std::uint64_t> reset_requested_{0U};
  std::atomic<std::uint64_t> reset_applied_{0U};
  std::int64_t last_feedback_ns_{0};
  std::atomic<std::uint64_t> trace_count_{0U};
  std::atomic<std::uint64_t> trace_id_{0U};
  std::array<std::atomic<std::uint64_t>, 8U> trace_payload_{};
  // Atomic fields plus an odd/even version prevent torn frame observations.
  // The consumer skips a busy observation; the control loop never waits.
  std::atomic<std::uint64_t> trace_version_{0U};
  std::atomic<std::uint64_t> observed_at_{0U};
  std::atomic<std::uint64_t> read_cycles_{0U};
  std::atomic<std::uint64_t> error_reason_{0U};
  std::atomic<std::uint64_t> error_quality_{0U};
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr feedback_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr trace_publisher_;
  rclcpp::TimerBase::SharedPtr trace_timer_;
};

}  // namespace mech::mech_bringup::test

PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::test::E5TestSystem,
                       hardware_interface::SystemInterface)
