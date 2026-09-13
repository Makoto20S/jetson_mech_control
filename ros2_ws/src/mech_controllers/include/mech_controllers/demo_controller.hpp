#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64.hpp"

namespace mech::mech_controllers {

struct BoundedTarget final {
  double minimum{-1.0};
  double maximum{1.0};
  double max_slew_per_second{1.0};
  // 03_mvp_delivery_plan.md:215 requires a stale command to lapse within
  // <=3 control cycles, i.e. <=6 ms at the 500 Hz target loop. Two cycles of
  // holding followed by a hard failure on the third keeps the whole watchdog
  // inside that budget.
  std::int64_t ttl_nanoseconds{4000000};
  std::int64_t hard_ttl_nanoseconds{6000000};
};

// Watchdog stage for the most recent TargetLimiter::update() call:
//   Following - within ttl_nanoseconds, tracking the submitted target.
//   Holding   - past ttl_nanoseconds but before hard_ttl_nanoseconds; the
//               last valid commanded value is frozen, no new motion.
//   Expired   - past hard_ttl_nanoseconds; caller must stop commanding.
enum class WatchdogStage { Following, Holding, Expired };

class TargetLimiter final {
 public:
  [[nodiscard]] bool configure(BoundedTarget limits) noexcept;
  [[nodiscard]] bool submit(double target, std::int64_t now_nanoseconds) noexcept;
  [[nodiscard]] double update(double previous, double period_seconds,
                              std::int64_t now_nanoseconds) noexcept;
  void clear() noexcept;
  [[nodiscard]] bool expired(std::int64_t now_nanoseconds) const noexcept;
  [[nodiscard]] WatchdogStage stage(std::int64_t now_nanoseconds) const noexcept;

 private:
  BoundedTarget limits_{};
  double target_{0.0};
  double held_{0.0};
  std::int64_t deadline_{0};
  bool configured_{false};
  bool has_target_{false};
  bool has_held_{false};
};

class DemoController final : public controller_interface::ControllerInterface {
 public:
  // A monotonic time source in nanoseconds, used for every watchdog deadline.
  // Injectable so tests can step the staged watchdog deterministically;
  // production uses steady_clock.
  //
  // Deliberately NOT the rclcpp::Time that update() is handed. That clock can
  // be ROS time, and a deployment with a frozen use_sim_time would stop the
  // watchdog's clock entirely - every TTL assertion would hold vacuously while
  // a stale target stayed alive on the motor.
  using MonotonicClock = std::function<std::int64_t()>;

  // A target as it crosses from the subscription thread into update(). The
  // generation is what distinguishes "a message arrived" from "the manager
  // cycled again": two messages carrying the same number are two targets and
  // both refresh the TTL, while any number of bare control cycles refresh
  // nothing.
  struct TargetCommand final {
    double value{0.0};
    std::uint64_t generation{0U};
  };

  DemoController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  [[nodiscard]] bool set_target(double target) noexcept;

  // Tests only; call before on_configure.
  void set_clock_for_testing(MonotonicClock clock) noexcept;

  // Test observability: how many targets have been accepted. A bare manager
  // cycle must never advance it.
  [[nodiscard]] std::uint64_t target_generation() const noexcept;

 private:
  void accept(double target) noexcept;

  std::string joint_name_;
  BoundedTarget limits_{};
  TargetLimiter limiter_;
  MonotonicClock clock_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<TargetCommand> inbox_;
  std::atomic<std::uint64_t> generation_{0U};
  std::uint64_t applied_generation_{0U};
  double command_{0.0};
  std::atomic<bool> active_{false};
};

}  // namespace mech::mech_controllers
