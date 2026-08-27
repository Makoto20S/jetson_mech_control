#pragma once

#include <cstdint>
#include <string>

#include "controller_interface/controller_interface.hpp"

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

  [[nodiscard]] bool set_target(double target,
                                std::int64_t now_nanoseconds) noexcept;

 private:
  std::string joint_name_;
  BoundedTarget limits_{};
  TargetLimiter limiter_;
  double command_{0.0};
  bool active_{false};
};

}  // namespace mech::mech_controllers
