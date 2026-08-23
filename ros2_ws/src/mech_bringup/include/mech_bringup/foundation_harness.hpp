#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mech_controllers/demo_controller.hpp"
#include "mech_hardware_ros2_control/composite_system.hpp"

namespace mech::mech_bringup {

struct HarnessMetrics final {
  std::uint64_t cycles{0U};
  std::uint64_t failures{0U};
  std::int64_t maximum_cycle_nanoseconds{0};
};

class FoundationHarness final {
 public:
  [[nodiscard]] bool configure(std::size_t joint_count) noexcept;
  [[nodiscard]] bool activate() noexcept;
  [[nodiscard]] bool switch_claim(bool claim) noexcept;
  [[nodiscard]] bool set_target(double target,
                                std::int64_t now_nanoseconds) noexcept;
  [[nodiscard]] bool cycle(std::int64_t now_nanoseconds,
                           std::int64_t period_nanoseconds) noexcept;
  [[nodiscard]] bool deactivate() noexcept;
  [[nodiscard]] bool cleanup() noexcept;
  [[nodiscard]] double position() const noexcept;
  [[nodiscard]] double command() const noexcept;
  [[nodiscard]] const HarnessMetrics& metrics() const noexcept { return metrics_; }

 private:
  mech_hardware_ros2_control::CompositeSystem hardware_;
  mech_controllers::TargetLimiter limiter_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;
  HarnessMetrics metrics_{};
  std::string command_name_;
  bool configured_{false};
  bool active_{false};
  bool claimed_{false};
};

}  // namespace mech::mech_bringup
