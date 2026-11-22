#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "mech_controllers/ref01_impedance_core.hpp"
#include "mech_controllers/ref01_predictive/ref01_predictive_session.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ph_exo_interfaces/msg/fixed_pose_plan.hpp"
#include "ph_exo_interfaces/msg/impedance_ref.hpp"
#include "ph_exo_interfaces/msg/impedance_status.hpp"

namespace ph_exo { class Ref01Model; }

namespace mech::mech_controllers {
// Coupled controller without device/codec dependencies. Select simulation_only
// or the explicit ref01_supported_v1 bench profile with its bounded adapter.
class Ref01ImpedanceController final : public controller_interface::ControllerInterface {
 public:
  Ref01ImpedanceController();
  ~Ref01ImpedanceController() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // Non-RT interface; ROS callback and tests use the same validation.
  bool set_plan(const std::array<double, 12>& frame,
                const ref01::Pair& velocity = {}, bool dynamic = false);
  // Read only in the thread invoking update (or after that thread has stopped).
  const ref01::Observation& observation() const noexcept { return predictive_?predictive_session_.observation():session_.observation(); }
  double compute_seconds() const noexcept { return compute_seconds_; }

 private:
  struct FaultTiming {
    double at_s{}, manager_period_s{}, update_gap_s{};
    ref01::Pair feedback_age_s{};
  };
  FaultTiming first_fault_{};
  double previous_update_s_{};
  struct SoftwareStatus {
    ref01::Observation observation{};
    double observed_at_monotonic_s{0.};
    double compute_seconds{0.};
    FaultTiming first_fault{};
  };
  ref01::Feedback feedback() const noexcept;
  void zero_effort() noexcept;
  // A contending service may skip one observation; the timestamp exposes its age.
  // Lifecycle hooks also use try_lock, since manager switching can run in update.
  void refresh_software_status(double observed_at) noexcept;
  std::vector<std::string> joints_{"motor104_joint", "motor105_joint"};
  std::unique_ptr<ph_exo::Ref01Model> model_;
  ref01::Session session_;
  ph_exo::control::PredictiveSession predictive_session_;
  void reject(ref01::Reason reason) noexcept {
    if(predictive_)predictive_session_.reject(reason);else session_.reject(reason);
  }
  ref01::Pair accepted_sequences_{};
  bool predictive_{false};
  ref01::Pair previous_alpha_{}, generation_{};
  realtime_tools::RealtimeBuffer<ref01::Plan> inbox_{ref01::Plan{}};
  std::atomic<double> epoch_{0.};
  std::mutex plan_mutex_;  // Non-RT writers only; update never locks it.
  double last_plan_sequence_{0.}, last_plan_source_{0.};
  std::array<double,2> last_reference_{}, last_reference_velocity_{};
  bool dynamic_{false};
  double compute_seconds_{0.};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  std::mutex status_mutex_;
  SoftwareStatus software_status_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_, stop_service_;
  std::atomic<double> stop_epoch_{0.};
  bool bench_{false};
  rclcpp::Subscription<ph_exo_interfaces::msg::FixedPosePlan>::SharedPtr typed_subscription_;
  rclcpp::Subscription<ph_exo_interfaces::msg::ImpedanceRef>::SharedPtr ref_subscription_;
  std::atomic<std::uint64_t> typed_session_{0};
  rclcpp::Publisher<ph_exo_interfaces::msg::ImpedanceStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace mech::mech_controllers
