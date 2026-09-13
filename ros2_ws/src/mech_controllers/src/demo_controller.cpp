#include "mech_controllers/demo_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mech::mech_controllers {

bool TargetLimiter::configure(BoundedTarget limits) noexcept {
  if (!std::isfinite(limits.minimum) || !std::isfinite(limits.maximum) ||
      !std::isfinite(limits.max_slew_per_second) ||
      limits.minimum > limits.maximum || limits.max_slew_per_second <= 0.0 ||
      limits.ttl_nanoseconds <= 0 || limits.hard_ttl_nanoseconds <= 0 ||
      limits.hard_ttl_nanoseconds <= limits.ttl_nanoseconds) {
    return false;
  }
  limits_ = limits;
  target_ = 0.0;
  held_ = 0.0;
  deadline_ = 0;
  configured_ = true;
  has_target_ = false;
  has_held_ = false;
  return true;
}

bool TargetLimiter::submit(double target,
                           std::int64_t now_nanoseconds) noexcept {
  if (!configured_ || !std::isfinite(target) || now_nanoseconds < 0 ||
      now_nanoseconds > std::numeric_limits<std::int64_t>::max() -
                            limits_.hard_ttl_nanoseconds) {
    return false;
  }
  target_ = std::clamp(target, limits_.minimum, limits_.maximum);
  deadline_ = now_nanoseconds + limits_.ttl_nanoseconds;
  has_target_ = true;
  return true;
}

double TargetLimiter::update(double previous, double period_seconds,
                             std::int64_t now_nanoseconds) noexcept {
  if (!configured_ || !std::isfinite(previous) || !std::isfinite(period_seconds) ||
      period_seconds < 0.0) {
    // Invalid input: never invent a value. Hold the last value we know to be
    // valid (seeded lazily from `previous` the first time it is finite) so we
    // never command a silent jump to zero.
    if (!has_held_ && std::isfinite(previous)) {
      held_ = previous;
      has_held_ = true;
    }
    return held_;
  }
  const auto current_stage = stage(now_nanoseconds);
  double result = previous;
  if (current_stage == WatchdogStage::Following) {
    const auto maximum_step = limits_.max_slew_per_second * period_seconds;
    result = previous + std::clamp(target_ - previous, -maximum_step, maximum_step);
  } else {
    // Holding or Expired: freeze at the last valid commanded value rather
    // than continuing to slew toward the (now stale) target.
    result = has_held_ ? held_ : previous;
  }
  held_ = result;
  has_held_ = true;
  return result;
}

void TargetLimiter::clear() noexcept {
  target_ = 0.0;
  held_ = 0.0;
  deadline_ = 0;
  has_target_ = false;
  has_held_ = false;
}

bool TargetLimiter::expired(std::int64_t now_nanoseconds) const noexcept {
  return stage(now_nanoseconds) != WatchdogStage::Following;
}

WatchdogStage TargetLimiter::stage(std::int64_t now_nanoseconds) const noexcept {
  // No target has ever been submitted, so there is nothing legitimate to
  // follow and nothing stale either: this watchdog measures how old a target
  // is, and a target that never existed has no age. Hold - the caller keeps
  // whatever value it seeded (for DemoController, the position just measured
  // at activation), and never slews toward the default target_ of 0.0, which
  // on a position interface is a commanded move to the calibrated zero.
  //
  // Deliberately NOT "expire after the hard TTL measured from zero": under a
  // real clock now_nanoseconds is time since boot, so that comparison made
  // every activation start out already Expired. Expiry stays reserved for a
  // target that was submitted and then went stale, which is the case ADR-012
  // is about.
  if (!has_target_ || now_nanoseconds < 0) {
    return WatchdogStage::Holding;
  }
  if (now_nanoseconds < deadline_) return WatchdogStage::Following;
  const auto hard_deadline =
      deadline_ + (limits_.hard_ttl_nanoseconds - limits_.ttl_nanoseconds);
  if (now_nanoseconds < hard_deadline) return WatchdogStage::Holding;
  return WatchdogStage::Expired;
}

DemoController::DemoController()
    : clock_([]() noexcept {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
      }) {}

void DemoController::set_clock_for_testing(MonotonicClock clock) noexcept {
  if (clock) clock_ = std::move(clock);
}

std::uint64_t DemoController::target_generation() const noexcept {
  return generation_.load(std::memory_order_acquire);
}

// Runs on the executor thread for a topic message, or on the caller's thread
// for set_target(). The generation is published last, with release ordering,
// so update() can never see a new generation paired with an older value.
void DemoController::accept(double target) noexcept {
  const auto generation = generation_.load(std::memory_order_relaxed) + 1U;
  inbox_.writeFromNonRT(TargetCommand{target, generation});
  generation_.store(generation, std::memory_order_release);
}

controller_interface::CallbackReturn DemoController::on_init() {
  try {
    auto_declare<std::string>("joint", "joint_1");
    auto_declare<double>("minimum", -1.0);
    auto_declare<double>("maximum", 1.0);
    auto_declare<double>("max_slew_per_second", 1.0);
    auto_declare<int64_t>("ttl_nanoseconds", 4000000);
    auto_declare<int64_t>("hard_ttl_nanoseconds", 6000000);
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_configure(
    const rclcpp_lifecycle::State&) {
  try {
    joint_name_ = get_node()->get_parameter("joint").as_string();
    limits_.minimum = get_node()->get_parameter("minimum").as_double();
    limits_.maximum = get_node()->get_parameter("maximum").as_double();
    limits_.max_slew_per_second =
        get_node()->get_parameter("max_slew_per_second").as_double();
    limits_.ttl_nanoseconds = get_node()->get_parameter("ttl_nanoseconds").as_int();
    limits_.hard_ttl_nanoseconds =
        get_node()->get_parameter("hard_ttl_nanoseconds").as_int();
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (joint_name_.empty() || !limiter_.configure(limits_)) {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Controller-relative, so the full topic name follows the controller's
  // namespace and two deployments of this controller cannot collide.
  subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      "~/target_position", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        if (message == nullptr) return;
        // A target only means something while this controller holds the claim.
        // Dropping here rather than buffering is what stops a re-activation
        // from replaying something a publisher sent while the joint belonged
        // to nobody.
        if (!active_.load(std::memory_order_acquire)) return;
        // Reject rather than clamp: a NaN or infinite target is a broken
        // publisher, not a request for the limit, and clamping it would turn
        // that bug into a full-scale motion command.
        if (!std::isfinite(message->data)) return;
        accept(message->data);
      });

  command_ = 0.0;
  active_.store(false, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_activate(
    const rclcpp_lifecycle::State&) {
  if (command_interfaces_.size() != 1U || state_interfaces_.size() != 1U ||
      command_interfaces_[0].get_name() !=
          joint_name_ + "/" + hardware_interface::HW_IF_POSITION ||
      state_interfaces_[0].get_name() !=
          joint_name_ + "/" + hardware_interface::HW_IF_POSITION) {
    return controller_interface::CallbackReturn::ERROR;
  }
  // Seed the hold value from the position just measured. ADR-016 already
  // refuses the claim while no valid sample exists, so reaching here should
  // mean feedback flowed; this check is the local guard against trusting that
  // reasoning rather than the number in front of us. Failing closed is right
  // because the only other seed available is a fabricated one, and on a
  // position interface a fabricated 0.0 is a commanded move to the calibrated
  // zero.
  const auto seed = state_interfaces_[0].get_value();
  if (!std::isfinite(seed)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  command_ = seed;
  limiter_.clear();
  // Ignore anything already in the inbox: a target that predates this
  // activation is not a target for this activation.
  applied_generation_ = generation_.load(std::memory_order_acquire);
  active_.store(true, std::memory_order_release);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DemoController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_.store(false, std::memory_order_release);
  limiter_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DemoController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/" + hardware_interface::HW_IF_POSITION}};
}

controller_interface::InterfaceConfiguration
DemoController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {joint_name_ + "/" + hardware_interface::HW_IF_POSITION}};
}

// The rclcpp::Time argument is deliberately unused: see MonotonicClock in the
// header. `period` is the control period rather than a deadline, so it still
// comes from the manager.
controller_interface::return_type DemoController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  if (!active_.load(std::memory_order_acquire) ||
      command_interfaces_.size() != 1U || period.nanoseconds() < 0) {
    return controller_interface::return_type::ERROR;
  }
  const auto now = clock_();

  // A new generation means a message really arrived, and that is the only
  // thing allowed to refresh the TTL. The acquire load pairs with accept()'s
  // release store.
  if (generation_.load(std::memory_order_acquire) != applied_generation_) {
    const TargetCommand pending = *inbox_.readFromRT();
    if (pending.generation != applied_generation_) {
      if (!limiter_.submit(pending.value, now)) {
        // submit() refuses a non-finite target (already filtered above) or a
        // clock so near int64 overflow that the deadline cannot be expressed.
        // Either way the inputs are broken, so say so rather than carry on
        // with a target whose deadline is unknown.
        return controller_interface::return_type::ERROR;
      }
      applied_generation_ = pending.generation;
    }
  }

  if (limiter_.stage(now) == WatchdogStage::Expired) {
    return controller_interface::return_type::ERROR;
  }
  command_ = limiter_.update(command_, period.seconds(), now);
  command_interfaces_[0].set_value(command_);
  return controller_interface::return_type::OK;
}

// The non-ROS entry, kept so unit tests can drive the watchdog without an
// executor. It routes through the same buffer and counter as the
// subscription, so the two entries cannot drift apart.
bool DemoController::set_target(double target) noexcept {
  if (!active_.load(std::memory_order_acquire) || !std::isfinite(target)) {
    return false;
  }
  accept(target);
  return true;
}

}  // namespace mech::mech_controllers

PLUGINLIB_EXPORT_CLASS(mech::mech_controllers::DemoController,
                       controller_interface::ControllerInterface)
