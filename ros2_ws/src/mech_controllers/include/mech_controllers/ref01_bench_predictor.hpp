#pragma once

#include "ref01_impedance_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace mech::mech_controllers::ref01 {

// Software-stage predictor for the explicit REF01 bench profile. Real sensor
// sequence/source-time validation and freshness gates remain with the caller.
// Neither command() nor advance() changes the original observation source time.
class BenchPredictor {
 public:
  void reset() noexcept {
    state_ = {};
    mass_ = {};
    external_ = {};
    source_ = 0.;
    initialized_ = false;
    failed_ = false;
  }

  bool observe(const ModelState& measured, const std::array<double, 4>& mass,
               double source) noexcept {
    if (failed_) return false;
    if (!valid_state(measured) || !valid_mass(mass) ||
        !std::isfinite(source) || source <= 0.) return fail();

    const auto& a = measured.A;
    const double det = a[0] * a[3] - a[1] * a[2];
    const double norm = std::max(std::abs(a[0]) + std::abs(a[1]),
                                 std::abs(a[2]) + std::abs(a[3]));
    if (!std::isfinite(det) || det == 0. || norm == 0.) return fail();
    const Pair inverse_row_norm{
        (std::abs(a[3]) + std::abs(a[1])) / std::abs(det),
        (std::abs(a[2]) + std::abs(a[0])) / std::abs(det)};
    const double condition = norm * std::max(inverse_row_norm[0], inverse_row_norm[1]);
    if (!std::isfinite(condition) || condition > 1e8) return fail();

    ModelState next = measured;
    Pair next_external = external_;
    if (initialized_) {
      const double elapsed = source - source_;
      // New observations only. Repeated/old observations cannot masquerade as
      // fresh model corrections or renew the downstream sensor lease.
      if (!std::isfinite(elapsed) || elapsed < .005 - 1e-12 ||
          elapsed >= feedback_ttl_s) return fail();
      Pair correction_velocity{};
      constexpr double position_half_count = .05 * pi / 180.;
      constexpr double speed_half_count = 5. * 2. * pi / (60. * 21. * 8.);
      for (unsigned i = 0; i < 2; ++i) {
        const double position_uncertainty = inverse_row_norm[i] * position_half_count;
        const double velocity_uncertainty = inverse_row_norm[i] * speed_half_count;
        correction_velocity[i] = outside_interval(
            measured.dq[i] - state_.dq[i], velocity_uncertainty);
        next.q[i] = state_.q[i] + outside_interval(
            measured.q[i] - state_.q[i], position_uncertainty);
        next.dq[i] = state_.dq[i] + correction_velocity[i];
      }
      for (unsigned i = 0; i < 2; ++i) {
        next_external[i] += (1. - std::pow(.75, elapsed / .020)) *
            (mass[2 * i] * correction_velocity[0] +
             mass[2 * i + 1] * correction_velocity[1]) / elapsed;
      }
    }
    // Reject invalid estimates; never silently clip an innovation into a
    // passing observation. This bound is on the software disturbance estimate,
    // not an actuator torque limit or an assertion about external human force.
    for (double value : next_external)
      if (!std::isfinite(value) || std::abs(value) > external_estimate_limit_nm)
        return fail();
    if (!valid_state(next)) return fail();
    state_ = next;
    mass_ = mass;
    external_ = next_external;
    source_ = source;
    initialized_ = true;
    return true;
  }

  bool command(const Plan& plan, Torque& torque) noexcept {
    torque = {};
    if (failed_) return false;
    if (!initialized_ || !plan.valid || !plan.enabled) return fail();
    for (unsigned i = 0; i < 2; ++i) {
      if (!std::isfinite(plan.reference[i]) || !std::isfinite(plan.stiffness[i]) ||
          !std::isfinite(plan.damping[i]) || !std::isfinite(plan.caps[i]) ||
          plan.stiffness[i] <= 0. || plan.damping[i] < 0. || plan.caps[i] <= 0.)
        return fail();
    }

    // Evaluate nominal K/B at a 5 ms predicted midpoint. The disturbance
    // estimate is used in the midpoint acceleration only, never added as
    // feedforward assistance to the actuator demand.
    constexpr double h = .005;
    auto implicit_mass = mass_;
    Pair force{};
    for (unsigned i = 0; i < 2; ++i) {
      implicit_mass[3 * i] += .5 * h * plan.damping[i] +
                              .25 * h * h * plan.stiffness[i];
      force[i] = plan.stiffness[i] *
                     (plan.reference[i] - state_.q[i] - .5 * h * state_.dq[i]) -
                 plan.damping[i] * state_.dq[i] + external_[i];
    }
    if (!valid_mass(implicit_mass)) return fail();
    const double det = implicit_mass[0] * implicit_mass[3] -
                       implicit_mass[1] * implicit_mass[2];
    const Pair acceleration{
        (implicit_mass[3] * force[0] - implicit_mass[1] * force[1]) / det,
        (implicit_mass[0] * force[1] - implicit_mass[2] * force[0]) / det};
    ModelState midpoint = state_;
    for (unsigned i = 0; i < 2; ++i) {
      midpoint.q[i] += .5 * h * state_.dq[i] + .25 * h * h * acceleration[i];
      midpoint.dq[i] += .5 * h * acceleration[i];
    }
    if (!valid_state(midpoint) || !fixed_pose_torque(midpoint, plan, torque)) {
      torque = {};
      return fail();
    }
    // fixed_pose_torque solves A^T*tau = gravity + K*error - B*velocity
    // and applies one common scale to both motor commands. advance() still
    // integrates the actual dequantized torque, including any saturation.
    return true;
  }

  bool advance(const Pair& actual_motor_torque, double dt) noexcept {
    if (failed_) return false;
    if (!initialized_ || !std::isfinite(dt) || dt <= 0. || dt >= max_update_s)
      return fail();
    for (double value : actual_motor_torque)
      if (!std::isfinite(value)) return fail();
    // Input is the actual dequantized wire command in [104,105] order, not
    // the desired floating-point command or an unquantized torque surrogate.
    const Pair alpha{actual_motor_torque[1], actual_motor_torque[0]};
    const auto& a = state_.A;
    const Pair force{
        a[0] * alpha[0] + a[2] * alpha[1] - state_.gravity[0] + external_[0],
        a[1] * alpha[0] + a[3] * alpha[1] - state_.gravity[1] + external_[1]};
    const double det = mass_[0] * mass_[3] - mass_[1] * mass_[2];
    const Pair acceleration{
        (mass_[3] * force[0] - mass_[1] * force[1]) / det,
        (mass_[0] * force[1] - mass_[2] * force[0]) / det};
    ModelState next = state_;
    for (unsigned i = 0; i < 2; ++i) {
      next.q[i] += dt * state_.dq[i] + .5 * dt * dt * acceleration[i];
      next.dq[i] += dt * acceleration[i];
    }
    if (!valid_state(next)) return fail();
    state_ = next;
    return true;
  }

  double source() const noexcept { return source_; }
  bool initialized() const noexcept { return initialized_; }
  bool failed() const noexcept { return failed_; }
  const ModelState& state() const noexcept { return state_; }
  const Pair& external() const noexcept { return external_; }

 private:
  static constexpr double external_estimate_limit_nm = 10.;

  static double outside_interval(double delta, double half_width) noexcept {
    return std::copysign(std::max(std::abs(delta) - half_width, 0.), delta);
  }

  static bool valid_state(const ModelState& state) noexcept {
    for (const auto& pair : {state.q, state.dq, state.gravity})
      for (double value : pair) if (!std::isfinite(value)) return false;
    for (double value : state.A) if (!std::isfinite(value)) return false;
    return std::isfinite(state.residual) && state.residual >= 0. &&
           state.q[0] >= -pi / 4. && state.q[0] <= pi / 4. &&
           state.q[1] >= -8. * pi / 180. && state.q[1] <= pi / 4.;
  }

  static bool valid_mass(const std::array<double, 4>& mass) noexcept {
    for (double value : mass) if (!std::isfinite(value)) return false;
    const double scale = std::max({std::abs(mass[0]), std::abs(mass[1]),
                                   std::abs(mass[2]), std::abs(mass[3])});
    if (!(scale > 0.) || mass[0] <= 0. || mass[3] <= 0. ||
        std::abs(mass[1] - mass[2]) > 1e-10 * scale) return false;
    const double det = mass[0] * mass[3] - mass[1] * mass[2];
    return std::isfinite(det) && det > 1e-12 * scale * scale;
  }

  bool fail() noexcept { failed_ = true; return false; }
  ModelState state_{};
  std::array<double, 4> mass_{};
  Pair external_{};
  double source_{};
  bool initialized_{false}, failed_{false};
};

}  // namespace mech::mech_controllers::ref01
