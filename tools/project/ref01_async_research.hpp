#ifndef PH_ASYNC_DAMPING_MIN
#define PH_ASYNC_DAMPING_MIN .8
#endif
#ifndef PH_ASYNC_IDENTIFY
#define PH_ASYNC_IDENTIFY 1
#endif
#ifndef PH_ASYNC_EXTERNAL
#define PH_ASYNC_EXTERNAL 1
#endif
#ifndef PH_ASYNC_OBSERVATION_PERIOD
#define PH_ASYNC_OBSERVATION_PERIOD .02
#endif
#ifndef PH_ASYNC_EXTERNAL_POLE
#define PH_ASYNC_EXTERNAL_POLE .75
#endif
#pragma once

#include "ref01_impedance_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace mech::mech_controllers::ref01 {

// Experimental inertia-adaptive predictor; opt-in Windows validation only.
// No plant parameters or truth state enter this class. Diagonal effective inertia
// is identified from source-aligned velocity increments and delivered torque
// impulses; nominal cross terms are retained. This is not a full rigid-body
// parameter identification or a proof of robustness over the projection bounds.
// Real sensor
// sequence/source-time validation and freshness gates remain with the caller.
// Neither command() nor advance() changes the original observation source time.
class AsyncResearchPredictor {
 public:
  void set_identified_mass(Pair value) noexcept { override_mass_=value; }
  void reset() noexcept {
    override_mass_={};impulse_={};last_measured_velocity_={};last_acceleration_={};last_force_={};
    covariance_={.25,.25};cross_={};estimate_={};have_interval_=false;
    integrated_s_=0.;strong_updates_={};weak_updates_={};nominal_diagonal_={};commanded_damping_={};
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

    auto adjusted_mass = mass;
    if (initialized_) {
      const double elapsed = source - source_;
      if (!std::isfinite(elapsed) || elapsed < .005-1e-12 || elapsed >= feedback_ttl_s)
        return fail();
      // Restored source-time history must contain every actually applied interval.
      // Late delivery may change wall time, never the duration of this impulse.
      if (std::abs(integrated_s_-elapsed)>1e-9) return fail();
      Pair acceleration{}, average_force{};
      for (unsigned i=0; i<2; ++i) {
        acceleration[i]=(measured.dq[i]-last_measured_velocity_[i])/elapsed;
        average_force[i]=impulse_[i]/elapsed;
      }
      if (have_interval_ && PH_ASYNC_IDENTIFY && override_mass_[0]<=0.) {
        for (unsigned i=0; i<2; ++i) {
          const double x=acceleration[i]-last_acceleration_[i];
          const double df=average_force[i]-last_force_[i];
          const double y=df-mass[1]*(acceleration[1-i]-last_acceleration_[1-i]);
          // Differencing adjacent source intervals cancels a constant unknown load.
          // Strong excitation drives weighted least squares. Quantized low-speed
          // observations must not repeatedly erase earlier informative evidence.
          if (std::abs(x)>strong_acceleration_change && std::abs(df)>strong_force_change) {
            covariance_[i]=.98*covariance_[i]+x*x;
            cross_[i]=.98*cross_[i]+x*y;
            estimate_[i]=std::clamp(cross_[i]/covariance_[i],.5*mass[3*i],30.*mass[3*i]);
            ++strong_updates_[i];
            if(covariance_[i]>1e10) {covariance_[i]*=.5;cross_[i]*=.5;}
          } else if (x*y>0. && std::abs(x)>weak_acceleration_change &&
                     std::abs(df)>weak_force_margin) {
            // With weak excitation only admit clear upward inertia evidence:
            // otherwise a quiet, unexcited axis can remain grossly under-modelled.
            // Margins are empirical noise tolerances, not statistical guarantees.
            const double lower=(std::abs(y)-weak_force_margin)/
                               (std::abs(x)+weak_acceleration_margin);
            if(lower>1.5*estimate_[i]) {
              estimate_[i]=std::clamp(lower,.5*mass[3*i],30.*mass[3*i]);
              ++weak_updates_[i];
            }
          }
        }
      }
      last_acceleration_=acceleration;last_force_=average_force;have_interval_=true;
      for (unsigned i=0;i<2;++i) { if(override_mass_[i]>0.) estimate_[i]=override_mass_[i]; adjusted_mass[3*i]=estimate_[i]; }
    } else {
      for(unsigned i=0;i<2;++i){estimate_[i]=mass[3*i];cross_[i]=.25*estimate_[i];}
    }
    if(!valid_mass(adjusted_mass)) return fail();
    ModelState next = measured;
    Pair next_external = external_;
    if (initialized_) {
      const double elapsed = source - source_;
      // New observations only. Repeated/old observations cannot masquerade as
      // fresh model corrections or renew the downstream sensor lease.
      if (!std::isfinite(elapsed) || elapsed < .005 - 1e-12 ||
          elapsed >= feedback_ttl_s) return fail();
      Pair correction_velocity{};
#ifdef PH_ASYNC_TIME_GAIN
      const double observation_gain=std::min(1.,elapsed/PH_ASYNC_OBSERVATION_PERIOD);
      const double external_gain=1.-std::pow(PH_ASYNC_EXTERNAL_POLE,elapsed/PH_ASYNC_OBSERVATION_PERIOD);
#else
      const double observation_gain=1.;
      const double external_gain=.25;
#endif
      constexpr double position_half_count = .05 * pi / 180.;
      constexpr double speed_half_count = 5. * 2. * pi / (60. * 21. * 8.);
      for (unsigned i = 0; i < 2; ++i) {
        const double position_uncertainty = inverse_row_norm[i] * position_half_count;
        const double velocity_uncertainty = inverse_row_norm[i] * speed_half_count;
        correction_velocity[i] = observation_gain*outside_interval(
            measured.dq[i] - state_.dq[i], velocity_uncertainty);
        next.q[i] = state_.q[i] + observation_gain*outside_interval(
            measured.q[i] - state_.q[i], position_uncertainty);
        next.dq[i] = state_.dq[i] + correction_velocity[i];
      }
      for (unsigned i = 0; i < 2; ++i) {
        // Use the measured source-interval impulse balance after adapting inertia.
        // Do not integrate the entire prediction error into an external force:
        // an inertia error would then be reinforced during delayed replay.
        const double residual = adjusted_mass[2*i]*last_acceleration_[0] +
            adjusted_mass[2*i+1]*last_acceleration_[1] - last_force_[i];
        next_external[i] = PH_ASYNC_EXTERNAL ? ((1.-external_gain)*external_[i] + external_gain*residual) : 0.;
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
    nominal_diagonal_={mass[0],mass[3]};
    mass_ = adjusted_mass;
    last_measured_velocity_=measured.dq;impulse_={};integrated_s_=0.;
    external_ = next_external;
    source_ = source;
    initialized_ = true;
    return true;
  }

  bool command(const Plan& requested_plan, Torque& torque) noexcept {
    // requested_plan.damping is the upper damping setting. For the light nominal
    // model use 80%, blending to 100% at twice the identified nominal inertia.
    // This reduces quantized low-inertia chatter while retaining heavy-axis damping.
    Plan plan=requested_plan;
    for(unsigned i=0;i<2;++i) {
      if(initialized_ && nominal_diagonal_[i]>0.)
        plan.damping[i] *= std::clamp(PH_ASYNC_DAMPING_MIN+(1.-PH_ASYNC_DAMPING_MIN)*(mass_[3*i]/nominal_diagonal_[i]-1.),PH_ASYNC_DAMPING_MIN,1.);
      commanded_damping_[i]=plan.damping[i];
    }
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
                 plan.damping[i] * state_.dq[i] + prediction_external(i,integrated_s_+.5*h);
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
        a[0] * alpha[0] + a[2] * alpha[1] - state_.gravity[0] + prediction_external(0,integrated_s_+.5*dt),
        a[1] * alpha[0] + a[3] * alpha[1] - state_.gravity[1] + prediction_external(1,integrated_s_+.5*dt)};
    for (unsigned i=0;i<2;++i) impulse_[i] += dt*(force[i]-prediction_external(i,integrated_s_+.5*dt));
    integrated_s_ += dt;
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

  const auto& commanded_damping() const noexcept { return commanded_damping_; }
  const auto& strong_updates() const noexcept { return strong_updates_; }
  const auto& weak_updates() const noexcept { return weak_updates_; }
  const auto& estimated_mass() const noexcept { return mass_; }
  double source() const noexcept { return source_; }
  bool initialized() const noexcept { return initialized_; }
  bool failed() const noexcept { return failed_; }
  const ModelState& state() const noexcept { return state_; }
  const Pair& external() const noexcept { return external_; }

 private:
  double prediction_external(unsigned axis,double age) const noexcept {
#ifdef PH_ASYNC_FORCE_DECAY
    // Disturbance is observed at a past source interval. Confidence in its
    // persistence decays across unobserved time; no actuator slew is imposed.
    return external_[axis]*std::exp(-age/PH_ASYNC_FORCE_DECAY);
#else
    (void)age;return external_[axis];
#endif
  }

  static constexpr double external_estimate_limit_nm = 10.;
  static constexpr double strong_acceleration_change = 1.;  // rad/s^2
  static constexpr double strong_force_change = .01;        // generalized N m
  static constexpr double weak_acceleration_change = .1;
  static constexpr double weak_force_margin = .001;
  static constexpr double weak_acceleration_margin = .2;

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
  Pair override_mass_{};
  Pair nominal_diagonal_{},commanded_damping_{};
  double integrated_s_{};
  std::array<unsigned,2> strong_updates_{},weak_updates_{};
  Pair impulse_{},last_measured_velocity_{},last_acceleration_{},last_force_{};
  Pair covariance_{.25,.25},cross_{},estimate_{};
  bool have_interval_{false};
  ModelState state_{};
  std::array<double, 4> mass_{};
  Pair external_{};
  double source_{};
  bool initialized_{false}, failed_{false};
};

}  // namespace mech::mech_controllers::ref01
