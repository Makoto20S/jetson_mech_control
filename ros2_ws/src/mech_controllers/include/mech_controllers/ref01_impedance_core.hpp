#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

// Portable PH-EXO simulation control contract; no ROS, device or model I/O.
namespace mech::mech_controllers::ref01 {
using Pair = std::array<double, 2>;
constexpr double pi = 3.14159265358979323846;
constexpr double plan_ttl_s = .100;
constexpr double feedback_ttl_s = .060;
constexpr double max_update_s = .010;
constexpr double max_pair_skew_s = .030;
constexpr double max_integer = 9007199254740991.0;

struct Plan {
  bool valid{false};
  bool enabled{false};
  double epoch{0.}, sequence{0.}, issued_at{0.};
  Pair reference{}, stiffness{}, damping{}, caps{};
  bool dynamic{false};
  Pair reference_velocity{};
};

// Only the standalone numerical probe instantiates the extended gain envelope.
template<bool offline_gain_sweep = false>
inline bool valid_plan(const std::array<double, 12>& f, double arrival,
                       double epoch, double previous_sequence,
                       double previous_source, Plan& result, double cap_limit=.2,
                       bool bench_gains=false, bool dynamic=false,
                       const Pair& reference_velocity={}) noexcept {
  constexpr double bench_k = offline_gain_sweep ? 60. : 30.;
  constexpr double bench_b = offline_gain_sweep ? 2. : 1.;
  if(!std::isfinite(cap_limit) || cap_limit<=0. || cap_limit>10.) return false;
  for (double value : f) if (!std::isfinite(value)) return false;
  for (double value : reference_velocity)
    if (!std::isfinite(value) || (!dynamic && value != 0.)) return false;
  if (!std::isfinite(arrival) || !std::isfinite(epoch) || epoch <= 0. ||
      f[0] != 1. || f[1] <= previous_sequence || f[1] <= 0. ||
      f[1] > max_integer || std::floor(f[1]) != f[1] ||
      f[2] < epoch || f[2] < previous_source || f[2] > arrival ||
      arrival - f[2] >= plan_ttl_s || (f[3] != 0. && f[3] != 1.) ||
      f[4] < -pi / 4. || f[4] > pi / 4. || f[5] < -8. * pi / 180. || f[5] > pi / 4. ||
      f[6] <= 0. || f[6] > (bench_gains?bench_k:2.4) || f[7] <= 0. || f[7] > (bench_gains?bench_k:2.) ||
      f[8] < 0. || f[8] > (bench_gains?bench_b:.18) || f[9] < 0. || f[9] > (bench_gains?bench_b:.15) ||
      f[10] <= 0. || f[10] > cap_limit || f[11] <= 0. || f[11] > cap_limit) return false;
  result = {true, f[3] == 1., epoch, f[1], f[2],
            {f[4], f[5]}, {f[6], f[7]}, {f[8], f[9]}, {f[10], f[11]},
            dynamic, reference_velocity};
  return true;
}

struct ModelState {
  Pair q{}, dq{}, gravity{};
  std::array<double, 4> A{};  // row-major: alpha_dot = A * q_dot
  double residual{0.};
  int iterations{0};
};

struct Torque {
  Pair alpha_nm{}, motor_nm{}, feedback_q_nm{};
  double scale{1.};
};

inline bool fixed_pose_torque(const ModelState& state, const Plan& plan,
                             Torque& result, double psi_weight=1., double beta_outward_weight=1.) noexcept {
  result = {};
  for (const auto& pair : {state.q, state.dq, state.gravity, plan.reference,
                          plan.stiffness, plan.damping, plan.caps, plan.reference_velocity})
    for (double value : pair) if (!std::isfinite(value)) return false;
  for (double value : state.A) if (!std::isfinite(value)) return false;
  for (unsigned i = 0; i < 2; ++i)
    if (plan.stiffness[i] <= 0. || plan.damping[i] < 0. || plan.caps[i] <= 0. ||
        (!plan.dynamic && plan.reference_velocity[i] != 0.)) return false;
  const auto& a = state.A;
  const double determinant = a[0] * a[3] - a[1] * a[2];
  const double norm = std::max(std::abs(a[0]) + std::abs(a[1]),
                               std::abs(a[2]) + std::abs(a[3]));
  if (!std::isfinite(determinant) || determinant == 0. || norm == 0.) return false;
  const double inverse_norm = std::max(std::abs(a[3]) + std::abs(a[1]),
                                       std::abs(a[2]) + std::abs(a[0])) / std::abs(determinant);
  if (!std::isfinite(norm * inverse_norm) || norm * inverse_norm > 1e8) return false;
  Pair demand{};
  for (unsigned i = 0; i < 2; ++i) {
    result.feedback_q_nm[i] = plan.stiffness[i] * (plan.reference[i] - state.q[i]) +
                             plan.damping[i] * (plan.reference_velocity[i] - state.dq[i]);
    // Suppress only measured psi impedance beyond the assist envelope.
    // The tolerance is for FK roundoff at the inclusive boundary.
    if (i == 0) result.feedback_q_nm[0] *= std::abs(state.q[0]) > pi / 4. + 1e-8 ? 0. : psi_weight;
    if (i == 1 && result.feedback_q_nm[1] > 0.) result.feedback_q_nm[1] *= beta_outward_weight;
    demand[i] = state.gravity[i] + result.feedback_q_nm[i];
  }
  // Solve A^T*tau = gravity + K*(reference-q) + B*(reference_velocity-dq).
  result.alpha_nm = {(a[3] * demand[0] - a[2] * demand[1]) / determinant,
                     (a[0] * demand[1] - a[1] * demand[0]) / determinant};
  for (double value : result.alpha_nm) if (!std::isfinite(value)) return false;
  const double ratio = std::max({1., std::abs(result.alpha_nm[0]) / plan.caps[0],
                                 std::abs(result.alpha_nm[1]) / plan.caps[1]});
  if (!std::isfinite(ratio)) return false;
  result.scale = 1. / ratio;
  // Common scaling can round one ULP beyond the cap. Preserve the exact
  // numeric contract expected by the downstream command lease.
  for (unsigned i = 0; i < 2; ++i)
    result.alpha_nm[i] = std::clamp(result.alpha_nm[i] * result.scale,
                                  -plan.caps[i], plan.caps[i]);
  // Confirmed simulation fixture mapping only: alpha1 -> 105, alpha2 -> 104.
  result.motor_nm = {result.alpha_nm[1], result.alpha_nm[0]};
  return true;
}

struct AxisSample {
  double position{0.}, velocity{0.}, source_time_s{0.}, sequence{0.};
};
using Feedback = std::array<AxisSample, 2>;  // motor104, motor105
enum class Mode { Inactive, Waiting, Following, Faulted };
enum class Reason {
  None, Inactive, Period, InvalidFeedback, FeedbackRollback, FeedbackMutation,
  FeedbackExpired, PairSkew, PlanExpired, Disabled, PlanChanged, Model,
  InitialReference, InvalidTorque, ComputeDeadline
};
inline const char* reason_name(Reason reason) noexcept {
  switch (reason) {
    case Reason::None: return "none";
    case Reason::Inactive: return "inactive";
    case Reason::Period: return "update_period";
    case Reason::InvalidFeedback: return "invalid_feedback";
    case Reason::FeedbackRollback: return "feedback_rollback";
    case Reason::FeedbackMutation: return "feedback_mutation";
    case Reason::FeedbackExpired: return "feedback_expired";
    case Reason::PairSkew: return "pair_skew";
    case Reason::PlanExpired: return "plan_expired";
    case Reason::Disabled: return "disabled";
    case Reason::PlanChanged: return "plan_changed";
    case Reason::Model: return "model_invalid";
    case Reason::InitialReference: return "initial_reference";
    case Reason::InvalidTorque: return "invalid_torque";
    case Reason::ComputeDeadline: return "compute_deadline";
  }
  return "unknown";
}

struct Observation {
  Mode mode{Mode::Inactive};
  Reason reason{Reason::Inactive};
  ModelState state{};
  Torque torque{};
  bool refresh{false};
  double source_time_s{0.};
  std::uint64_t generation{0};
};

class Session {
 public:
  explicit Session(double psi_ramp_s=.2) noexcept : psi_ramp_s_(psi_ramp_s) {}
  void activate(double now) noexcept {
    psi_outside_ = false;
    psi_return_at_ = -1.;
    epoch_ = now;
    previous_time_ = 0.;
    observed_ = {};
    commanded_sequence_ = {};
    locked_ = {};
    observation_ = {};
    observation_.mode = Mode::Waiting;
    observation_.reason = Reason::None;
  }
  void deactivate() noexcept {
    observation_.mode = Mode::Inactive;
    observation_.reason = Reason::Inactive;
    observation_.refresh = false;
    observation_.torque = {};
  }
  const Observation& observation() const noexcept { return observation_; }
  const Observation& reject(Reason reason) noexcept {
    if (observation_.mode == Mode::Inactive || observation_.mode == Mode::Faulted)
      return observation_;
    if (observation_.refresh && observation_.generation > 0) --observation_.generation;
    return fault(reason);
  }

  // Evaluate is called only for a new complete pair. It fills ModelState and
  // returns false on invalid geometry. No source or callback arrival is renewed.
  template<class Evaluate>
  const Observation& update(const Feedback& feedback, const Plan& plan,
                            double now, double dt, Evaluate&& evaluate) noexcept {
    observation_.refresh = false;
    if (observation_.mode == Mode::Inactive || observation_.mode == Mode::Faulted)
      return observation_;
    if (!std::isfinite(now) || !std::isfinite(dt) || now < epoch_ || dt <= 0. ||
        (locked_.valid && (dt >= max_update_s ||
          (previous_time_ > 0. && (now <= previous_time_ || now - previous_time_ >= max_update_s)))))
      return fault(Reason::Period);
    previous_time_ = now;
    for (unsigned i = 0; i < 2; ++i) {
      const auto& f = feedback[i];
      const auto& old = observed_[i];
      if (!std::isfinite(f.position) || !std::isfinite(f.velocity) ||
          !std::isfinite(f.source_time_s) || !std::isfinite(f.sequence) ||
          f.source_time_s <= 0. || f.source_time_s > now ||
          f.sequence <= 0. || f.sequence > max_integer || std::floor(f.sequence) != f.sequence)
        return fault(Reason::InvalidFeedback);
      if (old.sequence > 0.) {
        if (f.sequence < old.sequence || f.source_time_s < old.source_time_s)
          return fault(Reason::FeedbackRollback);
        if ((f.sequence > old.sequence && f.source_time_s <= old.source_time_s) ||
            (f.sequence == old.sequence &&
             (f.source_time_s != old.source_time_s || f.position != old.position || f.velocity != old.velocity)))
          return fault(Reason::FeedbackMutation);
      }
      if (now - f.source_time_s >= feedback_ttl_s) return fault(Reason::FeedbackExpired);
    }
    if (std::abs(feedback[0].source_time_s - feedback[1].source_time_s) >= max_pair_skew_s)
      return fault(Reason::PairSkew);
    observed_ = feedback;
    // A fresh activation must observe a post-activation pair before issuing any
    // effort. The sample used to warm the model during activation is not a lease.
    if (!locked_.valid && (feedback[0].source_time_s <= epoch_ ||
                          feedback[1].source_time_s <= epoch_)) return observation_;
    if (!plan.valid || plan.epoch != epoch_) {
      if (locked_.valid) return fault(Reason::PlanExpired);
      return observation_;
    }
    if (plan.issued_at > now || now - plan.issued_at >= plan_ttl_s) {
      if (locked_.valid) return fault(Reason::PlanExpired);
      return observation_;
    }
    if (!plan.enabled) {
      if (locked_.valid) return fault(Reason::Disabled);
      return observation_;
    }
    if (locked_.valid && (plan.dynamic != locked_.dynamic ||
                         (!plan.dynamic && plan.reference != locked_.reference) ||
                         plan.stiffness != locked_.stiffness ||
                         plan.damping != locked_.damping || plan.caps != locked_.caps))
      return fault(Reason::PlanChanged);
    // Holding a valid sample is permitted until its original source expires;
    // it does not count as a new command generation or renew the receiver lease.
    if (feedback[0].sequence <= commanded_sequence_[0] ||
        feedback[1].sequence <= commanded_sequence_[1]) return observation_;
    const Pair alpha{feedback[1].position - pi / 4., feedback[0].position - pi / 4.};
    const Pair velocity{feedback[1].velocity, feedback[0].velocity};
    ModelState state{};
    if (!evaluate(alpha, velocity, state)) return fault(Reason::Model);
    for (unsigned i = 0; i < 2; ++i) {
      if (!std::isfinite(state.q[i]) || !std::isfinite(state.dq[i])) return fault(Reason::Model);
      if (!locked_.valid && (std::abs(state.q[i] - plan.reference[i]) > .01 ||
                            std::abs(state.dq[i]) > .05 ||
                            std::abs(plan.reference_velocity[i]) > .05)) return fault(Reason::InitialReference);
    }
    Torque torque{};
    double weight = 1.;
    if (std::abs(state.q[0]) > pi / 4. + 1e-8) {
      psi_outside_ = true;
      psi_return_at_ = -1.;
      weight = 0.;
    } else {
      if (psi_outside_) {
        psi_outside_ = false;
        psi_return_at_ = now;
      }
      if (psi_return_at_ >= 0. && psi_ramp_s_ > 0.) {
        const double u = std::clamp((now - psi_return_at_) / psi_ramp_s_, 0., 1.);
        weight = u*u*u*(10. + u*(-15. + 6.*u));
        if (u == 1.) psi_return_at_ = -1.;
      }
    }
    // Use the measured input angles: phi = 135 deg - mean(theta104, theta105).
    // Fade only positive beta impedance over the final two degrees of phi.
    const double phi = pi / 2. - (alpha[0] + alpha[1]) / 2.;
    constexpr double fade_width = 2. * pi / 180.;
    const double u = std::clamp((pi / 2. - phi) / fade_width, 0., 1.);
    const double outward_weight = u * u * (3. - 2. * u);
    if (!fixed_pose_torque(state, plan, torque, weight, outward_weight)) return fault(Reason::InvalidTorque);
    locked_ = plan;
    commanded_sequence_ = {feedback[0].sequence, feedback[1].sequence};
    observation_.mode = Mode::Following;
    observation_.reason = Reason::None;
    observation_.state = state;
    observation_.torque = torque;
    observation_.source_time_s = std::min(feedback[0].source_time_s, feedback[1].source_time_s);
    observation_.refresh = true;
    ++observation_.generation;
    return observation_;
  }

 private:
  const Observation& fault(Reason reason) noexcept {
    observation_.mode = Mode::Faulted;
    observation_.reason = reason;
    observation_.refresh = false;
    observation_.torque = {};
    return observation_;
  }
  double psi_ramp_s_, psi_return_at_{-1.};
  bool psi_outside_{false};
  double epoch_{0.}, previous_time_{0.};
  Feedback observed_{};
  Pair commanded_sequence_{};
  Plan locked_{};
  Observation observation_{};
};
}  // namespace mech::mech_controllers::ref01
