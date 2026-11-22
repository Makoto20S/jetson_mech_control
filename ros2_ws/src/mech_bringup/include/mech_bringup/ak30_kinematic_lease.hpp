#pragma once
#include <cmath>
#include <algorithm>
#include <cstdint>
namespace mech::mech_bringup {
// A bounded hold between a slow observation-driven controller and a fast MIT
// session. Re-reading an external generation NEVER renews this lease.
class KinematicCommandLease {
 public:
  void reset(double baseline, double limit=.2) noexcept { limit_=limit; generation_=baseline; accepted_=0; last_=0; started_=false; fault_=false; effort_=0; }
  bool update(double effort, double generation, double source_s, bool feedback_ok,
              std::int64_t now) noexcept {
    if (fault_) return false;
    if (now<=0 || (last_ && now<last_)) return fail();
    last_=now;
    if (started_ && (now-accepted_>=40000000 || !feedback_ok ||
        !std::isfinite(source_s) || source_s<=0 || source_s>now*1e-9 ||
        now*1e-9-source_s>=.060)) return fail();
    if (!std::isfinite(generation) || generation<generation_ || generation<0 ||
        generation>9007199254740991. || std::floor(generation)!=generation) return fail();
    if (generation>generation_) {
      if (!feedback_ok || !std::isfinite(effort) || std::abs(effort)>limit_ || !std::isfinite(limit_) || limit_<=0. || limit_>10. ||
          !std::isfinite(source_s) || source_s<=0 || source_s>now*1e-9 ||
          now*1e-9-source_s>=.060) return fail();
      generation_=generation; accepted_=now; effort_=effort; started_=true;
    } else if (started_ && (!std::isfinite(effort) || effort!=effort_)) {
      // The controller may zero its buffer on fault without a new generation.
      // That is a cancellation, never a lease renewal or a fabricated command.
      return fail();
    }
    return started_;
  }
  bool faulted() const noexcept { return fault_; }
  double effort() const noexcept { return effort_; }
 private:
  bool fail() noexcept { fault_=true; return false; }
  double generation_{}, effort_{}, limit_{.2};
  std::int64_t accepted_{},last_{};
  bool started_{false},fault_{false};
};
}  // namespace mech::mech_bringup

namespace mech::mech_bringup {
// Fixed envelope for the explicitly selected supported bench session only.
class KinematicBenchEnvelope {
 public:
  // The supported ROS bench explicitly selects Direct. LegacyRamp remains
  // available for historical comparisons; freshness and fault gates are shared.
  enum class EffortPolicy { LegacyRamp, Direct };
  struct Limits {
    double torque_nm{10.}, current_a{6.}, travel_deg{90.};
    double startup_s{0.}; // optional target transition: travel only, fixed 90 deg
    std::int64_t duration_ns{30000000000LL}; // zero: explicit manual-stop mode
    bool valid() const noexcept {
      return std::isfinite(torque_nm) && torque_nm>0. && torque_nm<=10. &&
        std::isfinite(current_a) && current_a>0. && current_a<=6. &&
        std::isfinite(travel_deg) && travel_deg>0. && travel_deg<=90. &&
        std::isfinite(startup_s) && startup_s>=0. && startup_s<=61. &&
        duration_ns>=0 && duration_ns<=30000000000LL;
    }
  };
  void reset(double reference, EffortPolicy policy=EffortPolicy::LegacyRamp) noexcept {
    reset(reference,policy,Limits{});
  }
  void reset(double reference, EffortPolicy policy, Limits limits) noexcept {
    reference_=reference; start_=0; effort_=0; stopped_=false; policy_=policy; limits_=limits;
  }
  bool update(bool leased, bool cancelled, double desired, double position_deg,
              double erpm, double iq, double temperature, std::int64_t now,
              double period_s) noexcept {
    if(stopped_) return false;
    if(!start_ && !leased && !cancelled) return false;
    const double travel_limit=limits_.startup_s>0. &&
      (!start_ || (now>=start_ && (now-start_)*1e-9<limits_.startup_s)) ? 90. : limits_.travel_deg;
    const bool bad=cancelled || !limits_.valid() || !std::isfinite(position_deg) ||
      std::abs(position_deg-reference_)>travel_limit || !std::isfinite(erpm) ||
      !(std::abs(iq)<=limits_.current_a) || !(temperature>0. && temperature<60.) ||
      !std::isfinite(desired) || std::abs(desired)>limits_.torque_nm ||
      !std::isfinite(period_s) || period_s<=0. ||
      (start_ && (now<start_ || (limits_.duration_ns>0 && now-start_>=limits_.duration_ns)));
    if(bad || (start_ && !leased)) {
      stopped_=true; effort_=0.; return start_!=0; // one nominal-zero attempt, then no renewal
    }
    if(!leased) return false;
    if(!start_) start_=now;
    if(policy_==EffortPolicy::Direct) { effort_=desired; return true; }
    const double step=.5*std::min(period_s,.005);
    effort_+=std::clamp(desired-effort_,-step,step);
    return true;
  }
  double effort() const noexcept { return effort_; }
  bool stopped() const noexcept { return stopped_; }
 private:
  Limits limits_{};
  double reference_{},effort_{};
  std::int64_t start_{};
  bool stopped_{};
  EffortPolicy policy_{EffortPolicy::LegacyRamp};
};
}
