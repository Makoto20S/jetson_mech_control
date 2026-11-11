#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace mech::mech_bringup {

// Times use the controller host's steady clock. Array order is drive 104,105.
struct MitPairTarget {
  std::int64_t issued_ns{};
  std::array<double, 2> effort{};
};

struct MitPairAxis {
  double id{}, received_us{}, valid{}, fault{}, erpm{}, iq{}, temperature{};
};

struct MitPairLimits {
  double effort_nm{}, slew_nm_s{}, max_erpm{}, max_iq_a{}, max_temp_c{};
  std::int64_t target_ttl_ns{}, feedback_ttl_ns{}, max_skew_ns{};
  bool valid() const {
    for (double v : {effort_nm, slew_nm_s, max_erpm, max_iq_a, max_temp_c}) {
      if (!std::isfinite(v) || v <= 0.) return false;
    }
    return effort_nm <= 15. && target_ttl_ns > 0 && feedback_ttl_ns > 0 &&
           max_skew_ns >= 0 && max_skew_ns <= feedback_ttl_ns;
  }
};

enum class PairMode { Waiting, Following, TargetExpired, FeedbackInvalid, PairSkew, ClockInvalid, OperatorStop };

inline const char * pair_mode_name(PairMode mode) {
  switch (mode) {
    case PairMode::Waiting: return "waiting";
    case PairMode::Following: return "following";
    case PairMode::TargetExpired: return "target_expired";
    case PairMode::FeedbackInvalid: return "feedback_invalid";
    case PairMode::PairSkew: return "feedback_skew";
    case PairMode::ClockInvalid: return "invalid_clock";
    case PairMode::OperatorStop: return "operator_stop";
  }
  return "unknown";
}

struct MitPairWrite {
  std::array<double, 2> effort{};
  bool refresh{};
};

class MitPairGuard {
 public:
  void activate(MitPairLimits limits, std::int64_t now) {
    limits_ = limits;
    activated_ = last_tick_ = now;
    target_ = {};
    effort_ = {};
    mode_ = PairMode::Waiting;
  }

  PairMode mode() const { return mode_; }

  MitPairWrite request_stop() {
    if (mode_ == PairMode::Waiting || mode_ == PairMode::Following) {
      return stop(PairMode::OperatorStop);
    }
    return {};
  }

  MitPairWrite update(const MitPairTarget & incoming, const std::array<MitPairAxis, 2> & axes,
                      std::int64_t now, double period_s) {
    if (mode_ != PairMode::Waiting && mode_ != PairMode::Following) return {};
    if (now < last_tick_ || !std::isfinite(period_s) || period_s <= 0.) {
      return stop(PairMode::ClockInvalid);
    }
    last_tick_ = now;
    if (mode_ == PairMode::Following && now - target_.issued_ns >= limits_.target_ttl_ns) {
      return stop(PairMode::TargetExpired);
    }
    const bool fresh = incoming.issued_ns > activated_ && incoming.issued_ns > target_.issued_ns &&
        incoming.issued_ns <= now && now - incoming.issued_ns < limits_.target_ttl_ns &&
        std::isfinite(incoming.effort[0]) && std::isfinite(incoming.effort[1]);
    if (mode_ == PairMode::Waiting && !fresh) return {};
    const double now_us = static_cast<double>(now / 1000);
    for (unsigned i = 0; i < axes.size(); ++i) {
      const auto & a = axes[i];
      const bool ok = a.id == 104 + i && a.valid == 1. && a.fault == 0. &&
          std::isfinite(a.received_us) && a.received_us > 0 && a.received_us <= now_us &&
          now_us - a.received_us <= limits_.feedback_ttl_ns / 1000. &&
          std::abs(a.erpm) <= limits_.max_erpm && std::abs(a.iq) <= limits_.max_iq_a &&
          std::isfinite(a.temperature) && a.temperature <= limits_.max_temp_c;
      if (!ok) return stop(PairMode::FeedbackInvalid);
    }
    if (std::abs(axes[0].received_us - axes[1].received_us) > limits_.max_skew_ns / 1000.) {
      return stop(PairMode::PairSkew);
    }
    if (fresh) target_ = incoming;
    mode_ = PairMode::Following;
    // A late 500 Hz tick must not produce a larger torque step.
    const double step = limits_.slew_nm_s * std::min(period_s, .002);
    for (unsigned i = 0; i < effort_.size(); ++i) {
      const double goal = std::clamp(target_.effort[i], -limits_.effort_nm, limits_.effort_nm);
      effort_[i] += std::clamp(goal - effort_[i], -step, step);
    }
    return {effort_, true};
  }

 private:
  MitPairWrite stop(PairMode cause) {
    const bool previously_commanding = mode_ == PairMode::Following;
    mode_ = cause;
    effort_ = {};
    // Try nominal zero once for both axes, then cease lease renewal.
    return {effort_, previously_commanding};
  }
  MitPairLimits limits_{};
  MitPairTarget target_{};
  std::array<double, 2> effort_{};
  std::int64_t activated_{}, last_tick_{};
  PairMode mode_{PairMode::Waiting};
};

}  // namespace mech::mech_bringup
