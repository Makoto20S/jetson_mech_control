#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace mech::mech_simulation {

// Receiver-side contract for this software-only plant. It deliberately does
// not import controller code: a controller failure must not extend this lease.
class Ref01SimReceiver {
 public:
  static constexpr std::int64_t kTtlNs = 60000000;
  static constexpr double kEffortLimit = 0.2;
  static constexpr double kMaxExactGeneration = 9007199254740991.;
  enum class Fault { None, Clock, Generation, Effort, Expired, Runtime };

  void reset() noexcept { *this = Ref01SimReceiver{}; }

  // [effort104, generation104, effort105, generation105]. A repeated generation
  // is never applied again, even if its payload changes, and never renews TTL.
  bool accept(const std::array<double, 4>& command, std::int64_t now_ns) noexcept {
    if (latched()) return false;
    if (now_ns <= 0 || (last_clock_ns_ != 0 && now_ns < last_clock_ns_)) {
      fail(Fault::Clock, now_ns);
      return false;
    }
    last_clock_ns_ = now_ns;
    if (started() && now_ns - last_accept_ns_ >= kTtlNs) {
      fail(Fault::Expired, now_ns);
      return false;
    }
    const double first = command[1], second = command[3];
    if (!valid_generation(first) || !valid_generation(second) || first != second ||
        first < static_cast<double>(last_generation_)) {
      fail(Fault::Generation, now_ns);
      return false;
    }
    if (first == static_cast<double>(last_generation_)) return false;
    if (!std::isfinite(command[0]) || !std::isfinite(command[2]) ||
        std::abs(command[0]) > kEffortLimit + 1e-12 ||
        std::abs(command[2]) > kEffortLimit + 1e-12) {
      fail(Fault::Effort, now_ns);
      return false;
    }
    last_generation_ = static_cast<std::uint64_t>(first);
    last_accept_ns_ = now_ns;
    effort_ = {command[0], command[2]};
    return true;
  }

  void fail(Fault reason, std::int64_t now_ns) noexcept {
    if (!latched()) { fault_ = reason; zero_ns_ = now_ns; }
    effort_ = {0., 0.};
  }
  bool started() const noexcept { return last_generation_ != 0; }
  bool latched() const noexcept { return fault_ != Fault::None; }
  Fault fault() const noexcept { return fault_; }
  std::int64_t zero_ns() const noexcept { return zero_ns_; }
  std::int64_t last_accept_ns() const noexcept { return last_accept_ns_; }
  std::uint64_t generation() const noexcept { return last_generation_; }
  const std::array<double, 2>& effort() const noexcept { return effort_; }
  static const char* fault_name(Fault value) noexcept {
    switch (value) {
      case Fault::None: return "none";
      case Fault::Clock: return "clock";
      case Fault::Generation: return "generation";
      case Fault::Effort: return "effort";
      case Fault::Expired: return "expired";
      case Fault::Runtime: return "runtime";
    }
    return "unknown";
  }

 private:
  static bool valid_generation(double value) noexcept {
    return std::isfinite(value) && value >= 0. &&
           value <= kMaxExactGeneration && std::floor(value) == value;
  }
  Fault fault_{Fault::None};
  std::uint64_t last_generation_{};
  std::int64_t last_accept_ns_{}, last_clock_ns_{}, zero_ns_{};
  std::array<double, 2> effort_{};
};

}  // namespace mech::mech_simulation
