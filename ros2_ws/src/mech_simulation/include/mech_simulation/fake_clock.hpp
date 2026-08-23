#pragma once

#include <cstdint>
#include <limits>

#include "mech_control_core/time.hpp"

namespace mech::mech_simulation {

class FakeClock final {
 public:
  explicit FakeClock(mech_control_core::MonotonicTime initial)
      : now_(initial) {}

  [[nodiscard]] mech_control_core::MonotonicTime now() const noexcept {
    return now_;
  }

  [[nodiscard]] bool advance(
      mech_control_core::MonotonicDuration duration) noexcept {
    const auto current = now_.nanoseconds();
    const auto delta = duration.nanoseconds();
    if (delta > 0 &&
        current > std::numeric_limits<std::int64_t>::max() - delta) {
      return false;
    }
    now_ = *mech_control_core::MonotonicTime::from_nanoseconds(current + delta);
    return true;
  }

  [[nodiscard]] bool set(mech_control_core::MonotonicTime value) noexcept {
    if (value < now_) {
      return false;
    }
    now_ = value;
    return true;
  }

 private:
  mech_control_core::MonotonicTime now_;
};

}  // namespace mech::mech_simulation
