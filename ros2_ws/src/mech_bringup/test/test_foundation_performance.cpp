#include "mech_bringup/foundation_harness.hpp"
#include <gtest/gtest.h>

namespace mech::mech_bringup {
TEST(FoundationPerformance, MaximumHostCycleUnderTwoMilliseconds) {
  FoundationHarness harness;
  ASSERT_TRUE(harness.configure(1U));
  ASSERT_TRUE(harness.activate());
  ASSERT_TRUE(harness.switch_claim(true));
  constexpr std::int64_t period = 2000000;
  for (std::int64_t cycle = 1; cycle <= 1000; ++cycle) {
    ASSERT_TRUE(harness.set_target(1.0, cycle * period));
    ASSERT_TRUE(harness.cycle(cycle * period, period));
  }
  RecordProperty("maximum_cycle_nanoseconds",
                 std::to_string(harness.metrics().maximum_cycle_nanoseconds));
  EXPECT_LT(harness.metrics().maximum_cycle_nanoseconds, period);
}
}  // namespace mech::mech_bringup
