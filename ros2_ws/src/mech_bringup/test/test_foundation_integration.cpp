#include "mech_bringup/foundation_harness.hpp"

#include <gtest/gtest.h>

namespace mech::mech_bringup {
namespace {

TEST(FoundationIntegration, RepeatsLifecycleAndStrictSwitchOneHundredTimes) {
  for (int iteration = 0; iteration < 100; ++iteration) {
    FoundationHarness harness;
    ASSERT_TRUE(harness.configure(1U));
    ASSERT_TRUE(harness.activate());
    ASSERT_TRUE(harness.switch_claim(true));
    ASSERT_TRUE(harness.set_target(0.5, 0));
    ASSERT_TRUE(harness.cycle(2000000, 2000000));
    EXPECT_FALSE(harness.switch_claim(true));
    ASSERT_TRUE(harness.switch_claim(false));
    ASSERT_TRUE(harness.deactivate());
    ASSERT_TRUE(harness.cleanup());
  }
}

TEST(FoundationIntegration, RunsFiveHundredHertzAndExpiresCommandToNeutral) {
  FoundationHarness harness;
  ASSERT_TRUE(harness.configure(1U));
  ASSERT_TRUE(harness.activate());
  ASSERT_TRUE(harness.switch_claim(true));
  ASSERT_TRUE(harness.set_target(1.0, 0));
  constexpr std::int64_t period = 2000000;
  for (std::int64_t cycle = 1; cycle <= 1000; ++cycle) {
    ASSERT_TRUE(harness.cycle(cycle * period, period));
  }
  EXPECT_NEAR(harness.command(), 0.0, 1e-12);
  EXPECT_EQ(harness.metrics().cycles, 1000U);
  EXPECT_EQ(harness.metrics().failures, 0U);
  EXPECT_LT(harness.metrics().maximum_cycle_nanoseconds, period);
}

TEST(FoundationIntegration, RejectsInvalidLifecycleOrder) {
  FoundationHarness harness;
  EXPECT_FALSE(harness.activate());
  ASSERT_TRUE(harness.configure(1U));
  EXPECT_FALSE(harness.cycle(1, 1));
  EXPECT_TRUE(harness.cleanup());
  ASSERT_TRUE(harness.configure(1U));
  ASSERT_TRUE(harness.activate());
  EXPECT_TRUE(harness.deactivate());
  EXPECT_FALSE(harness.deactivate());
}

}  // namespace
}  // namespace mech::mech_bringup
