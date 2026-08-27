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

TEST(FoundationIntegration, RunsFiveHundredHertzAndHoldsCommandPastHardTtl) {
  FoundationHarness harness;
  ASSERT_TRUE(harness.configure(1U));
  ASSERT_TRUE(harness.activate());
  ASSERT_TRUE(harness.switch_claim(true));
  ASSERT_TRUE(harness.set_target(1.0, 0));
  constexpr std::int64_t period = 2000000;
  for (std::int64_t cycle = 1; cycle <= 1000; ++cycle) {
    ASSERT_TRUE(harness.cycle(cycle * period, period));
  }
  // The harness is configured with ttl=100ms / hard_ttl=200ms (see
  // FoundationHarness::configure). At 2ms/cycle the target (clamped to 1.0)
  // is followed at the 2.0/s slew limit for cycles 1..49 (t < 100ms), then
  // held from cycle 50 onward once the watchdog enters the Holding/Expired
  // stages -- it must freeze at the last followed value, NOT jump to 0.0
  // (that was the old, unsafe "expire to neutral" behaviour this test used
  // to encode). Last followed value: 49 cycles * (2.0 * 0.002s) = 0.196.
  constexpr double expected_hold = 0.196;
  EXPECT_NEAR(harness.command(), expected_hold, 1e-9);
  // With the corrected read -> update -> write ordering, the state read
  // immediately preceding a write always has one cycle to catch up to the
  // previously written command. Because the slew-limited command growth
  // (0.004/cycle) is well under the loopback runtime's per-cycle catch-up
  // cap (0.01/cycle), position() fully tracks command() by the next read,
  // so both converge to the same held value.
  EXPECT_NEAR(harness.position(), expected_hold, 1e-9);
  EXPECT_EQ(harness.metrics().cycles, 1000U);
  EXPECT_EQ(harness.metrics().failures, 0U);
  EXPECT_LT(harness.metrics().maximum_cycle_nanoseconds, period);
}

TEST(FoundationIntegration, FirstCycleReadsStaleStateBeforeCommandTakesEffect) {
  // ros2_control's real loop is read -> update -> write, so on the very
  // first cycle position() must still reflect the state from BEFORE this
  // cycle's command was computed and written (nothing has been written to
  // the runtime yet). command() reflects the new, slew-limited command.
  // With the old write -> read ordering this test would have observed
  // position() already reacting to the just-written command on cycle one.
  FoundationHarness harness;
  ASSERT_TRUE(harness.configure(1U));
  ASSERT_TRUE(harness.activate());
  ASSERT_TRUE(harness.switch_claim(true));
  ASSERT_TRUE(harness.set_target(0.5, 0));
  constexpr std::int64_t period = 2000000;
  ASSERT_TRUE(harness.cycle(period, period));
  EXPECT_NEAR(harness.position(), 0.0, 1e-12);
  EXPECT_NEAR(harness.command(), 0.004, 1e-12);
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
