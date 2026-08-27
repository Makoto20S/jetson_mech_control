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

TEST(FoundationIntegration, RunsFiveHundredHertzWhileTargetKeepsBeingRefreshed) {
  // A live upstream refreshes the target every cycle, so the watchdog never
  // leaves the Following stage and the loop runs indefinitely. This is the
  // 500 Hz endurance case; the watchdog itself is covered separately below.
  FoundationHarness harness;
  ASSERT_TRUE(harness.configure(1U));
  ASSERT_TRUE(harness.activate());
  ASSERT_TRUE(harness.switch_claim(true));
  constexpr std::int64_t period = 2000000;
  for (std::int64_t cycle = 1; cycle <= 1000; ++cycle) {
    const auto now = cycle * period;
    ASSERT_TRUE(harness.set_target(1.0, now));
    ASSERT_TRUE(harness.cycle(now, period));
  }
  // Slew is 2.0/s, so 0.004 per 2 ms cycle; the target (clamped to 1.0) is
  // reached after 250 cycles and held there for the rest of the run.
  EXPECT_NEAR(harness.command(), 1.0, 1e-9);
  EXPECT_NEAR(harness.position(), 1.0, 1e-9);
  EXPECT_EQ(harness.metrics().cycles, 1000U);
  EXPECT_EQ(harness.metrics().failures, 0U);
  EXPECT_LT(harness.metrics().maximum_cycle_nanoseconds, period);
}

TEST(FoundationIntegration, StaleCommandHoldsThenFaultsWithinThreeCycles) {
  // docs/planning/03_mvp_delivery_plan.md:215 requires a command to lapse
  // within <=3 control cycles (<=6 ms at 500 Hz) once the upstream stops
  // refreshing it. With ttl=4 ms and hard_ttl=6 ms at a 2 ms period:
  //   cycle 1 (t=2 ms) Following -> command advances one slew step
  //   cycle 2 (t=4 ms) Holding   -> command frozen at the last valid value,
  //                                 and specifically NOT driven toward 0.0,
  //                                 which on a position interface would be a
  //                                 commanded move to the zero position
  //   cycle 3 (t=6 ms) Expired   -> the cycle fails, entering a defined fault
  FoundationHarness harness;
  ASSERT_TRUE(harness.configure(1U));
  ASSERT_TRUE(harness.activate());
  ASSERT_TRUE(harness.switch_claim(true));
  ASSERT_TRUE(harness.set_target(1.0, 0));
  constexpr std::int64_t period = 2000000;

  ASSERT_TRUE(harness.cycle(1 * period, period));
  EXPECT_NEAR(harness.command(), 0.004, 1e-12);

  ASSERT_TRUE(harness.cycle(2 * period, period));
  EXPECT_NEAR(harness.command(), 0.004, 1e-12) << "must hold, not move";

  EXPECT_FALSE(harness.cycle(3 * period, period));
  EXPECT_EQ(harness.metrics().failures, 1U);
  EXPECT_NEAR(harness.command(), 0.004, 1e-12) << "must not jump to zero";
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
