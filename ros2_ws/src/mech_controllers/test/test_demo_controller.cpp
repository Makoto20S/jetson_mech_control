#include "mech_controllers/demo_controller.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace mech::mech_controllers {
namespace {

TEST(TargetLimiter, ClampsSlewWhileFollowing) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  EXPECT_EQ(limiter.stage(50), WatchdogStage::Following);
  EXPECT_DOUBLE_EQ(limiter.update(0.0, 0.25, 50), 0.5);
  EXPECT_FALSE(limiter.submit(std::numeric_limits<double>::quiet_NaN(), 0));
}

TEST(TargetLimiter, RejectsInvalidLimitsAndOverflow) {
  TargetLimiter limiter;
  EXPECT_FALSE(limiter.configure(BoundedTarget{1.0, -1.0, 1.0, 10, 20}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 0.0, 10, 20}));
  EXPECT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 10, 20}));
  EXPECT_FALSE(limiter.submit(0.0, std::numeric_limits<std::int64_t>::max()));
}

TEST(TargetLimiter, RejectsHardTtlNotGreaterThanTtl) {
  TargetLimiter limiter;
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 100}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 50}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 0}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, -1}));
  EXPECT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 100, 101}));
}

TEST(TargetLimiter, HoldsLastValidCommandBetweenTtlAndHardTtl) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  // Follow up to a non-zero, non-target value so "hold" and "go to zero" (the
  // old behaviour) and "keep following the target" are all distinguishable.
  const auto followed = limiter.update(0.2, 0.25, 50);
  EXPECT_DOUBLE_EQ(followed, 0.7);
  ASSERT_EQ(limiter.stage(100), WatchdogStage::Holding);
  const auto held = limiter.update(followed, 0.25, 100);
  EXPECT_DOUBLE_EQ(held, followed);
  EXPECT_NE(held, 0.0);
  // Holding must stay frozen across further cycles, not creep toward 0 or
  // resume slewing toward the stale target.
  const auto held_again = limiter.update(held, 0.25, 150);
  EXPECT_DOUBLE_EQ(held_again, followed);
}

TEST(TargetLimiter, ReportsExpiredStageAtHardTtl) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  EXPECT_EQ(limiter.stage(299), WatchdogStage::Holding);
  EXPECT_EQ(limiter.stage(300), WatchdogStage::Expired);
}

TEST(TargetLimiter, NonFiniteInputHoldsLastValidValueNotZero) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100, 300}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  const auto followed = limiter.update(0.2, 0.25, 50);
  EXPECT_DOUBLE_EQ(followed, 0.7);
  const auto held = limiter.update(std::numeric_limits<double>::quiet_NaN(), 0.25, 60);
  EXPECT_DOUBLE_EQ(held, followed);
  EXPECT_NE(held, 0.0);
  const auto held_negative_period = limiter.update(followed, -1.0, 70);
  EXPECT_DOUBLE_EQ(held_negative_period, followed);
}

TEST(DemoControllerConfigure, RejectsHardTtlNotGreaterThanTtl) {
  BoundedTarget limits{-1.0, 1.0, 1.0, 100, 100};
  TargetLimiter limiter;
  EXPECT_FALSE(limiter.configure(limits));
}

}  // namespace
}  // namespace mech::mech_controllers
