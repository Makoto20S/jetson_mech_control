#include "mech_controllers/demo_controller.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace mech::mech_controllers {
namespace {

TEST(TargetLimiter, ClampsSlewAndExpiresToNeutral) {
  TargetLimiter limiter;
  ASSERT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 2.0, 100}));
  ASSERT_TRUE(limiter.submit(5.0, 0));
  EXPECT_DOUBLE_EQ(limiter.update(0.0, 0.25, 50), 0.5);
  EXPECT_DOUBLE_EQ(limiter.update(0.5, 0.25, 100), 0.0);
  EXPECT_FALSE(limiter.submit(std::numeric_limits<double>::quiet_NaN(), 0));
}

TEST(TargetLimiter, RejectsInvalidLimitsAndOverflow) {
  TargetLimiter limiter;
  EXPECT_FALSE(limiter.configure(BoundedTarget{1.0, -1.0, 1.0, 10}));
  EXPECT_FALSE(limiter.configure(BoundedTarget{-1.0, 1.0, 0.0, 10}));
  EXPECT_TRUE(limiter.configure(BoundedTarget{-1.0, 1.0, 1.0, 10}));
  EXPECT_FALSE(limiter.submit(0.0, std::numeric_limits<std::int64_t>::max()));
}

}  // namespace
}  // namespace mech::mech_controllers
