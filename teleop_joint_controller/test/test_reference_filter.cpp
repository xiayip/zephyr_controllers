#include <gtest/gtest.h>
#include "teleop_joint_controller/reference_filter.hpp"
using teleop_joint_controller::ReferenceFilter;
TEST(ReferenceFilter, RejectsInvalidLimits) {
  ReferenceFilter f;
  EXPECT_THROW(f.configure({}, {}), std::invalid_argument);
  EXPECT_THROW(f.configure({1}, {0}), std::invalid_argument);
  EXPECT_THROW(f.configure({NAN}, {1}), std::invalid_argument);
  EXPECT_THROW(f.configure({1}, {1}, -.001), std::invalid_argument);
}
TEST(ReferenceFilter, BoundsReversalsWithJitterAndConverges) {
  ReferenceFilter f; f.configure({5}, {40}); f.reset({0});
  for (int k = 0; k < 1500; ++k) {
    double dt = (k % 9 == 0) ? .012 : .005;
    double target = k < 300 ? .3 : k < 600 ? -.2 : .1;
    double q = f.position[0], v = f.velocity[0]; f.step({target}, dt);
    EXPECT_LE(std::abs(f.velocity[0]), 5 + 1e-10);
    EXPECT_LE(std::abs(f.velocity[0] - v), 40 * dt + 1e-10);
    EXPECT_NEAR((f.position[0] - q) / dt, f.velocity[0], 1e-10);
  }
  EXPECT_NEAR(f.position[0], .1, 1e-7);
  EXPECT_NEAR(f.velocity[0], 0, 1e-7);
}
TEST(ReferenceFilter, RecordedCompressedSegmentDoesNotCreateSplineSpike) {
  ReferenceFilter f; f.configure({5}, {40}); f.reset({-.739901735599629});
  double peak = 0;
  for (int k = 0; k < 100; ++k) {
    f.step({-.7364739985603671}, .005);
    peak = std::max(peak, std::abs(f.velocity[0]));
  }
  // The recorded 0.640961 ms source interval is deliberately irrelevant.
  EXPECT_LT(peak, .5);
  EXPECT_NEAR(f.position[0], -.7364739985603671, 1e-8);
}
TEST(ReferenceFilter, SmoothsAsynchronousSampleHoldWithoutLargeFollowingDelay) {
  ReferenceFilter immediate, smooth;
  immediate.configure({5}, {40}, 0); smooth.configure({5}, {40}, .01);
  immediate.reset({0}); smooth.reset({0});
  double target = 0, raw_ripple = 0, smooth_ripple = 0;
  for (int k = 0; k < 500; ++k) {
    // A 100 Hz stream sampled by a 200 Hz controller: alternating hold/advance.
    if (k % 2 == 0) target = k * .005 * .1;
    const double before_raw = immediate.velocity[0], before_smooth = smooth.velocity[0];
    immediate.step({target}, .005); smooth.step({target}, .005);
    if (k > 100) {
      raw_ripple += std::pow(immediate.velocity[0] - before_raw, 2);
      smooth_ripple += std::pow(smooth.velocity[0] - before_smooth, 2);
      EXPECT_LT(std::abs(smooth.position[0] - target), .003);
    }
  }
  EXPECT_LT(smooth_ripple, raw_ripple * .4);
}
