#include <limits>
#include <string>
#include <vector>

#include "mit_impedance_controller/blocked_motion_detector.hpp"
#include "gtest/gtest.h"

namespace {

TEST(BlockedMotionDetectorTest, RequiresSustainedStationaryError) {
  mit_impedance_controller::BlockedMotionDetector detector;
  std::string error;
  ASSERT_TRUE(detector.configure({0.1}, {0.02}, 0.3, error)) << error;

  EXPECT_FALSE(detector.update({0.12}, {0.0}, 0.1));
  EXPECT_FALSE(detector.update({0.12}, {0.0}, 0.1));
  EXPECT_TRUE(detector.update({0.12}, {0.0}, 0.1));
  EXPECT_EQ(detector.last_triggered_joint(), 0U);
  EXPECT_FALSE(detector.update({0.12}, {0.0}, 0.1));
}

TEST(BlockedMotionDetectorTest, MovingOrRecoveredJointResetsTimer) {
  mit_impedance_controller::BlockedMotionDetector detector;
  std::string error;
  ASSERT_TRUE(detector.configure({0.1}, {0.02}, 0.3, error)) << error;

  EXPECT_FALSE(detector.update({0.12}, {0.0}, 0.2));
  EXPECT_FALSE(detector.update({0.12}, {0.03}, 0.1));
  EXPECT_FALSE(detector.update({0.12}, {0.0}, 0.2));
  EXPECT_FALSE(detector.update({0.05}, {0.0}, 0.1));
  EXPECT_FALSE(detector.update({0.12}, {0.0}, 0.2));
  EXPECT_TRUE(detector.update({0.12}, {0.0}, 0.1));
}

TEST(BlockedMotionDetectorTest, RejectsInvalidConfiguration) {
  mit_impedance_controller::BlockedMotionDetector detector;
  std::string error;
  EXPECT_FALSE(detector.configure({}, {}, 0.3, error));
  EXPECT_FALSE(detector.configure({0.1}, {0.02, 0.02}, 0.3, error));
  EXPECT_FALSE(detector.configure({0.1}, {0.02}, 0.0, error));
  EXPECT_FALSE(detector.configure({std::numeric_limits<double>::infinity()},
                                  {0.02}, 0.3, error));
}

} // namespace
