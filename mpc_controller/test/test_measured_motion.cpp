// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <mpc_controller/control/MeasuredMotion.h>
#include <limits>

using namespace zephyr::mpc;
namespace {
const std::array<double, 7> hold{0, 0, 0, 0, 0, 0, 1};
}
TEST(MeasuredMotion, RequiresAFullFreshWindowAndResetDiscardsOldQuietState) {
  MeasuredMotion m; m.configure(6, false);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  m.update(1, q, hold); EXPECT_FALSE(m.speeds().valid);
  m.update(1.03, q, hold); EXPECT_FALSE(m.speeds().valid);
  m.update(1.06, q, hold); ASSERT_TRUE(m.speeds().valid);
  for (double v : m.speeds().values) EXPECT_DOUBLE_EQ(v, 0);
  m.reset(); EXPECT_FALSE(m.speeds().valid);
  m.update(2, q, hold); EXPECT_FALSE(m.speeds().valid);
}
TEST(MeasuredMotion, CapturesTcpAndJointMotionWithoutCommandInputs) {
  MeasuredMotion m; m.configure(6, false);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  m.update(1, q, hold);
  auto p = hold; p[0] = 0.006; p[5] = std::sin(0.03); p[6] = std::cos(0.03);
  q[2] = 0.012;
  m.update(1.06, q, p); ASSERT_TRUE(m.speeds().valid);
  EXPECT_NEAR(m.speeds().values[MotionSpeeds::TcpLinear], 0.1, 1e-10);
  EXPECT_NEAR(m.speeds().values[MotionSpeeds::TcpAngular], 1, 1e-10);
  EXPECT_NEAR(m.speeds().values[MotionSpeeds::ArmJoint], 0.2, 1e-10);
}
TEST(MeasuredMotion, OppositeMotionWithinWindowDoesNotCancelAndDetectsNullspaceMotion) {
  MeasuredMotion m; m.configure(6, false);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  m.update(1, q, hold);
  q[0] = 0.03; m.update(1.03, q, hold);
  q[0] = 0; m.update(1.06, q, hold);
  ASSERT_TRUE(m.speeds().valid);
  EXPECT_DOUBLE_EQ(m.speeds().values[MotionSpeeds::TcpLinear], 0);
  EXPECT_NEAR(m.speeds().values[MotionSpeeds::ArmJoint], 1, 1e-10);
}
TEST(MeasuredMotion, MobileTranslationAndYawWrapAreIndependentOfQuietTcp) {
  MeasuredMotion m; m.configure(6, true);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(9); q[2] = M_PI - 0.01;
  m.update(1, q, hold);
  q[0] = 0.0036; q[1] = 0.0048; q[2] = -M_PI + 0.01;
  m.update(1.06, q, hold); ASSERT_TRUE(m.speeds().valid);
  EXPECT_NEAR(m.speeds().values[MotionSpeeds::BaseLinear], 0.1, 1e-10);
  EXPECT_NEAR(m.speeds().values[MotionSpeeds::BaseAngular], 1.0 / 3, 1e-10);
  EXPECT_DOUBLE_EQ(m.speeds().values[MotionSpeeds::ArmJoint], 0);
}
TEST(MeasuredMotion, InvalidInputsRepeatedClockAndFeedbackGapInvalidateSpeed) {
  MeasuredMotion m; m.configure(6, false);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  m.update(1, q, hold); m.update(1.06, q, hold); ASSERT_TRUE(m.speeds().valid);
  m.update(1.06, q, hold); EXPECT_FALSE(m.speeds().valid);
  m.update(1.12, q, hold); ASSERT_TRUE(m.speeds().valid);
  m.update(1.30, q, hold); EXPECT_FALSE(m.speeds().valid);
  m.update(1.36, q, hold); ASSERT_TRUE(m.speeds().valid);
  auto p = hold; p[0] = std::numeric_limits<double>::quiet_NaN();
  m.update(1.37, q, p); EXPECT_FALSE(m.speeds().valid);
  m.update(1.38, q, hold); EXPECT_FALSE(m.speeds().valid);
  m.update(1.44, q, hold); ASSERT_TRUE(m.speeds().valid);
  m.update(1.45, Eigen::VectorXd::Zero(3), hold); EXPECT_FALSE(m.speeds().valid);
}
TEST(MeasuredMotion, QuaternionSignDoesNotInventAngularSpeed) {
  MeasuredMotion m; m.configure(6, false);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  m.update(1, q, hold);
  auto p = hold; p[6] = -1;
  m.update(1.06, q, p); ASSERT_TRUE(m.speeds().valid);
  EXPECT_DOUBLE_EQ(m.speeds().values[MotionSpeeds::TcpAngular], 0);
}
