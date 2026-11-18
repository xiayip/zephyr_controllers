// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <mpc_controller/control/ContinuousTcpPath.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace zephyr::mpc;
namespace {
Eigen::VectorXd pose(double x, double y = 0, double z = 0, double yaw = 0) {
  Eigen::VectorXd p(7);
  p << x, y, z, 0, 0, std::sin(yaw / 2), std::cos(yaw / 2);
  return p;
}
size_t nearest(const SampledTcpPath& path, const Eigen::VectorXd& p) {
  size_t best = 0;
  for (size_t i = 1; i < path.poses.size(); ++i)
    if ((path.poses[i] - p).norm() < (path.poses[best] - p).norm()) best = i;
  return best;
}
std::vector<Eigen::VectorXd> line() {
  std::vector<Eigen::VectorXd> p;
  for (int i = 0; i <= 8; ++i) p.push_back(pose(i * 0.01));
  return p;
}
}  // namespace

TEST(ContinuousTcpPath, DenseStraightFramesDoNotStopAndDoNotInflateDuration) {
  const auto points = line();
  TcpPathLimits limits;
  limits.sample_interval = 0.001;
  const auto path = makeContinuousTcpPath(points, limits);
  const auto sparse = makeContinuousTcpPath({points.front(), points.back()}, limits);
  EXPECT_NEAR(path.times.back(), sparse.times.back(), 1e-8);
  EXPECT_NEAR((path.poses.front() - points.front()).norm(), 0, 1e-12);
  EXPECT_NEAR((path.poses.back() - points.back()).norm(), 0, 1e-12);
  for (size_t j = 1; j + 1 < points.size(); ++j) {
    const auto i = nearest(path, points[j]);
    ASSERT_GT(i, 0u); ASSERT_LT(i + 1, path.times.size());
    EXPECT_LT((path.poses[i] - points[j]).norm(), 1e-12);
    const double before = (path.poses[i][0] - path.poses[i - 1][0]) / (path.times[i] - path.times[i - 1]);
    const double after = (path.poses[i + 1][0] - path.poses[i][0]) / (path.times[i + 1] - path.times[i]);
    EXPECT_GT(before, 0.01); EXPECT_GT(after, 0.01);
    EXPECT_NEAR(before, after, 0.001);
  }
  const double initial_speed = (path.poses[1] - path.poses[0]).norm() / path.times[1];
  EXPECT_LT(initial_speed, 1e-5);
  const size_t end = path.times.size() - 1;
  EXPECT_LT((path.poses[end] - path.poses[end - 1]).norm() / (path.times[end] - path.times[end - 1]), 1e-5);
}

TEST(ContinuousTcpPath, CurvedTranslationAndRotationRespectDerivativeBounds) {
  TcpPathLimits limits;
  limits.sample_interval = 0.001;
  std::vector<Eigen::VectorXd> points;
  for (int i = 0; i <= 12; ++i) {
    const double a = i * M_PI / 12;
    points.push_back(pose(0.02 * std::sin(a), 0.02 * std::cos(a), 0, a / 4));
  }
  const auto path = makeContinuousTcpPath(points, limits);
  const size_t steps = std::ceil(path.times.back() / limits.sample_interval);
  const double dt = path.times.back() / steps;
  std::vector<Eigen::Vector3d> positions;
  std::vector<double> angles;
  // Exclude extra exact waypoint samples to take differences at a uniform dt.
  for (size_t i = 0; i < path.times.size(); ++i) {
    if (std::abs(path.times[i] / dt - std::round(path.times[i] / dt)) > 1e-7) continue;
    positions.push_back(path.poses[i].head<3>());
    angles.push_back(2 * std::atan2(path.poses[i][5], path.poses[i][6]));
    EXPECT_NEAR(path.poses[i].tail<4>().norm(), 1, 1e-12);
  }
  ASSERT_EQ(positions.size(), steps + 1);
  for (size_t i = 3; i < positions.size(); ++i) {
    EXPECT_LE(((positions[i] - positions[i - 1]) / dt).norm(), limits.linear_velocity * 1.001);
    EXPECT_LE(((positions[i] - 2 * positions[i - 1] + positions[i - 2]) / (dt * dt)).norm(), limits.linear_acceleration * 1.001);
    EXPECT_LE(((positions[i] - 3 * positions[i - 1] + 3 * positions[i - 2] - positions[i - 3]) / std::pow(dt, 3)).norm(), limits.linear_jerk * 1.01);
    EXPECT_LE(std::abs(angles[i] - angles[i - 1]) / dt, limits.angular_velocity * 1.001);
    EXPECT_LE(std::abs(angles[i] - 2 * angles[i - 1] + angles[i - 2]) / (dt * dt), limits.angular_acceleration * 1.001);
    EXPECT_LE(std::abs(angles[i] - 3 * angles[i - 1] + 3 * angles[i - 2] - angles[i - 3]) / std::pow(dt, 3), limits.angular_jerk * 1.01);
  }
}

TEST(ContinuousTcpPath, SlowerLimitsAndMinimumDurationRetainGeometry) {
  TcpPathLimits limits;
  const auto fast = makeContinuousTcpPath(line(), limits);
  limits.linear_velocity *= 0.2; limits.linear_acceleration *= 0.2; limits.linear_jerk *= 0.2;
  const auto slow = makeContinuousTcpPath(line(), limits);
  EXPECT_GT(slow.times.back(), fast.times.back());
  EXPECT_DOUBLE_EQ(makeContinuousTcpPath(line(), limits, 20).times.back(), 20);
}

TEST(ContinuousTcpPath, DuplicateFramesAndAntipodalQuaternionsAreNotDwellsOrTurns) {
  auto a = pose(0), b = pose(0.02);
  auto opposite = b; opposite.tail<4>() *= -1;
  const auto path = makeContinuousTcpPath({a, a, b, opposite}, {});
  const auto simple = makeContinuousTcpPath({a, b}, {});
  EXPECT_DOUBLE_EQ(path.times.back(), simple.times.back());
  EXPECT_NEAR((path.poses.back() - b).norm(), 0, 1e-12);
  const auto hold = makeContinuousTcpPath({a, a}, {}, 2);
  EXPECT_EQ(hold.poses.size(), 2u); EXPECT_DOUBLE_EQ(hold.times.back(), 2);
}

TEST(ContinuousTcpPath, PureOrientationPassesThroughIntermediateAngles) {
  const auto path = makeContinuousTcpPath({pose(0), pose(0, 0, 0, 0.2), pose(0, 0, 0, 0.4)}, {});
  const auto i = nearest(path, pose(0, 0, 0, 0.2));
  ASSERT_GT(i, 0u); ASSERT_LT(i + 1, path.poses.size());
  EXPECT_LT((path.poses[i] - pose(0, 0, 0, 0.2)).norm(), 1e-12);
  EXPECT_GT(path.poses[i + 1][5], path.poses[i - 1][5]);
}

TEST(ContinuousTcpPath, OptionalDeviationBoundAndRotationChartRemainEnforced) {
  const std::vector<Eigen::VectorXd> corner{pose(0), pose(0.2), pose(0.2, 0.2)};
  EXPECT_NO_THROW(makeContinuousTcpPath(corner, {}));
  TcpPathLimits strict; strict.max_deviation = 0.005;
  EXPECT_THROW(makeContinuousTcpPath(corner, strict), std::invalid_argument);
  EXPECT_THROW(makeContinuousTcpPath({pose(0), pose(0, 0, 0, M_PI)}, {}), std::invalid_argument);
  EXPECT_THROW(makeContinuousTcpPath({pose(0), pose(0, 0, 0, 2.8), pose(0, 0, 0, -2.8)}, {}), std::invalid_argument);
}

TEST(ContinuousTcpPath, LongSparseReferencePassesWithoutRelaxingKinematicLimits) {
  const std::vector<Eigen::VectorXd> points{
      pose(0.70668238, -0.09309751, 0.69396237),
      pose(1.13137833, -0.09320061, 0.72395522),
      pose(1.95637833, -0.09320061, 0.75395522)};
  TcpPathLimits limits;
  limits.linear_velocity = 0.05;
  limits.linear_acceleration = 0.15;
  limits.linear_jerk = 0.5;
  const auto path = makeContinuousTcpPath(points, limits);
  EXPECT_GT(path.times.back(), 46.0); EXPECT_LT(path.times.back(), 48.0);
  EXPECT_NEAR((path.poses.back() - points.back()).norm(), 0, 1e-12);
  const auto via = nearest(path, points[1]);
  ASSERT_GT(via, 0u); ASSERT_LT(via + 1, path.poses.size());
  EXPECT_LT((path.poses[via] - points[1]).norm(), 1e-12);
  EXPECT_GT((path.poses[via + 1].head<3>() - path.poses[via - 1].head<3>()).norm() /
      (path.times[via + 1] - path.times[via - 1]), 0.01);
  for (size_t i = 1; i < path.poses.size(); ++i)
    EXPECT_LE((path.poses[i].head<3>() - path.poses[i - 1].head<3>()).norm() /
        (path.times[i] - path.times[i - 1]), limits.linear_velocity * 1.00001);
  limits.max_deviation = 0.005;
  EXPECT_THROW(makeContinuousTcpPath(points, limits), std::invalid_argument);
  limits.max_deviation = 0;
  EXPECT_NO_THROW(makeContinuousTcpPath({pose(0), pose(25, 0, 1), pose(50, 0, 1)}, limits));
}

TEST(ContinuousTcpPath, ValidatesInputAndAllocationBudget) {
  EXPECT_THROW(makeContinuousTcpPath({}, {}), std::invalid_argument);
  EXPECT_THROW(makeContinuousTcpPath({pose(0)}, {}), std::invalid_argument);
  auto invalid = pose(0); invalid[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(makeContinuousTcpPath({pose(0), invalid}, {}), std::invalid_argument);
  invalid = pose(0); invalid.tail<4>().setZero();
  EXPECT_THROW(makeContinuousTcpPath({pose(0), invalid}, {}), std::invalid_argument);
  TcpPathLimits limits; limits.linear_jerk = 0;
  EXPECT_THROW(makeContinuousTcpPath(line(), limits), std::invalid_argument);
  limits = {}; limits.max_deviation = -0.001;
  EXPECT_THROW(makeContinuousTcpPath(line(), limits), std::invalid_argument);
  limits.max_deviation = std::numeric_limits<double>::infinity();
  EXPECT_THROW(makeContinuousTcpPath(line(), limits), std::invalid_argument);
  limits.max_deviation = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(makeContinuousTcpPath(line(), limits), std::invalid_argument);
  EXPECT_THROW(makeContinuousTcpPath(line(), {}, 1e9), std::invalid_argument);
  EXPECT_THROW(makeContinuousTcpPath(std::vector<Eigen::VectorXd>(1002, pose(0)), {}), std::invalid_argument);
}
