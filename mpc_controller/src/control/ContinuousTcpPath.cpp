// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/ContinuousTcpPath.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace zephyr::mpc {
namespace {
using Pose6 = Eigen::Matrix<double, 6, 1>;
using Bezier = std::array<Pose6, 4>;
constexpr double chart_limit = 3.0;
constexpr size_t sample_budget = 200000;
double smooth(double t) { return t * t * t * (10 + t * (-15 + 6 * t)); }
double inverseSmooth(double s) {
  double lo = 0, hi = 1;
  for (int i = 0; i < 52; ++i) {
    const double mid = (lo + hi) * 0.5;
    if (smooth(mid) < s) lo = mid; else hi = mid;
  }
  return (lo + hi) * 0.5;
}
double segmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& a,
                       const Eigen::Vector3d& b) {
  const Eigen::Vector3d d = b - a;
  const double t = d.squaredNorm() > 1e-24 ?
      std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
  return (p - a - t * d).norm();
}
}  // namespace

SampledTcpPath makeContinuousTcpPath(const std::vector<Eigen::VectorXd>& poses,
                                    const TcpPathLimits& l, double minimum_duration,
                                    const std::vector<double>& progress_steps) {
  if (poses.size() < 2 || poses.size() > 1001)
    throw std::invalid_argument("Continuous path needs start + 1..1000 poses");
  if (!progress_steps.empty()) {
    if (progress_steps.size() + 1 != poses.size()) throw std::invalid_argument("Invalid whole-body progress dimensions");
    for (double s : progress_steps)
      if (!std::isfinite(s) || s <= 0) throw std::invalid_argument("Invalid whole-body progress step");
  }
  const double positive[] = {l.linear_velocity, l.angular_velocity, l.linear_acceleration,
      l.angular_acceleration, l.linear_jerk, l.angular_jerk, l.sample_interval};
  for (double v : positive)
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("Invalid continuous path limit");
  if (!std::isfinite(l.max_deviation) || l.max_deviation < 0)
    throw std::invalid_argument("Continuous path max deviation must be finite and nonnegative");
  if (!std::isfinite(minimum_duration) || minimum_duration < 0)
    throw std::invalid_argument("Invalid continuous path duration");

  std::vector<Pose6> points;
  std::vector<double> knots{0.0};
  Eigen::Quaterniond origin;
  for (const auto& pose : poses) {
    if (pose.size() != 7 || !pose.allFinite() ||
        !std::isfinite(pose.tail<4>().norm()) || pose.tail<4>().norm() < 1e-9)
      throw std::invalid_argument("Invalid continuous path pose");
    const Eigen::Quaterniond q = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).normalized();
    if (points.empty()) origin = q;
    Eigen::Quaterniond relative = origin.conjugate() * q;
    if (relative.w() < 0) relative.coeffs() *= -1;
    const Eigen::AngleAxisd rotation(relative);
    Pose6 p;
    p.head<3>() = pose.head<3>();
    p.tail<3>() = rotation.axis() * rotation.angle();
    if (p.tail<3>().norm() >= chart_limit)
      throw std::invalid_argument("Continuous orientation exceeds 3 rad local chart; split into stopped paths");
    if (!points.empty()) {
      const Pose6 delta = p - points.back();
      if (delta.tail<3>().norm() >= chart_limit)
        throw std::invalid_argument("Continuous orientation crosses rotation-vector branch");
      double step = std::max(delta.head<3>().norm() / l.linear_velocity,
                                   delta.tail<3>().norm() / l.angular_velocity);
      if (!progress_steps.empty()) step = std::max(step, progress_steps[points.size()-1]);
      if (!std::isfinite(step)) throw std::invalid_argument("Continuous path overflow");
      if (step < 1e-9) continue;  // Duplicate pose, not a dwell instruction.
      knots.push_back(knots.back() + step);
    }
    points.push_back(p);
  }
  if (points.size() == 1) {
    Eigen::VectorXd hold = poses.front();
    hold.tail<4>().normalize();
    return {{0.0, std::max(l.sample_interval, minimum_duration)}, {hold, hold}, {}};
  }
  if (!std::isfinite(knots.back())) throw std::invalid_argument("Continuous path overflow");
  const double length = knots.back();
  for (auto& s : knots) s /= length;
  const size_t n = points.size();
  std::vector<double> h(n - 1);
  for (size_t i = 0; i + 1 < n; ++i) {
    h[i] = knots[i + 1] - knots[i];
    if (h[i] < 1e-9) throw std::invalid_argument("Continuous path has ill-conditioned waypoint spacing");
  }

  // Natural cubic spline: solve the tridiagonal system for second derivatives.
  std::vector<Pose6> second(n, Pose6::Zero());
  std::vector<double> upper(n, 0.0);
  for (size_t i = 1; i + 1 < n; ++i) {
    const double diagonal = 2 * (h[i - 1] + h[i]) - h[i - 1] * upper[i - 1];
    upper[i] = h[i] / diagonal;
    second[i] = (6 * ((points[i + 1] - points[i]) / h[i] -
                      (points[i] - points[i - 1]) / h[i - 1]) -
                  h[i - 1] * second[i - 1]) / diagonal;
  }
  for (size_t i = n - 2; i > 0; --i) second[i] -= upper[i] * second[i + 1];

  std::vector<Bezier> segments;
  std::array<double, 3> linear{}, angular{};
  for (size_t i = 0; i + 1 < n; ++i) {
    const Pose6 slope = (points[i + 1] - points[i]) / h[i];
    Bezier b{points[i], points[i] + h[i] / 3 * (slope - h[i] / 6 * (2 * second[i] + second[i + 1])),
             points[i + 1] - h[i] / 3 * (slope + h[i] / 6 * (second[i] + 2 * second[i + 1])),
             points[i + 1]};
    for (const auto& p : b) {
      if (!p.allFinite() || p.tail<3>().norm() >= chart_limit)
        throw std::invalid_argument("Continuous spline leaves valid rotation chart");
      // Distance to a convex segment is convex, so this bounds the entire curve.
      if (l.max_deviation > 0 &&
          segmentDistance(p.head<3>(), points[i].head<3>(), points[i + 1].head<3>()) > l.max_deviation)
        throw std::invalid_argument("Continuous path exceeds movel_continuous_max_deviation; add waypoints or use PATH");
    }
    segments.push_back(b);
    // Derivative Bezier control hulls bound norms at ALL path parameters.
    for (size_t order = 0; order < 3; ++order) {
      for (size_t j = 0; j < 3 - order; ++j) {
        b[j] = (3 - order) * (b[j + 1] - b[j]) / h[i];
        linear[order] = std::max(linear[order], b[j].head<3>().norm());
        angular[order] = std::max(angular[order], b[j].tail<3>().norm());
      }
    }
  }
  // s = quintic(t/T), with max |s'|=1.875, |s''|<5.774, |s'''|=60 for T=1.
  const auto timedBounds = [](const std::array<double, 3>& d) {
    return std::array<double, 3>{1.875 * d[0], 1.875 * 1.875 * d[1] + 5.774 * d[0],
        std::pow(1.875, 3) * d[2] + 3 * 1.875 * 5.774 * d[1] + 60 * d[0]};
  };
  const auto p = timedBounds(linear);
  const auto r = timedBounds(angular);
  // SO(3) exp Jacobian integral bounds: ||J||<=1, ||Jdot||<=|rdot|/2,
  // ||Jddot||<=|rddot|/2+|rdot|^2/3. These also bound physical angular derivatives.
  const double angular_acc = r[1] + 0.5 * r[0] * r[0];
  const double angular_jerk = r[2] + 1.5 * r[0] * r[1] + std::pow(r[0], 3) / 3;
  const double duration = std::max({l.sample_interval, minimum_duration,
      p[0] / l.linear_velocity, r[0] / l.angular_velocity,
      std::sqrt(p[1] / l.linear_acceleration), std::sqrt(angular_acc / l.angular_acceleration),
      std::cbrt(p[2] / l.linear_jerk), std::cbrt(angular_jerk / l.angular_jerk)});
  if (!std::isfinite(duration) || duration / l.sample_interval > sample_budget - n)
    throw std::invalid_argument("Continuous path exceeds sampling budget");

  const size_t count = static_cast<size_t>(std::ceil(duration / l.sample_interval));
  std::vector<double> times;
  times.reserve(count + n + 1);
  for (size_t i = 0; i <= count; ++i) times.push_back(duration * i / count);
  for (size_t i = 1; i + 1 < n; ++i) times.push_back(duration * inverseSmooth(knots[i]));
  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end(), [](double a, double b) {
    return std::abs(a - b) < 1e-10;
  }), times.end());
  SampledTcpPath result;
  if (!progress_steps.empty())
    for (double s : knots) result.waypoint_times.push_back(duration * inverseSmooth(s));
  if (!result.waypoint_times.empty()) {
    result.waypoint_times.front() = 0;
    result.waypoint_times.back() = duration;
  }
  result.times = std::move(times);
  result.poses.reserve(result.times.size());
  for (double t : result.times) {
    const double s = std::clamp(smooth(t / duration), 0.0, 1.0);
    const auto it = std::upper_bound(knots.begin(), knots.end(), s);
    const size_t i = std::min(n - 2, static_cast<size_t>(it - knots.begin() - 1));
    const double u = std::clamp((s - knots[i]) / h[i], 0.0, 1.0);
    auto b = segments[i];
    for (size_t level = 3; level > 0; --level)
      for (size_t j = 0; j < level; ++j) b[j] = (1 - u) * b[j] + u * b[j + 1];
    const double angle = b[0].tail<3>().norm();
    Eigen::Quaterniond q = origin;
    if (angle > 1e-12) q = origin * Eigen::Quaterniond(Eigen::AngleAxisd(angle, b[0].tail<3>() / angle));
    Eigen::VectorXd pose(7);
    pose.head<3>() = b[0].head<3>();
    pose.tail<4>() = q.normalized().coeffs();
    result.poses.push_back(std::move(pose));
  }
  return result;
}
}  // namespace zephyr::mpc
