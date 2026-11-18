// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/WholeBodyReference.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zephyr::mpc {
namespace {
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double smooth(double t) { return t*t*t*(10+t*(-15+6*t)); }
}

ocs2::vector_t WholeBodyTrajectory::sample(double time) const {
  if (time <= times.front()) return states.front();
  if (time >= times.back()) return states.back();
  const double s = smooth((time-times.front())/(times.back()-times.front()));
  const size_t i = std::min(progress.size()-2,
      static_cast<size_t>(std::upper_bound(progress.begin(), progress.end(), s) - progress.begin() - 1));
  const double h = progress[i + 1] - progress[i], u = std::clamp((s-progress[i])/h,0.0,1.0);
  return (2*u*u*u - 3*u*u + 1)*states[i] + (u*u*u - 2*u*u + u)*h*slopes[i] +
      (-2*u*u*u + 3*u*u)*states[i + 1] + (u*u*u - u*u)*h*slopes[i + 1];
}
void WholeBodyTrajectory::stretch(double factor) {
  if (!std::isfinite(factor) || factor < 1) throw std::invalid_argument("Invalid whole-body time scale");
  for (auto& t : times) t = times.front() + (t - times.front()) * factor;
}

std::shared_ptr<WholeBodyTrajectory> makeWholeBodyTrajectory(
    const std::vector<double>& times, ocs2::vector_array_t states,
    const ocs2::vector_t& weights, int yaw_index, const ocs2::vector_t& limits) {
  const size_t n = times.size();
  const auto dim = weights.size();
  if (n < 2 || states.size() != n || dim == 0 || limits.size() != dim ||
      !weights.allFinite() || (weights.array() < 0).any() ||
      !limits.allFinite() || (limits.array() <= 0).any() || yaw_index < -1 || yaw_index >= dim)
    throw std::invalid_argument("Invalid whole-body trajectory dimensions/weights/limits");
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(times[i]) || (i && times[i] - times[i-1] < 1e-8) ||
        states[i].size() != dim || !states[i].allFinite())
      throw std::invalid_argument("Invalid whole-body trajectory sample");
    if (i && yaw_index >= 0)
      states[i][yaw_index] = states[i-1][yaw_index] + wrap(states[i][yaw_index] - states[i-1][yaw_index]);
  }
  auto path = std::make_shared<WholeBodyTrajectory>();
  path->times = times; path->states = std::move(states); path->weights = weights;
  path->yaw_index = yaw_index;
  const double duration=times.back()-times.front();
  for (double t : times) path->progress.push_back(smooth((t-times.front())/duration));
  for (size_t i=1;i<n;++i)
    if (path->progress[i]-path->progress[i-1] < 1e-10)
      throw std::invalid_argument("Whole-body progress has ill-conditioned spacing");
  path->slopes.assign(n, ocs2::vector_t::Zero(dim));
  path->slopes.front()=(path->states[1]-path->states[0])/(path->progress[1]-path->progress[0]);
  path->slopes.back()=(path->states[n-1]-path->states[n-2])/(path->progress[n-1]-path->progress[n-2]);
  for (size_t i = 1; i + 1 < n; ++i) {
    const double a = path->progress[i] - path->progress[i-1], b = path->progress[i+1] - path->progress[i];
    const auto left = ((path->states[i] - path->states[i-1]) / a).eval();
    const auto right = ((path->states[i+1] - path->states[i]) / b).eval();
    for (int j = 0; j < dim; ++j) {
      if (left[j] * right[j] <= 0) continue;
      const double w1 = 2*b+a, w2 = b+2*a;
      path->slopes[i][j] = (w1+w2) / (w1/left[j] + w2/right[j]);
    }
  }
  double scale = 1;
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto secant = ((path->states[i+1] - path->states[i]) / (path->progress[i+1]-path->progress[i])).eval();
    // Cubic Bezier derivative hull [v0, 3*secant-v0-v1, v1].
    const auto middle = (3*secant - path->slopes[i] - path->slopes[i+1]).eval();
    for (int j = 0; j < dim; ++j) if (weights[j] > 0) {
      scale = std::max(scale, 1.875 / duration * std::max({std::abs(path->slopes[i][j]),
          std::abs(middle[j]), std::abs(path->slopes[i+1][j])}) / limits[j]);
    }
  }
  if (!std::isfinite(scale)) throw std::invalid_argument("Whole-body reference overflow");
  path->stretch(scale);
  return path;
}

WholeBodyStateCost::WholeBodyStateCost(std::shared_ptr<WholeBodyReference> r) : reference_(std::move(r)) {}
WholeBodyStateCost* WholeBodyStateCost::clone() const { return new WholeBodyStateCost(*this); }
bool WholeBodyStateCost::isActive(ocs2::scalar_t) const { return bool(reference_->solver); }
ocs2::scalar_t WholeBodyStateCost::getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
    const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const {
  const auto& path = reference_->solver;
  if (!path) return 0;
  auto error = (state - path->sample(time)).eval();
  if (path->yaw_index >= 0) error[path->yaw_index] = wrap(error[path->yaw_index]);
  return 0.5 * (path->weights.array() * error.array().square()).sum();
}
ocs2::ScalarFunctionQuadraticApproximation WholeBodyStateCost::getQuadraticApproximation(
    ocs2::scalar_t time, const ocs2::vector_t& state,
    const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const {
  auto a = ocs2::ScalarFunctionQuadraticApproximation::Zero(state.size(), 0);
  const auto& path = reference_->solver;
  if (!path) return a;
  auto error = (state - path->sample(time)).eval();
  if (path->yaw_index >= 0) error[path->yaw_index] = wrap(error[path->yaw_index]);
  a.f = 0.5 * (path->weights.array() * error.array().square()).sum();
  a.dfdx = path->weights.cwiseProduct(error);
  a.dfdxx = path->weights.asDiagonal();
  return a;
}
}  // namespace zephyr::mpc
