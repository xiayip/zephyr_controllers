#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace teleop_joint_controller {
// Runs at the controller clock, independent of Servo callback timestamps.
// Normal output obeys |dq/dt| <= vmax and |dv/dt| <= amax. Fault stops
// intentionally override acceleration limits and require reactivation.
class ReferenceFilter {
public:
  void configure(std::vector<double> vmax, std::vector<double> amax, double response_time = 0.01) {
    if (vmax.empty() || vmax.size() != amax.size())
      throw std::invalid_argument("velocity/acceleration limits must match joints");
    for (size_t i = 0; i < vmax.size(); ++i)
      if (!std::isfinite(vmax[i]) || vmax[i] <= 0 ||
          !std::isfinite(amax[i]) || amax[i] <= 0)
        throw std::invalid_argument("limits must be finite and positive");
    if (!std::isfinite(response_time) || response_time < 0)
      throw std::invalid_argument("reference response time must be finite and nonnegative");
    response_time_ = response_time;
    vmax_ = std::move(vmax); amax_ = std::move(amax);
    position.resize(vmax_.size()); velocity.resize(vmax_.size());
  }
  void reset(const std::vector<double>& q) {
    position = q;
    std::fill(velocity.begin(), velocity.end(), 0.0);
  }
  void stop() { std::fill(velocity.begin(), velocity.end(), 0.0); }
  void step(const std::vector<double>& target, double dt) {
    for (size_t i = 0; i < position.size(); ++i) {
      const double error = target[i] - position[i];
      const double dv = amax_[i] * dt;
      // Leave room to brake at a stationary target. A target reversal can
      // require several cycles of braking; never jump the reference to it.
      const double braking_velocity = std::sqrt(dv * dv + 2 * amax_[i] * std::abs(error)) - dv;
      // The two 200 Hz clocks are asynchronous: a cycle may see the same
      // waypoint twice, then skip to the next one. A short first-order response
      // avoids turning that sample/hold pattern directly into velocity ripple.
      const double desired = std::copysign(
          std::min({vmax_[i], std::abs(error) / (dt + response_time_), braking_velocity}), error);
      velocity[i] = std::clamp(desired, velocity[i] - dv, velocity[i] + dv);
      position[i] += velocity[i] * dt;
    }
  }
  std::vector<double> position, velocity;
private:
  std::vector<double> vmax_, amax_;
  double response_time_{0.01};
};
} // namespace teleop_joint_controller
