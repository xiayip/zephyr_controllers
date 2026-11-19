// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace zephyr::mpc {
/** Normalized closure, NOT an angle: 0=open, 1=closed.
 * rate is closure/s; speed/effort are device-normalized caps, NOT signed qdot/Nm.
 * Optional and initialization-only; real contact dynamics are not modeled.
 */
struct GripperConfig {
  std::string joint, command_prefix;
  double max_rate{0.5}, speed_limit{0.5}, effort_limit{0.4};
  double weight{10.0}, tolerance{0.01};
  bool enabled() const { return !joint.empty(); }
  void validate() const {
    if (!enabled()) return;
    if (command_prefix.empty() || !std::isfinite(max_rate) || max_rate <= 0 ||
        !std::isfinite(speed_limit) || speed_limit <= 0 || speed_limit > 1 ||
        !std::isfinite(effort_limit) || effort_limit < 0 || effort_limit > 1 ||
        !std::isfinite(weight) || weight <= 0 || weight > 1000 ||
        !std::isfinite(tolerance) || tolerance <= 0 || tolerance > 1)
      throw std::invalid_argument("Invalid normalized gripper configuration");
  }
  double step(double measured, double requested, double dt) const {
    if (!validPosition(measured) || !std::isfinite(requested) || !std::isfinite(dt) || dt <= 0)
      throw std::invalid_argument("Invalid gripper feedback/policy");
    return std::clamp(requested, std::max(0.0, measured-max_rate*dt),
                      std::min(1.0, measured+max_rate*dt));
  }
  static bool validPosition(double value) {
    return std::isfinite(value) && value >= 0 && value <= 1;
  }
};
}  // namespace zephyr::mpc
