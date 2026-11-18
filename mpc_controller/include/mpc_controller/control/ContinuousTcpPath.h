// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Core>
#include <vector>

namespace zephyr::mpc {
struct TcpPathLimits {
  double linear_velocity{0.3}, angular_velocity{1.0};
  double linear_acceleration{1.0}, angular_acceleration{2.0};
  double linear_jerk{2.0}, angular_jerk{4.0};
  double sample_interval{0.04}, max_deviation{0.0}; // 0 disables the optional chord-deviation bound.
};
struct SampledTcpPath {
  std::vector<double> times;  // Relative to the start, including every waypoint.
  std::vector<Eigen::VectorXd> poses;  // xyz, quaternion xyzw.
};

/** Non-RT natural cubic pose path with one rest-to-rest quintic time law.
 * Intermediate poses are interpolated, not independently stopped at. Translation
 * blending can optionally be bounded relative to each input segment (not a collision check).
 * Orientation uses a local rotation-vector chart < 3 rad from the first pose;
 * ambiguous/winding paths are rejected, never silently changed to another turn.
 * Bounds apply to the analytic curve; OCS2 consumes its sampled pose reference.
 */
SampledTcpPath makeContinuousTcpPath(const std::vector<Eigen::VectorXd>& poses,
                                    const TcpPathLimits& limits,
                                    double minimum_duration = 0.0);
}  // namespace zephyr::mpc
