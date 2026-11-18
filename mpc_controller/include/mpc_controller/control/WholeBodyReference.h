// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <vector>
#include <ocs2_core/cost/StateCost.h>

namespace zephyr::mpc {
/** Immutable, C1, shape-preserving state preference on the TCP's time axis.
 * Only yaw wraps; bounded arm joints must never take a modulo-2pi shortcut.
 * Weights and active state indices are constant for one finite motion.
 */
struct WholeBodyTrajectory {
  std::vector<double> times, progress;
  ocs2::vector_array_t states, slopes;
  ocs2::vector_t weights;
  int yaw_index{-1};
  ocs2::vector_t sample(double time) const;
  void stretch(double factor);
};

// Monotone cubic in path progress + the TCP's global quintic time law gives
// zero endpoint velocities and computes
// an analytic derivative bound per coordinate. No claim of jerk-limited motion.
std::shared_ptr<WholeBodyTrajectory> makeWholeBodyTrajectory(
    const std::vector<double>& times, ocs2::vector_array_t states,
    const ocs2::vector_t& weights, int yaw_index,
    const ocs2::vector_t& velocity_limits);

/** Set only at preSolverRun, after workers from the previous solve have joined.
 * All cloned costs share the same immutable snapshot for the entire solve.
 */
struct WholeBodyReference {
  std::shared_ptr<const WholeBodyTrajectory> solver;
};

class WholeBodyStateCost final : public ocs2::StateCost {
 public:
  explicit WholeBodyStateCost(std::shared_ptr<WholeBodyReference> reference);
  WholeBodyStateCost* clone() const override;
  bool isActive(ocs2::scalar_t time) const override;
  ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const override;
  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::TargetTrajectories&, const ocs2::PreComputation&) const override;
 private:
  std::shared_ptr<WholeBodyReference> reference_;
};
}  // namespace zephyr::mpc
