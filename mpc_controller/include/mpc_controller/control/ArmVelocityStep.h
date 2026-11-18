// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Core>
#include <cmath>

namespace zephyr::mpc {

/** Convert a kinematic qdot policy into one cycle's position reference.
 * Mobile base output is projected/deadbanded, so its nominal rollout and actual
 * state can diverge. Sending absolute rollout q then creates jumps on each
 * replan. Anchor the arm step to measured q; never substitute it for feedback.
 * Only the arm tail is changed. Invalid input leaves the entire command intact.
 */
inline bool stepArmVelocity(const Eigen::VectorXd& measured,
                            const Eigen::VectorXd& input,
                            Eigen::Index arm_offset, double period,
                            Eigen::VectorXd& command) {
    if (measured.size() != input.size() || measured.size() != command.size() ||
        arm_offset < 0 || arm_offset >= measured.size() ||
        !std::isfinite(period) || period <= 0.0 ||
        !measured.allFinite() || !input.allFinite()) return false;
    for (Eigen::Index i = arm_offset; i < measured.size(); ++i)
        if (!std::isfinite(measured[i] + period * input[i])) return false;
    command.tail(measured.size() - arm_offset) =
        measured.tail(measured.size() - arm_offset) + period * input.tail(input.size() - arm_offset);
    return true;
}

} // namespace zephyr::mpc
