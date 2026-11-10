// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <stdexcept>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>

namespace zephyr::mpc {

// Legacy Ranger compatibility: inputs are [vx, vy, wz, joint velocities...].
// Keep vy=0 in the optimizer, leaving vx, wz and all arm inputs free together.
// This is a chassis capability constraint, not a base/arm motion schedule.
inline std::unique_ptr<ocs2::StateInputConstraint> makeNoLateralVelocityConstraint(
    int state_dim, int input_dim) {
    if (state_dim < 3 || input_dim < 3)
        throw std::invalid_argument("Mobile base state/input dimensions must include x/y/yaw");
    auto input_matrix = ocs2::matrix_t::Zero(1, input_dim).eval();
    input_matrix(0, 1) = 1.0;
    return std::make_unique<ocs2::LinearStateInputConstraint>(
        ocs2::vector_t::Zero(1), ocs2::matrix_t::Zero(1, state_dim), input_matrix);
}

} // namespace zephyr::mpc
