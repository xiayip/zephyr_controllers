// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <ocs2_core/PreComputation.h>
#include "mpc_controller/control/BaseKinematics.h"

using namespace zephyr::mpc;

TEST(BaseKinematics, AllowsSimultaneousBaseAndArmMotionInBothDirections) {
    const auto constraint = makeNoLateralVelocityConstraint(9, 9);
    const ocs2::PreComputation pre;
    const auto state = ocs2::vector_t::LinSpaced(9, -1.0, 1.0).eval();
    for (double sign : {-1.0, 1.0}) {
        auto input = ocs2::vector_t::Constant(9, sign * 0.2).eval();
        input(1) = 0.0;
        EXPECT_TRUE(constraint->isActive(0.0));
        EXPECT_EQ(constraint->getNumConstraints(0.0), 1U);
        EXPECT_DOUBLE_EQ(constraint->getValue(0.0, state, input, pre).norm(), 0.0);
        input(1) = sign * 0.1;
        EXPECT_DOUBLE_EQ(constraint->getValue(0.0, state, input, pre)(0), sign * 0.1);
    }
}

TEST(BaseKinematics, OnlyLateralInputIsConstrainedAtEveryTimeAndState) {
    const auto constraint = makeNoLateralVelocityConstraint(9, 9);
    const ocs2::PreComputation pre;
    const auto state = ocs2::vector_t::Constant(9, 2.0).eval();
    const auto input = ocs2::vector_t::Constant(9, 0.3).eval();
    for (double time : {0.0, 1.0, 100.0}) {
        const auto approximation = constraint->getLinearApproximation(time, state, input, pre);
        EXPECT_DOUBLE_EQ(approximation.f(0), 0.3);
        EXPECT_TRUE(approximation.dfdx.isZero());
        ASSERT_EQ(approximation.dfdu.rows(), 1);
        ASSERT_EQ(approximation.dfdu.cols(), 9);
        for (int i = 0; i < 9; ++i)
            EXPECT_DOUBLE_EQ(approximation.dfdu(0, i), i == 1 ? 1.0 : 0.0);
    }
    const std::unique_ptr<ocs2::StateInputConstraint> copy(constraint->clone());
    EXPECT_EQ(copy->getValue(0.0, state, input, pre), constraint->getValue(0.0, state, input, pre));
}

TEST(BaseKinematics, RejectsMissingBaseDimensions) {
    EXPECT_THROW(makeNoLateralVelocityConstraint(2, 9), std::invalid_argument);
    EXPECT_THROW(makeNoLateralVelocityConstraint(9, 2), std::invalid_argument);
}
