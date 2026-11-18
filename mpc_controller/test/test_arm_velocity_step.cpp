// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "mpc_controller/control/ArmVelocityStep.h"
#include <limits>

using namespace zephyr::mpc;

TEST(ArmVelocityStep, UsesMeasuredArmAndOneCycleNotAbsoluteRollout) {
    const Eigen::VectorXd measured = Eigen::VectorXd::LinSpaced(9, 0, 0.8);
    const Eigen::VectorXd input = Eigen::VectorXd::Constant(9, -0.2);
    Eigen::VectorXd command = Eigen::VectorXd::Constant(9, 10.0);
    ASSERT_TRUE(stepArmVelocity(measured, input, 3, .005, command));
    EXPECT_TRUE(command.head(3).isConstant(10));
    EXPECT_TRUE(command.tail(6).isApprox((measured.tail(6).array() - .001).matrix()));
    // No synthetic velocity when the optimizer requests rest, even if each new
    // nominal rollout disagrees with measured q by a different offset.
    command.tail(6).setConstant(-2);
    ASSERT_TRUE(stepArmVelocity(measured, Eigen::VectorXd::Zero(9), 3, .005, command));
    EXPECT_TRUE(command.tail(6).isApprox(measured.tail(6)));
}

TEST(ArmVelocityStep, NoSawtoothOnPolicyReplacementAndNoObservationMutation) {
    Eigen::VectorXd measured = Eigen::VectorXd::Zero(9);
    const Eigen::VectorXd input = Eigen::VectorXd::Constant(9, .002);
    Eigen::VectorXd command(9);
    for (int cycle = 0; cycle < 200; ++cycle) {
        const auto before = measured.eval();
        command.setConstant(cycle % 4 == 0 ? -.0002 : .0002);
        ASSERT_TRUE(stepArmVelocity(measured, input, 3, .005, command));
        EXPECT_TRUE(measured.isApprox(before));
        for (int i = 3; i < 9; ++i) EXPECT_NEAR(command[i]-measured[i], .00001, 1e-15);
        measured.tail(6) = command.tail(6); // An ideal mock plant, NOT controller feedback caching.
    }
    for (int i = 3; i < 9; ++i) EXPECT_NEAR(measured[i], .002, 1e-14);
}

TEST(ArmVelocityStep, RejectsInvalidDataAtomically) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(9), u = q, command = q;
    command.setConstant(42);
    const auto before = command.eval();
    for (double period : {0., -1., std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
        EXPECT_FALSE(stepArmVelocity(q, u, 3, period, command));
        EXPECT_TRUE(command.isApprox(before));
    }
    EXPECT_FALSE(stepArmVelocity(q, u, -1, .005, command));
    EXPECT_FALSE(stepArmVelocity(q, u, 9, .005, command));
    EXPECT_FALSE(stepArmVelocity(q, Eigen::VectorXd::Zero(6), 3, .005, command));
    q[8] = NAN;
    EXPECT_FALSE(stepArmVelocity(q, u, 3, .005, command));
    q[8] = 0; u[8] = INFINITY;
    EXPECT_FALSE(stepArmVelocity(q, u, 3, .005, command));
    q[8] = u[8] = std::numeric_limits<double>::max();
    EXPECT_FALSE(stepArmVelocity(q, u, 3, 2, command));
    EXPECT_TRUE(command.isApprox(before));
}
