// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <array>
#include <limits>
#include <thread>
#include "mpc_controller/control/ControlSafety.h"

using namespace zephyr::mpc;
namespace {
using Vectors = std::vector<Eigen::VectorXd>;
const double invalid_number = std::numeric_limits<double>::quiet_NaN();
}

TEST(ControlPolicy, AcceptsFixedAndMobileDimensions) {
    for (int size : {6, 9}) {
        const Vectors values(3, Eigen::VectorXd::Zero(size));
        EXPECT_TRUE(validPolicy(std::vector<double>{1, 1.1, 1.2}, values, values, size, size));
    }
}
TEST(ControlPolicy, RejectsEmptySingletonOrMismatchedTrajectories) {
    const Vectors values(2, Eigen::VectorXd::Zero(6));
    EXPECT_FALSE(validPolicy(std::vector<double>{}, values, values, 6, 6));
    EXPECT_FALSE(validPolicy(std::vector<double>{1}, values, values, 6, 6));
    EXPECT_FALSE(validPolicy(std::vector<double>{1, 2, 3}, values, values, 6, 6));
    EXPECT_FALSE(validPolicy(std::vector<double>{1, 2}, values, Vectors{}, 6, 6));
    EXPECT_FALSE(validPolicy(std::vector<double>{1, 2}, values, values, 9, 9));
}
TEST(ControlPolicy, RejectsNonfiniteStatesAndInputs) {
    Vectors states(2, Eigen::VectorXd::Zero(6)), inputs = states;
    states[1](3) = invalid_number;
    EXPECT_FALSE(validPolicy(std::vector<double>{1, 2}, states, inputs, 6, 6));
    states[1].setZero(); inputs[0](2) = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(validPolicy(std::vector<double>{1, 2}, states, inputs, 6, 6));
}
TEST(ControlPolicy, ValidatesTimeOrderAndAllowsOcs2EventDuplicates) {
    const Vectors values(3, Eigen::VectorXd::Zero(6));
    EXPECT_TRUE(validPolicy(std::vector<double>{1, 1, 2}, values, values, 6, 6));
    EXPECT_FALSE(validPolicy(std::vector<double>{1, 1, 1}, values, values, 6, 6));
    EXPECT_FALSE(validPolicy(std::vector<double>{1, 0.5, 2}, values, values, 6, 6));
    EXPECT_FALSE(validPolicy(std::vector<double>{1, invalid_number, 2}, values, values, 6, 6));
}
TEST(ControlPolicy, FutureSampleMustRemainInsideHorizon) {
    EXPECT_TRUE(usablePolicy(1.1, 1.2, 1, 1.2, 0.1, 0.5));
    EXPECT_FALSE(usablePolicy(1.19, 1.21, 1, 1.2, 0.1, 0.5));
    EXPECT_FALSE(usablePolicy(1.21, 1.22, 1, 1.2, 0.1, 0.5));
}
TEST(ControlPolicy, WallWatchdogWorksWhenRosTimeStops) {
    EXPECT_TRUE(usablePolicy(1.1, 1.11, 1, 3, 0.1, 0.5));
    EXPECT_FALSE(usablePolicy(1.1, 1.11, 1, 3, 0.51, 0.5));
}
TEST(ControlPolicy, ReceivingAnOldPolicyDoesNotMakeItFresh) {
    EXPECT_FALSE(usablePolicy(2, 2.01, 1, 3, 0, 0.5));
}
TEST(ControlPolicy, RejectsTimeReversalAndNonfiniteValues) {
    EXPECT_FALSE(usablePolicy(0.9, 0.91, 1, 2, 0.1, 0.5));
    EXPECT_FALSE(usablePolicy(1.1, 1.09, 1, 2, 0.1, 0.5));
    EXPECT_FALSE(usablePolicy(1.1, 1.11, 1, 2, -0.1, 0.5));
    EXPECT_FALSE(usablePolicy(1.1, invalid_number, 1, 2, 0.1, 0.5));
    EXPECT_FALSE(usablePolicy(1.1, 1.11, 1, 2, 0.1, invalid_number));
}
TEST(GravityOutput, FixedArmUsesItsNamedDofs) {
    const auto full = Eigen::VectorXd::LinSpaced(6, 10, 15).eval();
    Eigen::VectorXd arm(6);
    ASSERT_TRUE(selectJointTorques(full, {0, 1, 2, 3, 4, 5}, arm));
    EXPECT_EQ(arm, full);
}
TEST(GravityOutput, MobileArmNeverReceivesBaseGeneralizedForces) {
    Eigen::VectorXd full(9); full << 100, 200, 300, 1, 2, 3, 4, 5, 6;
    Eigen::VectorXd arm(6);
    ASSERT_TRUE(selectJointTorques(full, {3, 4, 5, 6, 7, 8}, arm));
    EXPECT_EQ(arm, full.tail(6));
}
TEST(GravityOutput, MappingUsesIndicesRatherThanTheFirstSixEntries) {
    const auto full = Eigen::VectorXd::LinSpaced(9, 0, 8).eval();
    Eigen::VectorXd arm(3);
    ASSERT_TRUE(selectJointTorques(full, {8, 3, 6}, arm));
    EXPECT_DOUBLE_EQ(arm(0), 8); EXPECT_DOUBLE_EQ(arm(1), 3); EXPECT_DOUBLE_EQ(arm(2), 6);
}
TEST(GravityOutput, InvalidMappingDoesNotPartiallyChangeOutput) {
    auto full = Eigen::VectorXd::Zero(9).eval();
    auto arm = Eigen::VectorXd::Constant(3, 42).eval();
    EXPECT_FALSE(selectJointTorques(full, {3, 4, 9}, arm));
    EXPECT_FALSE(selectJointTorques(full, {-1, 4, 5}, arm));
    EXPECT_FALSE(selectJointTorques(full, {3, 4}, arm));
    full(4) = invalid_number;
    EXPECT_FALSE(selectJointTorques(full, {3, 4, 5}, arm));
    EXPECT_TRUE((arm.array() == 42).all());
}
TEST(CollisionMonitor, StartupWaitsForAValidCheckButHasADeadline) {
    CollisionWatchdog monitor;
    EXPECT_EQ(monitor.status(100, 100, 250), CollisionStatus::Pending);
    EXPECT_EQ(monitor.status(351, 100, 250), CollisionStatus::Stale);
    monitor.record(true, false, 351);
    EXPECT_EQ(monitor.status(352, 100, 250), CollisionStatus::Safe);
}
TEST(CollisionMonitor, CheckFailureIsNeverClearance) {
    CollisionWatchdog monitor;
    monitor.record(true, false, 100);
    monitor.record(false, false, 110);
    EXPECT_EQ(monitor.status(111, 100, 250), CollisionStatus::Invalid);
    monitor.record(true, true, 120);
    EXPECT_EQ(monitor.status(121, 100, 250), CollisionStatus::Collision);
}
TEST(CollisionMonitor, RecheckingOldSnapshotDoesNotRefreshItsAge) {
    CollisionWatchdog monitor;
    monitor.record(true, false, 100);
    EXPECT_EQ(monitor.status(300, 100, 250), CollisionStatus::Safe);
    monitor.record(true, false, 100);
    EXPECT_EQ(monitor.status(351, 100, 250), CollisionStatus::Stale);
}
TEST(CollisionMonitor, ReactivationCannotReusePreviousClearance) {
    CollisionWatchdog monitor;
    monitor.record(true, false, 100);
    monitor.reset();
    EXPECT_EQ(monitor.status(150, 150, 250), CollisionStatus::Pending);
}
TEST(CollisionMonitor, RejectsFutureObservationTimestamp) {
    CollisionWatchdog monitor;
    monitor.record(true, false, 200);
    EXPECT_EQ(monitor.status(150, 100, 250), CollisionStatus::Stale);
}
TEST(TargetSession, RejectsTargetsPreparedBeforePauseAndResume) {
    TargetSession session;
    EXPECT_EQ(session.token(), 0u);
    session.setAccepting(true);
    const auto old = session.token();
    EXPECT_TRUE(session.accepts(old));
    session.setAccepting(false);
    EXPECT_FALSE(session.accepts(old));
    session.setAccepting(true);
    EXPECT_FALSE(session.accepts(old));
    EXPECT_TRUE(session.accepts(session.token()));
}
TEST(TargetSession, RepeatedStatusDoesNotInvalidateCurrentCommands) {
    TargetSession session;
    session.setAccepting(true);
    const auto token = session.token();
    session.setAccepting(true);
    EXPECT_TRUE(session.accepts(token));
    EXPECT_FALSE(session.accepts(0));
}
TEST(ReferenceMailbox, ReadsCoherentSnapshotsUnderConcurrentUpdates) {
    struct Request { uint64_t sequence{0}; std::array<uint64_t, 7> pose{}; };
    TryWriteMailbox<Request> mailbox;
    std::atomic_bool finished{false};
    std::thread consumer([&] {
        while (!finished.load()) {
            const auto value = mailbox.read();
            for (auto element : value.pose) EXPECT_EQ(element, value.sequence);
        }
    });
    for (uint64_t i = 1; i <= 10000; ++i) {
        Request value; value.sequence = i; value.pose.fill(i);
        while (!mailbox.tryWrite(value)) std::this_thread::yield();
    }
    finished = true;
    consumer.join();
    EXPECT_EQ(mailbox.read().sequence, 10000u);
}
