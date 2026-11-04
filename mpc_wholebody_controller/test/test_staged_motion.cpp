// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <cmath>
#include "mpc_wholebody_controller/control/StagedMotion.h"

using namespace zephyr::mpc_wholebody;
namespace {
ReachCoordinator makeCoordinator() {
    StagedMotionConfig config;
    config.reach_radius = 1.0;
    return ReachCoordinator(config);
}
}

TEST(StagedMotion, FullReachKeepsTrackingAndReturningTargetReleasesArm) {
    auto coordinator = makeCoordinator();
    EXPECT_EQ(coordinator.update(1.0, {0,0,0}, {0,0,0}, {0.7,0,0}).phase, MotionPhase::Arm);
    EXPECT_EQ(coordinator.update(1.1, {0,0,0}, {0,0,0}, {0.95,0,0}).phase, MotionPhase::Arm);
    EXPECT_EQ(coordinator.update(1.2, {0,0,0}, {0,0,0}, {1.1,0,0}).phase, MotionPhase::Base);
    EXPECT_EQ(coordinator.update(1.3, {0,0,0}, {0,0,0}, {0.9,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.4, {0,0,0}, {0,0,0}, {0.9,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.7, {0,0,0}, {0,0,0}, {0.9,0,0}).phase, MotionPhase::Arm);
}

TEST(StagedMotion, FarTargetWaitsForObservedStopBeforeArmRelease) {
    auto coordinator = makeCoordinator();
    const auto far = coordinator.update(1.0, {0,0,0}, {0,0,0}, {1.2,0,0});
    EXPECT_EQ(far.phase, MotionPhase::Base);
    EXPECT_NEAR(far.base_goal.x(), 0.55, 1e-12);
    EXPECT_EQ(coordinator.update(1.1, {0.54,0,0}, {0.54,0,0}, {1.2,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.2, {0.54,0,0}, {0.54,0,0}, {1.2,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.3, {0.55,0,0}, {0.55,0,0}, {1.2,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.4, {0.55,0,0}, {0.55,0,0}, {1.2,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.6, {0.55,0,0}, {0.55,0,0}, {1.2,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.7, {0.55,0,0}, {0.55,0,0}, {1.2,0,0}).phase, MotionPhase::Arm);
}

TEST(StagedMotion, ANewFarTargetInterruptsSettling) {
    auto coordinator = makeCoordinator();
    coordinator.update(1.0, {0,0,0}, {0,0,0}, {1.2,0,0});
    EXPECT_EQ(coordinator.update(1.1, {0.54,0,0}, {0.54,0,0}, {1.2,0,0}).phase, MotionPhase::Settling);
    EXPECT_EQ(coordinator.update(1.2, {0.54,0,0}, {0.54,0,0}, {2,0,0}).phase, MotionPhase::Base);
}

TEST(StagedMotion, UsesArmMountAndThreeDimensionalRadius) {
    auto coordinator = makeCoordinator();
    const auto stage = coordinator.update(1.0, {2,3,0}, {2.2,3,0.5}, {3.2,3,1.1});
    EXPECT_EQ(stage.phase, MotionPhase::Base);
    EXPECT_NEAR(stage.distance, std::hypot(1.0,0.6), 1e-12);
    EXPECT_NEAR(stage.base_goal.x(), 3.0 - std::sqrt(0.65*0.65-0.6*0.6), 1e-12);
    EXPECT_NEAR(stage.base_goal.y(), 3.0, 1e-12);
}

TEST(StagedMotion, DoesNotChaseImpossibleHeightAndRecoversOnNewGoal) {
    auto coordinator = makeCoordinator();
    EXPECT_EQ(coordinator.update(1.0, {0,0,0}, {0,0,0}, {2,0,1.1}).phase, MotionPhase::Unreachable);
    EXPECT_EQ(coordinator.update(1.1, {0,0,0}, {0,0,0}, {1.2,0,0}).phase, MotionPhase::Base);
    coordinator.reset();
    EXPECT_EQ(coordinator.update(1.2, {0,0,0}, {0,0,0}, {0.7,0,0}).phase, MotionPhase::Arm);
}

TEST(StagedMotion, BoundaryNoiseKeepsCurrentApproachStable) {
    auto coordinator = makeCoordinator();
    EXPECT_EQ(coordinator.update(1.0, {0,0,0}, {0,0,0}, {1.01,0,0}).phase, MotionPhase::Arm);
    EXPECT_EQ(coordinator.update(1.1, {0,0,0}, {0,0,0}, {1.03,0,0}).phase, MotionPhase::Base);
    EXPECT_EQ(coordinator.update(1.2, {0,0,0}, {0,0,0}, {0.99,0,0}).phase, MotionPhase::Base);
    EXPECT_EQ(coordinator.update(1.3, {0,0,0}, {0,0,0}, {1.01,0,0}).phase, MotionPhase::Base);
    EXPECT_EQ(coordinator.update(1.4, {0,0,0}, {0,0,0}, {1.03,0,0}).phase, MotionPhase::Base);
}


namespace {
TrackingFeedback feedback(double position, double orientation = 0.0) {
    TrackingFeedback result;
    result.position_error = position;
    result.orientation_error = orientation;
    return result;
}
}

TEST(StagedMotion, FarApproachDoesNotStopAtTheOuterReachBoundary) {
    auto c = makeCoordinator();
    const auto first = c.update(1.0, {0,0,0}, {0,0,0}, {1.2,0,0});
    EXPECT_NEAR(first.stop_radius, 0.65, 1e-12);
    EXPECT_EQ(c.update(1.1, {0.25,0,0}, {0.25,0,0}, {1.2,0,0}).phase, MotionPhase::Base);
    EXPECT_NEAR(c.update(1.2, {0.4,0,0}, {0.4,0,0}, {1.2,0,0}).base_goal.x(), first.base_goal.x(), 1e-12);
    EXPECT_EQ(c.update(1.3, {0.54,0,0}, {0.54,0,0}, {1.2,0,0}).phase, MotionPhase::Settling);
}

TEST(StagedMotion, StalledTargetInsideReachUnlocksWholeBody) {
    auto c = makeCoordinator();
    for (int i=0; i<20; ++i)
        EXPECT_EQ(c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.10)).phase, MotionPhase::Arm);
    EXPECT_EQ(c.update(3.1, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.10)).phase, MotionPhase::Assist);
    EXPECT_EQ(c.update(3.2, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.005)).phase, MotionPhase::Settling);
    EXPECT_EQ(c.update(3.3, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.005)).phase, MotionPhase::Settling);
    EXPECT_EQ(c.update(3.6, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.005)).phase, MotionPhase::Arm);
}

TEST(StagedMotion, ProgressAndSmallResidualErrorsKeepTheBaseLocked) {
    auto c = makeCoordinator();
    for (int i=0; i<60; ++i)
        EXPECT_EQ(c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.4-0.005*i)).phase, MotionPhase::Arm);
    for (int i=60; i<100; ++i)
        EXPECT_EQ(c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.005,0.05)).phase, MotionPhase::Arm);
}

TEST(StagedMotion, OrientationStallAlsoAllowsBaseAssistance) {
    auto c = makeCoordinator();
    for (int i=0; i<22; ++i) c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.005,0.5));
    EXPECT_EQ(c.update(3.3, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.005,0.5)).phase, MotionPhase::Assist);
}

TEST(StagedMotion, ANewTargetAndObservationGapsRestartTheStallTimer) {
    auto c = makeCoordinator();
    for (int i=0; i<20; ++i) c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.10));
    EXPECT_EQ(c.update(3.0, {0,0,0}, {0,0,0}, {0.90,0,0}, feedback(0.10)).phase, MotionPhase::Arm);
    EXPECT_EQ(c.update(10.0, {0,0,0}, {0,0,0}, {0.90,0,0}, feedback(0.10)).phase, MotionPhase::Arm);
}

TEST(StagedMotion, FailedAssistDoesNotRepeatForTheSameInfeasibleTarget) {
    auto c = makeCoordinator();
    for (int i=0; i<22; ++i) c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.10));
    EXPECT_EQ(c.update(3.3, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.10)).phase, MotionPhase::Assist);
    for (int i=23; i<70; ++i) c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.10));
    for (int i=70; i<110; ++i)
        EXPECT_EQ(c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.85,0,0}, feedback(0.10)).phase, MotionPhase::Arm);
    for (int i=110; i<133; ++i) c.update(1.0+0.1*i, {0,0,0}, {0,0,0}, {0.90,0,0}, feedback(0.10));
    EXPECT_EQ(c.update(14.4, {0,0,0}, {0,0,0}, {0.90,0,0}, feedback(0.10)).phase, MotionPhase::Assist);
}

TEST(StagedMotion, ParkingRequiresBothPositionAndHeading) {
    auto c = makeCoordinator();
    const Eigen::Vector3d target(0,1.2,0);
    const auto plan = c.update(1.0, {0,0,0}, {0,0,0}, target);
    EXPECT_NEAR(plan.base_goal.x(), 0.0, 1e-12);
    EXPECT_NEAR(plan.base_goal.y(), 0.55, 1e-12);
    EXPECT_NEAR(plan.base_yaw_goal, 0.5*std::acos(-1.0), 1e-12);
    EXPECT_EQ(c.update(1.1, {0,0.55,0}, {0,0.55,0}, target).phase, MotionPhase::Base);
    EXPECT_EQ(c.update(1.2, {0,0.55,plan.base_yaw_goal}, {0,0.55,0}, target).phase, MotionPhase::Settling);
}

TEST(StagedMotion, ParkingCompensatesForMountTranslationWhenTurning) {
    auto c = makeCoordinator();
    const Eigen::Vector3d target(0.2,1.2,0);
    const auto plan = c.update(1.0, {0,0,0}, {0.2,0,0}, target);
    EXPECT_NEAR(plan.base_yaw_goal, 0.5*std::acos(-1.0), 1e-12);
    EXPECT_NEAR(plan.base_goal.x(), 0.2, 1e-12);
    EXPECT_NEAR(plan.base_goal.y(), 0.35, 1e-12);
    EXPECT_EQ(c.update(1.1, {0.2,0.35,plan.base_yaw_goal}, {0.2,0.55,0}, target).phase, MotionPhase::Settling);
}

TEST(StagedMotion, HeadingUsesShortestTurnAcrossPiBoundary) {
    auto c = makeCoordinator();
    const double initial_yaw = std::acos(-1.0)-0.02;
    const auto plan = c.update(1.0, {0,0,initial_yaw}, {0,0,0}, {-1.2,-0.02,0});
    EXPECT_GT(plan.base_yaw_goal-initial_yaw, 0.0);
    EXPECT_LT(plan.base_yaw_goal-initial_yaw, 0.06);
}

TEST(StagedMotion, ApproachUsesHeadingGuidanceWhileLockingArmForBothBackends) {
    const ocs2::vector_t x = ocs2::vector_t::Zero(9);
    ocs2::vector_t u = ocs2::vector_t::Zero(9);
    u(0)=0.1;u(2)=0.2;
    ocs2::PreComputation pre;
    for (bool ranger : {false,true}) {
        auto stage = std::make_shared<StagedMotionState>();
        stage->phase=MotionPhase::Base;
        stage->base_yaw_goal=1.0;
        stage->parking_yaw=1.0;
        stage->base_goal=Eigen::Vector2d(1,1);
        if (ranger) u.head<3>()=rangerApproachCommand(x.head<3>(),*stage,0.15,0.3).input;
        else {u(0)=0.1;u(2)=rangerHeadingCommand(x(2),stage->parking_yaw,0.3).velocity;}
        auto constraint=makeStageInputConstraint(stage,ranger);
        EXPECT_DOUBLE_EQ(constraint->getValue(0,x,u,pre).norm(),0.0);
        EXPECT_DOUBLE_EQ(constraint->getLinearApproximation(0,x,u,pre).dfdu.col(2).norm(),1.0);
        ocs2::vector_t moving_arm=u;moving_arm(3)=0.1;
        EXPECT_GT(constraint->getValue(0,x,moving_arm,pre).norm(),0.09);
        ocs2::vector_t lateral=u;lateral(1)=0.1;
        EXPECT_EQ(constraint->getValue(0,x,lateral,pre).norm()>0.09,ranger);
    }
}

TEST(StagedMotion, RangerAssistanceAllowsYawWithoutLateralVelocity) {
    auto stage=std::make_shared<StagedMotionState>();stage->phase=MotionPhase::Assist;
    auto constraint=makeStageInputConstraint(stage,true);
    const ocs2::vector_t x=ocs2::vector_t::Zero(9);
    ocs2::vector_t u=ocs2::vector_t::Constant(9,0.1);u(1)=0.0;
    ocs2::PreComputation pre;
    EXPECT_TRUE(constraint->isActive(0));
    EXPECT_DOUBLE_EQ(constraint->getValue(0,x,u,pre).norm(),0.0);
    u(1)=0.1;
    EXPECT_NEAR(constraint->getValue(0,x,u,pre).norm(),0.1,1e-12);
}

TEST(StagedMotion, RangerDrivesIntoParkingBeforeFinalInPlaceAlignment) {
    StagedMotionConfig config;config.reach_radius=1.0;config.ranger_base=true;
    ReachCoordinator c(config);
    const Eigen::Vector3d target(0.2,1.2,0);
    const auto start=c.update(1.0,{0,0,0},{0.2,0,0},target);
    EXPECT_FALSE(start.aligning_at_parking);
    EXPECT_NEAR(start.parking_yaw,0.5*std::acos(-1.0),1e-12);
    EXPECT_NEAR(start.base_yaw_goal,std::atan2(0.35,0.2),1e-12);
    const auto alignment=c.update(1.1,{0.2,0.34,0.8},{0.3393413,0.4834712,0},target);
    EXPECT_EQ(alignment.phase,MotionPhase::Base);
    EXPECT_TRUE(alignment.aligning_at_parking);
    EXPECT_NEAR(alignment.base_yaw_goal,start.parking_yaw,1e-12);
    auto stage=std::make_shared<StagedMotionState>(alignment);
    auto constraint=makeStageInputConstraint(stage,true);
    ocs2::PreComputation pre;const auto x=ocs2::vector_t::Zero(9).eval();
    ocs2::vector_t u=ocs2::vector_t::Zero(9);u(2)=rangerHeadingCommand(0.0,alignment.base_yaw_goal,0.3).velocity;
    EXPECT_DOUBLE_EQ(constraint->getValue(0,x,u,pre).norm(),0.0);
    u(0)=0.1;EXPECT_GT(constraint->getValue(0,x,u,pre).norm(),0.09);
    // Move beyond 2 * parking_tolerance, rather than exactly onto its boundary.
    const auto drift=c.update(1.2,{0.2,0.29,0.8},{0.3393413,0.4334712,0},target);
    EXPECT_FALSE(drift.aligning_at_parking);
}

TEST(StagedMotion, RangerHeadingGuidanceIsBoundedAndHasCorrectJacobian) {
    for (double yaw : {-3.1, -0.1, 0.0, 0.1, 3.1}) {
        const auto command=rangerHeadingCommand(yaw,0.0,0.2);
        EXPECT_LE(std::abs(command.velocity),0.2);
        const double numeric=(rangerHeadingCommand(yaw+1e-6,0.0,0.2).velocity-command.velocity)/1e-6;
        EXPECT_NEAR(numeric,command.yaw_derivative,1e-5);
    }
    const auto wrapped=rangerHeadingCommand(std::acos(-1.0)-0.02,-std::acos(-1.0)+0.02,0.2);
    EXPECT_GT(wrapped.velocity,0.0);
    EXPECT_DOUBLE_EQ(rangerHeadingCommand(1.0,1.0,0.2).velocity,0.0);
}

TEST(StagedMotion, RangerParkingGuidanceJacobianAndSpeedLimits) {
    StagedMotionState goal;goal.base_goal=Eigen::Vector2d(0.5,0.6);goal.parking_yaw=1.0;
    for (const Eigen::Vector3d base : {Eigen::Vector3d(0,0,0.2),Eigen::Vector3d(0.45,0.52,0.8),Eigen::Vector3d(0.7,0.9,-2.0)}) {
        const auto command=rangerApproachCommand(base,goal,0.15,0.2);
        EXPECT_LE(std::abs(command.input.x()),0.15);EXPECT_LE(std::abs(command.input.z()),0.2);
        EXPECT_DOUBLE_EQ(command.input.y(),0.0);
        for (int axis=0;axis<3;++axis) {
            Eigen::Vector3d shifted=base;shifted(axis)+=1e-7;
            const Eigen::Vector3d numeric=(rangerApproachCommand(shifted,goal,0.15,0.2).input-command.input)/1e-7;
            EXPECT_LT((numeric-command.state_jacobian.col(axis)).norm(),2e-5);
        }
    }
}

TEST(StagedMotion, RangerGuidanceConvergesWithoutOrbitingTheParkingPoint) {
    StagedMotionConfig config;config.reach_radius=1.0;config.ranger_base=true;
    ReachCoordinator c(config);Eigen::Vector3d base=Eigen::Vector3d::Zero();
    const Eigen::Vector3d target(0.2,1.2,0.0);
    MotionPhase phase=MotionPhase::Base;
    for (int i=0;i<6000 && phase==MotionPhase::Base;++i) {
        const Eigen::Vector2d arm_xy=base.head<2>()+Eigen::Rotation2Dd(base.z())*Eigen::Vector2d(0.2,0);
        const auto plan=c.update(1.0+0.02*i,base,{arm_xy.x(),arm_xy.y(),0},target);
        phase=plan.phase;
        if (phase!=MotionPhase::Base) break;
        const auto input=rangerApproachCommand(base,plan,0.15,0.2).input;
        base.head<2>()+=0.02*(Eigen::Rotation2Dd(base.z())*input.head<2>());
        base.z()+=0.02*input.z();
    }
    EXPECT_EQ(phase,MotionPhase::Settling);
    EXPECT_LT((base.head<2>()-Eigen::Vector2d(0.2,0.35)).norm(),0.025);
    EXPECT_LT(std::abs(std::remainder(base.z()-0.5*std::acos(-1.0),2*std::acos(-1.0))),0.08);
}
