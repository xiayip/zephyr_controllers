// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <Eigen/LU>
#include <limits>
#include <memory>
#include <thread>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/MRT_BASE.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "mpc_controller/control/BaseKinematics.h"
#include "mpc_controller/control/RuntimeMotionProfile.h"

namespace {
using namespace zephyr::mpc;
using namespace ocs2;

OptimalControlProblem makeProblem(size_t dimension = 4) {
  OptimalControlProblem problem;
  const auto identity = matrix_t::Identity(dimension, dimension).eval();
  problem.stateSoftConstraintPtr->add("endEffector", std::make_unique<QuadraticStateCost>(2.0 * identity));
  problem.finalSoftConstraintPtr->add("finalEndEffector", std::make_unique<QuadraticStateCost>(5.0 * identity));
  problem.costPtr->add("inputCost", std::make_unique<QuadraticStateInputCost>(
      0.2 * identity, 3.0 * identity, 0.1 * identity));
  problem.dynamicsPtr = std::make_unique<LinearSystemDynamics>(matrix_t::Zero(dimension, dimension), identity);
  return problem;
}

void synchronize(const std::shared_ptr<RuntimeMotionProfile>& runtime) {
  ReferenceManager reference;
  runtime->preSolverRun(0.0, 1.0, vector_t::Zero(4), reference);
}

void expectScaled(const ScalarFunctionQuadraticApproximation& scaled,
                  const ScalarFunctionQuadraticApproximation& original, double scale) {
  EXPECT_NEAR(scaled.f, scale * original.f, 1e-12);
  EXPECT_TRUE(scaled.dfdx.isApprox(scale * original.dfdx));
  EXPECT_TRUE(scaled.dfdu.isApprox(scale * original.dfdu));
  EXPECT_TRUE(scaled.dfdxx.isApprox(scale * original.dfdxx));
  EXPECT_TRUE(scaled.dfdux.isApprox(scale * original.dfdux));
  EXPECT_TRUE(scaled.dfduu.isApprox(scale * original.dfduu));
}

// Exercise the actual MRT buffer and callback locking without needing ROS.
class MrtHarness final : public MRT_BASE {
 public:
  void resetMpcNode(const TargetTrajectories&) override {}
  void setCurrentObservation(const SystemObservation&) override {}
  void publish(double marker) {
    auto command = std::make_unique<CommandData>();
    command->mpcInitObservation_.time = marker;
    auto policy = std::make_unique<PrimalSolution>();
    policy->timeTrajectory_ = {marker, marker + 0.1};
    policy->stateTrajectory_ = {vector_t::Constant(4, marker), vector_t::Constant(4, marker)};
    policy->inputTrajectory_ = {vector_t::Zero(4), vector_t::Zero(4)};
    moveToBuffer(std::move(command), std::move(policy), std::make_unique<PerformanceIndex>());
  }
};

TEST(RuntimeMotionProfile, ValidatesBoundsWithoutChangingRequestedOrSolverState) {
  RuntimeMotionProfile runtime;
  EXPECT_EQ(runtime.requestedRevision(), 1U);
  EXPECT_EQ(runtime.activeConfig().revision, 0U);
  std::string reason;
  RuntimeProfileConfig bad;
  bad.revision = 2;
  for (double value : {0.0, -1.0, 0.09, 10.01, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
    bad.tcp_cost_scale = value;
    EXPECT_FALSE(runtime.submit(bad, reason));
    EXPECT_FALSE(reason.empty());
    bad.tcp_cost_scale = 1.0;
    bad.input_cost_scale = value;
    EXPECT_FALSE(runtime.submit(bad, reason));
    bad.input_cost_scale = 1.0;
  }
  bad.base_mode = 7;
  EXPECT_FALSE(runtime.submit(bad, reason));
  bad.base_mode = RuntimeProfileConfig::LOCKED;
  bad.revision = 0;
  EXPECT_FALSE(runtime.submit(bad, reason));
  bad.revision = 1;
  EXPECT_FALSE(runtime.submit(bad, reason));
  EXPECT_EQ(runtime.requestedRevision(), 1U);
  EXPECT_EQ(runtime.requestedConfig().base_mode, RuntimeProfileConfig::COORDINATED);
  EXPECT_EQ(runtime.solverConfig().revision, 1U);
  bad.revision = 2;
  bad.tcp_cost_scale = 0.1;
  bad.input_cost_scale = 10.0;
  EXPECT_TRUE(runtime.submit(bad, reason));
  EXPECT_EQ(runtime.requestedRevision(), 2U);
  EXPECT_EQ(runtime.solverConfig().revision, 1U);
  bad.tcp_cost_scale = 0.0;
  EXPECT_THROW(RuntimeMotionProfile invalid(bad), std::invalid_argument);
}

TEST(RuntimeMotionProfile, DefaultCostsAndAllDerivativesMatchAndClonesShareFrozenProfile) {
  const auto original = makeProblem();
  auto problem = original;
  const auto runtime = std::make_shared<RuntimeMotionProfile>();
  decorateProblem(problem, runtime, true, true, 4, 4);
  auto solver_copy = problem;
  const auto x = vector_t::Constant(4, 0.4).eval();
  const auto u = vector_t::Constant(4, -0.2).eval();
  const TargetTrajectories target({0.0}, {vector_t::Zero(4)}, {vector_t::Constant(4, 0.1)});
  const PreComputation pre;
  const auto base_tcp = original.stateSoftConstraintPtr->getQuadraticApproximation(0, x, target, pre);
  const auto base_terminal = original.finalSoftConstraintPtr->getQuadraticApproximation(0, x, target, pre);
  const auto base_input = original.costPtr->getQuadraticApproximation(0, x, u, target, pre);
  for (auto* copy : {&problem, &solver_copy}) {
    expectScaled(copy->stateSoftConstraintPtr->getQuadraticApproximation(0, x, target, pre), base_tcp, 1.0);
    expectScaled(copy->finalSoftConstraintPtr->getQuadraticApproximation(0, x, target, pre), base_terminal, 1.0);
    expectScaled(copy->costPtr->getQuadraticApproximation(0, x, u, target, pre), base_input, 1.0);
  }
  RuntimeProfileConfig update{2, RuntimeProfileConfig::LOCKED, 3.0, 4.0};
  std::string reason;
  ASSERT_TRUE(runtime->submit(update, reason));
  // Submission cannot mutate an in-flight solve or any of its clones.
  expectScaled(solver_copy.stateSoftConstraintPtr->getQuadraticApproximation(0, x, target, pre), base_tcp, 1.0);
  synchronize(runtime);
  for (auto* copy : {&problem, &solver_copy}) {
    expectScaled(copy->stateSoftConstraintPtr->getQuadraticApproximation(0, x, target, pre), base_tcp, 3.0);
    expectScaled(copy->finalSoftConstraintPtr->getQuadraticApproximation(0, x, target, pre), base_terminal, 3.0);
    expectScaled(copy->costPtr->getQuadraticApproximation(0, x, u, target, pre), base_input, 4.0);
    EXPECT_DOUBLE_EQ(copy->stateSoftConstraintPtr->getValue(0, x, target, pre), 3.0 * base_tcp.f);
    EXPECT_DOUBLE_EQ(copy->costPtr->getValue(0, x, u, target, pre), 4.0 * base_input.f);
  }
}

TEST(RuntimeMotionProfile, RequestBetweenReferenceSwapAndModuleCallbackWaitsForNextSolve) {
  const auto runtime = std::make_shared<RuntimeMotionProfile>();
  runtime->captureBeforeReferenceUpdate();
  // The ReferenceManager is about to acquire its buffered target under rev1.
  std::string reason;
  ASSERT_TRUE(runtime->submit({2, RuntimeProfileConfig::LOCKED, 2.0, 3.0}, reason));
  synchronize(runtime);
  EXPECT_EQ(runtime->solverConfig().revision, 1U);
  EXPECT_EQ(runtime->solverConfig().base_mode, RuntimeProfileConfig::COORDINATED);
  EXPECT_EQ(runtime->requestedRevision(), 2U);
  runtime->captureBeforeReferenceUpdate();
  synchronize(runtime);
  EXPECT_EQ(runtime->solverConfig().revision, 2U);
  EXPECT_EQ(runtime->solverConfig().base_mode, RuntimeProfileConfig::LOCKED);
  EXPECT_DOUBLE_EQ(runtime->solverConfig().tcp_cost_scale, 2.0);
  // Standalone use without a ReferenceManager hook still snapshots normally.
  ASSERT_TRUE(runtime->submit({3, RuntimeProfileConfig::COORDINATED, 1.0, 1.0}, reason));
  synchronize(runtime);
  EXPECT_EQ(runtime->solverConfig().revision, 3U);
}

TEST(RuntimeMotionProfile, LockHasFullRankAndDoesNotDuplicateRangerLateralConstraint) {
  for (const bool ranger : {false, true}) {
    auto problem = makeProblem();
    if (ranger) problem.equalityConstraintPtr->add("baseNoLateralVelocity", makeNoLateralVelocityConstraint(4, 4));
    const auto runtime = std::make_shared<RuntimeMotionProfile>();
    decorateProblem(problem, runtime, true, ranger, 4, 4);
    auto clone = problem;
    auto& constraint = clone.equalityConstraintPtr->get("runtimeBaseMotion");
    EXPECT_EQ(constraint.getNumConstraints(0), ranger ? 1U : 0U);
    EXPECT_EQ(constraint.isActive(0), ranger);
    const auto state = vector_t::Zero(4).eval();
    const auto input = (vector_t(4) << 0.2, 0.1, 0.3, 0.4).finished();
    const PreComputation pre;
    std::string reason;
    ASSERT_TRUE(runtime->submit({2, RuntimeProfileConfig::LOCKED, 1.0, 1.0}, reason));
    synchronize(runtime);
    ASSERT_EQ(constraint.getNumConstraints(0), 3U);
    const auto approximation = constraint.getLinearApproximation(0, state, input, pre);
    EXPECT_EQ(approximation.dfdu.fullPivLu().rank(), 3);
    EXPECT_TRUE(approximation.dfdx.isZero());
    EXPECT_TRUE(approximation.f.isApprox(input.head<3>()));
    EXPECT_TRUE(approximation.dfdu.rightCols(1).isZero());
    size_t index = 0;
    EXPECT_FALSE(clone.equalityConstraintPtr->getTermIndex("baseNoLateralVelocity", index));
    ASSERT_TRUE(runtime->submit({3, RuntimeProfileConfig::COORDINATED, 1.0, 1.0}, reason));
    synchronize(runtime);
    EXPECT_EQ(constraint.getNumConstraints(0), ranger ? 1U : 0U);
  }
}

TEST(RuntimeMotionProfile, FixedBaseDoesNotConstrainArmInputsAndInvalidMobileDimensionsReject) {
  auto problem = makeProblem(2);
  const auto runtime = std::make_shared<RuntimeMotionProfile>(RuntimeProfileConfig{1, RuntimeProfileConfig::LOCKED, 1, 1});
  decorateProblem(problem, runtime, false, true, 2, 2);
  EXPECT_TRUE(problem.equalityConstraintPtr->empty());
  auto invalid = makeProblem(2);
  EXPECT_THROW(decorateProblem(invalid, runtime, true, true, 2, 2), std::invalid_argument);
  EXPECT_THROW(decorateProblem(invalid, nullptr, false, false, 2, 2), std::invalid_argument);
}

TEST(RuntimeMotionProfile, MrtPairsLatestBufferedPolicyAndNeverLabelsOldPolicyWithNewRequest) {
  const auto runtime = std::make_shared<RuntimeMotionProfile>();
  MrtHarness mrt;
  mrt.addMrtObserver(runtime);
  synchronize(runtime);
  mrt.publish(1.0);
  ASSERT_TRUE(mrt.updatePolicy());
  EXPECT_EQ(runtime->activeConfig().revision, 1U);
  std::string reason;
  ASSERT_TRUE(runtime->submit({2, RuntimeProfileConfig::LOCKED, 2, 3}, reason));
  synchronize(runtime);
  mrt.publish(2.0);
  EXPECT_EQ(runtime->activeConfig().revision, 1U);
  ASSERT_TRUE(runtime->submit({3, RuntimeProfileConfig::COORDINATED, 3, 4}, reason));
  synchronize(runtime);
  mrt.publish(3.0);  // New solve replaces the previous buffered policy.
  ASSERT_TRUE(runtime->submit({4, RuntimeProfileConfig::LOCKED, 4, 5}, reason));
  synchronize(runtime);  // A new solve starts, but has not published its result.
  ASSERT_TRUE(mrt.updatePolicy());
  EXPECT_EQ(mrt.getCommand().mpcInitObservation_.time, 3.0);
  EXPECT_EQ(runtime->activeConfig().revision, 3U);
  EXPECT_EQ(runtime->activeConfig().base_mode, RuntimeProfileConfig::COORDINATED);
  EXPECT_DOUBLE_EQ(runtime->activeConfig().tcp_cost_scale, 3.0);
  EXPECT_EQ(runtime->requestedRevision(), 4U);
  EXPECT_FALSE(mrt.updatePolicy());
  EXPECT_EQ(runtime->activeConfig().revision, 3U);
  mrt.publish(4.0);
  ASSERT_TRUE(mrt.updatePolicy());
  EXPECT_EQ(runtime->activeConfig().revision, 4U);
  EXPECT_EQ(runtime->activeConfig().base_mode, RuntimeProfileConfig::LOCKED);
}

TEST(RuntimeMotionProfile, ConcurrentMrtBufferOverwritePreservesPolicyRevisionPair) {
  const auto runtime = std::make_shared<RuntimeMotionProfile>();
  MrtHarness mrt;
  mrt.addMrtObserver(runtime);
  std::atomic<bool> done{false};
  std::atomic<bool> writes_valid{true};
  std::thread writer([&] {
    std::string reason;
    for (uint64_t revision = 2; revision <= 200; ++revision) {
      if (!runtime->submit({revision, static_cast<uint8_t>(revision % 2), 1, 1}, reason)) writes_valid = false;
      synchronize(runtime);
      mrt.publish(static_cast<double>(revision));
    }
    done = true;
  });
  while (!done.load()) {
    if (mrt.updatePolicy()) {
      EXPECT_DOUBLE_EQ(mrt.getCommand().mpcInitObservation_.time, runtime->activeConfig().revision);
      EXPECT_EQ(runtime->activeConfig().base_mode, runtime->activeConfig().revision % 2);
    }
    std::this_thread::yield();
  }
  writer.join();
  mrt.updatePolicy();
  EXPECT_TRUE(writes_valid);
  EXPECT_EQ(runtime->activeConfig().revision, 200U);
  EXPECT_DOUBLE_EQ(mrt.getCommand().mpcInitObservation_.time, 200.0);
}

TEST(RuntimeMotionProfile, RealMpcUsesNewConstraintAndCostWithoutReconstructingController) {
  auto problem = makeProblem();
  const auto runtime = std::make_shared<RuntimeMotionProfile>();
  decorateProblem(problem, runtime, true, true, 4, 4);
  rollout::Settings rollout_settings;
  rollout_settings.timeStep = 0.01;
  TimeTriggeredRollout rollout(*problem.dynamicsPtr, rollout_settings);
  DefaultInitializer initializer(4);
  ocs2::mpc::Settings mpc_settings;
  mpc_settings.timeHorizon_ = 0.5;
  mpc_settings.solutionTimeWindow_ = 0.2;
  ddp::Settings ddp_settings;
  ddp_settings.nThreads_ = 1;
  ddp_settings.threadPriority_ = 0;
  ddp_settings.maxNumIterations_ = 10;
  ddp_settings.displayInfo_ = false;
  ddp_settings.displayShortSummary_ = false;
  ddp_settings.checkNumericalStability_ = false;
  GaussNewtonDDP_MPC mpc(mpc_settings, ddp_settings, rollout, problem, initializer);
  auto desired = (vector_t(4) << 0.5, 0.0, 0.2, 1.0).finished();
  const TargetTrajectories target({0.0}, {desired}, {vector_t::Zero(4)});
  const auto references = std::make_shared<ReferenceManager>(target);
  mpc.getSolverPtr()->setReferenceManager(references);
  mpc.getSolverPtr()->addSynchronizedModule(runtime);
  MPC_MRT_Interface mrt(mpc);
  mrt.addMrtObserver(runtime);
  SystemObservation observation;
  observation.time = 0.0;
  observation.state = vector_t::Zero(4);
  observation.input = vector_t::Zero(4);
  mrt.setCurrentObservation(observation);
  ASSERT_NO_THROW(mrt.advanceMpc());
  ASSERT_TRUE(mrt.updatePolicy());
  EXPECT_EQ(runtime->activeConfig().revision, 1U);
  ASSERT_FALSE(mrt.getPolicy().inputTrajectory_.empty());
  EXPECT_GT(std::abs(mrt.getPolicy().inputTrajectory_.front()(0)), 0.01);
  std::string reason;
  ASSERT_TRUE(runtime->submit({2, RuntimeProfileConfig::LOCKED, 2.0, 1.0}, reason));
  observation.time = 0.02;
  mrt.setCurrentObservation(observation);
  ASSERT_NO_THROW(mrt.advanceMpc());
  EXPECT_EQ(runtime->activeConfig().revision, 1U);
  ASSERT_TRUE(mrt.updatePolicy());
  EXPECT_EQ(runtime->activeConfig().revision, 2U);
  ASSERT_FALSE(mrt.getPolicy().inputTrajectory_.empty());
  for (const auto& input : mrt.getPolicy().inputTrajectory_) EXPECT_LT(input.head<3>().norm(), 1e-5);
  EXPECT_GT(std::abs(mrt.getPolicy().inputTrajectory_.front()(3)), 0.01);
}

}  // namespace
