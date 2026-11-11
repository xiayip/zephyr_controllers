// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/RuntimeMotionProfile.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputCost.h>

namespace zephyr::mpc {
namespace {

class ScaledStateCost final : public ocs2::StateCost {
 public:
  ScaledStateCost(std::unique_ptr<ocs2::StateCost> cost,
                  std::shared_ptr<RuntimeMotionProfile> runtime)
      : cost_(std::move(cost)), runtime_(std::move(runtime)) {}
  ScaledStateCost(const ScaledStateCost& other)
      : cost_(other.cost_->clone()), runtime_(other.runtime_) {}
  ScaledStateCost* clone() const override { return new ScaledStateCost(*this); }
  bool isActive(ocs2::scalar_t time) const override { return cost_->isActive(time); }
  ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                          const ocs2::TargetTrajectories& target,
                          const ocs2::PreComputation& pre) const override {
    return runtime_->solverConfig().tcp_cost_scale * cost_->getValue(time, state, target, pre);
  }
  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state,
      const ocs2::TargetTrajectories& target,
      const ocs2::PreComputation& pre) const override {
    auto approximation = cost_->getQuadraticApproximation(time, state, target, pre);
    approximation *= runtime_->solverConfig().tcp_cost_scale;
    return approximation;
  }
 private:
  std::unique_ptr<ocs2::StateCost> cost_;
  std::shared_ptr<RuntimeMotionProfile> runtime_;
};

class ScaledInputCost final : public ocs2::StateInputCost {
 public:
  ScaledInputCost(std::unique_ptr<ocs2::StateInputCost> cost,
                  std::shared_ptr<RuntimeMotionProfile> runtime)
      : cost_(std::move(cost)), runtime_(std::move(runtime)) {}
  ScaledInputCost(const ScaledInputCost& other)
      : cost_(other.cost_->clone()), runtime_(other.runtime_) {}
  ScaledInputCost* clone() const override { return new ScaledInputCost(*this); }
  bool isActive(ocs2::scalar_t time) const override { return cost_->isActive(time); }
  ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                          const ocs2::vector_t& input,
                          const ocs2::TargetTrajectories& target,
                          const ocs2::PreComputation& pre) const override {
    return runtime_->solverConfig().input_cost_scale * cost_->getValue(time, state, input, target, pre);
  }
  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
      const ocs2::TargetTrajectories& target,
      const ocs2::PreComputation& pre) const override {
    auto approximation = cost_->getQuadraticApproximation(time, state, input, target, pre);
    approximation *= runtime_->solverConfig().input_cost_scale;
    return approximation;
  }
 private:
  std::unique_ptr<ocs2::StateInputCost> cost_;
  std::shared_ptr<RuntimeMotionProfile> runtime_;
};

class RuntimeBaseConstraint final : public ocs2::StateInputConstraint {
 public:
  RuntimeBaseConstraint(std::shared_ptr<RuntimeMotionProfile> runtime, bool ranger,
                        size_t state_dim, size_t input_dim)
      : StateInputConstraint(ocs2::ConstraintOrder::Linear), runtime_(std::move(runtime)),
        ranger_(ranger), state_dim_(state_dim), input_dim_(input_dim) {}
  RuntimeBaseConstraint* clone() const override { return new RuntimeBaseConstraint(*this); }
  bool isActive(ocs2::scalar_t time) const override { return getNumConstraints(time) != 0; }
  size_t getNumConstraints(ocs2::scalar_t) const override {
    return runtime_->solverConfig().base_mode == RuntimeProfileConfig::LOCKED ? 3 : (ranger_ ? 1 : 0);
  }
  ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t&,
                          const ocs2::vector_t& input, const ocs2::PreComputation&) const override {
    const auto count = getNumConstraints(time);
    if (count == 3) return input.head<3>();
    if (count == 1) return ocs2::vector_t::Constant(1, input(1));
    return ocs2::vector_t(0);
  }
  ocs2::VectorFunctionLinearApproximation getLinearApproximation(
      ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
      const ocs2::PreComputation& pre) const override {
    const auto count = getNumConstraints(time);
    auto approximation = ocs2::VectorFunctionLinearApproximation::Zero(count, state_dim_, input_dim_);
    approximation.f = getValue(time, state, input, pre);
    if (count == 3) approximation.dfdu.leftCols<3>().setIdentity();
    else if (count == 1) approximation.dfdu(0, 1) = 1.0;
    return approximation;
  }
 private:
  std::shared_ptr<RuntimeMotionProfile> runtime_;
  bool ranger_;
  size_t state_dim_;
  size_t input_dim_;
};

}  // namespace

bool validateRuntimeProfileConfig(const RuntimeProfileConfig& config, std::string& reason) {
  reason.clear();
  if (config.revision == 0) reason = "profile revision must be positive";
  else if (config.base_mode != RuntimeProfileConfig::COORDINATED && config.base_mode != RuntimeProfileConfig::LOCKED)
    reason = "base_mode must be COORDINATED or LOCKED";
  else if (!std::isfinite(config.tcp_cost_scale) || config.tcp_cost_scale < 0.1 || config.tcp_cost_scale > 10.0)
    reason = "tcp_cost_scale must be finite and within [0.1, 10]";
  else if (!std::isfinite(config.input_cost_scale) || config.input_cost_scale < 0.1 || config.input_cost_scale > 10.0)
    reason = "input_cost_scale must be finite and within [0.1, 10]";
  return reason.empty();
}

RuntimeMotionProfile::RuntimeMotionProfile(RuntimeProfileConfig initial)
    : pending_config_(initial), requested_revision_(initial.revision), solver_config_(initial),
      buffered_config_(initial), active_config_(initial) {
  std::string reason;
  if (!validateRuntimeProfileConfig(initial, reason)) throw std::invalid_argument(reason);
  buffered_config_.revision = active_config_.revision = 0;
}

bool RuntimeMotionProfile::submit(const RuntimeProfileConfig& config, std::string& reason) {
  if (!validateRuntimeProfileConfig(config, reason)) return false;
  std::lock_guard<std::mutex> lock(pending_mutex_);
  if (config.revision <= pending_config_.revision) {
    reason = "profile revision must increase";
    return false;
  }
  pending_config_ = config;
  requested_revision_.store(config.revision, std::memory_order_release);
  return true;
}

RuntimeProfileConfig RuntimeMotionProfile::requestedConfig() const {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  return pending_config_;
}

void RuntimeMotionProfile::preSolverRun(ocs2::scalar_t, ocs2::scalar_t,
                                       const ocs2::vector_t&, const ocs2::ReferenceManagerInterface&) {
  if (!captured_before_reference_) captureBeforeReferenceUpdate();
  captured_before_reference_ = false;
}

void RuntimeMotionProfile::captureBeforeReferenceUpdate() {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  solver_config_ = pending_config_;
  captured_before_reference_ = true;
}

void RuntimeMotionProfile::modifyBufferedSolution(const ocs2::CommandData&, ocs2::PrimalSolution&) {
  buffered_config_ = solver_config_;
}

void RuntimeMotionProfile::modifyActiveSolution(const ocs2::CommandData&, ocs2::PrimalSolution&) {
  active_config_ = buffered_config_;
}

void decorateProblem(ocs2::OptimalControlProblem& problem,
                     const std::shared_ptr<RuntimeMotionProfile>& runtime,
                     bool mobile, bool ranger, size_t state_dim, size_t input_dim) {
  if (!runtime) throw std::invalid_argument("runtime profile is required");
  if (mobile && (state_dim < 3 || input_dim < 3))
    throw std::invalid_argument("Mobile profile requires planar base state and input dimensions");
  // Validate all expected terms before mutating the copied problem.
  problem.stateSoftConstraintPtr->get("endEffector");
  problem.finalSoftConstraintPtr->get("finalEndEffector");
  problem.costPtr->get("inputCost");
  auto tcp = problem.stateSoftConstraintPtr->extract("endEffector");
  auto final_tcp = problem.finalSoftConstraintPtr->extract("finalEndEffector");
  auto input = problem.costPtr->extract("inputCost");
  problem.stateSoftConstraintPtr->add("endEffector", std::make_unique<ScaledStateCost>(std::move(tcp), runtime));
  problem.finalSoftConstraintPtr->add("finalEndEffector", std::make_unique<ScaledStateCost>(std::move(final_tcp), runtime));
  problem.costPtr->add("inputCost", std::make_unique<ScaledInputCost>(std::move(input), runtime));
  if (mobile) {
    // Replace our former Ranger-only equality; never duplicate its vy row.
    problem.equalityConstraintPtr->erase("baseNoLateralVelocity");
    problem.equalityConstraintPtr->add("runtimeBaseMotion",
        std::make_unique<RuntimeBaseConstraint>(runtime, ranger, state_dim, input_dim));
  }
}

}  // namespace zephyr::mpc
