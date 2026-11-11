// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <ocs2_mpc/MrtObserver.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace zephyr::mpc {

struct RuntimeProfileConfig {
  static constexpr uint8_t COORDINATED = 0;
  static constexpr uint8_t LOCKED = 1;
  uint64_t revision{1};
  uint8_t base_mode{COORDINATED};
  double tcp_cost_scale{1.0};
  double input_cost_scale{1.0};
};

// Multipliers apply to configured costs; they are not velocity limits.
bool validateRuntimeProfileConfig(const RuntimeProfileConfig& config, std::string& reason);

/** One immutable configuration per solve, paired with each buffered MRT policy.
 * Register this object with both SolverBase::addSynchronizedModule() and
 * MRT_BASE::addMrtObserver() before starting the solver/control threads.
 */
class RuntimeMotionProfile final : public ocs2::SolverSynchronizedModule, public ocs2::MrtObserver {
 public:
  explicit RuntimeMotionProfile(RuntimeProfileConfig initial = {});

  // Non-RT writers only. Rejects invalid/non-increasing revisions atomically.
  bool submit(const RuntimeProfileConfig& config, std::string& reason);
  uint64_t requestedRevision() const { return requested_revision_.load(std::memory_order_acquire); }
  RuntimeProfileConfig requestedConfig() const;

  // Solver thread and its joined worker jobs only; frozen throughout each solve.
  const RuntimeProfileConfig& solverConfig() const { return solver_config_; }
  // RT thread only, after updatePolicy(). Revision 0 means no activated policy.
  const RuntimeProfileConfig& activeConfig() const { return active_config_; }

  // Solver thread, before the wrapped ReferenceManager swaps references.
  // A profile request queues its measured hold before publishing the revision;
  // freezing here prevents a newer mode being paired with an older reference.
  void captureBeforeReferenceUpdate();

  void preSolverRun(ocs2::scalar_t init_time, ocs2::scalar_t final_time,
                    const ocs2::vector_t& initial_state,
                    const ocs2::ReferenceManagerInterface& reference_manager) override;
  void postSolverRun(const ocs2::PrimalSolution&) override {}
  void modifyBufferedSolution(const ocs2::CommandData&, ocs2::PrimalSolution&) override;
  void modifyActiveSolution(const ocs2::CommandData&, ocs2::PrimalSolution&) override;

 private:
  mutable std::mutex pending_mutex_;
  RuntimeProfileConfig pending_config_;
  std::atomic<uint64_t> requested_revision_;
  RuntimeProfileConfig solver_config_;
  bool captured_before_reference_{false};  // Solver thread only.
  // Both callbacks use MRT's buffer mutex, so these mirror its policy slots.
  RuntimeProfileConfig buffered_config_;
  RuntimeProfileConfig active_config_;
};

// Decorates a copy of the upstream problem before constructing the solver.
// Named TCP/input costs retain their original implementation and default values.
void decorateProblem(ocs2::OptimalControlProblem& problem,
                     const std::shared_ptr<RuntimeMotionProfile>& runtime,
                     bool mobile, bool ranger, size_t state_dim, size_t input_dim);

}  // namespace zephyr::mpc
