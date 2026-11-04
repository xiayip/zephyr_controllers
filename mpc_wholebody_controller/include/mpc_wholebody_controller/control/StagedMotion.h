// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <limits>
#include <cmath>
#include <Eigen/Geometry>
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/logger.hpp>

namespace zephyr::mpc_wholebody {
enum class MotionPhase { Arm, Base, Assist, Settling, Unreachable };
const char* motionPhaseName(MotionPhase phase);

struct StagedMotionConfig {
    bool enabled{false};
    std::string arm_base_frame{"base_link"};
    double reach_radius{0.0}; // Zero: estimate from the current model and joint limits.
    double reach_switch_margin{0.02};
    double parking_radius_ratio{0.65};
    double parking_tolerance{0.025};
    double heading_tolerance{0.08};
    bool ranger_base{false};
    double heading_max_velocity{0.3};
    double approach_max_linear_velocity{0.15};
    double tracking_stall_seconds{2.0};
    double tracking_position_tolerance{0.02};
    double tracking_orientation_tolerance{0.12};
    double tracking_progress_threshold{0.003};
    double assist_no_progress_seconds{4.0};
    double settle_seconds{0.25};
    double stopped_linear_speed{0.01};
    double stopped_angular_speed{0.02};
};

struct StagedMotionState {
    MotionPhase phase{MotionPhase::Arm};
    double policy_start{0.0};
    double distance{0.0};
    double stop_radius{0.0};
    Eigen::Vector2d base_goal{Eigen::Vector2d::Zero()};
    double base_yaw_goal{0.0}; // Current heading used by the optimizer.
    double parking_yaw{0.0}; // Final heading, latched with the parking point.
    double drive_direction{1.0}; // Latch forward/reverse for one Ranger approach.
    bool aligning_at_parking{false};
};

std::unique_ptr<ocs2::StateInputConstraint> makeStageInputConstraint(
    std::shared_ptr<StagedMotionState> stage, bool ranger_base, double max_angular_speed = 0.3,
    double max_linear_speed = 0.15);

struct HeadingCommand { double velocity, yaw_derivative; };
HeadingCommand rangerHeadingCommand(double yaw, double goal, double max_velocity);
struct RangerApproachCommand {
    Eigen::Vector3d input{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d state_jacobian{Eigen::Matrix3d::Zero()};
};
RangerApproachCommand rangerApproachCommand(const Eigen::Vector3d& base, const StagedMotionState& goal,
                                           double max_linear_speed, double max_angular_speed);

struct TrackingFeedback {
    double position_error{std::numeric_limits<double>::quiet_NaN()};
    double orientation_error{std::numeric_limits<double>::quiet_NaN()};
    Eigen::Quaterniond target_orientation{Eigen::Quaterniond::Identity()};
    bool valid() const {
        return std::isfinite(position_error) && position_error >= 0.0 &&
               std::isfinite(orientation_error) && orientation_error >= 0.0 &&
               target_orientation.coeffs().allFinite() && target_orientation.norm() > 1e-9;
    }
};

/** Pure range/settling logic. Used only by the solver thread. */
class ReachCoordinator {
public:
    explicit ReachCoordinator(StagedMotionConfig config) : config_(std::move(config)) {}
    void reset();
    const StagedMotionState& update(double time, const Eigen::Vector3d& base_xy_yaw,
                                   const Eigen::Vector3d& arm_origin, const Eigen::Vector3d& target,
                                   const TrackingFeedback& tracking = {});
private:
    StagedMotionConfig config_;
    StagedMotionState state_;
    Eigen::Vector3d previous_base_{Eigen::Vector3d::Zero()};
    double previous_time_{-1.0};
    double stopped_since_{-1.0};
    Eigen::Vector3d target_anchor_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation_anchor_{Eigen::Quaterniond::Identity()};
    bool target_initialized_{false};
    bool assist_attempted_{false};
    double progress_since_{-1.0};
    double best_tracking_error_{std::numeric_limits<double>::infinity()};
};

/** Wrap upstream references without changing their ROS topics or target caches.
 * preSolverRun freezes phase/goal for this solve, shared read-only by all cost clones.
 */
class StagedReferenceManager final : public ocs2::ReferenceManagerDecorator {
public:
    StagedReferenceManager(std::shared_ptr<ocs2::ReferenceManagerInterface> wrapped,
                          const ocs2::PinocchioInterface& pinocchio, std::string tcp_frame,
                          StagedMotionConfig config, rclcpp::Logger logger);
    void preSolverRun(ocs2::scalar_t init_time, ocs2::scalar_t final_time,
                      const ocs2::vector_t& init_state) override;
    void augmentProblem(ocs2::OptimalControlProblem& problem) const;
    void reset(); // Caller must have stopped/waited for the solver.
    const StagedMotionState& state() const { return *solve_state_; } // Solver thread only.
    double reachRadius() const { return config_.reach_radius; }
private:
    StagedMotionConfig config_;
    ocs2::PinocchioInterface pinocchio_;
    pinocchio::FrameIndex arm_frame_;
    pinocchio::FrameIndex tcp_frame_;
    std::shared_ptr<StagedMotionState> solve_state_;
    std::unique_ptr<ReachCoordinator> coordinator_;
    rclcpp::Logger logger_;
};
} // namespace zephyr::mpc_wholebody
