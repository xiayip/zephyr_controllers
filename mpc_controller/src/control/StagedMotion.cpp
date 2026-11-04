// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/StagedMotion.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateCost.h>
#include <pinocchio/algorithm/frames.hpp>
#include <rclcpp/logging.hpp>

namespace zephyr::mpc {
using namespace ocs2;

const char* motionPhaseName(MotionPhase phase) {
    switch (phase) {
        case MotionPhase::Arm: return "arm_tracking";
        case MotionPhase::Base: return "base_approach";
        case MotionPhase::Assist: return "whole_body_tracking";
        case MotionPhase::Settling: return "base_settling";
        case MotionPhase::Unreachable: return "unreachable_height";
    }
    return "unknown";
}

void ReachCoordinator::reset() {
    state_ = StagedMotionState{};
    previous_time_ = stopped_since_ = progress_since_ = -1.0;
    target_initialized_ = assist_attempted_ = false;
    best_tracking_error_ = std::numeric_limits<double>::infinity();
}

const StagedMotionState& ReachCoordinator::update(
    double time, const Eigen::Vector3d& base, const Eigen::Vector3d& arm_origin,
    const Eigen::Vector3d& target, const TrackingFeedback& tracking) {
    const Eigen::Vector3d offset = target - arm_origin;
    const double radius = config_.reach_radius;
    const auto previous_phase = state_.phase;
    state_.policy_start = time;
    state_.distance = offset.norm();
    // Parking is an approach destination, never a condition for ordinary arm tracking.
    const double desired_radius = std::min(radius - 0.005,
        std::max(config_.parking_radius_ratio * radius, std::hypot(offset.z(), 0.20)));
    state_.stop_radius = desired_radius;

    const double dt = time - previous_time_;
    const bool velocity_valid = previous_time_ >= 0.0 && dt > 1e-5 && dt < 0.5;
    const bool stopped = velocity_valid &&
        (base.head<2>() - previous_base_.head<2>()).norm() / dt <= config_.stopped_linear_speed &&
        std::abs(std::remainder(base.z() - previous_base_.z(), 2.0 * std::acos(-1.0))) / dt
            <= config_.stopped_angular_speed;
    if (!velocity_valid) progress_since_ = -1.0;
    previous_time_ = time;
    previous_base_ = base;

    if (!offset.allFinite() || !base.allFinite() || std::abs(offset.z()) >= radius - 0.005) {
        state_.phase = MotionPhase::Unreachable;
        stopped_since_ = progress_since_ = -1.0;
        return state_;
    }
    const Eigen::Quaterniond orientation = tracking.valid()
        ? tracking.target_orientation.normalized() : Eigen::Quaterniond::Identity();
    const bool changed = !target_initialized_ || (target - target_anchor_).norm() > 0.03 ||
        orientation.angularDistance(orientation_anchor_) > 0.15;
    if (changed) {
        target_anchor_ = target;
        orientation_anchor_ = orientation;
        target_initialized_ = true;
        assist_attempted_ = false;
        progress_since_ = -1.0;
        best_tracking_error_ = std::numeric_limits<double>::infinity();
    }
    if (changed && previous_phase == MotionPhase::Assist)
        state_.phase = state_.distance > radius + config_.reach_switch_margin ? MotionPhase::Base : MotionPhase::Settling;
    if (state_.phase == MotionPhase::Unreachable)
        state_.phase = state_.distance > radius ? MotionPhase::Base : MotionPhase::Settling;
    if ((state_.phase == MotionPhase::Arm || state_.phase == MotionPhase::Settling) &&
        state_.distance > radius + config_.reach_switch_margin)
        state_.phase = MotionPhase::Base;

    // A newly selected near target cancels an old approach; entering the sphere
    // through chassis motion alone does not cancel the closer parking destination.
    if (state_.phase == MotionPhase::Base && previous_phase == MotionPhase::Base &&
        changed && state_.distance <= radius)
        state_.phase = MotionPhase::Settling;
    if (state_.phase == MotionPhase::Base) {
        if (previous_phase != MotionPhase::Base || changed) {
            const double planar_radius = std::sqrt(std::max(0.0,
                desired_radius * desired_radius - offset.z() * offset.z()));
            const double planar_distance = offset.head<2>().norm();
            state_.base_yaw_goal = base.z();
            if (planar_distance > 1e-6)
                state_.base_yaw_goal += std::remainder(std::atan2(offset.y(), offset.x()) - base.z(), 2.0 * std::acos(-1.0));
            // Rotate the actual mount offset with the chassis. Otherwise turning
            // would shift the arm away from the intended parking circle.
            const Eigen::Vector2d mount = Eigen::Rotation2Dd(-base.z()) * (arm_origin.head<2>() - base.head<2>());
            const Eigen::Vector2d forward(std::cos(state_.base_yaw_goal), std::sin(state_.base_yaw_goal));
            state_.base_goal = target.head<2>() - planar_radius * forward -
                Eigen::Rotation2Dd(state_.base_yaw_goal) * mount;
            state_.parking_yaw = state_.base_yaw_goal;
            const Eigen::Vector2d travel = state_.base_goal - base.head<2>();
            state_.drive_direction = Eigen::Vector2d(std::cos(base.z()), std::sin(base.z())).dot(travel) < 0.0 ? -1.0 : 1.0;
            state_.aligning_at_parking = false;
        }
        const Eigen::Vector2d to_parking = state_.base_goal - base.head<2>();
        const double parking_distance = to_parking.norm();
        state_.base_yaw_goal = state_.parking_yaw;
        if (config_.ranger_base) {
            // A no-slip chassis cannot remove a lateral position residual while
            // also keeping its final heading. Drive into the point first, then
            // hold translation and finish the heading with an in-place turn.
            if (parking_distance <= 0.6 * config_.parking_tolerance) state_.aligning_at_parking = true;
            else if (parking_distance > 2.0 * config_.parking_tolerance) state_.aligning_at_parking = false;
            if (!state_.aligning_at_parking && parking_distance > 1e-6) {
                const double turn = std::remainder(std::atan2(to_parking.y(), to_parking.x()) +
                    (state_.drive_direction < 0.0 ? std::acos(-1.0) : 0.0) - base.z(), 2.0 * std::acos(-1.0));
                state_.base_yaw_goal = base.z() + turn;
            }
        }
        if (parking_distance <= config_.parking_tolerance &&
            std::abs(std::remainder(base.z() - state_.parking_yaw, 2.0 * std::acos(-1.0))) <= config_.heading_tolerance)
            state_.phase = MotionPhase::Settling;
    }

    if ((state_.phase == MotionPhase::Arm || state_.phase == MotionPhase::Assist) && tracking.valid()) {
        const bool reached = tracking.position_error <= config_.tracking_position_tolerance &&
            tracking.orientation_error <= config_.tracking_orientation_tolerance;
        const double score = std::hypot(tracking.position_error, 0.1 * tracking.orientation_error);
        if (reached) {
            progress_since_ = -1.0;
            best_tracking_error_ = score;
            if (state_.phase == MotionPhase::Assist) state_.phase = MotionPhase::Settling;
            else assist_attempted_ = false;
        } else if (progress_since_ < 0.0 || score < best_tracking_error_ - config_.tracking_progress_threshold) {
            best_tracking_error_ = score;
            progress_since_ = time;
        } else if (state_.phase == MotionPhase::Arm && !assist_attempted_ &&
                   time - progress_since_ >= config_.tracking_stall_seconds) {
            state_.phase = MotionPhase::Assist;
            assist_attempted_ = true;
            progress_since_ = time;
        } else if (state_.phase == MotionPhase::Assist &&
                   time - progress_since_ >= config_.assist_no_progress_seconds) {
            // Do not keep moving indefinitely for an infeasible fixed pose.
            state_.phase = MotionPhase::Settling;
        }
    } else progress_since_ = -1.0;

    if (state_.phase == MotionPhase::Settling) {
        if (previous_phase != MotionPhase::Settling || !stopped) stopped_since_ = -1.0;
        else if (stopped_since_ < 0.0) stopped_since_ = time;
        else if (time - stopped_since_ >= config_.settle_seconds) state_.phase = MotionPhase::Arm;
    } else stopped_since_ = -1.0;
    return state_;
}

namespace {
double halton(size_t index, unsigned base) {
    double value = 0.0, scale = 1.0;
    while (index) { scale /= base; value += scale * (index % base); index /= base; }
    return value;
}

// Position envelope estimate, not an orientation/collision-aware reachability certificate.
double estimateRadius(PinocchioInterface& pin, pinocchio::FrameIndex origin, pinocchio::FrameIndex tcp) {
    const auto& model = pin.getModel();
    auto& data = pin.getData();
    if (model.nq != 9) throw std::runtime_error("Automatic reach estimate expects planar base + six arm joints");
    vector_t q = vector_t::Zero(model.nq), best = q;
    const auto distance = [&] {
        pinocchio::framesForwardKinematics(model, data, q);
        return (data.oMf[tcp].translation() - data.oMf[origin].translation()).norm();
    };
    constexpr unsigned primes[] = {2, 3, 5, 7, 11, 13};
    double radius = 0.0;
    for (int i = 3; i < model.nq; ++i) {
        if (!std::isfinite(model.lowerPositionLimit[i]) || !std::isfinite(model.upperPositionLimit[i]) ||
            model.lowerPositionLimit[i] >= model.upperPositionLimit[i])
            throw std::runtime_error("Finite arm joint limits required for automatic reach estimate");
    }
    for (size_t sample = 1; sample <= 4096; ++sample) {
        for (int i = 3; i < model.nq; ++i)
            q[i] = model.lowerPositionLimit[i] + halton(sample, primes[i - 3]) *
                (model.upperPositionLimit[i] - model.lowerPositionLimit[i]);
        const double candidate = distance();
        if (candidate > radius) { radius = candidate; best = q; }
    }
    // Refine the longest sampled posture deterministically, within URDF limits.
    double step = 0.25;
    for (int pass = 0; pass < 32; ++pass) {
        bool improved = false;
        for (int i = 3; i < model.nq; ++i) for (double sign : {-1.0, 1.0}) {
            q = best;
            q[i] = std::clamp(q[i] + sign * step, model.lowerPositionLimit[i], model.upperPositionLimit[i]);
            const double candidate = distance();
            if (candidate > radius) { radius = candidate; best = q; improved = true; }
        }
        if (!improved) step *= 0.5;
    }
    return radius;
}

// Exact input equalities are visible to the optimizer and its exported prediction.
class StageInputConstraint final : public StateInputConstraint {
public:
    StageInputConstraint(std::shared_ptr<StagedMotionState> stage, bool ranger_base, double max_angular_speed, double max_linear_speed)
        : StateInputConstraint(ConstraintOrder::Linear), stage_(std::move(stage)), ranger_base_(ranger_base),
          max_angular_speed_(max_angular_speed), max_linear_speed_(max_linear_speed) {}
    StageInputConstraint* clone() const override { return new StageInputConstraint(*this); }
    bool isActive(scalar_t) const override { return stage_->phase != MotionPhase::Assist || ranger_base_; }
    size_t getNumConstraints(scalar_t) const override {
        if (stage_->phase == MotionPhase::Assist) return ranger_base_ ? 1 : 0;
        if (stage_->phase == MotionPhase::Arm) return 3; // vx, vy, wz
        if (stage_->phase == MotionPhase::Base) return ranger_base_ ? 9 : 7;
        return 9; // Settle or impossible height: hold everything.
    }
    vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) const override {
        const auto count = static_cast<Eigen::Index>(getNumConstraints(time));
        if (stage_->phase == MotionPhase::Assist) return vector_t::Constant(count, input(1));
        if (stage_->phase == MotionPhase::Base) {
            vector_t value(count);
            value.tail(6) = input.tail(6);
            if (ranger_base_)
                value.head<3>() = input.head<3>() - rangerApproachCommand(state.head<3>(), *stage_,
                    max_linear_speed_, max_angular_speed_).input;
            else
                value(0) = input(2) - rangerHeadingCommand(state(2), stage_->parking_yaw, max_angular_speed_).velocity;
            return value;
        }
        return input.head(count);
    }
    VectorFunctionLinearApproximation getLinearApproximation(
        scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& pre) const override {
        const auto count = static_cast<int>(getNumConstraints(time));
        auto result = VectorFunctionLinearApproximation::Zero(count, state.size(), input.size());
        result.f = getValue(time, state, input, pre);
        if (stage_->phase == MotionPhase::Assist) {
            if (ranger_base_) result.dfdu(0, 1) = 1.0;
        } else if (stage_->phase == MotionPhase::Base) {
            result.dfdu.bottomRightCorner(6, 6).setIdentity();
            if (ranger_base_) {
                result.dfdu.topLeftCorner(3, 3).setIdentity();
                result.dfdx.topLeftCorner(3, 3) = -rangerApproachCommand(state.head<3>(), *stage_,
                    max_linear_speed_, max_angular_speed_).state_jacobian;
            } else {
                result.dfdu(0, 2) = 1.0;
                result.dfdx(0, 2) = -rangerHeadingCommand(state(2), stage_->parking_yaw, max_angular_speed_).yaw_derivative;
            }
        } else result.dfdu.topLeftCorner(count, count).setIdentity();
        return result;
    }
private:
    std::shared_ptr<StagedMotionState> stage_;
    bool ranger_base_;
    double max_angular_speed_;
    double max_linear_speed_;
};

class ArmStageCost final : public StateCost {
public:
    ArmStageCost(std::unique_ptr<StateCost> wrapped, std::shared_ptr<StagedMotionState> stage)
        : wrapped_(std::move(wrapped)), stage_(std::move(stage)) {}
    ArmStageCost(const ArmStageCost& other) : wrapped_(other.wrapped_->clone()), stage_(other.stage_) {}
    ArmStageCost* clone() const override { return new ArmStageCost(*this); }
    bool isActive(scalar_t time) const override { return (stage_->phase == MotionPhase::Arm || stage_->phase == MotionPhase::Assist) && wrapped_->isActive(time); }
    scalar_t getValue(scalar_t t, const vector_t& x, const TargetTrajectories& target, const PreComputation& pre) const override {
        return wrapped_->getValue(t, x, target, pre);
    }
    ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        scalar_t t, const vector_t& x, const TargetTrajectories& target, const PreComputation& pre) const override {
        return wrapped_->getQuadraticApproximation(t, x, target, pre);
    }
private:
    std::unique_ptr<StateCost> wrapped_;
    std::shared_ptr<StagedMotionState> stage_;
};

class BaseApproachCost final : public StateCost {
public:
    explicit BaseApproachCost(std::shared_ptr<StagedMotionState> stage) : stage_(std::move(stage)) {}
    BaseApproachCost* clone() const override { return new BaseApproachCost(*this); }
    bool isActive(scalar_t) const override { return stage_->phase == MotionPhase::Base; }
    scalar_t getValue(scalar_t, const vector_t& x, const TargetTrajectories&, const PreComputation&) const override {
        const double yaw_error = std::remainder(x(2) - stage_->base_yaw_goal, 2.0 * std::acos(-1.0));
        return 0.5 * weight_ * (x.head<2>() - stage_->base_goal).squaredNorm() +
               0.5 * yaw_weight_ * yaw_error * yaw_error;
    }
    ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        scalar_t t, const vector_t& x, const TargetTrajectories& target, const PreComputation& pre) const override {
        auto result = ScalarFunctionQuadraticApproximation::Zero(x.size());
        result.f = getValue(t, x, target, pre);
        result.dfdx.head<2>() = weight_ * (x.head<2>() - stage_->base_goal);
        result.dfdxx.topLeftCorner<2, 2>().diagonal().setConstant(weight_);
        result.dfdx(2) = yaw_weight_ * std::remainder(x(2) - stage_->base_yaw_goal, 2.0 * std::acos(-1.0));
        result.dfdxx(2, 2) = yaw_weight_;
        return result;
    }
private:
    std::shared_ptr<StagedMotionState> stage_;
    static constexpr double weight_ = 50.0;
    static constexpr double yaw_weight_ = 1.0;
};
} // namespace

HeadingCommand rangerHeadingCommand(double yaw, double goal, double max_velocity) {
    const double error = std::remainder(goal - yaw, 2.0 * std::acos(-1.0));
    // A C1 dead zone avoids chasing tiny heading residuals without a discontinuity
    // in the equality constraint's derivative. Arrival tolerance is larger (0.08).
    constexpr double deadband = 0.02;
    const double excess = std::abs(error) - deadband;
    if (excess <= 0.0) return {0.0, 0.0};
    const double denominator = excess + deadband;
    const double effective_error = std::copysign(excess * excess / denominator, error);
    const double slope = excess * (excess + 2.0 * deadband) / (denominator * denominator);
    const double shaped = std::tanh(1.5 * effective_error / max_velocity);
    return {max_velocity * shaped, -1.5 * slope * (1.0 - shaped * shaped)};
}

RangerApproachCommand rangerApproachCommand(const Eigen::Vector3d& base, const StagedMotionState& goal,
                                           double max_linear_speed, double max_angular_speed) {
    RangerApproachCommand command;
    const Eigen::Vector2d delta = goal.base_goal - base.head<2>();
    const double squared_distance = delta.squaredNorm();
    if (goal.aligning_at_parking || squared_distance < 1e-10) {
        const auto heading = rangerHeadingCommand(base.z(), goal.parking_yaw, max_angular_speed);
        command.input.z() = heading.velocity;
        command.state_jacobian(2, 2) = heading.yaw_derivative;
        return command;
    }
    const double angle = std::remainder(std::atan2(delta.y(), delta.x()) +
        (goal.drive_direction < 0.0 ? std::acos(-1.0) : 0.0) - base.z(), 2.0 * std::acos(-1.0));
    const Eigen::Vector3d angle_gradient(delta.y()/squared_distance, -delta.x()/squared_distance, -1.0);
    const auto heading = rangerHeadingCommand(base.z(), base.z() + angle, max_angular_speed);
    command.input.z() = heading.velocity;
    command.state_jacobian.row(2) = (-heading.yaw_derivative * angle_gradient).transpose();

    // Reduce translation as the parking point approaches and while turning.
    // This avoids orbiting the point at a saturated yaw rate.
    const double distance = std::sqrt(squared_distance);
    const Eigen::Vector3d distance_gradient(-delta.x()/distance, -delta.y()/distance, 0.0);
    constexpr double position_gain = 0.4;
    const double shaped_linear = std::tanh(position_gain * distance / max_linear_speed);
    const double aligned = std::max(0.0, std::cos(angle));
    const double alignment = aligned * aligned;
    command.input.x() = goal.drive_direction * max_linear_speed * shaped_linear * alignment;
    command.state_jacobian.row(0) = (goal.drive_direction * (
        position_gain*(1.0-shaped_linear*shaped_linear)*alignment*distance_gradient -
        max_linear_speed*shaped_linear*2.0*aligned*std::sin(angle)*angle_gradient)).transpose();
    return command;
}

std::unique_ptr<StateInputConstraint> makeStageInputConstraint(
    std::shared_ptr<StagedMotionState> stage, bool ranger_base, double max_angular_speed, double max_linear_speed) {
    return std::make_unique<StageInputConstraint>(std::move(stage), ranger_base, max_angular_speed, max_linear_speed);
}

StagedReferenceManager::StagedReferenceManager(
    std::shared_ptr<ReferenceManagerInterface> wrapped, const PinocchioInterface& pin,
    std::string tcp_frame, StagedMotionConfig config, rclcpp::Logger logger)
    : ReferenceManagerDecorator(std::move(wrapped)), config_(std::move(config)), pinocchio_(pin),
      solve_state_(std::make_shared<StagedMotionState>()), logger_(std::move(logger)) {
    const auto& model = pinocchio_.getModel();
    arm_frame_ = model.getFrameId(config_.arm_base_frame);
    tcp_frame_ = model.getFrameId(tcp_frame);
    if (arm_frame_ >= model.frames.size() || tcp_frame_ >= model.frames.size() || model.nq != 9)
        throw std::runtime_error("Staged mobile MPC requires valid arm/TCP frames and a 9-state model");
    const auto parent = model.frames[arm_frame_].parentJoint;
    if (parent > 0 && model.joints[parent].idx_q() + model.joints[parent].nq() > 3)
        throw std::runtime_error("arm_workspace_frame must be fixed to the chassis, upstream of all arm joints");
    if (config_.reach_radius == 0.0) config_.reach_radius = estimateRadius(pinocchio_, arm_frame_, tcp_frame_);
    if (!std::isfinite(config_.reach_radius) || config_.reach_radius <= 0.05)
        throw std::runtime_error("Invalid arm reach radius");
    coordinator_ = std::make_unique<ReachCoordinator>(config_);
    RCLCPP_INFO(logger_, "Staged MPC reach radius %.3f m about %s; base restart margin %.3f m",
                config_.reach_radius, config_.arm_base_frame.c_str(), config_.reach_switch_margin);
}

void StagedReferenceManager::preSolverRun(scalar_t init_time, scalar_t final_time, const vector_t& state) {
    ReferenceManagerDecorator::preSolverRun(init_time, final_time, state);
    const auto& targets = getTargetTrajectories().stateTrajectory;
    if (targets.empty() || targets.back().size() < 7 || !targets.back().allFinite())
        throw std::runtime_error("Invalid TCP reference for staged motion");
    pinocchio::framesForwardKinematics(pinocchio_.getModel(), pinocchio_.getData(), state);
    const auto& tcp_pose = pinocchio_.getData().oMf[tcp_frame_];
    const auto& reference = targets.back();
    TrackingFeedback tracking;
    tracking.position_error = (tcp_pose.translation() - reference.head<3>()).norm();
    tracking.target_orientation = Eigen::Quaterniond(reference(6), reference(3), reference(4), reference(5));
    if (tracking.target_orientation.norm() <= 1e-9)
        throw std::runtime_error("Invalid target quaternion for staged motion");
    tracking.target_orientation.normalize();
    tracking.orientation_error = Eigen::Quaterniond(tcp_pose.rotation()).angularDistance(tracking.target_orientation);
    const auto previous_phase = solve_state_->phase;
    *solve_state_ = coordinator_->update(init_time, state.head<3>(),
        pinocchio_.getData().oMf[arm_frame_].translation(), reference.head<3>(), tracking);
    if (solve_state_->phase != previous_phase)
        RCLCPP_INFO(logger_, "Motion phase %s -> %s: target radius %.3f m, parking radius %.3f m, TCP error %.3f m / %.3f rad",
                    motionPhaseName(previous_phase), motionPhaseName(solve_state_->phase),
                    solve_state_->distance, solve_state_->stop_radius, tracking.position_error, tracking.orientation_error);
}

void StagedReferenceManager::augmentProblem(OptimalControlProblem& problem) const {
    auto arm_cost = problem.stateSoftConstraintPtr->extract("endEffector");
    auto final_cost = problem.finalSoftConstraintPtr->extract("finalEndEffector");
    if (!arm_cost || !final_cost) throw std::runtime_error("Missing upstream TCP costs for staged MPC");
    problem.stateSoftConstraintPtr->add("endEffector", std::make_unique<ArmStageCost>(std::move(arm_cost), solve_state_));
    problem.finalSoftConstraintPtr->add("finalEndEffector", std::make_unique<ArmStageCost>(std::move(final_cost), solve_state_));
    problem.stateCostPtr->add("baseApproach", std::make_unique<BaseApproachCost>(solve_state_));
    problem.finalCostPtr->add("baseApproach", std::make_unique<BaseApproachCost>(solve_state_));
    problem.equalityConstraintPtr->add("stageInputLock", makeStageInputConstraint(solve_state_, config_.ranger_base, config_.heading_max_velocity, config_.approach_max_linear_velocity));
}

void StagedReferenceManager::reset() {
    coordinator_->reset();
    *solve_state_ = StagedMotionState{};
}
} // namespace zephyr::mpc
