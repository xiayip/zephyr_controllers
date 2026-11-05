// SPDX-License-Identifier: Apache-2.0
// Local Pinocchio kinematics; standard MoveIt message contracts, no MoveIt
// runtime.
#include "mpc_controller/control/TcpKinematics.h"
#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace zephyr::mpc {
namespace {
using Error = moveit_msgs::msg::MoveItErrorCodes;
void status(Error &error, int code, const std::string &message) {
  error.val = code;
  error.message = message;
  error.source = "mpc_controller";
}
geometry_msgs::msg::Pose pose(const pinocchio::SE3 &transform) {
  geometry_msgs::msg::Pose p;
  const Eigen::Quaterniond r(transform.rotation());
  p.position.x = transform.translation().x();
  p.position.y = transform.translation().y();
  p.position.z = transform.translation().z();
  p.orientation.x = r.x();
  p.orientation.y = r.y();
  p.orientation.z = r.z();
  p.orientation.w = r.w();
  return p;
}
} // namespace
TcpKinematics::TcpKinematics(pinocchio::Model model,
                             std::vector<std::string> joints,
                             std::string tcp_frame, std::string base_frame,
                             KinematicsOptions options)
    : model_(std::move(model)), data_(model_), joints_(std::move(joints)),
      tcp_frame_(std::move(tcp_frame)), base_frame_(std::move(base_frame)),
      frame_(model_.getFrameId(tcp_frame_)), options_(std::move(options)) {
  if (frame_ >= model_.frames.size() || model_.nq != model_.nv ||
      model_.nq != static_cast<int>(joints_.size()) || base_frame_.empty())
    throw std::invalid_argument("Invalid single-arm kinematics model");
  std::unordered_set<std::string> unique;
  for (const auto &name : joints_) {
    const auto id = model_.getJointId(name);
    if (!unique.insert(name).second || id == 0 || id >= model_.joints.size() ||
        model_.joints[id].nq() != 1 || model_.joints[id].nv() != 1)
      throw std::invalid_argument("Invalid TCP joint: " + name);
    indices_.push_back(model_.joints[id].idx_q());
  }
  if ((options_.solver != "auto" && options_.solver != "dls" &&
       options_.solver != "bfgs") ||
      options_.group.empty() || options_.max_iterations < 1 ||
      options_.max_iterations > 10000 || !std::isfinite(options_.tolerance) ||
      options_.tolerance <= 0 || !std::isfinite(options_.damping) ||
      options_.damping <= 0 || !std::isfinite(options_.timeout) ||
      options_.timeout <= 0 || options_.timeout > 60)
    throw std::invalid_argument("Invalid kinematics options");
  if (options_.random_restart &&
      (!model_.lowerPositionLimit.allFinite() ||
       !model_.upperPositionLimit.allFinite() ||
       (model_.lowerPositionLimit.array() > model_.upperPositionLimit.array())
           .any()))
    throw std::invalid_argument(
        "Random IK restart requires finite joint bounds");
}
bool TcpKinematics::configuration(const moveit_msgs::msg::RobotState &state,
                                  Eigen::VectorXd &q, Error &error) const {
  const auto &js = state.joint_state;
  if (state.is_diff || !state.attached_collision_objects.empty() ||
      !state.multi_dof_joint_state.joint_names.empty() ||
      !state.multi_dof_joint_state.transforms.empty() ||
      !state.multi_dof_joint_state.twist.empty() ||
      !state.multi_dof_joint_state.wrench.empty() ||
      js.name.size() != joints_.size() ||
      js.position.size() != js.name.size()) {
    status(error, Error::INVALID_ROBOT_STATE,
           "Supply a complete, absolute named arm joint state only");
    return false;
  }
  q = pinocchio::neutral(model_);
  std::unordered_set<std::string> seen;
  for (size_t i = 0; i < js.name.size(); ++i) {
    const auto it = std::find(joints_.begin(), joints_.end(), js.name[i]);
    if (it == joints_.end() || !seen.insert(js.name[i]).second ||
        !std::isfinite(js.position[i])) {
      status(error, Error::INVALID_ROBOT_STATE,
             "Unknown/duplicate joint or nonfinite position");
      return false;
    }
    q(indices_[std::distance(joints_.begin(), it)]) = js.position[i];
  }
  return true;
}
void TcpKinematics::fk(const moveit_msgs::srv::GetPositionFK::Request &request,
                       moveit_msgs::srv::GetPositionFK::Response &response) {
  response = moveit_msgs::srv::GetPositionFK::Response();
  if (!request.header.frame_id.empty() &&
      request.header.frame_id != base_frame_) {
    status(response.error_code, Error::FRAME_TRANSFORM_FAILURE,
           "FK output frame must be " + base_frame_);
    return;
  }
  Eigen::VectorXd q;
  if (!configuration(request.robot_state, q, response.error_code))
    return;
  if (request.fk_link_names.empty()) {
    status(response.error_code, Error::INVALID_LINK_NAME,
           "No FK links requested");
    return;
  }
  for (const auto &name : request.fk_link_names)
    if (model_.getFrameId(name) >= model_.frames.size()) {
      status(response.error_code, Error::INVALID_LINK_NAME,
             "Unknown FK link: " + name);
      return;
    }
  std::lock_guard<std::timed_mutex> lock(mutex_);
  pinocchio::framesForwardKinematics(model_, data_, q);
  for (const auto &name : request.fk_link_names) {
    geometry_msgs::msg::PoseStamped result;
    result.header = request.header;
    result.header.frame_id = base_frame_;
    result.pose = pose(data_.oMf[model_.getFrameId(name)]);
    response.pose_stamped.push_back(result);
  }
  response.fk_link_names = request.fk_link_names;
  status(response.error_code, Error::SUCCESS,
         "FK complete (no collision checking)");
}
void TcpKinematics::ik(const moveit_msgs::srv::GetPositionIK::Request &service,
                       moveit_msgs::srv::GetPositionIK::Response &response) {
  using Clock = std::chrono::steady_clock;
  const auto began = Clock::now();
  response = moveit_msgs::srv::GetPositionIK::Response();
  const auto &request = service.ik_request;
  if (!request.group_name.empty() && request.group_name != options_.group) {
    status(response.error_code, Error::INVALID_GROUP_NAME,
           "Configured group is " + options_.group);
    return;
  }
  if ((!request.ik_link_name.empty() && request.ik_link_name != tcp_frame_) ||
      !request.ik_link_names.empty() || !request.pose_stamped_vector.empty()) {
    status(response.error_code, Error::INVALID_LINK_NAME,
           "Only the single configured TCP is supported");
    return;
  }
  if (request.avoid_collisions) {
    status(response.error_code, Error::COLLISION_CHECKING_UNAVAILABLE,
           "IK service does not check collision");
    return;
  }
  const auto &c = request.constraints;
  if (!c.joint_constraints.empty() || !c.position_constraints.empty() ||
      !c.orientation_constraints.empty() || !c.visibility_constraints.empty()) {
    status(response.error_code, Error::INVALID_GOAL_CONSTRAINTS,
           "Additional IK constraints are unsupported");
    return;
  }
  if (!request.pose_stamped.header.frame_id.empty() &&
      request.pose_stamped.header.frame_id != base_frame_) {
    status(response.error_code, Error::FRAME_TRANSFORM_FAILURE,
           "IK target frame must be " + base_frame_);
    return;
  }
  Eigen::VectorXd q;
  if (!configuration(request.robot_state, q, response.error_code))
    return;
  const auto &p = request.pose_stamped.pose;
  Eigen::Quaterniond orientation(p.orientation.w, p.orientation.x,
                                 p.orientation.y, p.orientation.z);
  const Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
  if (!position.allFinite() || !orientation.coeffs().allFinite() ||
      orientation.norm() < 1e-9) {
    status(response.error_code, Error::GOAL_STATE_INVALID,
           "Invalid IK target pose");
    return;
  }
  const double requested_timeout =
      request.timeout.sec + 1e-9 * request.timeout.nanosec;
  if (request.timeout.sec < 0 || request.timeout.nanosec >= 1000000000u ||
      requested_timeout > 60) {
    status(response.error_code, Error::FAILURE,
           "IK timeout must be in [0, 60] seconds");
    return;
  }
  const double seconds =
      requested_timeout > 0 ? requested_timeout : options_.timeout;
  const auto deadline = began + std::chrono::duration_cast<Clock::duration>(
                                    std::chrono::duration<double>(seconds));
  std::unique_lock<std::timed_mutex> lock(mutex_, std::defer_lock);
  if (!lock.try_lock_until(deadline)) {
    status(response.error_code, Error::TIMED_OUT,
           "IK timed out waiting for model access");
    return;
  }
  const auto clamp = [&](Eigen::VectorXd value) {
    return value.cwiseMax(model_.lowerPositionLimit)
        .cwiseMin(model_.upperPositionLimit)
        .eval();
  };
  bool success = false, timed_out = false;
  int iterations_used = 0;
  std::string used_solver;
  const pinocchio::SE3 target(orientation.normalized().toRotationMatrix(),
                              position);
  const auto &solver = options_.solver;
  const int iterations = options_.max_iterations;
  const double tolerance = options_.tolerance;
  const double damping = options_.damping;
  Eigen::VectorXd best = clamp(q);
  double best_error = std::numeric_limits<double>::infinity();
  const auto errorAt = [&](const Eigen::VectorXd &value) {
    pinocchio::framesForwardKinematics(model_, data_, value);
    return pinocchio::log6(data_.oMf[frame_].actInv(target)).toVector().eval();
  };
  std::mt19937 generator(0);
  const int attempts = options_.random_restart ? 4 : 1;
  for (int attempt = 0; attempt < attempts && !success && !timed_out;
       ++attempt) {
    q = best;
    if (attempt)
      for (int i : indices_)
        q(i) = std::uniform_real_distribution<double>(
            model_.lowerPositionLimit(i),
            model_.upperPositionLimit(i))(generator);
    for (const auto &method : solver == "auto"
                                  ? std::vector<std::string>{"dls", "bfgs"}
                                  : std::vector<std::string>{solver}) {
      Eigen::MatrixXd inverse_hessian =
          Eigen::MatrixXd::Identity(model_.nv, model_.nv);
      Eigen::VectorXd old_gradient, old_q;
      for (int iteration = 0; iteration < iterations; ++iteration) {
        if (Clock::now() >= deadline) {
          timed_out = true;
          break;
        }
        const Eigen::VectorXd error = errorAt(q);
        const double norm = error.norm();
        if (norm < best_error) {
          best_error = norm;
          best = q;
        }
        ++iterations_used;
        used_solver = method;
        if (norm <= tolerance) {
          success = true;
          break;
        }
        Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, model_.nv);
        jacobian.setZero();
        pinocchio::computeFrameJacobian(model_, data_, q, frame_,
                                        pinocchio::LOCAL, jacobian);
        Eigen::Matrix<double, 6, 6> log_jacobian;
        pinocchio::Jlog6(data_.oMf[frame_].actInv(target).inverse(),
                         log_jacobian);
        jacobian = -log_jacobian * jacobian;
        const Eigen::VectorXd gradient = jacobian.transpose() * error;
        Eigen::VectorXd step;
        if (method == "dls") {
          Eigen::Matrix<double, 6, 6> normal = jacobian * jacobian.transpose();
          normal.diagonal().array() += damping * damping;
          step = -jacobian.transpose() * normal.ldlt().solve(error);
        } else {
          if (old_gradient.size()) {
            const Eigen::VectorXd s = q - old_q, y = gradient - old_gradient;
            const double sy = s.dot(y);
            if (sy > 1e-12) {
              const Eigen::MatrixXd v =
                  Eigen::MatrixXd::Identity(model_.nv, model_.nv) -
                  s * y.transpose() / sy;
              inverse_hessian =
                  v * inverse_hessian * v.transpose() + s * s.transpose() / sy;
            }
          }
          step = -inverse_hessian * gradient;
          if (step.dot(gradient) >= 0.0) {
            inverse_hessian.setIdentity();
            step = -gradient;
          }
          old_q = q;
          old_gradient = gradient;
        }
        if (!step.allFinite())
          break;
        const double maximum = step.cwiseAbs().maxCoeff();
        if (maximum > 0.25)
          step *= 0.25 / maximum;
        bool improved = false;
        for (double scale = 1.0; scale >= 1e-5; scale *= 0.5) {
          if (Clock::now() >= deadline) {
            timed_out = true;
            break;
          }
          Eigen::VectorXd candidate = clamp(q + scale * step);
          if (errorAt(candidate).squaredNorm() < error.squaredNorm()) {
            q = std::move(candidate);
            improved = true;
            break;
          }
        }
        if (!improved)
          break;
      }
      if (success || timed_out)
        break;
      q = best;
    }
  }

  if (success) {
    response.solution.joint_state.header =
        request.robot_state.joint_state.header;
    response.solution.joint_state.name = request.robot_state.joint_state.name;
    for (const auto &name : response.solution.joint_state.name) {
      const auto i = std::distance(
          joints_.begin(), std::find(joints_.begin(), joints_.end(), name));
      response.solution.joint_state.position.push_back(best(indices_[i]));
    }
  }
  status(response.error_code,
         success     ? Error::SUCCESS
         : timed_out ? Error::TIMED_OUT
                     : Error::NO_IK_SOLUTION,
         "solver=" + used_solver +
             "; iterations=" + std::to_string(iterations_used) + "; residual=" +
             std::to_string(best_error) + "; no collision checking");
}
} // namespace zephyr::mpc
