// SPDX-License-Identifier: Apache-2.0
#include <mpc_controller/control/MotionActionServer.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>

namespace zephyr::mpc {
namespace {
bool validPose(const geometry_msgs::msg::Pose& p) {
  const double v[] = {p.position.x, p.position.y, p.position.z,
                     p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
  const double norm = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).norm();
  return std::all_of(std::begin(v), std::end(v), [](double x) { return std::isfinite(x); }) &&
      std::isfinite(norm) && norm > 1e-9;
}
const char* reasonText(uint16_t reason) {
  switch (reason) {
    case MotionStatus::NONE: return "Ready";
    case MotionStatus::CONTROLLER_INACTIVE: return "Controller inactive";
    case MotionStatus::FEEDBACK_UNAVAILABLE: return "Measured feedback unavailable or stale";
    case MotionStatus::SOLVER_FAILED: return "MPC solve failed";
    case MotionStatus::REFERENCE_FAILED: return "Reference update failed";
    case MotionStatus::POLICY_FAILED: return "Policy or output validation failed; inspect controller log";
    case MotionStatus::COLLISION_BLOCKED: return "Collision monitor blocked execution; inspect controller log";
    default: return "Motion stopped";
  }
}
std::string uuidString(const rclcpp_action::GoalUUID& id) {
  const char* hex = "0123456789abcdef";
  std::string out;
  for (auto byte : id) { out += hex[byte >> 4]; out += hex[byte & 15]; }
  return out;
}
}  // namespace

MotionActionServer::MotionActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<TcpReferenceManager> reference, std::shared_ptr<MotionRuntime> runtime)
    : node_(std::move(node)), reference_(std::move(reference)), runtime_(std::move(runtime)) {
  status_pub_ = node_->create_publisher<MotionStatus>("~/motion_status", rclcpp::QoS(1).transient_local());
  server_ = rclcpp_action::create_server<Action>(node_, "~/execute_motion",
      [this](const auto& id, auto request) { return goal(id, request); },
      [this](const std::shared_ptr<Handle> h) {
        return h == handle_ ? rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
      },
      [this](std::shared_ptr<Handle> h) { accepted(std::move(h)); });
  stop_service_ = node_->create_service<std_srvs::srv::Trigger>("~/stop",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!runtime_->active) {
          response->message = "Controller inactive; no motion accepted";
          return;
        }
        stop(MotionStatus::CANCELED, "Stopped by ~/stop");
        response->success = true;
        response->message = "Stop requested; wait for motion_status IDLE (or inspect FAULT), no resume needed";
      });
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
}

bool MotionActionServer::fresh(const MotionSample& sample) const {
  const auto age = motionSteadyNs() - sample.received_ns;
  return sample.valid && sample.activation_epoch == runtime_->activation_epoch.load() &&
      sample.received_ns > 0 && age >= 0 && age <= 200000000;
}
geometry_msgs::msg::PoseStamped MotionActionServer::actual(const MotionSample& s) const {
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = reference_->referenceFrame();
  p.header.stamp = rclcpp::Time(s.stamp_ns);
  p.pose.position.x = s.pose[0]; p.pose.position.y = s.pose[1]; p.pose.position.z = s.pose[2];
  p.pose.orientation.x = s.pose[3]; p.pose.orientation.y = s.pose[4];
  p.pose.orientation.z = s.pose[5]; p.pose.orientation.w = s.pose[6];
  return p;
}
rclcpp_action::GoalResponse MotionActionServer::goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const Action::Goal> g) {
  const auto reject = [this](const char* why) {
    RCLCPP_WARN(node_->get_logger(), "Motion rejected: %s", why);
    return rclcpp_action::GoalResponse::REJECT;
  };
  if (reserved_ || handle_ || runtime_->stopRequested()) return reject("another motion owns the controller");
  const auto sample = runtime_->samples.read();
  if (!runtime_->active || sample.state != MotionStatus::IDLE || !fresh(sample))
    return reject("controller is inactive, stopping, initializing, faulted or has stale feedback");
  const double values[] = {g->speed_scale, g->position_tolerance, g->orientation_tolerance,
                          g->settle_time, g->timeout};
  if (!std::all_of(std::begin(values), std::end(values), [](double x) { return std::isfinite(x) && x > 0; }) ||
      g->speed_scale > 1 || g->orientation_tolerance > M_PI || g->timeout <= g->settle_time)
    return reject("invalid speed scale, tolerance, settle time or timeout");
  if (g->mode == Action::Goal::POSE) {
    if (!g->waypoints.empty() || !validPose(g->target.pose)) return reject("invalid pose goal");
  } else if (g->mode == Action::Goal::PATH) {
    if (g->waypoints.empty() || g->waypoints.size() > 1000) return reject("path must contain 1..1000 poses");
    for (const auto& p : g->waypoints) if (!validPose(p.pose)) return reject("invalid path pose");
  } else if (g->mode == Action::Goal::RELATIVE) {
    if (!g->waypoints.empty()) return reject("relative goal cannot also contain a path");
    const auto& t = g->relative.twist;
    const double v[] = {t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z};
    if (!std::all_of(std::begin(v), std::end(v), [](double x) { return std::isfinite(x); }))
      return reject("nonfinite relative motion");
  } else return reject("unknown motion mode");
  if (!reference_->claimMotion()) return reject("reference ownership unavailable");
  reserved_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
void MotionActionServer::accepted(std::shared_ptr<Handle> handle) {
  handle_ = std::move(handle);
  activation_epoch_ = runtime_->activation_epoch.load();
  stopping_ = false;
  started_ns_ = motionSteadyNs();
  settled_ns_ = 0;
  reason_ = MotionStatus::NONE;
  message_ = "Executing";
  position_error_ = orientation_error_ = std::numeric_limits<double>::infinity();
  const auto sample = runtime_->samples.read();
  const auto pose = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(sample.pose.data()).eval();
  if (!runtime_->active || sample.state != MotionStatus::IDLE || !fresh(sample) ||
      !reference_->startMotion(*handle_->get_goal(), pose, target_, end_time_, message_)) {
    reason_ = MotionStatus::INVALID_GOAL;
    if (message_ == "Executing") message_ = "Controller stopped before goal acceptance";
    finish(false, sample);  // No motion reference was committed.
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Motion %s accepted", uuidString(handle_->get_goal_id()).c_str());
}
void MotionActionServer::stop(uint16_t reason, const std::string& message) {
  if (stopping_) return;
  if (!handle_) activation_epoch_ = runtime_->activation_epoch.load();
  stopping_ = true;
  reason_ = reason;
  message_ = message;
  stopped_ns_ = motionSteadyNs();
  stop_token_ = runtime_->stop_request.fetch_add(1) + 1;
  // The control loop is the sole TargetSession writer: it closes admission on
  // its next cycle, holds output, then resets the reference before reopening.
  RCLCPP_INFO(node_->get_logger(), "Motion stopping: %s", message.c_str());
}
void MotionActionServer::finish(bool success, const MotionSample& sample) {
  auto result = std::make_shared<Action::Result>();
  result->success = success;
  result->reason = reason_;
  result->message = success ? "Measured TCP settled at target" : message_;
  result->actual = actual(sample);
  result->position_error = position_error_;
  result->orientation_error = orientation_error_;
  reference_->releaseMotion();
  if (stopping_) runtime_->stop_release.store(stop_token_);
  if (handle_->is_canceling()) handle_->canceled(result);
  else if (success) handle_->succeed(result);
  else handle_->abort(result);
  RCLCPP_INFO(node_->get_logger(), "Motion finished: %s", result->message.c_str());
  handle_.reset();
  reserved_ = stopping_ = false;
}
void MotionActionServer::tick() {
  auto sample = runtime_->samples.read();
  const auto now = motionSteadyNs();
  MotionStatus status;
  status.header.stamp = node_->now();
  status.header.frame_id = reference_->referenceFrame();
  status.actual = actual(sample);
  status.feedback_valid = fresh(sample);
  status.state = runtime_->active ? sample.state : MotionStatus::INACTIVE;
  status.reason = runtime_->active ? sample.reason : MotionStatus::CONTROLLER_INACTIVE;
  if (runtime_->active && !fresh(sample)) {
    status.state = MotionStatus::FAULT;
    status.reason = MotionStatus::FEEDBACK_UNAVAILABLE;
  }
  status.message = status.state == MotionStatus::INITIALIZING && status.reason == MotionStatus::NONE ?
      "Initializing reference and waiting for a valid MPC policy" : reasonText(status.reason);
  if (handle_) {
    status.goal_id = uuidString(handle_->get_goal_id());
    const auto& g = *handle_->get_goal();
    if (handle_->is_canceling()) stop(MotionStatus::CANCELED, "Canceled by client");
    if (!runtime_->active || runtime_->activation_epoch.load() != activation_epoch_)
      stop(MotionStatus::CONTROLLER_INACTIVE, reasonText(MotionStatus::CONTROLLER_INACTIVE));
    else if (!fresh(sample)) stop(MotionStatus::FEEDBACK_UNAVAILABLE, reasonText(MotionStatus::FEEDBACK_UNAVAILABLE));
    else if (sample.state != MotionStatus::IDLE && !stopping_) {
      const auto code = sample.reason ? sample.reason : MotionStatus::REFERENCE_FAILED;
      stop(code, reasonText(code));
    }
    else if (!stopping_ && !reference_->motionSessionValid())
      stop(MotionStatus::REFERENCE_FAILED, "Reference session changed; old motion cannot resume");
    if ((now - started_ns_) * 1e-9 >= g.timeout) stop(MotionStatus::TIMED_OUT, "Motion deadline exceeded");

    if (fresh(sample)) {
      const auto& p = target_.pose;
      position_error_ = (Eigen::Vector3d(sample.pose[0], sample.pose[1], sample.pose[2]) -
                         Eigen::Vector3d(p.position.x, p.position.y, p.position.z)).norm();
      orientation_error_ = Eigen::Quaterniond(sample.pose[6], sample.pose[3], sample.pose[4], sample.pose[5])
          .angularDistance(Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z));
    }
    if (!stopping_) {
      const bool arrived = sample.reference_time >= end_time_ &&
          position_error_ <= g.position_tolerance && orientation_error_ <= g.orientation_tolerance;
      if (!arrived) settled_ns_ = 0;
      else if (!settled_ns_) settled_ns_ = sample.received_ns;
      status.state = arrived ? MotionStatus::SETTLING : MotionStatus::EXECUTING;
      status.message = arrived ? "Waiting for stable measured arrival" : "Executing";
      if (settled_ns_ && (sample.received_ns - settled_ns_) * 1e-9 >= g.settle_time) finish(true, sample);
    }
  }
  // Shared stop confirmation for finite Actions AND continuous topic commands.
  // A stop never clears a fault and never releases inputs before a hold write.
  if (stopping_) {
    status.state = MotionStatus::STOPPING;
    status.reason = reason_;
    status.message = message_;
    const bool lifecycle_stopped = activation_epoch_ != 0 &&
        runtime_->stopped_epoch.load() == activation_epoch_;
    if (lifecycle_stopped || (sample.stop_ack >= stop_token_ && sample.received_ns >= stopped_ns_)) {
      if (handle_) finish(false, sample);
      else {
        runtime_->stop_release.store(stop_token_);
        stopping_ = false;
      }
    } else if (now - stopped_ns_ > 2000000000LL) {
      status.reason = MotionStatus::STOP_UNCONFIRMED;
      status.message = "Waiting for control-loop stop acknowledgement; inputs remain blocked";
    }
  }
  if (handle_) {
    status.position_error = position_error_;
    status.orientation_error = orientation_error_;
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->state = status.state;
    feedback->actual = status.actual;
    feedback->target = target_;
    feedback->position_error = position_error_;
    feedback->orientation_error = orientation_error_;
    feedback->elapsed = (now - started_ns_) * 1e-9;
    handle_->publish_feedback(feedback);
  }
  status_pub_->publish(status);
}
}  // namespace zephyr::mpc
