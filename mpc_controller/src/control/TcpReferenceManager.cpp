// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
#include "mpc_controller/control/TcpReferenceManager.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

namespace zephyr::mpc {
// TCP target input, frame conversion, and OCS2 reference trajectories.

using namespace ocs2;
namespace {
vector_t poseVector(const geometry_msgs::msg::Pose &p) {
  vector_t v(7);
  v << p.position.x, p.position.y, p.position.z, p.orientation.x,
      p.orientation.y, p.orientation.z, p.orientation.w;
  if (!v.allFinite() || v.tail<4>().norm() < 1e-9)
    throw std::invalid_argument("Invalid TCP pose");
  v.tail<4>().normalize();
  return v;
}
geometry_msgs::msg::Pose rosPose(const vector_t &v) {
  geometry_msgs::msg::Pose p;
  p.position.x = v(0);
  p.position.y = v(1);
  p.position.z = v(2);
  p.orientation.x = v(3);
  p.orientation.y = v(4);
  p.orientation.z = v(5);
  p.orientation.w = v(6);
  return p;
}
Eigen::Quaterniond quaternion(const vector_t &p) {
  return Eigen::Quaterniond(p(6), p(3), p(4), p(5));
}
vector_t interpolate(const vector_t &a, const vector_t &b, double t) {
  vector_t p(7);
  p.head<3>() = (1 - t) * a.head<3>() + t * b.head<3>();
  p.tail<4>() = quaternion(a).slerp(t, quaternion(b)).normalized().coeffs();
  return p;
}
double smooth(double t) { return t * t * t * (10 + t * (-15 + 6 * t)); }
double minimumDuration(double distance, double velocity, double acceleration,
                       double jerk) {
  return std::max({1.875 * distance / velocity,
                   std::sqrt(5.774 * distance / acceleration),
                   std::cbrt(60 * distance / jerk)});
}
} // namespace
TcpReferenceManager::TcpReferenceManager(
    std::shared_ptr<ReferenceManagerInterface> wrapped, std::string base_frame,
    std::string tcp_frame, size_t input_dim)
    : ReferenceManagerDecorator(std::move(wrapped)),
      base_frame_(std::move(base_frame)), tcp_frame_(std::move(tcp_frame)),
      input_dim_(input_dim), target_(vector_t::Zero(7)) {
  target_(6) = 1.0;
}
void TcpReferenceManager::subscribe(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
  node_ = node;
  const auto parameter = [&](const char *key, double fallback) {
    const double value = node->has_parameter(key)
                             ? node->get_parameter(key).as_double()
                             : node->declare_parameter(key, fallback);
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument(std::string(key) + " must be positive");
    return value;
  };
  duration_ = parameter("movel_duration", 2.0);
  interval_ = parameter("movel_sample_interval", 0.04);
  max_linear_ = parameter("movel_max_linear_velocity", 0.3);
  max_angular_ = parameter("movel_max_angular_velocity", 1.0);
  max_linear_acc_ = parameter("movel_max_linear_acceleration", 1.0);
  max_angular_acc_ = parameter("movel_max_angular_acceleration", 2.0);
  max_linear_jerk_ = parameter("movel_max_linear_jerk", 2.0);
  max_angular_jerk_ = parameter("movel_max_angular_jerk", 4.0);
  tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "tcp_current_target", rclcpp::QoS(1).transient_local());
  // All target integration, allocation and publishing runs on the executor.
  // The control loop supplies only an atomic clock and bounded reset requests.
  reference_timer_ =
      node->create_wall_timer(std::chrono::milliseconds(10), [this] {
        try {
          updateReference();
        } catch (const std::exception &error) {
          failed_.store(true, std::memory_order_release);
          RCLCPP_ERROR(node_->get_logger(), "TCP reference update failed: %s",
                       error.what());
        }
      });
  pose_sub_ = node->create_subscription<geometry_msgs::msg::Pose>(
      "tcp_target", 1, [this](geometry_msgs::msg::Pose::ConstSharedPtr p) {
        geometry_msgs::msg::PoseStamped s;
        s.header.frame_id = base_frame_;
        s.pose = *p;
        acceptPose(s, false);
      });
  stamped_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "tcp_target/stamped", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) {
        acceptPose(*p, true);
      });
  relative_sub_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      "tcp_target/relative", 1,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr p) {
        acceptRelative(*p);
      });
  twist_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "tcp_target/twist", 1,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr p) {
        const auto session = target_session_.token();
        if (!session)
          return;
        const double values[] = {p->linear.x,  p->linear.y,  p->linear.z,
                                 p->angular.x, p->angular.y, p->angular.z};
        for (double v : values)
          if (!std::isfinite(v))
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!target_session_.accepts(session))
          return;
        target_ = sample(time_);
        twist_ = *p;
        twist_active_ = true;
        twist_received_ = std::chrono::steady_clock::now();
        last_time_ = observation_time_.load(std::memory_order_acquire);
      });
  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "target_path", 1,
      [this](nav_msgs::msg::Path::ConstSharedPtr p) { acceptPath(*p); });
}
bool TcpReferenceManager::transform(const geometry_msgs::msg::PoseStamped &p,
                                    vector_t &result) const {
  try {
    auto converted = p;
    if (!p.header.frame_id.empty() && p.header.frame_id != base_frame_)
      tf2::doTransform(p, converted,
                       tf_->lookupTransform(base_frame_, p.header.frame_id,
                                            tf2::TimePointZero));
    result = poseVector(converted.pose);
    return true;
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "TCP target rejected: %s", e.what());
    return false;
  }
}
void TcpReferenceManager::acceptPose(const geometry_msgs::msg::PoseStamped &p,
                                     bool interpolate_target) {
  const auto session = target_session_.token();
  if (!session)
    return;
  vector_t goal;
  if (!transform(p, goal))
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!target_session_.accepts(session))
    return;
  twist_active_ = false;
  try {
    if (interpolate_target)
      plan({goal}, duration_);
    else {
      target_ = goal;
      commitImmediate();
    }
    publish();
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "TCP target rejected: %s", e.what());
  }
}
void TcpReferenceManager::acceptRelative(
    const geometry_msgs::msg::TwistStamped &delta) {
  const auto session = target_session_.token();
  if (!session)
    return;
  try {
    Eigen::Vector3d translation(delta.twist.linear.x, delta.twist.linear.y,
                                delta.twist.linear.z);
    Eigen::Vector3d rotation(delta.twist.angular.x, delta.twist.angular.y,
                             delta.twist.angular.z);
    if (!translation.allFinite() || !rotation.allFinite())
      throw std::invalid_argument("Invalid relative target");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!target_session_.accepts(session))
      return;
    vector_t goal = sample(time_);
    Eigen::Quaterniond frame = Eigen::Quaterniond::Identity();
    if (delta.header.frame_id == tcp_frame_)
      frame = quaternion(goal);
    else if (!delta.header.frame_id.empty() &&
             delta.header.frame_id != base_frame_) {
      const auto tf = tf_->lookupTransform(base_frame_, delta.header.frame_id,
                                           tf2::TimePointZero);
      frame =
          Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                             tf.transform.rotation.y, tf.transform.rotation.z)
              .normalized();
    }
    goal.head<3>() += frame * translation;
    const double angle = rotation.norm();
    if (angle > 1e-12)
      goal.tail<4>() =
          (frame *
           Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation / angle)) *
           frame.conjugate() * quaternion(goal))
              .normalized()
              .coeffs();
    twist_active_ = false;
    plan({goal}, duration_);
    publish();
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "Relative TCP target rejected: %s",
                e.what());
  }
}
void TcpReferenceManager::acceptPath(const nav_msgs::msg::Path &path) {
  const auto session = target_session_.token();
  if (!session || path.poses.empty())
    return;
  vector_array_t goals;
  if (!transformPath(path, goals))
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!target_session_.accepts(session))
    return;
  try {
    twist_active_ = false;
    plan(goals, duration_);
    publish();
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(), "TCP path rejected: %s", e.what());
  }
}
bool TcpReferenceManager::transformPath(const nav_msgs::msg::Path &path,
                                        vector_array_t &goals) const {
  goals.clear();
  goals.reserve(path.poses.size());
  for (auto pose : path.poses) {
    if (pose.header.frame_id.empty())
      pose.header = path.header;
    vector_t goal;
    if (!transform(pose, goal))
      return false;
    goals.push_back(std::move(goal));
  }
  return true;
}
void TcpReferenceManager::plan(const std::vector<vector_t> &goals,
                               double requested_duration) {
  vector_t start = sample(time_);
  scalar_array_t times{time_};
  vector_array_t poses{start};
  double elapsed = 0.0;
  for (const auto &goal : goals) {
    const double distance = (goal.head<3>() - start.head<3>()).norm();
    const double angle = quaternion(start).angularDistance(quaternion(goal));
    const double duration =
        std::max({interval_, requested_duration / goals.size(),
                  minimumDuration(distance, max_linear_, max_linear_acc_,
                                  max_linear_jerk_),
                  minimumDuration(angle, max_angular_, max_angular_acc_,
                                  max_angular_jerk_)});
    const size_t count = std::max<size_t>(1, std::ceil(duration / interval_));
    if (count > 100000 || poses.size() + count > 200000)
      throw std::invalid_argument("TCP path too long");
    for (size_t i = 1; i <= count; ++i) {
      const double t = static_cast<double>(i) / count;
      times.push_back(time_ + elapsed + t * duration);
      poses.push_back(interpolate(start, goal, smooth(t)));
    }
    elapsed += duration;
    start = goal;
  }
  referenceManagerPtr_->setTargetTrajectories(TargetTrajectories(
      times, poses, vector_array_t(times.size(), vector_t::Zero(input_dim_))));
  times_ = std::move(times);
  poses_ = std::move(poses);
  target_ = goals.back();
}
vector_t TcpReferenceManager::sample(double time) const {
  if (times_.empty() || time >= times_.back())
    return target_;
  if (time <= times_.front())
    return poses_.front();
  const auto upper = std::upper_bound(times_.begin(), times_.end(), time);
  const size_t end = upper - times_.begin(), begin = end - 1;
  return interpolate(poses_[begin], poses_[end],
                     (time - times_[begin]) / (times_[end] - times_[begin]));
}
void TcpReferenceManager::commitImmediate() {
  times_.clear();
  poses_.clear();
  referenceManagerPtr_->setTargetTrajectories(
      TargetTrajectories({time_}, {target_}, {vector_t::Zero(input_dim_)}));
}
void TcpReferenceManager::setCurrentObservation(
    const SystemObservation &observation) {
  observation_time_.store(observation.time, std::memory_order_release);
}
bool TcpReferenceManager::requestReset(const vector_t &pose, double time) {
  if (pose.size() != 7 || !pose.allFinite() || !std::isfinite(time))
    return false;
  ResetRequest request;
  std::copy_n(pose.data(), 7, request.pose.begin());
  request.time = time;
  request.sequence = requested_reset_ + 1;
  if (!reset_mailbox_.tryWrite(request))
    return false;
  requested_reset_ = request.sequence;
  return true;
}
bool TcpReferenceManager::resetReady() const {
  return requested_reset_ != 0 &&
         applied_reset_.load(std::memory_order_acquire) == requested_reset_;
}
void TcpReferenceManager::updateReference() {
  // Never called by the realtime loop. Waiting behind a large path only
  // delays initialization/next references, not hardware updates or stopping.
  std::lock_guard<std::mutex> lock(mutex_);
  const auto reset = reset_mailbox_.read();
  if (reset.sequence != applied_reset_.load(std::memory_order_relaxed)) {
    target_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(reset.pose.data());
    time_ = last_time_ = reset.time;
    twist_active_ = false;
    commitImmediate();
    publish();
    applied_reset_.store(reset.sequence, std::memory_order_release);
  }
  const double now = observation_time_.load(std::memory_order_acquire);
  const double dt = now - last_time_;
  time_ = now;
  last_time_ = time_;
  if (!target_session_.token()) {
    twist_active_ = false;
    return;
  }
  if (!twist_active_)
    return;
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    twist_received_)
          .count() > 0.2) {
    twist_active_ = false;
    return;
  }
  if (dt <= 0.0 || dt > 0.5)
    return;
  target_.head<3>() +=
      dt * Eigen::Vector3d(twist_.linear.x, twist_.linear.y, twist_.linear.z);
  const Eigen::Vector3d rotation(twist_.angular.x, twist_.angular.y,
                                 twist_.angular.z);
  if (rotation.norm() > 1e-12)
    target_.tail<4>() = (Eigen::Quaterniond(Eigen::AngleAxisd(
                             dt * rotation.norm(), rotation.normalized())) *
                         quaternion(target_))
                            .normalized()
                            .coeffs();
  commitImmediate();
  publish();
}
void TcpReferenceManager::setAcceptingTargets(bool accept) {
  target_session_.setAccepting(accept);
}
void TcpReferenceManager::publish() const {
  if (!target_pub_)
    return;
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = base_frame_;
  p.header.stamp = node_->now();
  p.pose = rosPose(target_);
  target_pub_->publish(p);
}

} // namespace zephyr::mpc
