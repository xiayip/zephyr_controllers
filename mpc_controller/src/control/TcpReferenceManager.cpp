// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
#include "mpc_controller/control/TcpReferenceManager.h"
#include "mpc_controller/control/ContinuousTcpPath.h"
#include "mpc_controller/control/RuntimeMotionProfile.h"

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
  if (!v.allFinite() || !std::isfinite(v.tail<4>().norm()) || v.tail<4>().norm() < 1e-9)
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
void TcpReferenceManager::setRuntimeProfile(std::shared_ptr<RuntimeMotionProfile> profile) {
  runtime_profile_ = std::move(profile);
}
void TcpReferenceManager::configureWholeBody(bool mobile, std::vector<std::string> joints,
                                           vector_t lower, vector_t upper) {
  if (joints.empty() ||
      lower.size() != static_cast<int>(joints.size()) || upper.size() != lower.size() ||
      !lower.allFinite() || !upper.allFinite() || (lower.array() > upper.array()).any())
    throw std::invalid_argument("Invalid whole-body model limits");
  mobile_ = mobile; joints_ = std::move(joints); lower_ = std::move(lower); upper_ = std::move(upper);
}
void TcpReferenceManager::configureGripper(GripperConfig config) {
  config.validate();
  gripper_ = std::move(config);
  if (!gripper_.enabled()) return;
  if (joints_.empty() || joints_.back() != gripper_.joint || lower_.tail<1>()[0] != 0 || upper_.tail<1>()[0] != 1)
    throw std::invalid_argument("Gripper must be last normalized whole-body coordinate");
  gripper_index_ = joints_.size() - 1 + (mobile_ ? 3 : 0);
}
void TcpReferenceManager::commit(TargetTrajectories target, std::shared_ptr<const WholeBodyTrajectory> body) {
  std::lock_guard<std::mutex> lock(commit_mutex_);
  if (gripper_.enabled()) {
    if (!body) {
      // TCP-only commands retain closure, never leave the extra DoF cost-free.
      vector_t state = vector_t::Zero(joints_.size() + (mobile_ ? 3 : 0));
      vector_t weights = vector_t::Zero(state.size());
      state[gripper_index_] = gripper_target_;
      weights[gripper_index_] = gripper_.weight;
      body = makeWholeBodyTrajectory({0, 1}, {state, state}, weights, -1, vector_t::Ones(state.size()));
    }
    gripper_target_ = body->states.back()[gripper_index_];
  }
  referenceManagerPtr_->setTargetTrajectories(std::move(target));
  pending_body_ = std::move(body);
}
void TcpReferenceManager::preSolverRun(scalar_t init_time, scalar_t final_time,
                                     const vector_t& initial_state) {
  // Capture profile FIRST. A new profile is submitted only after its measured
  // hold reference was queued; the reverse ordering could pair a new revision
  // with the preceding motion's reference for one solve.
  if (runtime_profile_) runtime_profile_->captureBeforeReferenceUpdate();
  std::lock_guard<std::mutex> lock(commit_mutex_);
  referenceManagerPtr_->preSolverRun(init_time, final_time, initial_state);
  whole_body_->solver = pending_body_;
}
void TcpReferenceManager::subscribe(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
  node_ = node;
  const auto parameter = [&](const char *key, double fallback, bool allow_zero = false) {
    const double value = node->has_parameter(key)
                             ? node->get_parameter(key).as_double()
                             : node->declare_parameter(key, fallback);
    if (!std::isfinite(value) || value < 0.0 || (!allow_zero && value == 0.0))
      throw std::invalid_argument(std::string(key) + (allow_zero ? " must be nonnegative" : " must be positive"));
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
  continuous_max_deviation_ = parameter("movel_continuous_max_deviation", 0.0, true);
  body_velocity_limits_ = vector_t::Constant(joints_.size() + (mobile_ ? 3 : 0),
      parameter("whole_body_max_joint_velocity", 0.2));
  if (gripper_.enabled()) body_velocity_limits_[gripper_index_] = gripper_.max_rate;
  if (mobile_) {
    // Component-wise bound inscribed in the translational speed disk.
    const double linear = parameter("base_max_linear_speed", 0.1, true);
    const double angular = parameter("base_max_angular_speed", 0.2, true);
    // Existing profiles may use 0 for uncapped output. Do not change their
    // TCP-only behavior; use conservative limits for the new reference instead.
    body_velocity_limits_.head<2>().setConstant((linear > 0 ? linear : 0.1) / std::sqrt(2.0));
    body_velocity_limits_[2] = angular > 0 ? angular : 0.2;
  }
  tf_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
  target_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "tcp_current_target", rclcpp::QoS(1).transient_local());
  if (node->has_parameter("mock_debug_enabled") && node->get_parameter("mock_debug_enabled").as_bool()) {
    reference_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
        "tcp_reference_path", rclcpp::QoS(1).transient_local());
    base_reference_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(
        "base_reference_path", rclcpp::QoS(1).transient_local());
  }
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
        const auto generation = input_generation_.load();
        if (!session)
          return;
        const double values[] = {p->linear.x,  p->linear.y,  p->linear.z,
                                 p->angular.x, p->angular.y, p->angular.z};
        for (double v : values)
          if (!std::isfinite(v))
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!target_session_.accepts(session) || motion_owned_ || profile_change_pending_ ||
            generation != input_generation_.load())
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
  continuous_path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "target_path/continuous", 1,
      [this](nav_msgs::msg::Path::ConstSharedPtr p) { acceptPath(*p, true); });
}
bool TcpReferenceManager::transform(const geometry_msgs::msg::PoseStamped &p,
                                    vector_t &result) const {
  try {
    auto converted = p;
    if (!p.header.frame_id.empty() && p.header.frame_id != base_frame_)
      tf2::doTransform(p, converted,
                       tf_->lookupTransform(base_frame_, p.header.frame_id,
                                            rclcpp::Time(p.header.stamp)));
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
  const auto generation = input_generation_.load();
  if (!session)
    return;
  vector_t goal;
  if (!transform(p, goal))
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!target_session_.accepts(session) || motion_owned_ || profile_change_pending_ ||
      generation != input_generation_.load())
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
  const auto generation = input_generation_.load();
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
    if (!target_session_.accepts(session) || motion_owned_ || profile_change_pending_ ||
        generation != input_generation_.load())
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
void TcpReferenceManager::acceptPath(const nav_msgs::msg::Path &path, bool continuous) {
  const auto session = target_session_.token();
  const auto generation = input_generation_.load();
  if (!session || path.poses.empty() || path.poses.size() > 1000)
    return;
  vector_array_t goals;
  if (!transformPath(path, goals))
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!target_session_.accepts(session) || motion_owned_ || profile_change_pending_ ||
      generation != input_generation_.load())
    return;
  try {
    plan(goals, duration_, nullptr, 1.0, continuous);
    twist_active_ = false;
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
                               double requested_duration, const vector_t* measured_start,
                               double speed_scale, bool continuous,
                               const vector_array_t& body_states, const vector_t& body_weights) {
  vector_t start = measured_start ? *measured_start : sample(time_);
  const auto finish = [&](scalar_array_t times, vector_array_t poses, scalar_array_t knots) {
    std::shared_ptr<WholeBodyTrajectory> body;
    if (!body_states.empty()) {
      body = makeWholeBodyTrajectory(knots, body_states, body_weights, mobile_ ? 2 : -1,
                                     body_velocity_limits_ * speed_scale);
      const double scale = (body->times.back()-time_) / (times.back()-time_);
      for (auto& t : times) t = time_ + (t-time_) * scale;
    }
    if (!std::isfinite(times.back()) || (body && times.back()-time_ > 3600))
      throw std::invalid_argument("Motion exceeds one-hour reference budget");
    commit(TargetTrajectories(times, poses, vector_array_t(times.size(), vector_t::Zero(input_dim_))), body);
    times_ = std::move(times); poses_ = std::move(poses); target_ = poses_.back();
  };
  if (continuous) {
    TcpPathLimits limits{max_linear_ * speed_scale, max_angular_ * speed_scale,
        max_linear_acc_ * speed_scale, max_angular_acc_ * speed_scale,
        max_linear_jerk_ * speed_scale, max_angular_jerk_ * speed_scale,
        interval_, continuous_max_deviation_};
    std::vector<vector_t> points{start};
    points.insert(points.end(), goals.begin(), goals.end());
    std::vector<double> steps;
    if (!body_states.empty()) {
      for (size_t i = 1; i < body_states.size(); ++i) {
        auto delta = (body_states[i] - body_states[i-1]).eval();
        if (mobile_) delta[2] = std::atan2(std::sin(delta[2]), std::cos(delta[2]));
        double step = interval_;
        for (int j = 0; j < delta.size(); ++j) if (body_weights[j] > 0)
          step = std::max(step, std::abs(delta[j]) / (body_velocity_limits_[j]*speed_scale));
        steps.push_back(step);
      }
    }
    auto path = makeContinuousTcpPath(points, limits, requested_duration, steps);
    for (size_t i = 0; i < path.times.size(); ++i) {
      path.times[i] += time_;
      if (!std::isfinite(path.times[i]) || (i && path.times[i] <= path.times[i - 1]))
        throw std::invalid_argument("Continuous path timestamps lose precision");
    }
    // Build fully on the executor; OCS2 swaps the buffered trajectory only in
    // preSolverRun. No partial path is exposed and no work moves into RT update.
    for (auto& t : path.waypoint_times) t += time_;
    finish(std::move(path.times), std::move(path.poses), std::move(path.waypoint_times));
    return;
  }
  scalar_array_t times{time_};
  scalar_array_t knots{time_};
  vector_array_t poses{start};
  double elapsed = 0.0;
  for (const auto &goal : goals) {
    const double distance = (goal.head<3>() - start.head<3>()).norm();
    const double angle = quaternion(start).angularDistance(quaternion(goal));
    const double duration =
        std::max({interval_, requested_duration / goals.size(),
                  minimumDuration(distance, max_linear_ * speed_scale, max_linear_acc_ * speed_scale,
                                  max_linear_jerk_ * speed_scale),
                  minimumDuration(angle, max_angular_ * speed_scale, max_angular_acc_ * speed_scale,
                                  max_angular_jerk_ * speed_scale)});
    if (!std::isfinite(duration) || duration / interval_ > 100000)
      throw std::invalid_argument("TCP path duration exceeds sampling budget");
    const size_t count = std::max<size_t>(1, std::ceil(duration / interval_));
    if (count > 100000 || poses.size() + count > 200000)
      throw std::invalid_argument("TCP path too long");
    for (size_t i = 1; i <= count; ++i) {
      const double t = static_cast<double>(i) / count;
      times.push_back(time_ + elapsed + t * duration);
      poses.push_back(interpolate(start, goal, smooth(t)));
    }
    elapsed += duration;
    knots.push_back(time_ + elapsed);
    start = goal;
  }
  finish(std::move(times), std::move(poses), std::move(knots));
}

bool TcpReferenceManager::claimMotion() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (motion_owned_ || profile_change_pending_ || !target_session_.token()) return false;
  motion_owned_ = true;
  motion_session_ = target_session_.token();
  ++input_generation_;
  twist_active_ = false;
  return true;
}
void TcpReferenceManager::releaseMotion() {
  std::lock_guard<std::mutex> lock(mutex_);
  motion_owned_ = false;
  motion_session_ = 0;
  ++input_generation_;
}
bool TcpReferenceManager::motionSessionValid() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return motion_owned_ && !profile_change_pending_ && target_session_.accepts(motion_session_);
}
bool TcpReferenceManager::beginProfileChange(const vector_t& measured_pose) {
  if (measured_pose.size() != 7 || !measured_pose.allFinite() ||
      !std::isfinite(measured_pose.tail<4>().norm()) || measured_pose.tail<4>().norm() < 1e-9)
    return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (motion_owned_) return false;
  profile_change_pending_ = true;
  ++input_generation_;
  twist_active_ = false;
  target_ = measured_pose;
  target_.tail<4>().normalize();
  time_ = last_time_ = observation_time_.load(std::memory_order_acquire);
  commitImmediate();
  publish();
  return true;
}
void TcpReferenceManager::endProfileChange() {
  std::lock_guard<std::mutex> lock(mutex_);
  profile_change_pending_ = false;
  ++input_generation_;
}
bool TcpReferenceManager::validateWholeBodyGoal(
    const zephyr_msgs::action::ExecuteCartesianMotion::Goal& g, std::string& error) const {
  using Goal = zephyr_msgs::action::ExecuteCartesianMotion::Goal;
  try {
    for (double w : {g.base_position_weight, g.base_yaw_weight, g.posture_weight})
      if (!std::isfinite(w) || w < 0 || w > 1000)
        throw std::invalid_argument("Whole-body weights must be finite within [0,1000]");
    const bool base = g.base_position_weight > 0 || g.base_yaw_weight > 0;
    const bool posture = g.posture_weight > 0;
    if (g.whole_body_references.empty()) {
      if (base || posture) throw std::invalid_argument("Whole-body weights require references");
      return true;
    }
    if (joints_.empty()) throw std::invalid_argument("Whole-body model not configured");
    if (joints_.size() + (mobile_ ? 3 : 0) > 32)
      throw std::invalid_argument("Whole-body reference supports up to 32 scalar states");
    if (!base && !posture) throw std::invalid_argument("Whole-body references require a positive weight");
    if (base && !mobile_) throw std::invalid_argument("Base reference requires a mobile model");
    if (g.mode != Goal::POSE && g.mode != Goal::CONTINUOUS_PATH)
      throw std::invalid_argument("Whole-body references support POSE or CONTINUOUS_PATH only");
    if (g.whole_body_references.size() != (g.mode == Goal::POSE ? 1 : g.waypoints.size()))
      throw std::invalid_argument("Whole-body reference count must match TCP targets");
    if ((g.base_position_weight > 0 && (!std::isfinite(g.base_position_tolerance) || g.base_position_tolerance <= 0)) ||
        (g.base_yaw_weight > 0 && (!std::isfinite(g.base_yaw_tolerance) ||
                                 g.base_yaw_tolerance <= 0 || g.base_yaw_tolerance > M_PI)))
      throw std::invalid_argument("Invalid base arrival tolerance");
    const auto& names = g.whole_body_references.front().posture.name;
    for (const auto& r : g.whole_body_references) {
      if (base) {
        if (r.base_pose.header.frame_id != base_frame_)
          throw std::invalid_argument("Base reference must use fixed MPC reference frame " + base_frame_);
        const auto p = poseVector(r.base_pose.pose);
        if (std::abs(p[2]) > 1e-6 || std::abs(p[3]) > 1e-6 || std::abs(p[4]) > 1e-6)
          throw std::invalid_argument("Base reference must be planar: z=roll=pitch=0");
      } else if (!r.base_pose.header.frame_id.empty())
        throw std::invalid_argument("Base reference supplied without an active base weight");
      const auto& q = r.posture;
      if (!q.velocity.empty() || !q.effort.empty() || !q.header.frame_id.empty())
        throw std::invalid_argument("Posture supports named joint positions only");
      if (!posture) {
        if (!q.name.empty() || !q.position.empty()) throw std::invalid_argument("Posture supplied without posture weight");
        continue;
      }
      if (names.empty() || q.name != names || q.name.size() != q.position.size())
        throw std::invalid_argument("Posture must use the same nonempty joint names at every waypoint");
      for (size_t i = 0; i < names.size(); ++i) {
        const auto it = std::find(joints_.begin(), joints_.end(), names[i]);
        if (it == joints_.end() || std::find(names.begin(), names.begin()+i, names[i]) != names.begin()+i)
          throw std::invalid_argument("Unknown or duplicate posture joint " + names[i]);
        const size_t index = it-joints_.begin();
        if (!std::isfinite(q.position[i]) || q.position[i] < lower_[index] || q.position[i] > upper_[index])
          throw std::invalid_argument("Posture outside model joint limits: " + names[i]);
      }
    }
    return true;
  } catch (const std::exception& e) { error = e.what(); return false; }
}
void TcpReferenceManager::prepareWholeBody(
    const zephyr_msgs::action::ExecuteCartesianMotion::Goal& g, const vector_t& measured,
    vector_array_t& states, vector_t& weights) const {
  std::string error;
  if (!validateWholeBodyGoal(g, error)) throw std::invalid_argument(error);
  if (g.whole_body_references.empty() && !gripper_.enabled()) return;
  const size_t offset = mobile_ ? 3 : 0, dim = joints_.size()+offset;
  if (measured.size() != static_cast<int>(dim) || !measured.allFinite())
    throw std::invalid_argument("Whole-body measured state unavailable");
  states = {measured}; weights = vector_t::Zero(dim);
  if (gripper_.enabled()) weights[gripper_index_] = gripper_.weight;
  if (mobile_) weights.head<3>() << g.base_position_weight, g.base_position_weight, g.base_yaw_weight;
  const size_t count = g.mode == zephyr_msgs::action::ExecuteCartesianMotion::Goal::POSE ||
      g.mode == zephyr_msgs::action::ExecuteCartesianMotion::Goal::RELATIVE ? 1 : g.waypoints.size();
  for (size_t k = 0; k < count; ++k) {
    const zephyr_msgs::msg::WholeBodyReference empty;
    const auto& r = g.whole_body_references.empty() ? empty : g.whole_body_references[k];
    vector_t state = measured;
    if (gripper_.enabled()) state[gripper_index_] = gripper_target_;
    if (g.base_position_weight > 0 || g.base_yaw_weight > 0) {
      const auto p = poseVector(r.base_pose.pose);
      if (g.base_position_weight > 0) state.head<2>() = p.head<2>();
      if (g.base_yaw_weight > 0) state[2] = 2*std::atan2(p[5], p[6]);
    }
    for (size_t i = 0; i < r.posture.name.size(); ++i) {
      const size_t j = std::find(joints_.begin(), joints_.end(), r.posture.name[i])-joints_.begin()+offset;
      state[j] = r.posture.position[i];
      weights[j] = static_cast<int>(j) == gripper_index_ ? gripper_.weight : g.posture_weight;
    }
    states.push_back(std::move(state));
  }
}
bool TcpReferenceManager::gripperAtTarget(const vector_t& state, double speed) const {
  if (!gripper_.enabled()) return true;
  std::lock_guard<std::mutex> lock(mutex_);
  return state.size() > gripper_index_ && GripperConfig::validPosition(state[gripper_index_]) &&
      std::isfinite(speed) && speed >= 0 && speed <= 0.02 &&
      std::abs(state[gripper_index_] - gripper_target_) <= gripper_.tolerance;
}
std::array<double, 2> TcpReferenceManager::wholeBodyErrors(const vector_t& state) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_body_ || !mobile_) return {0, 0};
  if (state.size() != pending_body_->weights.size() || !state.allFinite())
    return {INFINITY, INFINITY};
  const auto& target = pending_body_->states.back();
  const double yaw = state[2]-target[2];
  return {pending_body_->weights[0] > 0 ? (state.head<2>()-target.head<2>()).norm() : 0,
          pending_body_->weights[2] > 0 ? std::abs(std::atan2(std::sin(yaw), std::cos(yaw))) : 0};
}
bool TcpReferenceManager::startMotion(
    const zephyr_msgs::action::ExecuteCartesianMotion::Goal& request,
    const vector_t& measured_pose, geometry_msgs::msg::PoseStamped& target,
    double& end_time, std::string& error, const vector_t& measured_state) {
  using Goal = zephyr_msgs::action::ExecuteCartesianMotion::Goal;
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!motion_owned_ || profile_change_pending_ || !target_session_.accepts(motion_session_))
      throw std::runtime_error("Controller no longer ready for this motion");
    vector_array_t goals;
    if (request.mode == Goal::RELATIVE) {
      vector_t p = measured_pose;
      const auto& d = request.relative;
      Eigen::Vector3d translation(d.twist.linear.x, d.twist.linear.y, d.twist.linear.z);
      Eigen::Vector3d rotation(d.twist.angular.x, d.twist.angular.y, d.twist.angular.z);
      Eigen::Quaterniond frame = Eigen::Quaterniond::Identity();
      if (d.header.frame_id == tcp_frame_) frame = quaternion(p);
      else if (!d.header.frame_id.empty() && d.header.frame_id != base_frame_) {
        const auto tf = tf_->lookupTransform(base_frame_, d.header.frame_id, rclcpp::Time(d.header.stamp));
        frame = Eigen::Quaterniond(tf.transform.rotation.w, tf.transform.rotation.x,
                                  tf.transform.rotation.y, tf.transform.rotation.z).normalized();
      }
      p.head<3>() += frame * translation;
      if (rotation.norm() > 1e-12)
        p.tail<4>() = (frame * Eigen::Quaterniond(Eigen::AngleAxisd(rotation.norm(), rotation.normalized())) *
                       frame.conjugate() * quaternion(p)).normalized().coeffs();
      if (!p.allFinite()) throw std::invalid_argument("Relative motion overflow");
      goals.push_back(std::move(p));
    } else {
      const auto poses = request.mode == Goal::POSE ?
          std::vector<geometry_msgs::msg::PoseStamped>{request.target} : request.waypoints;
      for (const auto& p : poses) {
        vector_t converted;
        if (!transform(p, converted)) throw std::invalid_argument("Invalid pose or unavailable timestamped TF");
        goals.push_back(std::move(converted));
      }
    }
    if (goals.empty()) throw std::invalid_argument("Empty motion");
    vector_array_t body_states;
    vector_t body_weights;
    prepareWholeBody(request, measured_state, body_states, body_weights);
    time_ = observation_time_.load(std::memory_order_acquire);
    plan(goals, 0.0, &measured_pose, request.speed_scale, request.mode == Goal::CONTINUOUS_PATH,
         body_states, body_weights);
    end_time = times_.back();
    target.header.frame_id = base_frame_;
    target.header.stamp = node_->now();
    target.pose = rosPose(target_);
    publish();
    return true;
  } catch (const std::exception& e) {
    error = e.what();
    return false;
  }
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
  commit(TargetTrajectories({time_}, {target_}, {vector_t::Zero(input_dim_)}));
}
void TcpReferenceManager::setCurrentObservation(
    const SystemObservation &observation) {
  observation_time_.store(observation.time, std::memory_order_release);
}
bool TcpReferenceManager::requestReset(const vector_t &pose, double time, double gripper_position) {
  if (pose.size() != 7 || !pose.allFinite() || !std::isfinite(time))
    return false;
  if (gripper_.enabled() && !GripperConfig::validPosition(gripper_position)) return false;
  ResetRequest request;
  std::copy_n(pose.data(), 7, request.pose.begin());
  request.time = time;
  request.gripper_position = gripper_position;
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
    gripper_target_ = reset.gripper_position;
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
  if (!target_session_.token() || profile_change_pending_) {
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
  if (reference_path_pub_) {
    nav_msgs::msg::Path path;
    path.header = p.header;
    for (size_t i = 0; i < times_.size(); ++i) {
      geometry_msgs::msg::PoseStamped sample;
      sample.header.frame_id = base_frame_;
      sample.header.stamp = rclcpp::Time(static_cast<int64_t>(times_[i] * 1e9));
      sample.pose = rosPose(poses_[i]);
      path.poses.push_back(std::move(sample));
    }
    reference_path_pub_->publish(path);
    nav_msgs::msg::Path base_path;
    base_path.header = p.header;
    if (pending_body_ && mobile_) for (double time : times_) {
      const auto state = pending_body_->sample(time);
      geometry_msgs::msg::PoseStamped base;
      base.header.frame_id = base_frame_;
      base.header.stamp = rclcpp::Time(static_cast<int64_t>(time*1e9));
      base.pose.position.x = state[0]; base.pose.position.y = state[1];
      base.pose.orientation.z = std::sin(state[2]/2); base.pose.orientation.w = std::cos(state[2]/2);
      base_path.poses.push_back(base);
    }
    base_reference_path_pub_->publish(base_path); // Empty explicitly clears retired reference.
  }
}

} // namespace zephyr::mpc
