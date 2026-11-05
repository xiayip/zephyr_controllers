// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mpc_controller/control/ControlSafety.h>
#include <nav_msgs/msg/path.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace zephyr::mpc {
/** One real TCP, with no arm selector or ROS name translation. */
class TcpReferenceManager final : public ocs2::ReferenceManagerDecorator {
public:
  TcpReferenceManager(std::shared_ptr<ocs2::ReferenceManagerInterface> wrapped,
                      std::string base_frame, std::string tcp_frame,
                      size_t input_dim);
  void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
  void setAcceptingTargets(bool accept);
  void setCurrentObservation(const ocs2::SystemObservation &observation);
  /** Nonblocking request; caller retries while the mailbox is busy. */
  bool requestReset(const ocs2::vector_t &pose, double time);
  bool resetReady() const;
  bool consumeFailure() {
    return failed_.exchange(false, std::memory_order_acq_rel);
  }

private:
  void
  updateReference(); // Executor timer only, never controller_manager::update.
  struct ResetRequest {
    std::array<double, 7> pose{};
    double time{0};
    uint64_t sequence{0};
  };
  TryWriteMailbox<ResetRequest> reset_mailbox_;
  uint64_t requested_reset_{0}; // RT writer only
  std::atomic<uint64_t> applied_reset_{0};
  std::atomic<double> observation_time_{0};
  std::atomic_bool failed_{false};
  rclcpp::TimerBase::SharedPtr reference_timer_;
  bool transform(const geometry_msgs::msg::PoseStamped &pose,
                 ocs2::vector_t &result) const;
  bool transformPath(const nav_msgs::msg::Path &path,
                     ocs2::vector_array_t &goals) const;
  void acceptPose(const geometry_msgs::msg::PoseStamped &pose,
                  bool interpolate);
  void acceptRelative(const geometry_msgs::msg::TwistStamped &delta);
  void acceptPath(const nav_msgs::msg::Path &path);
  void plan(const std::vector<ocs2::vector_t> &poses, double duration);
  ocs2::vector_t sample(double time) const;
  void publish() const;
  void commitImmediate();
  mutable std::mutex mutex_;
  TargetSession target_session_;
  std::string base_frame_, tcp_frame_;
  size_t input_dim_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  ocs2::vector_t target_;
  double time_{0.0}, last_time_{-1.0};
  ocs2::scalar_array_t times_;
  ocs2::vector_array_t poses_;
  geometry_msgs::msg::Twist twist_;
  std::chrono::steady_clock::time_point twist_received_{};
  bool twist_active_{false};
  double duration_{2.0}, interval_{0.04}, max_linear_{0.3}, max_angular_{1.0};
  double max_linear_acc_{1.0}, max_angular_acc_{2.0}, max_linear_jerk_{2.0},
      max_angular_jerk_{4.0};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr stamped_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      relative_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
};
} // namespace zephyr::mpc
