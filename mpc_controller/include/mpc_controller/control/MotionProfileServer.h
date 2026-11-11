// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <mutex>
#include <mpc_controller/control/MotionRuntime.h>
#include <mpc_controller/control/RuntimeMotionProfile.h>
#include <mpc_controller/control/TcpReferenceManager.h>
#include <zephyr_msgs/msg/motion_profile_status.hpp>
#include <zephyr_msgs/srv/set_motion_profile.hpp>

namespace zephyr::mpc {
/** Executor-side coordinator. No service, mutex or DDS work runs in update().
 * Request acceptance is separate from activation of a matching MPC policy.
 * This component neither arbitrates chassis commands nor confirms a base stop.
 */
class MotionProfileServer {
 public:
  MotionProfileServer(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                      std::shared_ptr<TcpReferenceManager> reference,
                      std::shared_ptr<RuntimeMotionProfile> profile,
                      std::shared_ptr<MotionRuntime> runtime);
 private:
  using SetProfile = zephyr_msgs::srv::SetMotionProfile;
  using ProfileStatus = zephyr_msgs::msg::MotionProfileStatus;
  void request(const SetProfile::Request&, SetProfile::Response&);
  void tick();
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<TcpReferenceManager> reference_;
  std::shared_ptr<RuntimeMotionProfile> profile_;
  std::shared_ptr<MotionRuntime> runtime_;
  std::mutex mutex_;
  RuntimeProfileConfig requested_config_;
  bool profile_submitted_{true};
  bool transition_pending_{false}, owns_reference_gate_{false};
  bool handover_failed_{false};
  uint64_t request_activation_epoch_{0};
  std::string detail_;
  rclcpp::Service<SetProfile>::SharedPtr service_;
  rclcpp::Publisher<ProfileStatus>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace zephyr::mpc
