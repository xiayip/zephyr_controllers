// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zephyr_msgs/action/execute_cartesian_motion.hpp>
#include <mpc_controller/control/MotionRuntime.h>
#include <mpc_controller/control/TcpReferenceManager.h>

namespace zephyr::mpc {
// All callbacks use the lifecycle node's mutually-exclusive default callback group.
// RT and lifecycle code communicate only through MotionRuntime.
class MotionActionServer {
public:
  using Action = zephyr_msgs::action::ExecuteCartesianMotion;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  MotionActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                    std::shared_ptr<TcpReferenceManager> reference,
                    std::shared_ptr<MotionRuntime> runtime);
private:
  rclcpp_action::GoalResponse goal(const rclcpp_action::GoalUUID&,
                                 std::shared_ptr<const Action::Goal>);
  void accepted(std::shared_ptr<Handle>);
  void tick();
  void stop(uint16_t reason, const std::string& message);
  void finish(bool success, const MotionSample& sample);
  geometry_msgs::msg::PoseStamped actual(const MotionSample&) const;
  bool fresh(const MotionSample&) const;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<TcpReferenceManager> reference_;
  std::shared_ptr<MotionRuntime> runtime_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Publisher<MotionStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Handle> handle_;
  geometry_msgs::msg::PoseStamped target_;
  bool reserved_{false}, stopping_{false};
  double end_time_{0}, position_error_{0}, orientation_error_{0};
  int64_t started_ns_{0}, settled_ns_{0}, stopped_ns_{0};
  uint64_t stop_token_{0}, activation_epoch_{0};
  uint16_t reason_{MotionStatus::NONE};
  std::string message_;
};
}  // namespace zephyr::mpc
