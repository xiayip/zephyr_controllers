#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <moveit_msgs/msg/servo_status.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include "teleop_joint_controller/reference_filter.hpp"

namespace teleop_joint_controller {
class TeleopJointController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
private:
  struct Waypoint {
    std::vector<double> positions;
    int64_t stamp_ns{0};
  };
  struct Command {
    std::vector<Waypoint> points;
    int64_t received_ns{0};
    int64_t source_ns{0};
    uint64_t session{0};
  };
  static int64_t steady_now();
  void receive(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void receive_servo_status(const moveit_msgs::msg::ServoStatus::SharedPtr msg);
  void fault(const char* reason);
  bool write();
  void publish(const rclcpp::Time&, double dt);
  bool read_positions();
  std::vector<std::string> joints_;
  std::string downstream_;
  std::vector<size_t> pos_commands_, vel_commands_, pos_states_;
  std::vector<double> lower_, upper_, actual_, target_, previous_target_, sampled_target_;
  ReferenceFilter filter_;
  double command_timeout_{0.1}, startup_timeout_{1.0}, max_period_{0.05};
  double max_target_step_{0.15}, max_tracking_error_{0.15};
  bool seen_command_{false}, faulted_{false};
  const char* reason_{"inactive"};
  int64_t activated_ns_{0}, last_received_ns_{0}, last_source_ns_{0};
  uint64_t current_session_{0};
  // Accessed only by the mutually exclusive subscription callback. Reject
  // reordered packets before they can replace the window used by update().
  uint64_t callback_session_{0};
  int64_t callback_source_ns_{0};
  std::atomic<uint64_t> active_session_{0}, invalid_session_{0};
  std::atomic<uint64_t> servo_stop_session_{0};
  std::atomic<int> servo_stop_code_{0};
  realtime_tools::RealtimeBuffer<Command> commands_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
  rclcpp::Subscription<moveit_msgs::msg::ServoStatus>::SharedPtr servo_status_subscription_;
  using State = control_msgs::msg::JointTrajectoryControllerState;
  using Diagnostics = diagnostic_msgs::msg::DiagnosticArray;
  std::shared_ptr<realtime_tools::RealtimePublisher<State>> state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<Diagnostics>> status_pub_;
  double status_elapsed_{0.0};
};
} // namespace teleop_joint_controller
