#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/action/parallel_gripper_command.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "normalized_gripper_controller/normalized_gripper_controller_parameters.hpp"
#include "normalized_gripper_controller/visibility_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_server_goal_handle.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace normalized_gripper_controller {

class NormalizedGripperController
    : public controller_interface::ChainableControllerInterface {
public:
  struct Commands {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  NormalizedGripperController();

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  bool on_set_chained_mode(bool chained_mode) override;

  NORMALIZED_GRIPPER_CONTROLLER_PUBLIC
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override;

  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) override;

private:
  using GripperCommandAction = control_msgs::action::ParallelGripperCommand;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GripperCommandAction>;
  using RealtimeGoalHandle =
      realtime_tools::RealtimeServerGoalHandle<GripperCommandAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBox =
      realtime_tools::RealtimeThreadSafeBox<RealtimeGoalHandlePtr>;

  bool validate_interfaces() const;
  bool parse_command(const sensor_msgs::msg::JointState &message,
                     Commands &command) const;
  void receive_command(const sensor_msgs::msg::JointState &message);
  void set_hold_position();
  void preempt_active_goal();
  void check_for_success(const rclcpp::Time &time, double current_position,
                         double current_velocity);

  rclcpp_action::GoalResponse
  goal_callback(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const GripperCommandAction::Goal> goal);
  rclcpp_action::CancelResponse
  cancel_callback(const std::shared_ptr<GoalHandle> goal_handle);
  void accepted_callback(std::shared_ptr<GoalHandle> goal_handle);

  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      position_command_interface_;
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      velocity_command_interface_;
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      effort_command_interface_;

  std::optional<
      std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      position_state_interface_;
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      velocity_state_interface_;
  std::optional<
      std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      effort_state_interface_;

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;

  realtime_tools::RealtimeBuffer<Commands> command_;
  Commands command_from_subscriber_;
  Commands final_command_;
  std::vector<hardware_interface::CommandInterface::SharedPtr>
      reference_handles_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      command_subscriber_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
      state_publisher_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
      realtime_state_publisher_;

  rclcpp_action::Server<GripperCommandAction>::SharedPtr action_server_;
  RealtimeGoalHandleBox active_goal_;
  rclcpp::TimerBase::SharedPtr goal_handle_timer_;
  std::shared_ptr<GripperCommandAction::Result> action_result_;
  rclcpp::Duration action_monitor_period_{0, 0};
  rclcpp::Time last_movement_time_{0, 0, RCL_ROS_TIME};
  double active_goal_position_{0.0};
  std::atomic_bool active_{false};
  std::atomic_bool chained_mode_{false};
};

} // namespace normalized_gripper_controller
