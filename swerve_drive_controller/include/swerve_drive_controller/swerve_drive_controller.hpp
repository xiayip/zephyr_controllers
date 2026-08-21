#ifndef SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_
#define SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace swerve_drive_controller
{

class SwerveController : public controller_interface::ControllerInterface
{
public:
  SwerveController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Command
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;
  };

  bool assign_interface_indices();
  void receive_command(const geometry_msgs::msg::Twist & twist, const rclcpp::Time & stamp);
  void halt();
  void publish_odometry(const rclcpp::Time & time, const BaseVelocity & velocity);

  std::array<std::string, 4> wheel_joint_names_;
  std::array<std::string, 4> steering_joint_names_;
  std::array<double, 4> steering_directions_{{1.0, 1.0, 1.0, 1.0}};
  std::array<double, 4> max_wheel_velocities_{{0.0, 0.0, 0.0, 0.0}};
  std::array<std::size_t, 4> wheel_command_indices_{};
  std::array<std::size_t, 4> steering_command_indices_{};
  std::array<std::size_t, 4> wheel_state_indices_{};
  std::array<std::size_t, 4> steering_state_indices_{};

  SwerveDriveKinematics kinematics_;
  double steering_limit_ = 1.5707963267948966;
  std::chrono::milliseconds command_timeout_{500};
  bool use_stamped_vel_ = false;
  bool enable_odom_tf_ = true;
  bool open_loop_ = false;
  bool interfaces_assigned_ = false;
  std::string cmd_vel_topic_ = "~/cmd_vel";
  std::string odom_frame_id_ = "odom";
  std::string base_frame_id_ = "base_link";
  rclcpp::Duration publish_period_{0, 0};
  rclcpp::Time previous_publish_time_{0, 0, RCL_ROS_TIME};
  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_yaw_ = 0.0;

  std::atomic_bool subscriber_is_active_{false};
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_command_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
    realtime_odometry_publisher_;
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> transform_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
    realtime_transform_publisher_;
};

}  // namespace swerve_drive_controller

#endif  // SWERVE_DRIVE_CONTROLLER__SWERVE_DRIVE_CONTROLLER_HPP_