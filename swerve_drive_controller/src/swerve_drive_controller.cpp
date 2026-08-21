#include "swerve_drive_controller/swerve_drive_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace swerve_drive_controller
{

namespace
{

constexpr std::array<const char *, 4> WHEEL_PARAMETER_NAMES = {
  "front_left_wheel_joint", "front_right_wheel_joint",
  "rear_left_wheel_joint", "rear_right_wheel_joint"};
constexpr std::array<const char *, 4> LEGACY_WHEEL_PARAMETER_NAMES = {
  "joint_wheel_left_front", "joint_wheel_right_front",
  "joint_wheel_left_rear", "joint_wheel_right_rear"};
constexpr std::array<const char *, 4> STEERING_PARAMETER_NAMES = {
  "front_left_steering_joint", "front_right_steering_joint",
  "rear_left_steering_joint", "rear_right_steering_joint"};
constexpr std::array<const char *, 4> LEGACY_STEERING_PARAMETER_NAMES = {
  "joint_steering_left_front", "joint_steering_right_front",
  "joint_steering_left_rear", "joint_steering_right_rear"};
constexpr std::array<const char *, 4> STEERING_DIRECTION_PARAMETER_NAMES = {
  "front_left_steering_direction", "front_right_steering_direction",
  "rear_left_steering_direction", "rear_right_steering_direction"};
constexpr std::array<const char *, 4> LEGACY_VELOCITY_LIMIT_PARAMETER_NAMES = {
  "front_left_velocity_threshold", "front_right_velocity_threshold",
  "rear_left_velocity_threshold", "rear_right_velocity_threshold"};

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

SwerveController::SwerveController()
: kinematics_({{{0.25, 0.19}, {0.25, -0.19}, {-0.25, 0.19}, {-0.25, -0.19}}})
{
  command_buffer_.writeFromNonRT(Command{});
}

controller_interface::CallbackReturn SwerveController::on_init()
{
  try {
    for (const auto * parameter_name : WHEEL_PARAMETER_NAMES) {
      auto_declare<std::string>(parameter_name, "");
    }
    for (const auto * parameter_name : LEGACY_WHEEL_PARAMETER_NAMES) {
      auto_declare<std::string>(parameter_name, "");
    }
    for (const auto * parameter_name : STEERING_PARAMETER_NAMES) {
      auto_declare<std::string>(parameter_name, "");
    }
    for (const auto * parameter_name : LEGACY_STEERING_PARAMETER_NAMES) {
      auto_declare<std::string>(parameter_name, "");
    }
    for (const auto * parameter_name : STEERING_DIRECTION_PARAMETER_NAMES) {
      auto_declare<double>(parameter_name, 1.0);
    }
    for (const auto * parameter_name : LEGACY_VELOCITY_LIMIT_PARAMETER_NAMES) {
      auto_declare<double>(parameter_name, 0.0);
    }

    auto_declare<double>("wheel_base", 0.0);
    auto_declare<double>("track_width", 0.0);
    auto_declare<double>("chassis_length", 0.0);
    auto_declare<double>("chassis_width", 0.0);
    auto_declare<double>("wheel_radius", 0.0);
    auto_declare<double>("steering_limit", steering_limit_);
    auto_declare<double>("max_wheel_velocity", 0.0);
    auto_declare<double>("center_of_rotation", 0.0);
    auto_declare<double>("cmd_vel_timeout", 0.5);
    auto_declare<double>("publish_rate", 50.0);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
    auto_declare<bool>("enable_odom_tf", enable_odom_tf_);
    auto_declare<bool>("open_loop", open_loop_);
    auto_declare<std::string>("cmd_vel_topic", cmd_vel_topic_);
    auto_declare<std::string>("odom_frame_id", "");
    auto_declare<std::string>("base_frame_id", "");
    auto_declare<std::string>("odom", "");
    auto_declare<std::string>("base_footprint", "");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration()
const
{
  std::vector<std::string> interfaces;
  interfaces.reserve(8);
  for (const auto & joint_name : wheel_joint_names_) {
    interfaces.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint_name : steering_joint_names_) {
    interfaces.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces};
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  std::vector<std::string> interfaces;
  interfaces.reserve(8);
  for (const auto & joint_name : wheel_joint_names_) {
    interfaces.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (const auto & joint_name : steering_joint_names_) {
    interfaces.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces};
}

controller_interface::CallbackReturn SwerveController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  bool uses_legacy_parameters = false;
  try {
    for (std::size_t index = 0; index < wheel_joint_names_.size(); ++index) {
      const std::string wheel_joint =
        node->get_parameter(WHEEL_PARAMETER_NAMES[index]).as_string();
      const std::string legacy_wheel_joint =
        node->get_parameter(LEGACY_WHEEL_PARAMETER_NAMES[index]).as_string();
      wheel_joint_names_[index] = wheel_joint.empty() ? legacy_wheel_joint : wheel_joint;
      uses_legacy_parameters = uses_legacy_parameters ||
        (wheel_joint.empty() && !legacy_wheel_joint.empty());

      const std::string steering_joint =
        node->get_parameter(STEERING_PARAMETER_NAMES[index]).as_string();
      const std::string legacy_steering_joint =
        node->get_parameter(LEGACY_STEERING_PARAMETER_NAMES[index]).as_string();
      steering_joint_names_[index] =
        steering_joint.empty() ? legacy_steering_joint : steering_joint;
      uses_legacy_parameters = uses_legacy_parameters ||
        (steering_joint.empty() && !legacy_steering_joint.empty());

      steering_directions_[index] =
        node->get_parameter(STEERING_DIRECTION_PARAMETER_NAMES[index]).as_double();
      if (wheel_joint_names_[index].empty() || steering_joint_names_[index].empty()) {
        throw std::invalid_argument("All wheel and steering joint names must be set");
      }
      if (std::abs(std::abs(steering_directions_[index]) - 1.0) > 1.0e-9) {
        throw std::invalid_argument("Steering direction parameters must be either 1.0 or -1.0");
      }
    }

    double wheel_base = node->get_parameter("wheel_base").as_double();
    if (wheel_base <= 0.0) {
      wheel_base = node->get_parameter("chassis_length").as_double();
      uses_legacy_parameters = uses_legacy_parameters || wheel_base > 0.0;
    }
    double track_width = node->get_parameter("track_width").as_double();
    if (track_width <= 0.0) {
      track_width = node->get_parameter("chassis_width").as_double();
      uses_legacy_parameters = uses_legacy_parameters || track_width > 0.0;
    }
    double wheel_radius = node->get_parameter("wheel_radius").as_double();
    if (wheel_base <= 0.0 || track_width <= 0.0 || wheel_radius <= 0.0) {
      throw std::invalid_argument("wheel_base, track_width, and wheel_radius must be positive");
    }
    const double half_length = wheel_base / 2.0;
    const double half_width = track_width / 2.0;
    std::array<std::pair<double, double>, 4> wheel_positions = {{
      {half_length, half_width}, {half_length, -half_width},
      {-half_length, half_width}, {-half_length, -half_width}}};
    kinematics_.setWheelParams(wheel_positions, wheel_radius);

    steering_limit_ = node->get_parameter("steering_limit").as_double();
    if (steering_limit_ <= 0.0 || steering_limit_ > M_PI) {
      throw std::invalid_argument("steering_limit must be in the range (0, pi]");
    }

    const double max_wheel_velocity = node->get_parameter("max_wheel_velocity").as_double();
    for (std::size_t index = 0; index < max_wheel_velocities_.size(); ++index) {
      const double legacy_limit =
        node->get_parameter(LEGACY_VELOCITY_LIMIT_PARAMETER_NAMES[index]).as_double();
      max_wheel_velocities_[index] = legacy_limit > 0.0 ? legacy_limit : max_wheel_velocity;
    }

    command_timeout_ = std::chrono::milliseconds(
      static_cast<int64_t>(node->get_parameter("cmd_vel_timeout").as_double() * 1000.0));
    const double publish_rate = node->get_parameter("publish_rate").as_double();
    if (command_timeout_.count() <= 0 || publish_rate <= 0.0) {
      throw std::invalid_argument("cmd_vel_timeout and publish_rate must be positive");
    }
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
    use_stamped_vel_ = node->get_parameter("use_stamped_vel").as_bool();
    enable_odom_tf_ = node->get_parameter("enable_odom_tf").as_bool();
    open_loop_ = node->get_parameter("open_loop").as_bool();
    cmd_vel_topic_ = node->get_parameter("cmd_vel_topic").as_string();
    if (cmd_vel_topic_.empty()) {
      throw std::invalid_argument("cmd_vel_topic must not be empty");
    }

    odom_frame_id_ = node->get_parameter("odom_frame_id").as_string();
    if (odom_frame_id_.empty()) {
      odom_frame_id_ = node->get_parameter("odom").as_string();
      uses_legacy_parameters = uses_legacy_parameters || !odom_frame_id_.empty();
    }
    if (odom_frame_id_.empty()) {
      odom_frame_id_ = "odom";
    }
    base_frame_id_ = node->get_parameter("base_frame_id").as_string();
    if (base_frame_id_.empty()) {
      base_frame_id_ = node->get_parameter("base_footprint").as_string();
      uses_legacy_parameters = uses_legacy_parameters || !base_frame_id_.empty();
    }
    if (base_frame_id_.empty()) {
      base_frame_id_ = "base_link";
    }

    if (std::abs(node->get_parameter("center_of_rotation").as_double()) > 1.0e-9) {
      RCLCPP_WARN(
        node->get_logger(), "center_of_rotation is deprecated and currently ignored");
    }
    if (uses_legacy_parameters) {
      RCLCPP_WARN(
        node->get_logger(),
        "Legacy swerve_drive_controller parameter names are deprecated; use the canonical names");
    }

    if (use_stamped_vel_) {
      stamped_command_subscriber_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr message)
        {
          const rclcpp::Time stamp =
            message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0 ?
            get_node()->now() : rclcpp::Time(message->header.stamp);
          receive_command(message->twist, stamp);
        });
    } else {
      command_subscriber_ = node->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr message)
        {
          receive_command(*message, get_node()->now());
        });
    }

    odometry_publisher_ =
      node->create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::SystemDefaultsQoS());
    realtime_odometry_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);
    transform_publisher_ =
      node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
    realtime_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      transform_publisher_);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure controller: %s", exception.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!assign_interface_indices()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  interfaces_assigned_ = true;
  odom_x_ = 0.0;
  odom_y_ = 0.0;
  odom_yaw_ = 0.0;
  previous_publish_time_ = get_node()->now();
  Command command;
  command.stamp = get_node()->now();
  command_buffer_.writeFromNonRT(command);
  subscriber_is_active_.store(true);
  halt();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_.store(false);
  if (interfaces_assigned_) {
    halt();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_.store(false);
  interfaces_assigned_ = false;
  stamped_command_subscriber_.reset();
  command_subscriber_.reset();
  realtime_odometry_publisher_.reset();
  odometry_publisher_.reset();
  realtime_transform_publisher_.reset();
  transform_publisher_.reset();
  command_buffer_.writeFromNonRT(Command{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_error(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_.store(false);
  if (interfaces_assigned_) {
    halt();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_.store(false);
  if (interfaces_assigned_) {
    halt();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::array<double, 4> feedback_wheel_velocities{};
  std::array<double, 4> feedback_steering_positions{};
  bool feedback_is_valid = true;
  for (std::size_t index = 0; index < feedback_wheel_velocities.size(); ++index) {
    feedback_wheel_velocities[index] =
      state_interfaces_[wheel_state_indices_[index]].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
    feedback_steering_positions[index] =
      state_interfaces_[steering_state_indices_[index]].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN()) * steering_directions_[index];
    feedback_is_valid = feedback_is_valid &&
      std::isfinite(feedback_wheel_velocities[index]) &&
      std::isfinite(feedback_steering_positions[index]);
  }

  std::array<double, 4> current_steering_positions = feedback_steering_positions;
  if (!feedback_is_valid) {
    current_steering_positions.fill(0.0);
  }

  const Command command = *command_buffer_.readFromRT();
  const bool command_expired =
    command.stamp.nanoseconds() == 0 ||
    (time - command.stamp).nanoseconds() >
    std::chrono::duration_cast<std::chrono::nanoseconds>(command_timeout_).count();
  auto wheel_commands = command_expired ?
    kinematics_.compute_wheel_commands(
    0.0, 0.0, 0.0, current_steering_positions, steering_limit_) :
    kinematics_.compute_wheel_commands(
    command.linear_x, command.linear_y, command.angular_z,
    current_steering_positions, steering_limit_);

  for (std::size_t index = 0; index < wheel_commands.size(); ++index) {
    if (max_wheel_velocities_[index] > 0.0) {
      wheel_commands[index].drive_velocity = std::clamp(
        wheel_commands[index].drive_velocity,
        -max_wheel_velocities_[index], max_wheel_velocities_[index]);
    }
    if (!command_interfaces_[wheel_command_indices_[index]].set_value(
        wheel_commands[index].drive_velocity) ||
      !command_interfaces_[steering_command_indices_[index]].set_value(
        steering_directions_[index] * wheel_commands[index].steering_angle))
    {
      return controller_interface::return_type::ERROR;
    }
  }

  std::array<double, 4> odometry_wheel_velocities{};
  std::array<double, 4> odometry_steering_positions{};
  if (open_loop_) {
    for (std::size_t index = 0; index < wheel_commands.size(); ++index) {
      odometry_wheel_velocities[index] = wheel_commands[index].drive_velocity;
      odometry_steering_positions[index] = wheel_commands[index].steering_angle;
    }
  } else {
    if (!feedback_is_valid) {
      return controller_interface::return_type::OK;
    }
    odometry_wheel_velocities = feedback_wheel_velocities;
    odometry_steering_positions = feedback_steering_positions;
  }

  const BaseVelocity velocity =
    kinematics_.estimate_velocity(odometry_wheel_velocities, odometry_steering_positions);
  const double dt = period.seconds();
  const double cos_yaw = std::cos(odom_yaw_);
  const double sin_yaw = std::sin(odom_yaw_);
  odom_x_ += (velocity.linear_x * cos_yaw - velocity.linear_y * sin_yaw) * dt;
  odom_y_ += (velocity.linear_x * sin_yaw + velocity.linear_y * cos_yaw) * dt;
  odom_yaw_ = normalize_angle(odom_yaw_ + velocity.angular_z * dt);
  publish_odometry(time, velocity);

  return controller_interface::return_type::OK;
}

bool SwerveController::assign_interface_indices()
{
  const auto find_index = [](const auto & interfaces, const std::string & name)
    -> std::optional<std::size_t>
    {
      for (std::size_t index = 0; index < interfaces.size(); ++index) {
        if (interfaces[index].get_name() == name) {
          return index;
        }
      }
      return std::nullopt;
    };

  for (std::size_t index = 0; index < wheel_joint_names_.size(); ++index) {
    const std::string wheel_interface =
      wheel_joint_names_[index] + "/" + hardware_interface::HW_IF_VELOCITY;
    const std::string steering_interface =
      steering_joint_names_[index] + "/" + hardware_interface::HW_IF_POSITION;
    const auto wheel_command = find_index(command_interfaces_, wheel_interface);
    const auto steering_command = find_index(command_interfaces_, steering_interface);
    const auto wheel_state = find_index(state_interfaces_, wheel_interface);
    const auto steering_state = find_index(state_interfaces_, steering_interface);
    if (!wheel_command || !steering_command || !wheel_state || !steering_state) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Missing command or state interface for wheel index %zu", index);
      return false;
    }
    wheel_command_indices_[index] = *wheel_command;
    steering_command_indices_[index] = *steering_command;
    wheel_state_indices_[index] = *wheel_state;
    steering_state_indices_[index] = *steering_state;
  }
  return true;
}

void SwerveController::receive_command(
  const geometry_msgs::msg::Twist & twist, const rclcpp::Time & stamp)
{
  if (!subscriber_is_active_.load()) {
    return;
  }
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
    !std::isfinite(twist.angular.z))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Ignoring cmd_vel containing a non-finite value");
    return;
  }
  Command command;
  command.stamp = stamp;
  command.linear_x = twist.linear.x;
  command.linear_y = twist.linear.y;
  command.angular_z = twist.angular.z;
  command_buffer_.writeFromNonRT(command);
}

void SwerveController::halt()
{
  for (std::size_t index = 0; index < wheel_command_indices_.size(); ++index) {
    const bool wheel_stopped =
      command_interfaces_[wheel_command_indices_[index]].set_value(0.0);
    const bool steering_centered =
      command_interfaces_[steering_command_indices_[index]].set_value(0.0);
    if (!wheel_stopped || !steering_centered) {
      RCLCPP_WARN(get_node()->get_logger(), "Failed to halt wheel index %zu", index);
    }
  }
}

void SwerveController::publish_odometry(
  const rclcpp::Time & time, const BaseVelocity & velocity)
{
  if (time - previous_publish_time_ < publish_period_) {
    return;
  }
  previous_publish_time_ = time;
  const double half_yaw = odom_yaw_ / 2.0;
  const double quaternion_z = std::sin(half_yaw);
  const double quaternion_w = std::cos(half_yaw);

  if (realtime_odometry_publisher_ && realtime_odometry_publisher_->trylock()) {
    auto & message = realtime_odometry_publisher_->msg_;
    message.header.stamp = time;
    message.header.frame_id = odom_frame_id_;
    message.child_frame_id = base_frame_id_;
    message.pose.pose.position.x = odom_x_;
    message.pose.pose.position.y = odom_y_;
    message.pose.pose.orientation.z = quaternion_z;
    message.pose.pose.orientation.w = quaternion_w;
    message.twist.twist.linear.x = velocity.linear_x;
    message.twist.twist.linear.y = velocity.linear_y;
    message.twist.twist.angular.z = velocity.angular_z;
    realtime_odometry_publisher_->unlockAndPublish();
  }

  if (enable_odom_tf_ && realtime_transform_publisher_ &&
    realtime_transform_publisher_->trylock())
  {
    auto & message = realtime_transform_publisher_->msg_;
    message.transforms.resize(1);
    auto & transform = message.transforms.front();
    transform.header.stamp = time;
    transform.header.frame_id = odom_frame_id_;
    transform.child_frame_id = base_frame_id_;
    transform.transform.translation.x = odom_x_;
    transform.transform.translation.y = odom_y_;
    transform.transform.rotation.z = quaternion_z;
    transform.transform.rotation.w = quaternion_w;
    realtime_transform_publisher_->unlockAndPublish();
  }
}

}  // namespace swerve_drive_controller

PLUGINLIB_EXPORT_CLASS(
  swerve_drive_controller::SwerveController,
  controller_interface::ControllerInterface)