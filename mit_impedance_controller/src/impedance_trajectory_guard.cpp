#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "mit_impedance_controller/blocked_motion_detector.hpp"

namespace mit_impedance_controller {

class ImpedanceTrajectoryGuard final : public rclcpp::Node {
public:
  ImpedanceTrajectoryGuard() : Node("impedance_trajectory_guard") {
    joint_names_ = declare_parameter<std::vector<std::string>>(
        "joints", std::vector<std::string>{});
    const auto position_error_thresholds =
        declare_parameter<std::vector<double>>("position_error_threshold",
                                               std::vector<double>{});
    const auto stationary_velocity_thresholds =
        declare_parameter<std::vector<double>>("stationary_velocity_threshold",
                                               std::vector<double>{});
    const double blocking_duration =
        declare_parameter<double>("blocking_duration", 0.3);
    const std::string state_topic =
        declare_parameter<std::string>("state_topic", "impedance_state");
    const std::string action_name = declare_parameter<std::string>(
        "trajectory_action", "follow_joint_trajectory");

    if (joint_names_.empty() || state_topic.empty() || action_name.empty() ||
        position_error_thresholds.size() != joint_names_.size() ||
        stationary_velocity_thresholds.size() != joint_names_.size()) {
      throw std::invalid_argument(
          "trajectory guard requires one threshold per configured joint");
    }
    std::string error;
    if (!detector_.configure(position_error_thresholds,
                             stationary_velocity_thresholds, blocking_duration,
                             error)) {
      throw std::invalid_argument(error);
    }

    trajectory_client_ =
        rclcpp_action::create_client<TrajectoryAction>(this, action_name);
    state_subscription_ =
        create_subscription<control_msgs::msg::DynamicJointState>(
            state_topic, rclcpp::SensorDataQoS(),
            [this](const control_msgs::msg::DynamicJointState::SharedPtr msg) {
              process_state(*msg);
            });
    RCLCPP_INFO(get_logger(),
                "Monitoring '%s' and canceling blocked trajectories on '%s'",
                state_topic.c_str(), action_name.c_str());
  }

private:
  using TrajectoryAction = control_msgs::action::FollowJointTrajectory;

  static bool
  read_interface(const control_msgs::msg::DynamicJointState &message,
                 const std::string &joint_name,
                 const std::string &interface_name, double &value) {
    const auto joint = std::find(message.joint_names.begin(),
                                 message.joint_names.end(), joint_name);
    if (joint == message.joint_names.end()) {
      return false;
    }
    const auto joint_index = static_cast<std::size_t>(
        std::distance(message.joint_names.begin(), joint));
    if (joint_index >= message.interface_values.size()) {
      return false;
    }
    const auto &interfaces = message.interface_values[joint_index];
    const auto interface =
        std::find(interfaces.interface_names.begin(),
                  interfaces.interface_names.end(), interface_name);
    if (interface == interfaces.interface_names.end()) {
      return false;
    }
    const auto interface_index = static_cast<std::size_t>(
        std::distance(interfaces.interface_names.begin(), interface));
    if (interface_index >= interfaces.values.size()) {
      return false;
    }
    value = interfaces.values[interface_index];
    return true;
  }

  void process_state(const control_msgs::msg::DynamicJointState &message) {
    const auto now = std::chrono::steady_clock::now();
    double period_seconds = 0.0;
    if (last_sample_time_.time_since_epoch().count() != 0) {
      period_seconds =
          std::chrono::duration<double>(now - last_sample_time_).count();
    }
    last_sample_time_ = now;

    std::vector<double> position_errors(joint_names_.size());
    std::vector<double> velocities(joint_names_.size());
    for (std::size_t index = 0; index < joint_names_.size(); ++index) {
      if (!read_interface(message, joint_names_[index], "position_error",
                          position_errors[index]) ||
          !read_interface(message, joint_names_[index], "velocity",
                          velocities[index])) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Impedance state is missing position_error or velocity");
        detector_.reset();
        return;
      }
    }

    if (!detector_.update(position_errors, velocities, period_seconds)) {
      return;
    }
    const auto joint_index = detector_.last_triggered_joint();
    if (!trajectory_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(),
                  "Blocked motion detected on '%s', but the trajectory action "
                  "server is unavailable",
                  joint_names_[joint_index].c_str());
      detector_.reset();
      return;
    }

    RCLCPP_WARN(get_logger(),
                "Blocked motion detected on '%s'; canceling the active "
                "impedance trajectory",
                joint_names_[joint_index].c_str());
    (void)trajectory_client_->async_cancel_all_goals();
  }

  std::vector<std::string> joint_names_;
  BlockedMotionDetector detector_;
  std::chrono::steady_clock::time_point last_sample_time_{};
  rclcpp_action::Client<TrajectoryAction>::SharedPtr trajectory_client_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr
      state_subscription_;
};

} // namespace mit_impedance_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<mit_impedance_controller::ImpedanceTrajectoryGuard>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("impedance_trajectory_guard"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
