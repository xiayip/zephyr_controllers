// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <mutex>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <string>
#include <vector>

namespace zephyr::mpc {
struct KinematicsOptions {
  std::string group{"arm"}, solver{"auto"};
  int max_iterations{300};
  double tolerance{1e-4}, damping{0.01}, timeout{0.1};
  bool random_restart{false};
};

/** Pure kinematics services; never execute motion or claim collision-free
 * results. This reduced model is expressed in base_frame and contains only arm
 * joints.
 */
class TcpKinematics {
public:
  TcpKinematics(pinocchio::Model model, std::vector<std::string> joints,
                std::string tcp_frame, std::string base_frame,
                KinematicsOptions options = {});
  void fk(const moveit_msgs::srv::GetPositionFK::Request &request,
          moveit_msgs::srv::GetPositionFK::Response &response);
  void ik(const moveit_msgs::srv::GetPositionIK::Request &request,
          moveit_msgs::srv::GetPositionIK::Response &response);

private:
  bool configuration(const moveit_msgs::msg::RobotState &state,
                     Eigen::VectorXd &q,
                     moveit_msgs::msg::MoveItErrorCodes &error) const;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<std::string> joints_;
  std::string tcp_frame_, base_frame_;
  pinocchio::FrameIndex frame_;
  std::vector<int> indices_;
  KinematicsOptions options_;
  std::timed_mutex mutex_;
};
} // namespace zephyr::mpc
