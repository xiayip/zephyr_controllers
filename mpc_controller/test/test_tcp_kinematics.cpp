// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/TcpKinematics.h"
#include <gtest/gtest.h>
#include <limits>
#include <pinocchio/multibody/joint/joint-prismatic.hpp>

using namespace zephyr::mpc;
using Error = moveit_msgs::msg::MoveItErrorCodes;
using FK = moveit_msgs::srv::GetPositionFK;
using IK = moveit_msgs::srv::GetPositionIK;

namespace {
// Analytic two-axis stage: TCP=(qx, 0, qz+0.5), identity orientation.
pinocchio::Model model() {
  pinocchio::Model m;
  const auto x =
      m.addJoint(0, pinocchio::JointModelPX(), pinocchio::SE3::Identity(), "x");
  const auto z =
      m.addJoint(x, pinocchio::JointModelPZ(), pinocchio::SE3::Identity(), "z");
  m.addFrame(pinocchio::Frame(
      "tcp", z, 0,
      pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.5)),
      pinocchio::BODY));
  m.lowerPositionLimit.setConstant(-1.0);
  m.upperPositionLimit.setConstant(1.0);
  return m;
}
moveit_msgs::msg::RobotState state() {
  moveit_msgs::msg::RobotState s;
  s.joint_state.name = {"z", "x"}; // Deliberately not model order.
  s.joint_state.position = {0.0, 0.0};
  return s;
}
IK::Request request() {
  IK::Request r;
  r.ik_request.group_name = "arm";
  r.ik_request.ik_link_name = "tcp";
  r.ik_request.robot_state = state();
  r.ik_request.pose_stamped.header.frame_id = "base";
  r.ik_request.pose_stamped.pose.position.x = 0.2;
  r.ik_request.pose_stamped.pose.position.z = 0.8;
  r.ik_request.pose_stamped.pose.orientation.w = 1.0;
  r.ik_request.timeout.sec = 1;
  return r;
}
} // namespace

TEST(TcpKinematics, ForwardUsesNamedOrderAndRequestedFrames) {
  TcpKinematics k(model(), {"x", "z"}, "tcp", "base");
  FK::Request r;
  r.robot_state = state();
  r.robot_state.joint_state.position = {0.3, 0.2};
  r.fk_link_names = {"tcp"};
  r.header.stamp.sec = 42;
  FK::Response out;
  k.fk(r, out);
  ASSERT_EQ(out.error_code.val, Error::SUCCESS);
  ASSERT_EQ(out.pose_stamped.size(), 1u);
  EXPECT_EQ(out.fk_link_names, r.fk_link_names);
  EXPECT_EQ(out.pose_stamped[0].header.frame_id, "base");
  EXPECT_EQ(out.pose_stamped[0].header.stamp.sec, 42);
  EXPECT_NEAR(out.pose_stamped[0].pose.position.x, 0.2, 1e-12);
  EXPECT_NEAR(out.pose_stamped[0].pose.position.z, 0.8, 1e-12);
}

TEST(TcpKinematics, RejectsInvalidFkFrameAndNames) {
  TcpKinematics k(model(), {"x", "z"}, "tcp", "base");
  FK::Request r;
  r.robot_state = state();
  r.fk_link_names = {"tcp"};
  FK::Response out;
  r.header.frame_id = "world";
  k.fk(r, out);
  EXPECT_EQ(out.error_code.val, Error::FRAME_TRANSFORM_FAILURE);
  r.header.frame_id = "base";
  r.fk_link_names = {"missing"};
  k.fk(r, out);
  EXPECT_EQ(out.error_code.val, Error::INVALID_LINK_NAME);
  EXPECT_TRUE(out.pose_stamped.empty());
  r.fk_link_names.clear();
  k.fk(r, out);
  EXPECT_EQ(out.error_code.val, Error::INVALID_LINK_NAME);
}

TEST(TcpKinematics, AllSolversRoundTripWithNamedSolution) {
  for (const auto *solver : {"dls", "bfgs", "auto"}) {
    KinematicsOptions options;
    options.solver = solver;
    TcpKinematics k(model(), {"z", "x"}, "tcp", "base", options);
    auto r = request();
    IK::Response out;
    k.ik(r, out);
    ASSERT_EQ(out.error_code.val, Error::SUCCESS)
        << solver << ": " << out.error_code.message;
    EXPECT_EQ(out.solution.joint_state.name,
              r.ik_request.robot_state.joint_state.name);
    ASSERT_EQ(out.solution.joint_state.position.size(), 2u);
    EXPECT_NEAR(out.solution.joint_state.position[0], 0.3, options.tolerance);
    EXPECT_NEAR(out.solution.joint_state.position[1], 0.2, options.tolerance);
  }
}

TEST(TcpKinematics, NeverSilentlyIgnoresUnsupportedIkRequests) {
  TcpKinematics k(model(), {"x", "z"}, "tcp", "base");
  const auto check = [&](IK::Request r, int expected) {
    IK::Response out;
    k.ik(r, out);
    EXPECT_EQ(out.error_code.val, expected) << out.error_code.message;
    EXPECT_TRUE(out.solution.joint_state.name.empty());
  };
  auto r = request();
  r.ik_request.avoid_collisions = true;
  check(r, Error::COLLISION_CHECKING_UNAVAILABLE);
  r = request();
  r.ik_request.constraints.joint_constraints.emplace_back();
  check(r, Error::INVALID_GOAL_CONSTRAINTS);
  r = request();
  r.ik_request.group_name = "other";
  check(r, Error::INVALID_GROUP_NAME);
  r = request();
  r.ik_request.ik_link_name = "other";
  check(r, Error::INVALID_LINK_NAME);
  r = request();
  r.ik_request.ik_link_names = {"tcp"};
  check(r, Error::INVALID_LINK_NAME);
  r = request();
  r.ik_request.pose_stamped.header.frame_id = "world";
  check(r, Error::FRAME_TRANSFORM_FAILURE);
  r = request();
  r.ik_request.pose_stamped.pose.orientation.w = 0.0;
  check(r, Error::GOAL_STATE_INVALID);
  r = request();
  r.ik_request.pose_stamped.pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  check(r, Error::GOAL_STATE_INVALID);
}

TEST(TcpKinematics, ValidatesCompleteAbsoluteRobotState) {
  TcpKinematics k(model(), {"x", "z"}, "tcp", "base");
  const auto check = [&](moveit_msgs::msg::RobotState s) {
    auto r = request();
    r.ik_request.robot_state = s;
    IK::Response out;
    k.ik(r, out);
    EXPECT_EQ(out.error_code.val, Error::INVALID_ROBOT_STATE);
  };
  auto s = state();
  s.joint_state.name[1] = "z";
  check(s);
  s = state();
  s.joint_state.name[1] = "unknown";
  check(s);
  s = state();
  s.joint_state.position.pop_back();
  check(s);
  s = state();
  s.joint_state.position[0] = std::numeric_limits<double>::infinity();
  check(s);
  s = state();
  s.is_diff = true;
  check(s);
  s = state();
  s.attached_collision_objects.emplace_back();
  check(s);
  s = state();
  s.multi_dof_joint_state.joint_names = {"base"};
  check(s);
}

TEST(TcpKinematics, EnforcesTimeoutAndJointBounds) {
  TcpKinematics k(model(), {"x", "z"}, "tcp", "base");
  auto r = request();
  IK::Response out;
  r.ik_request.timeout.sec = 0;
  r.ik_request.timeout.nanosec = 1;
  k.ik(r, out);
  EXPECT_EQ(out.error_code.val, Error::TIMED_OUT);
  r.ik_request.timeout.sec = -1;
  k.ik(r, out);
  EXPECT_EQ(out.error_code.val, Error::FAILURE);
  r = request();
  r.ik_request.pose_stamped.pose.position.x = 2.0;
  k.ik(r, out);
  EXPECT_EQ(out.error_code.val, Error::NO_IK_SOLUTION);
  EXPECT_TRUE(out.solution.joint_state.position.empty());
  r = request();
  r.ik_request.timeout.sec = 0; // Configured default budget.
  k.ik(r, out);
  EXPECT_EQ(out.error_code.val, Error::SUCCESS);
}

TEST(TcpKinematics, RejectsInvalidInitialization) {
  EXPECT_THROW(TcpKinematics(model(), {"x", "x"}, "tcp", "base"),
               std::invalid_argument);
  EXPECT_THROW(TcpKinematics(model(), {"x", "z"}, "missing", "base"),
               std::invalid_argument);
  KinematicsOptions o;
  o.solver = "bad";
  EXPECT_THROW(TcpKinematics(model(), {"x", "z"}, "tcp", "base", o),
               std::invalid_argument);
  o = {};
  o.timeout = 0;
  EXPECT_THROW(TcpKinematics(model(), {"x", "z"}, "tcp", "base", o),
               std::invalid_argument);
}
