// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/JointInterfaces.h"
#include "mpc_controller/control/CtrlComponent.h"
#include "mpc_controller/runtime/MpcExecutionParams.hpp"
#include "mpc_controller/runtime/RtCycleTiming.h"
#include <gtest/gtest.h>

using namespace zephyr::mpc;

TEST(JointInterfaces, PositionWritesAndClearDropsHandlesAndMode) {
  double value = 0.0;
  auto command = std::make_shared<hardware_interface::CommandInterface>(
      "joint", "position", &value);
  hardware_interface::LoanedCommandInterface loan(command, nullptr);
  JointInterfaces interfaces;
  interfaces.joint_position_command_interface_.push_back(loan);
  interfaces.last_sent_joint_positions_.resize(1);
  interfaces.setJointPositionCommand(0, 0.25);
  EXPECT_DOUBLE_EQ(value, 0.25);
  EXPECT_DOUBLE_EQ(interfaces.last_sent_joint_positions_[0], 0.25);
  interfaces.detectAndSetControlMode();
  EXPECT_EQ(interfaces.control_mode_, ControlMode::POSITION);
  interfaces.control_mode_ = ControlMode::MIX;
  interfaces.clear();
  EXPECT_EQ(interfaces.control_mode_, ControlMode::POSITION);
  EXPECT_TRUE(interfaces.joint_position_command_interface_.empty());
  EXPECT_TRUE(interfaces.last_sent_joint_positions_.empty());
}

TEST(JointInterfaces, RedetectsModeAfterInterfaceChange) {
  double value = 0;
  auto command = std::make_shared<hardware_interface::CommandInterface>(
      "joint", "position", &value);
  auto state = std::make_shared<hardware_interface::StateInterface>(
      "joint", "position", &value);
  hardware_interface::LoanedCommandInterface c(command, nullptr);
  hardware_interface::LoanedStateInterface s(state);
  JointInterfaces i;
  i.joint_position_command_interface_.push_back(c);
  i.joint_velocity_command_interface_.push_back(c);
  i.joint_force_command_interface_.push_back(c);
  i.joint_position_state_interface_.push_back(s);
  i.joint_velocity_state_interface_.push_back(s);
  i.joint_force_state_interface_.push_back(s);
  i.detectAndSetControlMode();
  EXPECT_EQ(i.control_mode_, ControlMode::MIX);
  i.joint_force_command_interface_.clear();
  i.detectAndSetControlMode();
  EXPECT_EQ(i.control_mode_, ControlMode::POSITION);
}

TEST(JointInterfaces, PositionAdapterReceivesVelocityWithoutEnablingTorqueOutput) {
  double position = 0.0, velocity = 9.0;
  auto pc = std::make_shared<hardware_interface::CommandInterface>("adapter/joint", "position", &position);
  auto vc = std::make_shared<hardware_interface::CommandInterface>("adapter/joint", "velocity", &velocity);
  hardware_interface::LoanedCommandInterface pl(pc, nullptr), vl(vc, nullptr);
  JointInterfaces interfaces;
  interfaces.joint_position_command_interface_.push_back(pl);
  interfaces.last_sent_joint_positions_.resize(1);
  interfaces.setJointMotionCommand(0, 0.1, 0.2); // Position-only mock is unchanged.
  EXPECT_DOUBLE_EQ(position, 0.1);
  EXPECT_DOUBLE_EQ(velocity, 9.0);
  interfaces.joint_velocity_command_interface_.push_back(vl);
  interfaces.detectAndSetControlMode();
  EXPECT_EQ(interfaces.control_mode_, ControlMode::POSITION);
  interfaces.setJointMotionCommand(0, 0.3, -0.2);
  EXPECT_DOUBLE_EQ(position, 0.3);
  EXPECT_DOUBLE_EQ(velocity, -0.2);
  EXPECT_DOUBLE_EQ(interfaces.last_sent_joint_positions_[0], 0.3);
  interfaces.setJointMotionCommand(0, 0.3, 0.0);
  EXPECT_DOUBLE_EQ(velocity, 0.0);
}

TEST(RuntimeHelpers, ExecutionCadencePreservesPreviousDefaults) {
  const auto logger = rclcpp::get_logger("cadence_test");
  const auto explicit_rate =
      runtime::computeMpcExecutionParams(200, 50, logger);
  EXPECT_DOUBLE_EQ(explicit_rate.mpc_period_sec, 0.02);
  EXPECT_EQ(explicit_rate.thread_sleep_ms, 10);
  const auto fallback = runtime::computeMpcExecutionParams(200, 0, logger);
  EXPECT_DOUBLE_EQ(fallback.mpc_period_sec, 0.02);
  const auto excessive = runtime::computeMpcExecutionParams(200, 400, logger);
  EXPECT_DOUBLE_EQ(excessive.mpc_period_sec, 0.02);
}

TEST(RuntimeHelpers, TimingSamplesAreBoundedAndDisabledScopeDoesNothing) {
  runtime::RtSampleWindow window;
  EXPECT_EQ(window.maxv(), 0u);
  for (unsigned i = 0; i < 2000; ++i)
    window.add(i);
  EXPECT_EQ(window.n, window.kCap);
  EXPECT_EQ(window.maxv(), 1023u);
  EXPECT_EQ(window.percentile(0.5), 512u);
  window.reset();
  EXPECT_EQ(window.percentile(0.5), 0u);
  runtime::RtCycleTiming timer;
  uint32_t elapsed = 123;
  { auto scope = timer.scope(&elapsed); }
  EXPECT_EQ(elapsed, 123u);
}

TEST(RuntimeHelpers, ProductionRejectsDiagnosticsBeforeLoadingAnyRobotModel) {
  rclcpp::init(0, nullptr);
  for (const auto* parameter : {"prediction_visualization_enabled", "traj_record_enabled"}) {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false).parameter_overrides({rclcpp::Parameter(parameter, true)});
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("diagnostics_gate_test", options);
    JointInterfaces interfaces;
    try {
      CtrlComponent component(node, interfaces);
      ADD_FAILURE() << "Production accepted " << parameter;
    } catch (const std::runtime_error& error) {
      EXPECT_NE(std::string(error.what()).find("mock-only"), std::string::npos);
    }
    EXPECT_FALSE(node->get_parameter("mock_debug_enabled").as_bool());
  }
  rclcpp::shutdown();
}
