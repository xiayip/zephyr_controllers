// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/JointInterfaces.h"
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
