// SPDX-License-Identifier: Apache-2.0
// Adapted from fiveages-sim/arms_ros2_control; see UPSTREAM.md.
#pragma once
#include <functional>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <tuple>
#include <optional>
#include <vector>

namespace zephyr::mpc {
enum class ControlMode { POSITION, MIX };

/** Interface handles and the last requested position command.
 * No FSM or derivative-history cache: neither is consumed by the MPC path.
 */
struct JointInterfaces {
  using Commands = std::vector<
      std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;
  using States = std::vector<
      std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
  Commands joint_position_command_interface_, joint_velocity_command_interface_,
      joint_force_command_interface_, joint_kp_command_interface_,
      joint_kd_command_interface_;
  States joint_position_state_interface_, joint_velocity_state_interface_,
      joint_force_state_interface_;
  int frequency_{1000};
  ControlMode control_mode_{ControlMode::POSITION};
  std::vector<double> pd_gains_;
  std::vector<double> last_sent_joint_positions_;
  // Kept separate: normalized speed/effort caps are not arm qdot/torque.
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>> gripper_position_state;
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      gripper_position, gripper_speed, gripper_effort;
  double last_gripper_position{0}, gripper_speed_limit{0.5}, gripper_effort_limit{0.4};
  bool writeGripper(double position) {
    if (!gripper_position || !gripper_speed || !gripper_effort) return false;
    const bool ok = gripper_position->get().set_value(position) &&
        gripper_speed->get().set_value(gripper_speed_limit) &&
        gripper_effort->get().set_value(gripper_effort_limit);
    if (ok) last_gripper_position = position;
    return ok;
  }

  void detectAndSetControlMode() {
    control_mode_ = !joint_position_command_interface_.empty() &&
                            !joint_velocity_command_interface_.empty() &&
                            !joint_force_command_interface_.empty() &&
                            !joint_position_state_interface_.empty() &&
                            !joint_velocity_state_interface_.empty() &&
                            !joint_force_state_interface_.empty()
                        ? ControlMode::MIX
                        : ControlMode::POSITION;
  }
  void setJointPositionCommand(size_t index, double value) {
    std::ignore =
        joint_position_command_interface_[index].get().set_value(value);
    last_sent_joint_positions_[index] = value;
  }
  // Position adapters may expose velocity without effort. Do not mistake that
  // for torque/MIT output, or leave their velocity reference from a previous JTC.
  void setJointMotionCommand(size_t index, double position, double velocity) {
    setJointPositionCommand(index, position);
    if (!joint_velocity_command_interface_.empty())
      std::ignore = joint_velocity_command_interface_[index].get().set_value(velocity);
  }
  void clear() {
    gripper_position_state.reset(); gripper_position.reset();
    gripper_speed.reset(); gripper_effort.reset();
    joint_position_command_interface_.clear();
    joint_velocity_command_interface_.clear();
    joint_force_command_interface_.clear();
    joint_kp_command_interface_.clear();
    joint_kd_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();
    joint_force_state_interface_.clear();
    last_sent_joint_positions_.clear();
    control_mode_ = ControlMode::POSITION;
  }
};
} // namespace zephyr::mpc
