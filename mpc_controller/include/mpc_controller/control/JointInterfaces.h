// SPDX-License-Identifier: Apache-2.0
// Adapted from fiveages-sim/arms_ros2_control; see UPSTREAM.md.
#pragma once
#include <functional>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <tuple>
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
  void clear() {
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
