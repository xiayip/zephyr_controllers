#include "mit_controller/mit_controller.hpp"

#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mit_controller {
namespace {
constexpr std::array<const char *, 5> kMitInterfaces = {"position", "velocity",
                                                        "kp", "kd", "t_ff"};

bool finite_nonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}
} // namespace

controller_interface::CallbackReturn MitController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "MIT controller initialization failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitController::on_configure(const rclcpp_lifecycle::State &) {
  params_ = param_listener_->get_params();
  const auto joint_count = params_.joints.size();
  if (joint_count == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "joints must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  hold_positions_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  hold_kp_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  hold_kd_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  command_indices_.clear();
  position_state_indices_.clear();
  reference_handles_.clear();
  reference_handles_.reserve(joint_count * kMitInterfaces.size());
  for (const auto &joint : params_.joints) {
    for (const auto *interface_name : kMitInterfaces) {
      reference_handles_.push_back(
          std::make_shared<hardware_interface::CommandInterface>(
              get_node()->get_name(), joint + "/" + interface_name));
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : params_.joints) {
    for (const auto *interface_name : kMitInterfaces) {
      configuration.names.push_back(joint + "/" + interface_name);
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
MitController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : params_.joints) {
    configuration.names.push_back(joint + "/position");
  }
  return configuration;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
MitController::on_export_reference_interfaces_list() {
  return reference_handles_;
}

std::size_t
MitController::find_command_interface(const std::string &full_name) const {
  const auto found =
      std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                   [&full_name](const auto &interface) {
                     return interface.get_name() == full_name;
                   });
  if (found == command_interfaces_.end()) {
    throw std::runtime_error("missing command interface " + full_name);
  }
  return static_cast<std::size_t>(
      std::distance(command_interfaces_.begin(), found));
}

std::size_t
MitController::find_state_interface(const std::string &full_name) const {
  const auto found =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                   [&full_name](const auto &interface) {
                     return interface.get_name() == full_name;
                   });
  if (found == state_interfaces_.end()) {
    throw std::runtime_error("missing state interface " + full_name);
  }
  return static_cast<std::size_t>(
      std::distance(state_interfaces_.begin(), found));
}

bool MitController::bind_interfaces() {
  try {
    command_indices_.clear();
    position_state_indices_.clear();
    command_indices_.reserve(params_.joints.size());
    position_state_indices_.reserve(params_.joints.size());
    for (const auto &joint : params_.joints) {
      command_indices_.push_back({find_command_interface(joint + "/position"),
                                  find_command_interface(joint + "/velocity"),
                                  find_command_interface(joint + "/kp"),
                                  find_command_interface(joint + "/kd"),
                                  find_command_interface(joint + "/t_ff")});
      position_state_indices_.push_back(
          find_state_interface(joint + "/position"));
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "MIT interface binding failed: %s",
                 error.what());
    return false;
  }
  return true;
}

void MitController::set_reference(const std::size_t joint_index,
                                  const double position, const double velocity,
                                  const double kp, const double kd,
                                  const double t_ff) {
  const auto base = joint_index * kMitInterfaces.size();
  (void)reference_handles_[base]->set_value(position);
  (void)reference_handles_[base + 1]->set_value(velocity);
  (void)reference_handles_[base + 2]->set_value(kp);
  (void)reference_handles_[base + 3]->set_value(kd);
  (void)reference_handles_[base + 4]->set_value(t_ff);
}

bool MitController::write_joint_command(const std::size_t joint_index,
                                        const double position,
                                        const double velocity, const double kp,
                                        const double kd, const double t_ff) {
  const auto &indices = command_indices_[joint_index];
  return command_interfaces_[indices.position].set_value(position) &&
         command_interfaces_[indices.velocity].set_value(velocity) &&
         command_interfaces_[indices.kp].set_value(kp) &&
         command_interfaces_[indices.kd].set_value(kd) &&
         command_interfaces_[indices.t_ff].set_value(t_ff);
}

bool MitController::initialize_hold() {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto command_position =
        command_interfaces_[command_indices_[index].position].get_optional();
    const auto measured_position =
        state_interfaces_[position_state_indices_[index]].get_optional();
    const auto command_kp =
        command_interfaces_[command_indices_[index].kp].get_optional();
    const auto command_kd =
        command_interfaces_[command_indices_[index].kd].get_optional();
    if (command_position && std::isfinite(*command_position)) {
      hold_positions_[index] = *command_position;
    } else if (measured_position && std::isfinite(*measured_position)) {
      hold_positions_[index] = *measured_position;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "No finite startup position for joint '%s'",
                   params_.joints[index].c_str());
      return false;
    }
    if (!command_kp || !command_kd || !finite_nonnegative(*command_kp) ||
        !finite_nonnegative(*command_kd)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "No finite startup hold gains for joint '%s'",
                   params_.joints[index].c_str());
      return false;
    }
    hold_kp_[index] = *command_kp;
    hold_kd_[index] = *command_kd;
    set_reference(index, hold_positions_[index], 0.0, hold_kp_[index],
                  hold_kd_[index], 0.0);
  }
  return write_hold();
}

bool MitController::write_hold() {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    if (!std::isfinite(hold_positions_[index]) ||
        !write_joint_command(index, hold_positions_[index], 0.0,
                             hold_kp_[index], hold_kd_[index], 0.0)) {
      return false;
    }
  }
  return true;
}

controller_interface::CallbackReturn
MitController::on_activate(const rclcpp_lifecycle::State &) {
  if (!bind_interfaces() || !initialize_hold()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitController::on_deactivate(const rclcpp_lifecycle::State &) {
  const bool hold_written = command_indices_.empty() || write_hold();
  command_indices_.clear();
  position_state_indices_.clear();
  return hold_written ? controller_interface::CallbackReturn::SUCCESS
                      : controller_interface::CallbackReturn::ERROR;
}

bool MitController::on_set_chained_mode(const bool chained_mode) {
  if (!chained_mode) {
    for (std::size_t index = 0; index < params_.joints.size(); ++index) {
      set_reference(index, hold_positions_[index], 0.0, hold_kp_[index],
                    hold_kd_[index], 0.0);
    }
  }
  return true;
}

controller_interface::return_type
MitController::update_reference_from_subscribers(const rclcpp::Time &,
                                                 const rclcpp::Duration &) {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    set_reference(index, hold_positions_[index], 0.0, hold_kp_[index],
                  hold_kd_[index], 0.0);
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
MitController::update_and_write_commands(const rclcpp::Time &,
                                         const rclcpp::Duration &) {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto base = index * kMitInterfaces.size();
    const auto position = reference_handles_[base]->get_optional();
    const auto velocity = reference_handles_[base + 1]->get_optional();
    const auto kp = reference_handles_[base + 2]->get_optional();
    const auto kd = reference_handles_[base + 3]->get_optional();
    const auto t_ff = reference_handles_[base + 4]->get_optional();
    const bool valid = position && velocity && kp && kd && t_ff &&
                       std::isfinite(*position) && std::isfinite(*velocity) &&
                       finite_nonnegative(*kp) && finite_nonnegative(*kd) &&
                       std::isfinite(*t_ff);
    if (!valid) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Invalid MIT reference for joint '%s'; applying safe hold",
          params_.joints[index].c_str());
      set_reference(index, hold_positions_[index], 0.0, hold_kp_[index],
                    hold_kd_[index], 0.0);
      if (!write_joint_command(index, hold_positions_[index], 0.0,
                               hold_kp_[index], hold_kd_[index], 0.0)) {
        return controller_interface::return_type::ERROR;
      }
      continue;
    }

    hold_positions_[index] = *position;
    if (!write_joint_command(index, *position, *velocity, *kp, *kd, *t_ff)) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

} // namespace mit_controller

PLUGINLIB_EXPORT_CLASS(mit_controller::MitController,
                       controller_interface::ChainableControllerInterface)
