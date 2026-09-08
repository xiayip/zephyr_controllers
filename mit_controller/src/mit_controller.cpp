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
  if (!std::isfinite(params_.max_position_step) ||
      params_.max_position_step <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "max_position_step must be finite and positive");
    return controller_interface::CallbackReturn::ERROR;
  }

  hold_positions_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  hold_kp_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  hold_kd_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  hold_t_ff_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());
  hold_measured_velocities_.assign(joint_count,
                                   std::numeric_limits<double>::quiet_NaN());
  last_commands_.resize(joint_count);
  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
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
    configuration.names.push_back(joint + "/velocity");
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
    velocity_state_indices_.clear();
    command_indices_.reserve(params_.joints.size());
    position_state_indices_.reserve(params_.joints.size());
    velocity_state_indices_.reserve(params_.joints.size());
    for (const auto &joint : params_.joints) {
      command_indices_.push_back({find_command_interface(joint + "/position"),
                                  find_command_interface(joint + "/velocity"),
                                  find_command_interface(joint + "/kp"),
                                  find_command_interface(joint + "/kd"),
                                  find_command_interface(joint + "/t_ff")});
      position_state_indices_.push_back(
          find_state_interface(joint + "/position"));
      velocity_state_indices_.push_back(
          find_state_interface(joint + "/velocity"));
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
  Command applied{position, velocity, kp, kd, t_ff};
  const auto &previous = last_commands_[joint_index];
  const auto measured_position =
      state_interfaces_[position_state_indices_[joint_index]].get_optional();
  const auto measured_velocity =
      state_interfaces_[velocity_state_indices_[joint_index]].get_optional();
  const double q = measured_position && std::isfinite(*measured_position)
                       ? *measured_position
                       : hold_positions_[joint_index];
  const double dq = measured_velocity && std::isfinite(*measured_velocity)
                        ? *measured_velocity
                        : hold_measured_velocities_[joint_index];
  const double delta = position - previous.position;
  if (std::abs(delta) > params_.max_position_step) {
    const auto torque = [q, dq](const Command &c) {
      return c.kp * (c.position - q) + c.kd * (c.velocity - dq) + c.t_ff;
    };
    const double alpha = params_.max_position_step / std::abs(delta);
    const double previous_torque = torque(previous);
    const double blended_torque =
        previous_torque + alpha * (torque(applied) - previous_torque);
    applied.position =
        previous.position + std::clamp(delta, -params_.max_position_step,
                                       params_.max_position_step);
    // Limit the position AND interpolate modeled torque. For a pure hold
    // reparameterization both endpoint torques are equal, so a downstream
    // position-only clamp cannot create a preload/feedforward double count.
    // For a real position jump the torque progresses only by alpha too.
    applied.t_ff =
        blended_torque - kp * (applied.position - q) - kd * (velocity - dq);
  }
  const double projected_hold_ff = applied.t_ff + kp * (applied.position - q) +
                                   kd * velocity +
                                   (hold_kd_[joint_index] - kd) * dq;
  if (!std::isfinite(applied.position) || !std::isfinite(applied.t_ff) ||
      !std::isfinite(projected_hold_ff)) {
    return false;
  }
  const bool written =
      command_interfaces_[indices.position].set_value(applied.position) &&
      command_interfaces_[indices.velocity].set_value(velocity) &&
      command_interfaces_[indices.kp].set_value(kp) &&
      command_interfaces_[indices.kd].set_value(kd) &&
      command_interfaces_[indices.t_ff].set_value(applied.t_ff);
  if (written)
    last_commands_[joint_index] = applied;
  return written;
}

bool MitController::initialize_hold() {
  holding_ = false;
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto command_position =
        command_interfaces_[command_indices_[index].position].get_optional();
    const auto measured_position =
        state_interfaces_[position_state_indices_[index]].get_optional();
    const auto command_kp =
        command_interfaces_[command_indices_[index].kp].get_optional();
    const auto command_kd =
        command_interfaces_[command_indices_[index].kd].get_optional();
    const auto command_velocity =
        command_interfaces_[command_indices_[index].velocity].get_optional();
    const auto command_t_ff =
        command_interfaces_[command_indices_[index].t_ff].get_optional();
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
        !finite_nonnegative(*command_kd) || !command_velocity ||
        !std::isfinite(*command_velocity) || !command_t_ff ||
        !std::isfinite(*command_t_ff)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "No finite startup MIT tuple for joint '%s'",
                   params_.joints[index].c_str());
      return false;
    }
    hold_kp_[index] = *command_kp;
    hold_kd_[index] = *command_kd;
    last_commands_[index] = {hold_positions_[index], *command_velocity,
                             *command_kp, *command_kd, *command_t_ff};
    if (!capture_hold(index, last_commands_[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Cannot establish a finite hold for joint '%s'",
                   params_.joints[index].c_str());
      return false;
    }
  }
  // Activation adopts the whole downstream tuple; it must not erase an
  // already-running controller's feedforward or velocity contribution.
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto &command = last_commands_[index];
    set_reference(index, command.position, command.velocity, command.kp,
                  command.kd, command.t_ff);
    if (!write_joint_command(index, command.position, command.velocity,
                             command.kp, command.kd, command.t_ff)) {
      return false;
    }
  }
  return true;
}

bool MitController::capture_hold(const std::size_t index,
                                 const Command &command) {
  const auto position =
      state_interfaces_[position_state_indices_[index]].get_optional();
  const auto velocity =
      state_interfaces_[velocity_state_indices_[index]].get_optional();
  if (!position || !velocity || !std::isfinite(*position) ||
      !std::isfinite(*velocity)) {
    return false;
  }
  // Re-anchor at actual q with v_ref=0 and startup hold gains. Preserve the
  // last command's modeled torque at this q,dq, including moving references
  // and the derivative-gain change. Do not treat the old t_ff alone as load.
  const double t_ff = command.t_ff +
                      command.kp * (command.position - *position) +
                      command.kd * command.velocity +
                      (hold_kd_[index] - command.kd) * *velocity;
  if (!std::isfinite(t_ff)) {
    return false;
  }
  hold_positions_[index] = *position;
  hold_t_ff_[index] = t_ff;
  hold_measured_velocities_[index] = *velocity;
  return true;
}

bool MitController::enter_hold() {
  // Unchaining and deactivation can occur before the next hardware write.
  // Do not step/re-anchor a second time: retain the already latched tuple.
  if (holding_) {
    return true;
  }
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    if (!capture_hold(index, last_commands_[index])) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Invalid feedback for '%s'; retaining last finite hold",
                  params_.joints[index].c_str());
    }
  }
  return write_hold();
}

bool MitController::write_hold() {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    if (!std::isfinite(hold_positions_[index]) ||
        !std::isfinite(hold_t_ff_[index]) ||
        !write_joint_command(index, hold_positions_[index], 0.0,
                             hold_kp_[index], hold_kd_[index],
                             hold_t_ff_[index])) {
      return false;
    }
    set_reference(index, hold_positions_[index], 0.0, hold_kp_[index],
                  hold_kd_[index], hold_t_ff_[index]);
  }
  holding_ = true;
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
  const bool hold_written = command_indices_.empty() || enter_hold();
  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  return hold_written ? controller_interface::CallbackReturn::SUCCESS
                      : controller_interface::CallbackReturn::ERROR;
}

bool MitController::on_set_chained_mode(const bool chained_mode) {
  if (!chained_mode && !command_indices_.empty()) {
    return enter_hold();
  }
  return true;
}

controller_interface::return_type
MitController::update_reference_from_subscribers(const rclcpp::Time &,
                                                 const rclcpp::Duration &) {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    set_reference(index, hold_positions_[index], 0.0, hold_kp_[index],
                  hold_kd_[index], hold_t_ff_[index]);
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
MitController::update_and_write_commands(const rclcpp::Time &,
                                         const rclcpp::Duration &) {
  if (is_in_chained_mode()) {
    holding_ = false;
  }
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
    if (!valid ||
        (is_in_chained_mode() && !capture_hold(index, last_commands_[index]))) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000,
                            "Invalid MIT reference/feedback for joint '%s'; "
                            "applying last finite hold",
                            params_.joints[index].c_str());
      set_reference(index, hold_positions_[index], 0.0, hold_kp_[index],
                    hold_kd_[index], hold_t_ff_[index]);
      if (!write_joint_command(index, hold_positions_[index], 0.0,
                               hold_kp_[index], hold_kd_[index],
                               hold_t_ff_[index])) {
        return controller_interface::return_type::ERROR;
      }
      continue;
    }

    if (!write_joint_command(index, *position, *velocity, *kp, *kd, *t_ff)) {
      return controller_interface::return_type::ERROR;
    }
    if (is_in_chained_mode()) {
      // Cache the tuple actually written, not an unachieved stepped request.
      (void)capture_hold(index, last_commands_[index]);
    }
  }
  holding_ = !is_in_chained_mode();
  return controller_interface::return_type::OK;
}

} // namespace mit_controller

PLUGINLIB_EXPORT_CLASS(mit_controller::MitController,
                       controller_interface::ChainableControllerInterface)
