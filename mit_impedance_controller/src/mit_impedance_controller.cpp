#include "mit_impedance_controller/mit_impedance_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace mit_impedance_controller {
namespace {
constexpr std::array<const char *, 5> kMitInterfaces = {"position", "velocity",
                                                        "kp", "kd", "t_ff"};
constexpr std::array<const char *, 2> kReferenceInterfaces = {"position",
                                                              "velocity"};
constexpr std::array<const char *, 17> kStateFields = {
    "position",
    "target_position_reference",
    "active_position_reference",
    "position_error",
    "velocity",
    "velocity_reference",
    "raw_gravity_torque",
    "scaled_gravity_torque",
    "gravity_factor",
    "commanded_kp",
    "commanded_kd",
    "estimated_stiffness_torque",
    "estimated_damping_torque",
    "soft_limit_torque",
    "target_feedforward_torque",
    "commanded_feedforward_torque",
    "estimated_commanded_torque",
};

bool finite_nonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool finite_in_range(const double value, const double upper) {
  return finite_nonnegative(value) && value <= upper;
}

bool finite_absolute_bounded(const double value, const double limit) {
  return std::isfinite(value) && std::abs(value) <= limit;
}
} // namespace

MitImpedanceController::~MitImpedanceController() {
  if (param_listener_) {
    param_listener_->clearUserCallback();
  }
  runtime_validation_callback_.reset();
}

controller_interface::CallbackReturn MitImpedanceController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    runtime_validation_callback_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          const auto expected_size =
              param_listener_->get_params().joints.size();
          for (const auto &parameter : parameters) {
            const bool is_stiffness = parameter.get_name() == "stiffness";
            const bool is_damping =
                parameter.get_name() == "damping_coefficients";
            const bool is_gravity_scale =
                parameter.get_name() == "gravity_compensation.scale";
            const bool is_gravity_bias =
                parameter.get_name() == "gravity_compensation.bias";
            if (!is_stiffness && !is_damping && !is_gravity_scale &&
                !is_gravity_bias) {
              continue;
            }
            if (parameter.get_type() !=
                rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
              result.successful = false;
              result.reason = parameter.get_name() + " must be a double array";
              return result;
            }
            const auto values = parameter.as_double_array();
            bool values_valid = false;
            if (is_gravity_bias) {
              values_valid = std::all_of(
                  values.begin(), values.end(), [](const double value) {
                    return finite_absolute_bounded(value, 5.0);
                  });
            } else {
              const double upper = is_stiffness ? 300.0
                                   : is_damping ? 30.0
                                                : 2.0;
              values_valid = std::all_of(values.begin(), values.end(),
                                         [upper](double value) {
                                           return finite_in_range(value, upper);
                                         });
            }
            if (values.size() != expected_size || !values_valid) {
              result.successful = false;
              result.reason = parameter.get_name() +
                              " must contain one valid value per joint";
              return result;
            }
          }
          return result;
        });
    param_listener_->setUserCallback([this](const Params &updated_params) {
      target_gravity_factor_.store(updated_params.gravity_compensation.factor,
                                   std::memory_order_relaxed);
      const auto joint_count = updated_params.joints.size();
      const auto &stiffness = updated_params.stiffness;
      const auto &damping = updated_params.damping_coefficients;
      const auto &scale = updated_params.gravity_compensation.scale;
      const auto &bias = updated_params.gravity_compensation.bias;
      if (stiffness.size() == joint_count && damping.size() == joint_count &&
          scale.size() == joint_count && bias.size() == joint_count &&
          std::all_of(
              stiffness.begin(), stiffness.end(),
              [](double value) { return finite_in_range(value, 300.0); }) &&
          std::all_of(
              damping.begin(), damping.end(),
              [](double value) { return finite_in_range(value, 30.0); }) &&
          std::all_of(
              scale.begin(), scale.end(),
              [](double value) { return finite_in_range(value, 2.0); }) &&
          std::all_of(bias.begin(), bias.end(), [](double value) {
            return finite_absolute_bounded(value, 5.0);
          })) {
        runtime_parameters_buffer_.writeFromNonRT(
            {stiffness, damping, scale, bias});
      }
    });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "MIT impedance controller initialization failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitImpedanceController::on_configure(const rclcpp_lifecycle::State &) {
  params_ = param_listener_->get_params();
  const auto joint_count = params_.joints.size();
  const auto has_joint_count = [joint_count](const auto &values) {
    return values.size() == joint_count;
  };
  if (joint_count == 0 || params_.downstream_controller.empty() ||
      !has_joint_count(params_.stiffness) ||
      !has_joint_count(params_.damping_coefficients) ||
      !has_joint_count(params_.reference.position_rate_limit) ||
      !has_joint_count(params_.reference.velocity_limit) ||
      !has_joint_count(params_.gravity_compensation.scale) ||
      !has_joint_count(params_.gravity_compensation.bias) ||
      !has_joint_count(params_.command_shaping.torque_limit) ||
      !has_joint_count(params_.command_shaping.torque_rate_limit) ||
      !has_joint_count(params_.soft_limits.margin) ||
      !has_joint_count(params_.soft_limits.stiffness) ||
      !has_joint_count(params_.soft_limits.torque_limit)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "All per-joint impedance parameters must match joints");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.gravity_compensation.gravity.size() != 3) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "gravity_compensation.gravity must contain three values");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string error;
  const Eigen::Vector3d gravity(params_.gravity_compensation.gravity[0],
                                params_.gravity_compensation.gravity[1],
                                params_.gravity_compensation.gravity[2]);
  if (!gravity_model_.configure(get_robot_description(), params_.joints,
                                gravity, error)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Pinocchio model configuration failed: %s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  const auto &lower = gravity_model_.lower_position_limits();
  const auto &upper = gravity_model_.upper_position_limits();
  for (std::size_t index = 0; index < joint_count; ++index) {
    const double margin = params_.soft_limits.margin[index];
    if (std::isfinite(lower[static_cast<Eigen::Index>(index)]) &&
        std::isfinite(upper[static_cast<Eigen::Index>(index)]) &&
        2.0 * margin >= upper[static_cast<Eigen::Index>(index)] -
                            lower[static_cast<Eigen::Index>(index)]) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Soft-limit margin is too large for joint '%s'",
                   params_.joints[index].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  gravity_compensation_model::GravityFeedforwardShaperConfig shaper_config;
  shaper_config.scale =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(joint_count));
  shaper_config.torque_limit = Eigen::Map<const Eigen::VectorXd>(
      params_.command_shaping.torque_limit.data(),
      static_cast<Eigen::Index>(joint_count));
  shaper_config.torque_rate_limit = Eigen::Map<const Eigen::VectorXd>(
      params_.command_shaping.torque_rate_limit.data(),
      static_cast<Eigen::Index>(joint_count));
  shaper_config.transition_duration =
      params_.command_shaping.transition_duration;
  if (!command_shaper_.configure(shaper_config, error)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Torque shaper configuration failed: %s", error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  runtime_parameters_buffer_.initRT(
      {params_.stiffness, params_.damping_coefficients,
       params_.gravity_compensation.scale, params_.gravity_compensation.bias});
  gravity_compensation_enabled_ = params_.gravity_compensation.enabled;
  soft_limits_enabled_ = params_.soft_limits.enabled;
  gravity_factor_transition_duration_ =
      params_.gravity_compensation.factor_transition_duration;
  target_gravity_factor_.store(params_.gravity_compensation.factor,
                               std::memory_order_relaxed);
  current_gravity_factor_ = params_.gravity_compensation.factor;

  const auto size = static_cast<Eigen::Index>(joint_count);
  measured_positions_.setZero(size);
  measured_velocities_.setZero(size);
  target_position_references_.setZero(size);
  active_position_references_.setZero(size);
  velocity_references_.setZero(size);
  raw_gravity_torques_.setZero(size);
  scaled_gravity_torques_.setZero(size);
  stiffness_torques_.setZero(size);
  damping_torques_.setZero(size);
  soft_limit_torques_.setZero(size);
  target_feedforward_torques_.setZero(size);
  commanded_feedforward_torques_.setZero(size);
  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();

  reference_handles_.clear();
  reference_handles_.reserve(joint_count * kReferenceInterfaces.size());
  for (const auto &joint : params_.joints) {
    for (const auto *interface_name : kReferenceInterfaces) {
      reference_handles_.push_back(
          std::make_shared<hardware_interface::CommandInterface>(
              get_node()->get_name(), joint + "/" + interface_name));
    }
  }
  ReferenceCommand initial_reference;
  initial_reference.positions.assign(joint_count,
                                     std::numeric_limits<double>::quiet_NaN());
  initial_reference.velocities.assign(joint_count, 0.0);
  subscriber_command_buffer_.initRT(initial_reference);

  state_publish_period_ = 1.0 / params_.state_publish_rate;
  state_publish_accumulator_ = 0.0;
  try {
    command_subscriber_ =
        get_node()
            ->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
                "~/commands", rclcpp::SystemDefaultsQoS(),
                [this](
                    const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr
                        msg) {
                  const auto joint_count = params_.joints.size();
                  if (msg->positions.size() != joint_count ||
                      (!msg->velocities.empty() &&
                       msg->velocities.size() != joint_count) ||
                      !std::all_of(
                          msg->positions.begin(), msg->positions.end(),
                          [](double value) { return std::isfinite(value); }) ||
                      !std::all_of(
                          msg->velocities.begin(), msg->velocities.end(),
                          [](double value) { return std::isfinite(value); })) {
                    RCLCPP_ERROR(
                        get_node()->get_logger(),
                        "Impedance command must contain one finite "
                        "position and zero or one finite velocity per joint");
                    return;
                  }
                  ReferenceCommand command;
                  command.positions = msg->positions;
                  command.velocities = msg->velocities;
                  if (command.velocities.empty()) {
                    command.velocities.assign(joint_count, 0.0);
                  }
                  subscriber_command_buffer_.writeFromNonRT(command);
                });
    state_publisher_ =
        get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
            "~/state", rclcpp::SensorDataQoS());
    realtime_state_publisher_ =
        std::make_unique<StatePublisher>(state_publisher_);
  } catch (const std::exception &publish_error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to create impedance interfaces: %s",
                 publish_error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  realtime_state_publisher_->lock();
  auto &message = realtime_state_publisher_->msg_;
  message.joint_names = params_.joints;
  message.interface_values.resize(joint_count);
  for (auto &interface_value : message.interface_values) {
    interface_value.interface_names.assign(kStateFields.begin(),
                                           kStateFields.end());
    interface_value.values.resize(kStateFields.size());
  }
  realtime_state_publisher_->unlock();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : params_.joints) {
    for (const auto *interface_name : kMitInterfaces) {
      configuration.names.push_back(params_.downstream_controller + "/" +
                                    joint + "/" + interface_name);
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
MitImpedanceController::state_interface_configuration() const {
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
MitImpedanceController::on_export_reference_interfaces_list() {
  return reference_handles_;
}

bool MitImpedanceController::on_set_chained_mode(const bool) { return true; }

std::size_t MitImpedanceController::find_command_interface(
    const std::string &full_name) const {
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

std::size_t MitImpedanceController::find_state_interface(
    const std::string &full_name) const {
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

bool MitImpedanceController::bind_interfaces() {
  try {
    command_indices_.clear();
    position_state_indices_.clear();
    velocity_state_indices_.clear();
    command_indices_.reserve(params_.joints.size());
    position_state_indices_.reserve(params_.joints.size());
    velocity_state_indices_.reserve(params_.joints.size());
    for (const auto &joint : params_.joints) {
      const auto prefix = params_.downstream_controller + "/" + joint + "/";
      command_indices_.push_back({find_command_interface(prefix + "position"),
                                  find_command_interface(prefix + "velocity"),
                                  find_command_interface(prefix + "kp"),
                                  find_command_interface(prefix + "kd"),
                                  find_command_interface(prefix + "t_ff")});
      position_state_indices_.push_back(
          find_state_interface(joint + "/position"));
      velocity_state_indices_.push_back(
          find_state_interface(joint + "/velocity"));
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "MIT impedance interface binding failed: %s", error.what());
    return false;
  }
  return true;
}

bool MitImpedanceController::update_measured_state() {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto position =
        state_interfaces_[position_state_indices_[index]].get_optional();
    const auto velocity =
        state_interfaces_[velocity_state_indices_[index]].get_optional();
    if (!position || !velocity || !std::isfinite(*position) ||
        !std::isfinite(*velocity)) {
      return false;
    }
    const auto eigen_index = static_cast<Eigen::Index>(index);
    measured_positions_[eigen_index] = *position;
    measured_velocities_[eigen_index] = *velocity;
  }
  return true;
}

bool MitImpedanceController::update_gravity_factor(
    const double period_seconds) noexcept {
  if (!std::isfinite(period_seconds) || period_seconds < 0.0) {
    return false;
  }
  const double target = target_gravity_factor_.load(std::memory_order_relaxed);
  if (!std::isfinite(target) || target < 0.0 || target > 1.0) {
    return false;
  }
  if (gravity_factor_transition_duration_ == 0.0) {
    current_gravity_factor_ = target;
  } else {
    const double maximum_delta =
        period_seconds / gravity_factor_transition_duration_;
    current_gravity_factor_ += std::clamp(target - current_gravity_factor_,
                                          -maximum_delta, maximum_delta);
  }
  return true;
}

controller_interface::return_type
MitImpedanceController::update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) {
  const auto *command = subscriber_command_buffer_.readFromRT();
  if (!command || command->positions.size() != params_.joints.size() ||
      command->velocities.size() != params_.joints.size()) {
    return controller_interface::return_type::ERROR;
  }
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto offset = index * kReferenceInterfaces.size();
    if (!reference_handles_[offset]->set_value(command->positions[index]) ||
        !reference_handles_[offset + 1]->set_value(
            command->velocities[index])) {
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

bool MitImpedanceController::update_reference_targets(
    const double period_seconds) {
  if (!std::isfinite(period_seconds) || period_seconds < 0.0) {
    return false;
  }
  const auto &lower = gravity_model_.lower_position_limits();
  const auto &upper = gravity_model_.upper_position_limits();
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto offset = index * kReferenceInterfaces.size();
    const auto position = reference_handles_[offset]->get_optional();
    const auto velocity = reference_handles_[offset + 1]->get_optional();
    const auto eigen_index = static_cast<Eigen::Index>(index);
    if (position && std::isfinite(*position)) {
      target_position_references_[eigen_index] =
          std::clamp(*position, lower[eigen_index], upper[eigen_index]);
      (void)reference_handles_[offset]->set_value(
          target_position_references_[eigen_index]);
    }
    if (velocity && std::isfinite(*velocity)) {
      const double limit = params_.reference.velocity_limit[index];
      velocity_references_[eigen_index] = std::clamp(*velocity, -limit, limit);
      (void)reference_handles_[offset + 1]->set_value(
          velocity_references_[eigen_index]);
    }
    const double maximum_delta =
        params_.reference.position_rate_limit[index] * period_seconds;
    active_position_references_[eigen_index] +=
        std::clamp(target_position_references_[eigen_index] -
                       active_position_references_[eigen_index],
                   -maximum_delta, maximum_delta);
  }
  return true;
}

bool MitImpedanceController::compute_target_command(
    const RuntimeParameters &runtime_parameters) {
  if (gravity_compensation_enabled_) {
    if (!gravity_model_.compute(measured_positions_, raw_gravity_torques_)) {
      return false;
    }
  } else {
    raw_gravity_torques_.setZero();
  }

  const auto &lower = gravity_model_.lower_position_limits();
  const auto &upper = gravity_model_.upper_position_limits();
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto eigen_index = static_cast<Eigen::Index>(index);
    scaled_gravity_torques_[eigen_index] =
        gravity_compensation_enabled_
            ? current_gravity_factor_ *
                  (runtime_parameters.gravity_scale[index] *
                       raw_gravity_torques_[eigen_index] +
                   runtime_parameters.gravity_bias[index])
            : 0.0;

    stiffness_torques_[eigen_index] =
        runtime_parameters.stiffness[index] *
        (active_position_references_[eigen_index] -
         measured_positions_[eigen_index]);

    damping_torques_[eigen_index] =
        runtime_parameters.damping[index] *
        (velocity_references_[eigen_index] - measured_velocities_[eigen_index]);

    double soft_limit_torque = 0.0;
    if (soft_limits_enabled_) {
      const double margin = params_.soft_limits.margin[index];
      const double stiffness = params_.soft_limits.stiffness[index];
      const double lower_boundary = lower[eigen_index] + margin;
      const double upper_boundary = upper[eigen_index] - margin;
      if (std::isfinite(lower_boundary) &&
          measured_positions_[eigen_index] < lower_boundary) {
        soft_limit_torque =
            stiffness * (lower_boundary - measured_positions_[eigen_index]);
      } else if (std::isfinite(upper_boundary) &&
                 measured_positions_[eigen_index] > upper_boundary) {
        soft_limit_torque =
            stiffness * (upper_boundary - measured_positions_[eigen_index]);
      }
      const double limit = params_.soft_limits.torque_limit[index];
      soft_limit_torque = std::clamp(soft_limit_torque, -limit, limit);
    }
    soft_limit_torques_[eigen_index] = soft_limit_torque;
    target_feedforward_torques_[eigen_index] =
        scaled_gravity_torques_[eigen_index] + soft_limit_torques_[eigen_index];
  }
  return stiffness_torques_.allFinite() && damping_torques_.allFinite() &&
         target_feedforward_torques_.allFinite();
}

bool MitImpedanceController::initialize_command(
    const RuntimeParameters &runtime_parameters) {
  if (!update_measured_state()) {
    return false;
  }
  Eigen::VectorXd initial_feedforward(
      static_cast<Eigen::Index>(params_.joints.size()));
  ReferenceCommand initial_reference;
  initial_reference.positions.resize(params_.joints.size());
  initial_reference.velocities.assign(params_.joints.size(), 0.0);
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto &indices = command_indices_[index];
    const auto read_or = [this](const std::size_t command_index,
                                const double fallback) {
      const auto value = command_interfaces_[command_index].get_optional();
      return value && std::isfinite(*value) ? *value : fallback;
    };
    const auto eigen_index = static_cast<Eigen::Index>(index);
    const double old_position =
        read_or(indices.position, measured_positions_[eigen_index]);
    const double old_velocity = read_or(indices.velocity, 0.0);
    const double old_kp = read_or(indices.kp, 0.0);
    const double old_kd = read_or(indices.kd, 0.0);
    const double old_feedforward = read_or(indices.t_ff, 0.0);
    if (!finite_nonnegative(old_kp) || !finite_nonnegative(old_kd)) {
      return false;
    }
    target_position_references_[eigen_index] = measured_positions_[eigen_index];
    active_position_references_[eigen_index] = measured_positions_[eigen_index];
    velocity_references_[eigen_index] = 0.0;
    const double previous_total_torque =
        old_kp * (old_position - measured_positions_[eigen_index]) +
        old_kd * (old_velocity - measured_velocities_[eigen_index]) +
        old_feedforward;
    const double new_feedback_torque =
        runtime_parameters.damping[index] *
        (velocity_references_[eigen_index] - measured_velocities_[eigen_index]);
    initial_feedforward[eigen_index] =
        previous_total_torque - new_feedback_torque;
    initial_reference.positions[index] = measured_positions_[eigen_index];
    const auto offset = index * kReferenceInterfaces.size();
    (void)reference_handles_[offset]->set_value(
        measured_positions_[eigen_index]);
    (void)reference_handles_[offset + 1]->set_value(0.0);
  }
  subscriber_command_buffer_.writeFromNonRT(initial_reference);
  current_gravity_factor_ =
      target_gravity_factor_.load(std::memory_order_relaxed);
  if (!command_shaper_.reset(initial_feedforward) ||
      !compute_target_command(runtime_parameters) ||
      !command_shaper_.update(target_feedforward_torques_, 0.0,
                              commanded_feedforward_torques_)) {
    return false;
  }
  return write_command(runtime_parameters);
}

bool MitImpedanceController::write_command(
    const RuntimeParameters &runtime_parameters) {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto &indices = command_indices_[index];
    const auto eigen_index = static_cast<Eigen::Index>(index);
    if (!command_interfaces_[indices.position].set_value(
            active_position_references_[eigen_index]) ||
        !command_interfaces_[indices.velocity].set_value(
            velocity_references_[eigen_index]) ||
        !command_interfaces_[indices.kp].set_value(
            runtime_parameters.stiffness[index]) ||
        !command_interfaces_[indices.kd].set_value(
            runtime_parameters.damping[index]) ||
        !command_interfaces_[indices.t_ff].set_value(
            commanded_feedforward_torques_[eigen_index])) {
      return false;
    }
  }
  return true;
}

void MitImpedanceController::publish_state(const rclcpp::Time &time,
                                           const double period_seconds) {
  if (!realtime_state_publisher_ || !std::isfinite(period_seconds) ||
      period_seconds < 0.0) {
    return;
  }
  state_publish_accumulator_ += period_seconds;
  if (state_publish_accumulator_ < state_publish_period_ ||
      !realtime_state_publisher_->trylock()) {
    return;
  }
  state_publish_accumulator_ =
      std::fmod(state_publish_accumulator_, state_publish_period_);
  auto &message = realtime_state_publisher_->msg_;
  message.header.stamp = time;
  const auto *runtime_parameters = runtime_parameters_buffer_.readFromRT();
  if (!runtime_parameters ||
      runtime_parameters->stiffness.size() != params_.joints.size() ||
      runtime_parameters->damping.size() != params_.joints.size()) {
    realtime_state_publisher_->unlock();
    return;
  }
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto eigen_index = static_cast<Eigen::Index>(index);
    auto &values = message.interface_values[index].values;
    values[0] = measured_positions_[eigen_index];
    values[1] = target_position_references_[eigen_index];
    values[2] = active_position_references_[eigen_index];
    values[3] = active_position_references_[eigen_index] -
                measured_positions_[eigen_index];
    values[4] = measured_velocities_[eigen_index];
    values[5] = velocity_references_[eigen_index];
    values[6] = raw_gravity_torques_[eigen_index];
    values[7] = scaled_gravity_torques_[eigen_index];
    values[8] = current_gravity_factor_;
    values[9] = runtime_parameters->stiffness[index];
    values[10] = runtime_parameters->damping[index];
    values[11] = stiffness_torques_[eigen_index];
    values[12] = damping_torques_[eigen_index];
    values[13] = soft_limit_torques_[eigen_index];
    values[14] = target_feedforward_torques_[eigen_index];
    values[15] = commanded_feedforward_torques_[eigen_index];
    values[16] = stiffness_torques_[eigen_index] +
                 damping_torques_[eigen_index] +
                 commanded_feedforward_torques_[eigen_index];
  }
  realtime_state_publisher_->unlockAndPublish();
}

controller_interface::CallbackReturn
MitImpedanceController::on_activate(const rclcpp_lifecycle::State &) {
  const auto *runtime_parameters = runtime_parameters_buffer_.readFromRT();
  if (!runtime_parameters || !bind_interfaces() ||
      !initialize_command(*runtime_parameters)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitImpedanceController::on_deactivate(const rclcpp_lifecycle::State &) {
  const auto *runtime_parameters = runtime_parameters_buffer_.readFromRT();
  const bool command_written =
      command_indices_.empty() ||
      (runtime_parameters && write_command(*runtime_parameters));
  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  return command_written ? controller_interface::CallbackReturn::SUCCESS
                         : controller_interface::CallbackReturn::ERROR;
}

controller_interface::return_type
MitImpedanceController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  const auto *runtime_parameters = runtime_parameters_buffer_.readFromRT();
  if (!runtime_parameters ||
      runtime_parameters->stiffness.size() != params_.joints.size() ||
      runtime_parameters->damping.size() != params_.joints.size() ||
      runtime_parameters->gravity_scale.size() != params_.joints.size() ||
      runtime_parameters->gravity_bias.size() != params_.joints.size() ||
      !update_measured_state() || !update_gravity_factor(period.seconds()) ||
      !update_reference_targets(period.seconds()) ||
      !compute_target_command(*runtime_parameters) ||
      !command_shaper_.update(target_feedforward_torques_, period.seconds(),
                              commanded_feedforward_torques_) ||
      !write_command(*runtime_parameters)) {
    return controller_interface::return_type::ERROR;
  }
  publish_state(time, period.seconds());
  return controller_interface::return_type::OK;
}

} // namespace mit_impedance_controller

PLUGINLIB_EXPORT_CLASS(mit_impedance_controller::MitImpedanceController,
                       controller_interface::ChainableControllerInterface)
