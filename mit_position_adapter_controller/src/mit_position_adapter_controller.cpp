#include "mit_position_adapter_controller/mit_position_adapter_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace mit_position_adapter_controller {
namespace {
constexpr std::array<const char *, 5> kMitInterfaces = {"position", "velocity",
                                                        "kp", "kd", "t_ff"};
constexpr std::array<const char *, 2> kStateInterfaces = {"position",
                                                          "velocity"};
constexpr std::array<const char *, 2> kReferenceInterfaces = {"position",
                                                              "velocity"};
constexpr std::array<const char *, 9> kMitStateInterfaces = {
    "position_reference",       "velocity_reference",
    "raw_gravity_torque",       "scaled_gravity_torque",
    "gravity_factor",           "feedforward_torque",
    "position_feedback_torque", "velocity_feedback_torque",
    "commanded_torque",
};

bool finite_nonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool finite_absolute_bounded(const double value, const double limit) {
  return std::isfinite(value) && std::abs(value) <= limit;
}

Eigen::VectorXd parameter_vector(const std::vector<double> &values,
                                 const std::size_t size,
                                 const double default_value) {
  if (values.empty()) {
    return Eigen::VectorXd::Constant(static_cast<Eigen::Index>(size),
                                     default_value);
  }
  return Eigen::Map<const Eigen::VectorXd>(
      values.data(), static_cast<Eigen::Index>(values.size()));
}
} // namespace

MitPositionAdapterController::~MitPositionAdapterController() {
  if (param_listener_) {
    param_listener_->clearUserCallback();
  }
  gain_validation_callback_.reset();
}

controller_interface::CallbackReturn MitPositionAdapterController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    gain_validation_callback_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          const auto expected_size =
              param_listener_->get_params().joints.size();
          for (const auto &parameter : parameters) {
            const bool is_position_gain =
                parameter.get_name() == "position_kp" ||
                parameter.get_name() == "position_kd";
            const bool is_gravity_scale =
                parameter.get_name() == "gravity_compensation.scale";
            const bool is_gravity_bias =
                parameter.get_name() == "gravity_compensation.bias";
            if (!is_position_gain && !is_gravity_scale && !is_gravity_bias) {
              continue;
            }
            if (parameter.get_type() !=
                rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
              result.successful = false;
              result.reason = parameter.get_name() + " must be a double array";
              return result;
            }
            const auto values = parameter.as_double_array();
            const bool values_valid =
                is_position_gain ? std::all_of(values.begin(), values.end(),
                                               finite_nonnegative)
                : is_gravity_scale
                    ? std::all_of(values.begin(), values.end(),
                                  [](const double value) {
                                    return finite_nonnegative(value) &&
                                           value <= 2.0;
                                  })
                    : std::all_of(values.begin(), values.end(),
                                  [](const double value) {
                                    return finite_absolute_bounded(value, 5.0);
                                  });
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
      if (updated_params.position_kp.size() == joint_count &&
          updated_params.position_kd.size() == joint_count &&
          std::all_of(updated_params.position_kp.begin(),
                      updated_params.position_kp.end(), finite_nonnegative) &&
          std::all_of(updated_params.position_kd.begin(),
                      updated_params.position_kd.end(), finite_nonnegative)) {
        position_gains_buffer_.writeFromNonRT(
            {updated_params.position_kp, updated_params.position_kd});
      }
      auto gravity_scale = updated_params.gravity_compensation.scale;
      auto gravity_bias = updated_params.gravity_compensation.bias;
      if (gravity_scale.empty()) {
        gravity_scale.assign(joint_count, 1.0);
      }
      if (gravity_bias.empty()) {
        gravity_bias.assign(joint_count, 0.0);
      }
      if (gravity_scale.size() == joint_count &&
          gravity_bias.size() == joint_count &&
          std::all_of(gravity_scale.begin(), gravity_scale.end(),
                      [](const double value) {
                        return finite_nonnegative(value) && value <= 2.0;
                      }) &&
          std::all_of(gravity_bias.begin(), gravity_bias.end(),
                      [](const double value) {
                        return finite_absolute_bounded(value, 5.0);
                      })) {
        gravity_calibration_buffer_.writeFromNonRT(
            {gravity_scale, gravity_bias});
      }
    });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "MIT position adapter initialization failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitPositionAdapterController::on_configure(const rclcpp_lifecycle::State &) {
  params_ = param_listener_->get_params();
  const auto joint_count = params_.joints.size();
  if (joint_count == 0 || params_.downstream_controller.empty() ||
      params_.position_kp.size() != joint_count ||
      params_.position_kd.size() != joint_count) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "joints, downstream_controller, position_kp, and position_kd "
                 "must be configured consistently");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < joint_count; ++index) {
    if (!finite_nonnegative(params_.position_kp[index]) ||
        !finite_nonnegative(params_.position_kd[index])) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid position gains for joint '%s'",
                   params_.joints[index].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  position_gains_buffer_.initRT({params_.position_kp, params_.position_kd});

  if (params_.gravity_compensation.gravity.size() != 3) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "gravity_compensation.gravity must contain exactly three values");
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto gravity_scale =
      parameter_vector(params_.gravity_compensation.scale, joint_count, 1.0);
  const auto gravity_bias =
      parameter_vector(params_.gravity_compensation.bias, joint_count, 0.0);
  const auto gravity_torque_limit = parameter_vector(
      params_.gravity_compensation.torque_limit, joint_count, 1.0e6);
  const auto gravity_torque_rate_limit = parameter_vector(
      params_.gravity_compensation.torque_rate_limit, joint_count, 1.0e6);
  if (gravity_scale.size() != static_cast<Eigen::Index>(joint_count) ||
      gravity_bias.size() != static_cast<Eigen::Index>(joint_count) ||
      gravity_torque_limit.size() != static_cast<Eigen::Index>(joint_count) ||
      gravity_torque_rate_limit.size() !=
          static_cast<Eigen::Index>(joint_count)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "gravity compensation scale, bias, and limits must be empty or match "
        "joints");
    return controller_interface::CallbackReturn::ERROR;
  }

  gravity_compensation_model::GravityFeedforwardShaperConfig shaper_config;
  gravity_calibration_buffer_.initRT(
      {std::vector<double>(gravity_scale.data(),
                           gravity_scale.data() + gravity_scale.size()),
       std::vector<double>(gravity_bias.data(),
                           gravity_bias.data() + gravity_bias.size())});
  shaper_config.scale = Eigen::VectorXd::Ones(gravity_scale.size());
  shaper_config.torque_limit = gravity_torque_limit;
  shaper_config.torque_rate_limit = gravity_torque_rate_limit;
  shaper_config.transition_duration =
      params_.gravity_compensation.transition_duration;
  std::string model_error;
  if (!gravity_shaper_.configure(shaper_config, model_error)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Invalid gravity feed-forward configuration: %s",
                 model_error.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  gravity_compensation_enabled_ = params_.gravity_compensation.enabled;
  gravity_factor_transition_duration_ =
      params_.gravity_compensation.factor_transition_duration;
  current_gravity_factor_ = 0.0;
  target_gravity_factor_.store(params_.gravity_compensation.factor,
                               std::memory_order_relaxed);
  if (gravity_compensation_enabled_) {
    const Eigen::Vector3d gravity(params_.gravity_compensation.gravity[0],
                                  params_.gravity_compensation.gravity[1],
                                  params_.gravity_compensation.gravity[2]);
    if (!gravity_model_.configure(get_robot_description(), params_.joints,
                                  gravity, model_error)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Pinocchio gravity model configuration failed: %s",
                   model_error.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  position_references_.assign(joint_count,
                              std::numeric_limits<double>::quiet_NaN());
  velocity_references_.assign(joint_count, 0.0);
  measured_positions_.setZero(static_cast<Eigen::Index>(joint_count));
  measured_velocities_.setZero(static_cast<Eigen::Index>(joint_count));
  raw_gravity_torques_.setZero(static_cast<Eigen::Index>(joint_count));
  factored_gravity_torques_.setZero(static_cast<Eigen::Index>(joint_count));
  feedforward_torques_.setZero(static_cast<Eigen::Index>(joint_count));
  reference_handles_.clear();
  reference_handles_.reserve(joint_count * kReferenceInterfaces.size());
  for (const auto &joint : params_.joints) {
    for (const auto *interface_name : kReferenceInterfaces) {
      reference_handles_.push_back(
          std::make_shared<hardware_interface::CommandInterface>(
              get_node()->get_name(), joint + "/" + interface_name));
    }
  }

  state_publish_period_ = 1.0 / params_.state_publish_rate;
  state_publish_accumulator_ = 0.0;
  try {
    mit_state_publisher_ =
        get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
            "~/mit_state", rclcpp::SensorDataQoS());
    realtime_mit_state_publisher_ =
        std::make_unique<MitStatePublisher>(mit_state_publisher_);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to create MIT state publisher: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  realtime_mit_state_publisher_->lock();
  auto &state_message = realtime_mit_state_publisher_->msg_;
  state_message.joint_names = params_.joints;
  state_message.interface_values.resize(joint_count);
  for (auto &interface_value : state_message.interface_values) {
    interface_value.interface_names.assign(kMitStateInterfaces.begin(),
                                           kMitStateInterfaces.end());
    interface_value.values.resize(kMitStateInterfaces.size());
  }
  realtime_mit_state_publisher_->unlock();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MitPositionAdapterController::command_interface_configuration() const {
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
MitPositionAdapterController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : params_.joints) {
    for (const auto *interface_name : kStateInterfaces) {
      configuration.names.push_back(joint + "/" + interface_name);
    }
  }
  return configuration;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
MitPositionAdapterController::on_export_reference_interfaces_list() {
  return reference_handles_;
}

std::size_t MitPositionAdapterController::find_command_interface(
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

std::size_t MitPositionAdapterController::find_state_interface(
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

bool MitPositionAdapterController::bind_interfaces() {
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
                 "MIT position adapter interface binding failed: %s",
                 error.what());
    return false;
  }
  return true;
}

bool MitPositionAdapterController::initialize_references() {
  const auto *gains = position_gains_buffer_.readFromRT();
  if (!gains || gains->kp.size() != params_.joints.size() ||
      gains->kd.size() != params_.joints.size()) {
    return false;
  }
  Eigen::VectorXd initial_feedforward(
      static_cast<Eigen::Index>(params_.joints.size()));
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto measured_position =
        state_interfaces_[position_state_indices_[index]].get_optional();
    if (!measured_position || !std::isfinite(*measured_position)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "No finite startup position for joint '%s'",
                   params_.joints[index].c_str());
      return false;
    }
    measured_positions_[static_cast<Eigen::Index>(index)] = *measured_position;

    const auto measured_velocity =
        state_interfaces_[velocity_state_indices_[index]].get_optional();
    measured_velocities_[static_cast<Eigen::Index>(index)] =
        measured_velocity && std::isfinite(*measured_velocity)
            ? *measured_velocity
            : 0.0;

    const auto &indices = command_indices_[index];
    const auto downstream_position =
        command_interfaces_[indices.position].get_optional();
    if (downstream_position && std::isfinite(*downstream_position)) {
      position_references_[index] = *downstream_position;
    } else {
      position_references_[index] = *measured_position;
    }

    const auto read_or = [this](const std::size_t command_index,
                                const double fallback) {
      const auto value = command_interfaces_[command_index].get_optional();
      return value && std::isfinite(*value) ? *value : fallback;
    };
    const double old_position = read_or(indices.position, *measured_position);
    const double old_velocity = read_or(indices.velocity, 0.0);
    velocity_references_[index] = old_velocity;
    const double old_kp = read_or(indices.kp, gains->kp[index]);
    const double old_kd = read_or(indices.kd, gains->kd[index]);
    const double old_feedforward = read_or(indices.t_ff, 0.0);
    const double measured_velocity_value =
        measured_velocities_[static_cast<Eigen::Index>(index)];
    const double old_torque =
        old_kp * (old_position - *measured_position) +
        old_kd * (old_velocity - measured_velocity_value) + old_feedforward;
    const double new_feedback_torque =
        gains->kp[index] * (position_references_[index] - *measured_position) +
        gains->kd[index] *
            (velocity_references_[index] - measured_velocity_value);
    initial_feedforward[static_cast<Eigen::Index>(index)] =
        old_torque - new_feedback_torque;

    const auto reference_offset = index * kReferenceInterfaces.size();
    (void)reference_handles_[reference_offset]->set_value(
        position_references_[index]);
    (void)reference_handles_[reference_offset + 1]->set_value(
        velocity_references_[index]);
  }

  raw_gravity_torques_.setZero();
  factored_gravity_torques_.setZero();
  current_gravity_factor_ = 0.0;
  if (gravity_compensation_enabled_ &&
      !gravity_model_.compute(measured_positions_, raw_gravity_torques_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to compute initial Pinocchio gravity torque");
    return false;
  }
  if (!gravity_shaper_.reset(initial_feedforward) ||
      !gravity_shaper_.update(factored_gravity_torques_, 0.0,
                              feedforward_torques_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to initialize gravity feed-forward transition");
    return false;
  }
  return write_position_mode(*gains);
}

bool MitPositionAdapterController::update_measured_state() {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto position =
        state_interfaces_[position_state_indices_[index]].get_optional();
    if (!position || !std::isfinite(*position)) {
      return false;
    }
    measured_positions_[static_cast<Eigen::Index>(index)] = *position;

    const auto velocity =
        state_interfaces_[velocity_state_indices_[index]].get_optional();
    if (velocity && std::isfinite(*velocity)) {
      measured_velocities_[static_cast<Eigen::Index>(index)] = *velocity;
    }
  }
  return true;
}

bool MitPositionAdapterController::update_gravity_factor(
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

bool MitPositionAdapterController::update_gravity_feedforward(
    const double period_seconds,
    const GravityCalibration &gravity_calibration) {
  if (!update_measured_state() || !update_gravity_factor(period_seconds)) {
    return false;
  }
  if (gravity_compensation_enabled_) {
    if (!gravity_model_.compute(measured_positions_, raw_gravity_torques_)) {
      return false;
    }
    for (Eigen::Index index = 0; index < raw_gravity_torques_.size(); ++index) {
      factored_gravity_torques_[index] =
          current_gravity_factor_ *
          (gravity_calibration.scale[static_cast<std::size_t>(index)] *
               raw_gravity_torques_[index] +
           gravity_calibration.bias[static_cast<std::size_t>(index)]);
    }
  } else {
    raw_gravity_torques_.setZero();
    factored_gravity_torques_.setZero();
  }
  return gravity_shaper_.update(factored_gravity_torques_, period_seconds,
                                feedforward_torques_);
}

bool MitPositionAdapterController::write_position_mode(
    const PositionGains &gains) {
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto &indices = command_indices_[index];
    if (!command_interfaces_[indices.position].set_value(
            position_references_[index]) ||
        !command_interfaces_[indices.velocity].set_value(
            velocity_references_[index]) ||
        !command_interfaces_[indices.kp].set_value(gains.kp[index]) ||
        !command_interfaces_[indices.kd].set_value(gains.kd[index]) ||
        !command_interfaces_[indices.t_ff].set_value(
            feedforward_torques_[static_cast<Eigen::Index>(index)])) {
      return false;
    }
  }
  return true;
}

void MitPositionAdapterController::publish_mit_state(
    const rclcpp::Time &time, const double period_seconds,
    const PositionGains &gains, const GravityCalibration &gravity_calibration) {
  if (!realtime_mit_state_publisher_ || !std::isfinite(period_seconds) ||
      period_seconds < 0.0) {
    return;
  }
  state_publish_accumulator_ += period_seconds;
  if (state_publish_accumulator_ < state_publish_period_ ||
      !realtime_mit_state_publisher_->trylock()) {
    return;
  }
  state_publish_accumulator_ =
      std::fmod(state_publish_accumulator_, state_publish_period_);

  auto &message = realtime_mit_state_publisher_->msg_;
  message.header.stamp = time;
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto eigen_index = static_cast<Eigen::Index>(index);
    const double position_feedback =
        gains.kp[index] *
        (position_references_[index] - measured_positions_[eigen_index]);
    const double velocity_feedback =
        gains.kd[index] *
        (velocity_references_[index] - measured_velocities_[eigen_index]);
    const double scaled_gravity =
        current_gravity_factor_ *
        (gravity_calibration.scale[index] * raw_gravity_torques_[eigen_index] +
         gravity_calibration.bias[index]);
    auto &values = message.interface_values[index].values;
    values[0] = position_references_[index];
    values[1] = velocity_references_[index];
    values[2] = raw_gravity_torques_[eigen_index];
    values[3] = scaled_gravity;
    values[4] = current_gravity_factor_;
    values[5] = feedforward_torques_[eigen_index];
    values[6] = position_feedback;
    values[7] = velocity_feedback;
    values[8] = feedforward_torques_[eigen_index] + position_feedback +
                velocity_feedback;
  }
  realtime_mit_state_publisher_->unlockAndPublish();
}

controller_interface::CallbackReturn
MitPositionAdapterController::on_activate(const rclcpp_lifecycle::State &) {
  if (!bind_interfaces() || !initialize_references()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitPositionAdapterController::on_deactivate(const rclcpp_lifecycle::State &) {
  const auto *gains = position_gains_buffer_.readFromRT();
  const bool hold_written =
      command_indices_.empty() || (gains && write_position_mode(*gains));
  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  return hold_written ? controller_interface::CallbackReturn::SUCCESS
                      : controller_interface::CallbackReturn::ERROR;
}

bool MitPositionAdapterController::on_set_chained_mode(const bool) {
  return true;
}

controller_interface::return_type
MitPositionAdapterController::update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) {
  return controller_interface::return_type::OK;
}

controller_interface::return_type
MitPositionAdapterController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  const auto *gains = position_gains_buffer_.readFromRT();
  const auto *gravity_calibration = gravity_calibration_buffer_.readFromRT();
  if (!gains || gains->kp.size() != params_.joints.size() ||
      gains->kd.size() != params_.joints.size() || !gravity_calibration ||
      gravity_calibration->scale.size() != params_.joints.size() ||
      gravity_calibration->bias.size() != params_.joints.size()) {
    return controller_interface::return_type::ERROR;
  }
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto reference_offset = index * kReferenceInterfaces.size();
    const auto position_reference =
        reference_handles_[reference_offset]->get_optional();
    if (position_reference && std::isfinite(*position_reference)) {
      position_references_[index] = *position_reference;
    } else {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000,
                            "Invalid position reference for joint '%s'; "
                            "retaining the previous command",
                            params_.joints[index].c_str());
      (void)reference_handles_[reference_offset]->set_value(
          position_references_[index]);
    }

    const auto velocity_reference =
        reference_handles_[reference_offset + 1]->get_optional();
    if (velocity_reference && std::isfinite(*velocity_reference)) {
      velocity_references_[index] = *velocity_reference;
    } else {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                            1000,
                            "Invalid velocity reference for joint '%s'; "
                            "retaining the previous command",
                            params_.joints[index].c_str());
      (void)reference_handles_[reference_offset + 1]->set_value(
          velocity_references_[index]);
    }
  }
  if (!update_gravity_feedforward(period.seconds(), *gravity_calibration)) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          1000,
                          "Invalid gravity compensation input; retaining the "
                          "previous feed-forward torque");
  }
  if (!write_position_mode(*gains)) {
    return controller_interface::return_type::ERROR;
  }
  publish_mit_state(time, period.seconds(), *gains, *gravity_calibration);
  return controller_interface::return_type::OK;
}

} // namespace mit_position_adapter_controller

PLUGINLIB_EXPORT_CLASS(
    mit_position_adapter_controller::MitPositionAdapterController,
    controller_interface::ChainableControllerInterface)
