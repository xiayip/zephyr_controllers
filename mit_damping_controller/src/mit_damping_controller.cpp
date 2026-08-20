#include "mit_damping_controller/mit_damping_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace mit_damping_controller {
namespace {
constexpr std::array<const char *, 5> kMitInterfaces = {"position", "velocity",
                                                        "kp", "kd", "t_ff"};
constexpr std::array<const char *, 10> kStateFields = {
    "position",           "raw_velocity",          "filtered_velocity",
    "raw_gravity_torque", "scaled_gravity_torque", "gravity_factor",
    "damping_torque",     "soft_limit_torque",     "target_torque",
    "commanded_torque"};

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

MitDampingController::~MitDampingController() {
  if (param_listener_) {
    param_listener_->clearUserCallback();
  }
  runtime_validation_callback_.reset();
}

controller_interface::CallbackReturn MitDampingController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    runtime_validation_callback_ = get_node()->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          const auto expected_size =
              param_listener_->get_params().joints.size();
          for (const auto &parameter : parameters) {
            const bool is_damping =
                parameter.get_name() == "damping_coefficients";
            const bool is_gravity_scale =
                parameter.get_name() == "gravity_compensation.scale";
            const bool is_gravity_bias =
                parameter.get_name() == "gravity_compensation.bias";
            if (!is_damping && !is_gravity_scale && !is_gravity_bias) {
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
                is_gravity_bias
                    ? std::all_of(values.begin(), values.end(),
                                  [](const double value) {
                                    return finite_absolute_bounded(value, 5.0);
                                  })
                    : std::all_of(values.begin(), values.end(),
                                  [is_damping](const double value) {
                                    return finite_in_range(
                                        value, is_damping ? 20.0 : 2.0);
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
      const auto &damping = updated_params.damping_coefficients;
      const auto &scale = updated_params.gravity_compensation.scale;
      const auto &bias = updated_params.gravity_compensation.bias;
      if (damping.size() == joint_count && scale.size() == joint_count &&
          bias.size() == joint_count &&
          std::all_of(damping.begin(), damping.end(),
                      [](const double value) {
                        return finite_in_range(value, 20.0);
                      }) &&
          std::all_of(
              scale.begin(), scale.end(),
              [](const double value) { return finite_in_range(value, 2.0); }) &&
          std::all_of(bias.begin(), bias.end(), [](const double value) {
            return finite_absolute_bounded(value, 5.0);
          })) {
        runtime_parameters_buffer_.writeFromNonRT({damping, scale, bias});
      }
    });
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "MIT damping controller initialization failed: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitDampingController::on_configure(const rclcpp_lifecycle::State &) {
  params_ = param_listener_->get_params();
  const auto joint_count = params_.joints.size();
  const auto has_joint_count = [joint_count](const auto &values) {
    return values.size() == joint_count;
  };
  if (joint_count == 0 || params_.downstream_controller.empty() ||
      !has_joint_count(params_.damping_coefficients) ||
      !has_joint_count(params_.damping_torque_limit) ||
      !has_joint_count(params_.gravity_compensation.scale) ||
      !has_joint_count(params_.gravity_compensation.bias) ||
      !has_joint_count(params_.command_shaping.torque_limit) ||
      !has_joint_count(params_.command_shaping.torque_rate_limit) ||
      !has_joint_count(params_.soft_limits.margin) ||
      !has_joint_count(params_.soft_limits.stiffness) ||
      !has_joint_count(params_.soft_limits.torque_limit)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "All per-joint damping parameters must match joints");
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

  runtime_parameters_buffer_.initRT({params_.damping_coefficients,
                                     params_.gravity_compensation.scale,
                                     params_.gravity_compensation.bias});
  gravity_compensation_enabled_ = params_.gravity_compensation.enabled;
  velocity_filter_enabled_ = params_.velocity_filter.enabled;
  soft_limits_enabled_ = params_.soft_limits.enabled;
  velocity_filter_cutoff_frequency_ = params_.velocity_filter.cutoff_frequency;
  gravity_factor_transition_duration_ =
      params_.gravity_compensation.factor_transition_duration;
  command_transition_duration_ = params_.command_shaping.transition_duration;
  target_gravity_factor_.store(params_.gravity_compensation.factor,
                               std::memory_order_relaxed);
  current_gravity_factor_ = params_.gravity_compensation.factor;

  const auto size = static_cast<Eigen::Index>(joint_count);
  measured_positions_.setZero(size);
  measured_velocities_.setZero(size);
  filtered_velocities_.setZero(size);
  raw_gravity_torques_.setZero(size);
  scaled_gravity_torques_.setZero(size);
  damping_torques_.setZero(size);
  soft_limit_torques_.setZero(size);
  target_torques_.setZero(size);
  commanded_torques_.setZero(size);
  transition_positions_.setZero(size);
  transition_velocities_.setZero(size);
  transition_kp_.setZero(size);
  transition_kd_.setZero(size);
  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();

  state_publish_period_ = 1.0 / params_.state_publish_rate;
  state_publish_accumulator_ = 0.0;
  try {
    state_publisher_ =
        get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
            "~/state", rclcpp::SensorDataQoS());
    realtime_state_publisher_ =
        std::make_unique<StatePublisher>(state_publisher_);
  } catch (const std::exception &publish_error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to create damping state publisher: %s",
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
MitDampingController::command_interface_configuration() const {
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
MitDampingController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint : params_.joints) {
    configuration.names.push_back(joint + "/position");
    configuration.names.push_back(joint + "/velocity");
  }
  return configuration;
}

std::size_t MitDampingController::find_command_interface(
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

std::size_t
MitDampingController::find_state_interface(const std::string &full_name) const {
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

bool MitDampingController::bind_interfaces() {
  try {
    command_indices_.clear();
    position_state_indices_.clear();
    velocity_state_indices_.clear();
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
                 "MIT damping interface binding failed: %s", error.what());
    return false;
  }
  return true;
}

bool MitDampingController::reset_velocity_filters() {
  velocity_filters_.clear();
  if (!velocity_filter_enabled_) {
    return true;
  }
  const double sample_frequency = static_cast<double>(get_update_rate());
  if (!std::isfinite(sample_frequency) || sample_frequency <= 0.0 ||
      velocity_filter_cutoff_frequency_ >= 0.5 * sample_frequency) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Velocity cutoff must be below the Nyquist frequency");
    return false;
  }
  velocity_filters_.reserve(params_.joints.size());
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    velocity_filters_.emplace_back(sample_frequency,
                                   velocity_filter_cutoff_frequency_, 0.0);
    if (!velocity_filters_.back().configure()) {
      return false;
    }
  }
  return true;
}

bool MitDampingController::update_measured_state() {
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
    if (velocity_filter_enabled_) {
      if (!velocity_filters_[index].update(*velocity,
                                           filtered_velocities_[eigen_index])) {
        return false;
      }
    } else {
      filtered_velocities_[eigen_index] = *velocity;
    }
  }
  return true;
}

bool MitDampingController::update_gravity_factor(
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

bool MitDampingController::update_command_transition(
    const double period_seconds) noexcept {
  if (!std::isfinite(period_seconds) || period_seconds < 0.0) {
    return false;
  }
  if (command_transition_duration_ == 0.0) {
    command_transition_progress_ = 1.0;
  } else {
    command_transition_progress_ =
        std::min(1.0, command_transition_progress_ +
                          period_seconds / command_transition_duration_);
  }
  return true;
}

bool MitDampingController::compute_target_torque(
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

    const double damping_limit = params_.damping_torque_limit[index];
    damping_torques_[eigen_index] =
        std::clamp(-runtime_parameters.damping_coefficients[index] *
                       filtered_velocities_[eigen_index],
                   -damping_limit, damping_limit);

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
    target_torques_[eigen_index] = scaled_gravity_torques_[eigen_index] +
                                   damping_torques_[eigen_index] +
                                   soft_limit_torques_[eigen_index];
  }
  return target_torques_.allFinite();
}

bool MitDampingController::initialize_command(
    const RuntimeParameters &runtime_parameters) {
  if (!reset_velocity_filters() || !update_measured_state()) {
    return false;
  }
  Eigen::VectorXd initial_torque(
      static_cast<Eigen::Index>(params_.joints.size()));
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
    transition_positions_[eigen_index] = old_position;
    transition_velocities_[eigen_index] = old_velocity;
    transition_kp_[eigen_index] = old_kp;
    transition_kd_[eigen_index] = old_kd;
    initial_torque[eigen_index] = old_feedforward;
  }
  command_transition_progress_ =
      command_transition_duration_ == 0.0 ? 1.0 : 0.0;
  current_gravity_factor_ =
      target_gravity_factor_.load(std::memory_order_relaxed);
  if (!command_shaper_.reset(initial_torque) ||
      !compute_target_torque(runtime_parameters) ||
      !command_shaper_.update(target_torques_, 0.0, commanded_torques_)) {
    return false;
  }
  return write_command();
}

bool MitDampingController::write_command() {
  const double old_command_weight = 1.0 - command_transition_progress_;
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto &indices = command_indices_[index];
    const auto eigen_index = static_cast<Eigen::Index>(index);
    const bool transition_complete = command_transition_progress_ >= 1.0;
    const double position_command = transition_complete
                                        ? measured_positions_[eigen_index]
                                        : transition_positions_[eigen_index];
    const double velocity_command =
        transition_complete ? 0.0 : transition_velocities_[eigen_index];
    if (!command_interfaces_[indices.position].set_value(position_command) ||
        !command_interfaces_[indices.velocity].set_value(velocity_command) ||
        !command_interfaces_[indices.kp].set_value(
            old_command_weight * transition_kp_[eigen_index]) ||
        !command_interfaces_[indices.kd].set_value(
            old_command_weight * transition_kd_[eigen_index]) ||
        !command_interfaces_[indices.t_ff].set_value(
            commanded_torques_[eigen_index])) {
      return false;
    }
  }
  return true;
}

void MitDampingController::publish_state(const rclcpp::Time &time,
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
  for (std::size_t index = 0; index < params_.joints.size(); ++index) {
    const auto eigen_index = static_cast<Eigen::Index>(index);
    auto &values = message.interface_values[index].values;
    values[0] = measured_positions_[eigen_index];
    values[1] = measured_velocities_[eigen_index];
    values[2] = filtered_velocities_[eigen_index];
    values[3] = raw_gravity_torques_[eigen_index];
    values[4] = scaled_gravity_torques_[eigen_index];
    values[5] = current_gravity_factor_;
    values[6] = damping_torques_[eigen_index];
    values[7] = soft_limit_torques_[eigen_index];
    values[8] = target_torques_[eigen_index];
    const double old_command_weight = 1.0 - command_transition_progress_;
    const bool transition_complete = command_transition_progress_ >= 1.0;
    const double position_command = transition_complete
                                        ? measured_positions_[eigen_index]
                                        : transition_positions_[eigen_index];
    const double velocity_command =
        transition_complete ? 0.0 : transition_velocities_[eigen_index];
    values[9] = old_command_weight * transition_kp_[eigen_index] *
                    (position_command - measured_positions_[eigen_index]) +
                old_command_weight * transition_kd_[eigen_index] *
                    (velocity_command - measured_velocities_[eigen_index]) +
                commanded_torques_[eigen_index];
  }
  realtime_state_publisher_->unlockAndPublish();
}

controller_interface::CallbackReturn
MitDampingController::on_activate(const rclcpp_lifecycle::State &) {
  const auto *runtime_parameters = runtime_parameters_buffer_.readFromRT();
  if (!runtime_parameters || !bind_interfaces() ||
      !initialize_command(*runtime_parameters)) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MitDampingController::on_deactivate(const rclcpp_lifecycle::State &) {
  const bool command_written = command_indices_.empty() || write_command();
  command_indices_.clear();
  position_state_indices_.clear();
  velocity_state_indices_.clear();
  return command_written ? controller_interface::CallbackReturn::SUCCESS
                         : controller_interface::CallbackReturn::ERROR;
}

controller_interface::return_type
MitDampingController::update(const rclcpp::Time &time,
                             const rclcpp::Duration &period) {
  const auto *runtime_parameters = runtime_parameters_buffer_.readFromRT();
  if (!runtime_parameters ||
      runtime_parameters->damping_coefficients.size() !=
          params_.joints.size() ||
      runtime_parameters->gravity_scale.size() != params_.joints.size() ||
      runtime_parameters->gravity_bias.size() != params_.joints.size() ||
      !update_measured_state() || !update_gravity_factor(period.seconds()) ||
      !update_command_transition(period.seconds()) ||
      !compute_target_torque(*runtime_parameters) ||
      !command_shaper_.update(target_torques_, period.seconds(),
                              commanded_torques_) ||
      !write_command()) {
    return controller_interface::return_type::ERROR;
  }
  publish_state(time, period.seconds());
  return controller_interface::return_type::OK;
}

} // namespace mit_damping_controller

PLUGINLIB_EXPORT_CLASS(mit_damping_controller::MitDampingController,
                       controller_interface::ControllerInterface)
