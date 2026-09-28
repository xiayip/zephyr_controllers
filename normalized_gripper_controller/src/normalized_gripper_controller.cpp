#include "normalized_gripper_controller/normalized_gripper_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace normalized_gripper_controller {
namespace {

constexpr std::size_t kPositionReference = 0;
constexpr std::size_t kVelocityReference = 1;
constexpr std::size_t kEffortReference = 2;

bool has_interface(const std::vector<std::string> &interfaces,
                   const std::string &name) {
  return std::find(interfaces.begin(), interfaces.end(), name) !=
         interfaces.end();
}

} // namespace

NormalizedGripperController::NormalizedGripperController()
    : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn NormalizedGripperController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to initialize parameters: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool NormalizedGripperController::validate_interfaces() const {
  const std::set<std::string> supported{"position", "velocity", "effort"};
  const auto valid = [&supported](const std::vector<std::string> &interfaces) {
    return std::all_of(interfaces.begin(), interfaces.end(),
                       [&supported](const std::string &interface) {
                         return supported.find(interface) != supported.end();
                       });
  };
  return valid(params_.command_interfaces) && valid(params_.state_interfaces) &&
         has_interface(params_.command_interfaces,
                       hardware_interface::HW_IF_POSITION) &&
         has_interface(params_.state_interfaces,
                       hardware_interface::HW_IF_POSITION);
}

controller_interface::CallbackReturn
NormalizedGripperController::on_configure(const rclcpp_lifecycle::State &) {
  active_.store(false);
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Parameter listener is not initialized");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();
  if (params_.joints.size() != 1 || params_.joints.front().empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exactly one non-empty joint is required");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (params_.topic_prefix.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "topic_prefix must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!validate_interfaces()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Only position, velocity, and effort interfaces are "
                 "supported; position is required");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_subscriber_ =
      get_node()->create_subscription<sensor_msgs::msg::JointState>(
          params_.topic_prefix + "/gripper_command",
          rclcpp::SystemDefaultsQoS(),
          [this](sensor_msgs::msg::JointState::SharedPtr message) {
            receive_command(*message);
          });

  state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      params_.topic_prefix + "/joint_states", rclcpp::SystemDefaultsQoS());
  realtime_state_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
      state_publisher_);
  auto &state = realtime_state_publisher_->msg_;
  state.name = params_.joints;
  state.position.resize(1, std::numeric_limits<double>::quiet_NaN());
  state.velocity.resize(1, std::numeric_limits<double>::quiet_NaN());
  state.effort.resize(1, std::numeric_limits<double>::quiet_NaN());

  reference_handles_.clear();
  reference_handles_.reserve(3);
  for (const auto *interface : {"position", "velocity", "effort"}) {
    reference_handles_.push_back(
        std::make_shared<hardware_interface::CommandInterface>(
            get_node()->get_name(), params_.joints.front() + "/" + interface));
  }
  action_monitor_period_ =
      rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);
  action_result_ = std::make_shared<GripperCommandAction::Result>();
  action_result_->state.name = params_.joints;
  action_result_->state.position.resize(1);
  action_result_->state.velocity.resize(1);
  action_result_->state.effort.resize(1);

  action_server_ = rclcpp_action::create_server<GripperCommandAction>(
      get_node(), "~/gripper_cmd",
      std::bind(&NormalizedGripperController::goal_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&NormalizedGripperController::cancel_callback, this,
                std::placeholders::_1),
      std::bind(&NormalizedGripperController::accepted_callback, this,
                std::placeholders::_1));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
NormalizedGripperController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!params_.joints.empty()) {
    for (const auto &interface : params_.command_interfaces) {
      configuration.names.push_back(params_.joints.front() + "/" + interface);
    }
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
NormalizedGripperController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!params_.joints.empty()) {
    for (const auto &interface : params_.state_interfaces) {
      configuration.names.push_back(params_.joints.front() + "/" + interface);
    }
  }
  return configuration;
}

controller_interface::CallbackReturn
NormalizedGripperController::on_activate(const rclcpp_lifecycle::State &) {
  const auto command_by_name = [this](const std::string &interface)
      -> std::optional<
          std::reference_wrapper<hardware_interface::LoanedCommandInterface>> {
    const auto expected = params_.joints.front() + "/" + interface;
    const auto item =
        std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                     [&expected](const auto &candidate) {
                       return candidate.get_name() == expected;
                     });
    if (item == command_interfaces_.end()) {
      return std::nullopt;
    }
    return std::ref(*item);
  };
  const auto state_by_name = [this](const std::string &interface)
      -> std::optional<
          std::reference_wrapper<hardware_interface::LoanedStateInterface>> {
    const auto expected = params_.joints.front() + "/" + interface;
    const auto item =
        std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                     [&expected](const auto &candidate) {
                       return candidate.get_name() == expected;
                     });
    if (item == state_interfaces_.end()) {
      return std::nullopt;
    }
    return std::ref(*item);
  };

  position_command_interface_ =
      command_by_name(hardware_interface::HW_IF_POSITION);
  velocity_command_interface_ =
      command_by_name(hardware_interface::HW_IF_VELOCITY);
  effort_command_interface_ = command_by_name(hardware_interface::HW_IF_EFFORT);
  position_state_interface_ = state_by_name(hardware_interface::HW_IF_POSITION);
  velocity_state_interface_ = state_by_name(hardware_interface::HW_IF_VELOCITY);
  effort_state_interface_ = state_by_name(hardware_interface::HW_IF_EFFORT);

  if (!position_command_interface_ || !position_state_interface_) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Required position interfaces were not assigned");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (has_interface(params_.command_interfaces,
                    hardware_interface::HW_IF_VELOCITY) !=
      velocity_command_interface_.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Configured velocity command interface was not assigned");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (has_interface(params_.command_interfaces,
                    hardware_interface::HW_IF_EFFORT) !=
      effort_command_interface_.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Configured effort command interface was not assigned");
    return controller_interface::CallbackReturn::ERROR;
  }

  const double current_position =
      position_state_interface_->get().get_optional().value_or(
          std::numeric_limits<double>::quiet_NaN());
  if (!std::isfinite(current_position) || current_position < 0.0 ||
      current_position > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Initial normalized position is invalid: %.6f",
                 current_position);
    return controller_interface::CallbackReturn::ERROR;
  }
  command_from_subscriber_ = {current_position, params_.max_velocity,
                              params_.max_effort};
  command_.initRT(command_from_subscriber_);
  (void)reference_handles_[kPositionReference]->set_value(current_position);
  (void)reference_handles_[kVelocityReference]->set_value(params_.max_velocity);
  (void)reference_handles_[kEffortReference]->set_value(params_.max_effort);
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
NormalizedGripperController::on_deactivate(const rclcpp_lifecycle::State &) {
  active_.store(false);
  preempt_active_goal();
  goal_handle_timer_.reset();
  position_command_interface_.reset();
  velocity_command_interface_.reset();
  effort_command_interface_.reset();
  position_state_interface_.reset();
  velocity_state_interface_.reset();
  effort_state_interface_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool NormalizedGripperController::on_set_chained_mode(bool chained_mode) {
  if (chained_mode) {
    preempt_active_goal();
  }
  chained_mode_.store(chained_mode);
  return true;
}

controller_interface::return_type
NormalizedGripperController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &) {
  const double current_position =
      position_state_interface_->get().get_optional().value_or(
          std::numeric_limits<double>::quiet_NaN());
  const double current_velocity =
      velocity_state_interface_
          ? velocity_state_interface_->get().get_optional().value_or(0.0)
          : 0.0;
  const double current_effort =
      effort_state_interface_
          ? effort_state_interface_->get().get_optional().value_or(
                std::numeric_limits<double>::quiet_NaN())
          : std::numeric_limits<double>::quiet_NaN();

  const auto position_reference =
      reference_handles_[kPositionReference]->get_optional();
  const auto velocity_reference =
      reference_handles_[kVelocityReference]->get_optional();
  const auto effort_reference =
      reference_handles_[kEffortReference]->get_optional();
  if (!position_reference || !velocity_reference || !effort_reference) {
    return controller_interface::return_type::ERROR;
  }
  final_command_ = {*position_reference, *velocity_reference,
                    *effort_reference};
  if (!std::isfinite(final_command_.position) ||
      !position_command_interface_->get().set_value(final_command_.position)) {
    return controller_interface::return_type::ERROR;
  }
  if (velocity_command_interface_ && std::isfinite(final_command_.velocity) &&
      !velocity_command_interface_->get().set_value(final_command_.velocity)) {
    return controller_interface::return_type::ERROR;
  }
  if (effort_command_interface_ && std::isfinite(final_command_.effort) &&
      !effort_command_interface_->get().set_value(final_command_.effort)) {
    return controller_interface::return_type::ERROR;
  }

  check_for_success(time, current_position, current_velocity);
  if (realtime_state_publisher_ && realtime_state_publisher_->trylock()) {
    auto &state = realtime_state_publisher_->msg_;
    state.header.stamp = time;
    state.position[0] = current_position;
    state.velocity[0] = current_velocity;
    state.effort[0] = current_effort;
    realtime_state_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
NormalizedGripperController::on_export_reference_interfaces_list() {
  return reference_handles_;
}

controller_interface::return_type
NormalizedGripperController::update_reference_from_subscribers(
    const rclcpp::Time &, const rclcpp::Duration &) {
  const auto command = command_.readFromRT();
  if (command == nullptr) {
    return controller_interface::return_type::ERROR;
  }
  if (!reference_handles_[kPositionReference]->set_value(command->position) ||
      !reference_handles_[kVelocityReference]->set_value(command->velocity) ||
      !reference_handles_[kEffortReference]->set_value(command->effort)) {
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

bool NormalizedGripperController::parse_command(
    const sensor_msgs::msg::JointState &message, Commands &command) const {
  if (message.position.size() != 1 || !std::isfinite(message.position[0]) ||
      message.position[0] < 0.0 || message.position[0] > 1.0) {
    return false;
  }
  if (!message.name.empty() &&
      (message.name.size() != 1 ||
       message.name.front() != params_.joints.front())) {
    return false;
  }
  const auto optional_normalized = [](const std::vector<double> &values,
                                      double fallback) {
    if (values.empty() || !std::isfinite(values.front())) {
      return fallback;
    }
    return std::clamp(values.front(), 0.0, 1.0);
  };
  command.position = message.position.front();
  command.velocity =
      optional_normalized(message.velocity, params_.max_velocity);
  command.effort = optional_normalized(message.effort, params_.max_effort);
  return true;
}

void NormalizedGripperController::receive_command(
    const sensor_msgs::msg::JointState &message) {
  if (!active_.load()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring topic command while controller is inactive");
    return;
  }
  if (chained_mode_.load()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring topic command while in chained mode");
    return;
  }
  Commands command;
  if (!parse_command(message, command)) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Ignoring gripper command: expected one position in [0, 1] for "
                "joint '%s'",
                params_.joints.front().c_str());
    return;
  }
  preempt_active_goal();
  command_from_subscriber_ = command;
  command_.writeFromNonRT(command_from_subscriber_);
}

rclcpp_action::GoalResponse NormalizedGripperController::goal_callback(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperCommandAction::Goal> goal) {
  Commands command;
  if (!active_.load() || chained_mode_.load() ||
      !parse_command(goal->command, command)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void NormalizedGripperController::accepted_callback(
    std::shared_ptr<GoalHandle> goal_handle) {
  Commands command;
  if (!parse_command(goal_handle->get_goal()->command, command)) {
    goal_handle->abort(std::make_shared<GripperCommandAction::Result>());
    return;
  }
  preempt_active_goal();
  command_from_subscriber_ = command;
  command_.writeFromNonRT(command_from_subscriber_);
  active_goal_position_ = command.position;
  last_movement_time_ = get_node()->now();

  action_result_->reached_goal = false;
  action_result_->stalled = false;
  auto realtime_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  realtime_goal->execute();
  active_goal_.set([realtime_goal](RealtimeGoalHandlePtr &stored) {
    stored = realtime_goal;
  });

  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(
      action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
      std::bind(&RealtimeGoalHandle::runNonRealtime, realtime_goal));
}

rclcpp_action::CancelResponse NormalizedGripperController::cancel_callback(
    const std::shared_ptr<GoalHandle> goal_handle) {
  RealtimeGoalHandlePtr active;
  active_goal_.get(
      [&active](const RealtimeGoalHandlePtr &stored) { active = stored; });
  if (active && active->gh_ == goal_handle) {
    set_hold_position();
    active->setCanceled(std::make_shared<GripperCommandAction::Result>());
    active_goal_.set([](RealtimeGoalHandlePtr &stored) { stored.reset(); });
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NormalizedGripperController::preempt_active_goal() {
  RealtimeGoalHandlePtr active;
  active_goal_.get(
      [&active](const RealtimeGoalHandlePtr &stored) { active = stored; });
  if (active) {
    active->setCanceled(std::make_shared<GripperCommandAction::Result>());
    active_goal_.set([](RealtimeGoalHandlePtr &stored) { stored.reset(); });
  }
}

void NormalizedGripperController::set_hold_position() {
  if (!position_state_interface_) {
    return;
  }
  const auto current = position_state_interface_->get().get_optional();
  if (!current || !std::isfinite(*current)) {
    return;
  }
  command_from_subscriber_ = {*current, params_.max_velocity,
                              params_.max_effort};
  command_.writeFromNonRT(command_from_subscriber_);
}

void NormalizedGripperController::check_for_success(const rclcpp::Time &time,
                                                    double current_position,
                                                    double current_velocity) {
  RealtimeGoalHandlePtr active;
  active_goal_.get(
      [&active](const RealtimeGoalHandlePtr &stored) { active = stored; });
  if (!active || !std::isfinite(current_position)) {
    return;
  }

  action_result_->state.header.stamp = time;
  action_result_->state.position[0] = current_position;
  action_result_->state.velocity[0] = current_velocity;
  action_result_->state.effort[0] =
      effort_state_interface_
          ? effort_state_interface_->get().get_optional().value_or(0.0)
          : 0.0;

  if (std::abs(active_goal_position_ - current_position) <=
      params_.goal_tolerance) {
    action_result_->reached_goal = true;
    action_result_->stalled = false;
    active->setSucceeded(action_result_);
    active_goal_.set([](RealtimeGoalHandlePtr &stored) { stored.reset(); });
    return;
  }
  if (std::abs(current_velocity) > params_.stall_velocity_threshold) {
    last_movement_time_ = time;
    return;
  }
  if ((time - last_movement_time_).seconds() <= params_.stall_timeout) {
    return;
  }

  action_result_->reached_goal = false;
  action_result_->stalled = true;
  if (params_.allow_stalling) {
    active->setSucceeded(action_result_);
  } else {
    active->setAborted(action_result_);
  }
  active_goal_.set([](RealtimeGoalHandlePtr &stored) { stored.reset(); });
}

} // namespace normalized_gripper_controller

PLUGINLIB_EXPORT_CLASS(
    normalized_gripper_controller::NormalizedGripperController,
    controller_interface::ChainableControllerInterface)
