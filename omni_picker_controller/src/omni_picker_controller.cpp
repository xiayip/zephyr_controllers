#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "omni_picker_controller/omni_picker_controller.hpp"

namespace omni_picker_controller
{
  OmniPickerController::OmniPickerController()
      : controller_interface::ChainableControllerInterface()
  {
  }

  controller_interface::CallbackReturn OmniPickerController::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OmniPickerController::on_configure(
      const rclcpp_lifecycle::State &)
  {
    const auto logger = get_node()->get_logger();
    if (!param_listener_)
    {
      RCLCPP_ERROR(logger, "Error encountered during init");
      return controller_interface::CallbackReturn::ERROR;
    }
    params_ = param_listener_->get_params();

    // Controlled joint
    if (params_.joints.size() != 1)
    {
      RCLCPP_ERROR(logger, "Joint size needs to equal 1");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.topic_prefix.empty())
    {
      RCLCPP_ERROR(logger, "Topic prefix is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    gripper_command_subscriber_ =
        get_node()->create_subscription<sensor_msgs::msg::JointState>(
            params_.topic_prefix + "/gripper_command", rclcpp::SystemDefaultsQoS(),
            std::bind(&OmniPickerController::topic_callback, this, std::placeholders::_1));

    gripper_joint_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        params_.topic_prefix + "/joint_states", rclcpp::SystemDefaultsQoS());
    realtime_gripper_joint_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(
            gripper_joint_publisher_);
    auto &msg = realtime_gripper_joint_publisher_->msg_;
    msg.name.resize(1);
    msg.position.resize(1);
    msg.velocity.resize(1);
    msg.effort.resize(1);
    msg.name = params_.joints;

    reference_interfaces_.resize(3, std::numeric_limits<double>::quiet_NaN());
    // debug
    // for (const auto &joint : params_.joints)
    // {
    //   RCLCPP_INFO(logger, "Joint name: %s", joint.c_str());
    // }
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  OmniPickerController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : params_.joints)
    {
      command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
      command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
      // command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_ACCELERATION);
    }
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  OmniPickerController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : params_.joints)
    {
      state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
      state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
      state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn OmniPickerController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // bind command interfaces
    auto position_command_interface_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [](const hardware_interface::LoanedCommandInterface &command_interface)
        { return command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
    auto velocity_command_interface_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [](const hardware_interface::LoanedCommandInterface &command_interface)
        { return command_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY; });
    auto effort_command_interface_it = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [](const hardware_interface::LoanedCommandInterface &command_interface)
        { return command_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT; });
    // TODO: bind acceleration command interface
    position_command_interface_ = *position_command_interface_it;
    velocity_command_interface_ = *velocity_command_interface_it;
    effort_command_interface_ = *effort_command_interface_it;
    // bind state interfaces
    const auto position_state_interface_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [](const hardware_interface::LoanedStateInterface &state_interface)
        { return state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
    const auto velocity_state_interface_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [](const hardware_interface::LoanedStateInterface &state_interface)
        { return state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY; });
    const auto effort_state_interface_it = std::find_if(
        state_interfaces_.begin(), state_interfaces_.end(),
        [](const hardware_interface::LoanedStateInterface &state_interface)
        { return state_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT; });
    joint_position_state_interface_ = *position_state_interface_it;
    joint_velocity_state_interface_ = *velocity_state_interface_it;
    joint_effort_state_interface_ = *effort_state_interface_it;

    // Command - non RT version
    command_from_subscriber_.position_cmd_ = joint_position_state_interface_->get().get_value();
    command_from_subscriber_.velocity_cmd_ = params_.max_effort;
    command_from_subscriber_.effort_cmd_ = params_.max_velocity;
    command_.initRT(command_from_subscriber_);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OmniPickerController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    position_command_interface_ = std::nullopt;
    velocity_command_interface_ = std::nullopt;
    effort_command_interface_ = std::nullopt;
    joint_position_state_interface_ = std::nullopt;
    joint_velocity_state_interface_ = std::nullopt;
    joint_effort_state_interface_ = std::nullopt;
    return CallbackReturn::SUCCESS;
  }

  bool OmniPickerController::on_set_chained_mode(bool chained_mode)
  {
    (void)chained_mode;
    // TODO: disable service and action server when chained mode is enabled
    return true;
  }

  controller_interface::return_type OmniPickerController::update_and_write_commands(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    (void)period;

    double current_position = joint_position_state_interface_->get().get_value();
    double current_velocity = joint_velocity_state_interface_->get().get_value();
    double current_effort = joint_effort_state_interface_->get().get_value();
    // RCLCPP_INFO(get_node()->get_logger(), "currnet position: %f, velocity: %f, effort: %f", current_position, current_velocity, current_effort);

    final_command_.position_cmd_ = reference_interfaces_[0];
    final_command_.velocity_cmd_ = reference_interfaces_[1];
    final_command_.effort_cmd_ = reference_interfaces_[2];
    // RCLCPP_INFO(get_node()->get_logger(), "final position: %f, velocity: %f, effort: %f", final_command_.position_cmd_, final_command_.velocity_cmd_, final_command_.effort_cmd_);

    position_command_interface_->get().set_value(final_command_.position_cmd_);
    if (!std::isnan(final_command_.velocity_cmd_))
    {
      velocity_command_interface_->get().set_value(final_command_.velocity_cmd_);
    }
    if (!std::isnan(final_command_.effort_cmd_))
    {
      effort_command_interface_->get().set_value(final_command_.effort_cmd_);
    }
    // publish realtime gripper joint state
    if (realtime_gripper_joint_publisher_->trylock())
    {
      auto &msg = realtime_gripper_joint_publisher_->msg_;
      msg.header.stamp = time;
      msg.position[0] = current_position; // position from 0 to 1
      msg.velocity[0] = current_velocity;
      msg.effort[0] = current_effort;
      realtime_gripper_joint_publisher_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }

  std::vector<hardware_interface::CommandInterface> OmniPickerController::on_export_reference_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> reference_interfaces;

    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), params_.joints[0] + "/" + hardware_interface::HW_IF_POSITION, &reference_interfaces_[0]));
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), params_.joints[0] + "/" + hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[1]));
    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), params_.joints[0] + "/" + hardware_interface::HW_IF_EFFORT, &reference_interfaces_[2]));
    return reference_interfaces;
  }

  // Is this function necessary?
  controller_interface::return_type OmniPickerController::update_reference_from_subscribers()
  {
    reference_interfaces_[0] = (*(command_.readFromRT())).position_cmd_;
    reference_interfaces_[1] = (*(command_.readFromRT())).velocity_cmd_;
    reference_interfaces_[2] = (*(command_.readFromRT())).effort_cmd_;
    return controller_interface::return_type::OK;
  }

  void OmniPickerController::topic_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
  {
    set_new_gripper_goal(*msg);
  };

  void OmniPickerController::set_new_gripper_goal(const sensor_msgs::msg::JointState &gripper_goal_msg)
  {
    if (gripper_goal_msg.position[0] < 0.0 || gripper_goal_msg.position[0] > 1.0)
    {
      // RCLCPP_ERROR(get_node()->get_logger(), "Gripper goal position out of range, must be between 0 and 1");
      return;
    }
    command_from_subscriber_.position_cmd_ = gripper_goal_msg.position[0];
    command_from_subscriber_.velocity_cmd_ = (gripper_goal_msg.velocity.empty() || std::abs(gripper_goal_msg.velocity[0]) < 1e5) ? params_.max_velocity : gripper_goal_msg.velocity[0];
    command_from_subscriber_.effort_cmd_ = (gripper_goal_msg.effort.empty() || std::abs(gripper_goal_msg.effort[0]) < 1e5) ? params_.max_effort : gripper_goal_msg.effort[0];
    command_.writeFromNonRT(command_from_subscriber_);
  }
} // namespace omni_picker_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    omni_picker_controller::OmniPickerController, controller_interface::ChainableControllerInterface)
