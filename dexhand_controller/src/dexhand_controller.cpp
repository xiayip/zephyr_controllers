#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "dexhand_controller/dexhand_controller.hpp"

namespace dexhand_controller
{
  DexHandController::DexHandController()
      : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn DexHandController::on_init()
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

  controller_interface::CallbackReturn DexHandController::on_configure(
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

    dexhand_command_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        "~/dexhand_command", rclcpp::SystemDefaultsQoS(),
        std::bind(&DexHandController::topic_callback, this, std::placeholders::_1));

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  DexHandController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : params_.joints)
    {
      command_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  DexHandController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &joint : params_.joints)
    {
      state_interfaces_config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn DexHandController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    position_command_interface_.clear();
    std::copy_if(command_interfaces_.begin(), command_interfaces_.end(), std::back_inserter(position_command_interface_),
                 [](const hardware_interface::LoanedCommandInterface &command_interface)
                 { return command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
    if (position_command_interface_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "No position command interface found");
      return controller_interface::CallbackReturn::ERROR;
    }

    joint_position_state_interface_.clear();
    std::copy_if(state_interfaces_.begin(), state_interfaces_.end(), std::back_inserter(joint_position_state_interface_),
                 [](const hardware_interface::LoanedStateInterface &state_interface)
                 { return state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION; });
    if (joint_position_state_interface_.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "No position state interface found");
      return controller_interface::CallbackReturn::ERROR;
    }

    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>(nullptr);

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn DexHandController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    position_command_interface_.clear();
    joint_position_state_interface_.clear();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type DexHandController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    auto joint_commands = rt_command_ptr_.readFromRT();
    if (!joint_commands || !(*joint_commands))
    {
      return controller_interface::return_type::OK;
    }
    for (size_t i = 0; i < (*joint_commands)->name.size(); ++i) {
      // assign by name
      for (auto &command_interface : position_command_interface_)
      {
        if (command_interface.get().get_name() == (*joint_commands)->name[i])
        {
          if (!command_interface.get().set_value((*joint_commands)->position[i]))
          {
            continue;
          }
        }
      }
    }
    return controller_interface::return_type::OK;
  }

  void DexHandController::topic_callback(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
  {
    set_dexhand_goal(msg);
  }

  void DexHandController::set_dexhand_goal(const std::shared_ptr<sensor_msgs::msg::JointState> msg)
  {
    // check the validity of the message
    if (msg->name.size() != msg->position.size() ||
        msg->name.size() != params_.joints.size() ||
        msg->name.size() == 0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid JointState message");
      return;
    }
    // stash the goal
    rt_command_ptr_.writeFromNonRT(msg);

    // Set the goal for the dexterous hand
  }

} // namespace dexhand_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dexhand_controller::DexHandController, controller_interface::ControllerInterface)
