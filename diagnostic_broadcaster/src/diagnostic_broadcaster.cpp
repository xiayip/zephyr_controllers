#include "diagnostic_broadcaster/diagnostic_broadcaster.hpp"

#include <limits>

namespace diagnostic_broadcaster
{
  controller_interface::CallbackReturn DiagnosticBroadcaster::on_init()
  {
    // Initialize parameter listener and load default parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    // Create diagnostic updater
    diagnostics_updater_ = std::make_shared<diagnostic_updater::Updater>(this->get_node());
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn DiagnosticBroadcaster::on_configure(
      const rclcpp_lifecycle::State &)
  {
    // Optionally reload parameters
    params_ = param_listener_->get_params();

    // Add a diagnostic task to check state interfaces
    const char* robot_name_env = std::getenv("ROBOT_NAME");
    std::string robot_name = robot_name_env ? robot_name_env : "";
    if (robot_name.empty())
    {
      RCLCPP_WARN(get_node()->get_logger(), "ROBOT_NAME environment variable is not set. Set to mh2 by default.");
      robot_name = "mh2";
    }
    const char* robot_id_env = std::getenv("ROS_DOMAIN_ID");
    std::string robot_id = robot_id_env ? robot_id_env : "";
    if (robot_id.empty())
    {
      RCLCPP_WARN(get_node()->get_logger(), "ROS_DOMAIN_ID environment variable is not set. Set to 000 by default.");
      robot_id = "000";
    }
    std::string hardware_id = robot_name + "_" + robot_id;
    diagnostics_updater_->setHardwareID("/" + hardware_id + "/hardware bridge");
    diagnostics_updater_->add("Motor status", std::bind(
                                                  &DiagnosticBroadcaster::update_motor_state, this, std::placeholders::_1));
    if (robot_name == "mh2") {
      diagnostics_updater_->add("USB device", std::bind(
        &DiagnosticBroadcaster::check_tty_usb_device, this, std::placeholders::_1));
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  DiagnosticBroadcaster::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration
  DiagnosticBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto &diagnostic : params_.hardware_diagnostic_interface)
    {
      state_interfaces_config.names.push_back(diagnostic);
    }
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn DiagnosticBroadcaster::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // set loaned interfaces
    MapOfReferencesToStateInterfaces map;
    for (const auto &interface_name : params_.hardware_diagnostic_interface)
    {
      auto interface = std::find_if(
          state_interfaces_.begin(), state_interfaces_.end(),
          [&](const auto &configured_interface)
          {
            const auto full_name_interface_name = configured_interface.get_name();
            return full_name_interface_name == interface_name;
          });
      if (interface != state_interfaces_.end())
      {
        map.emplace(interface_name, std::ref(*interface));
      }
    }

    hardware_diagnostic_interfaces_ = std::move(map);
    // update the diagnostic interfaces
    diagnostics_updater_->force_update();
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn DiagnosticBroadcaster::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type DiagnosticBroadcaster::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return controller_interface::return_type::OK;
  }

  void DiagnosticBroadcaster::update_motor_state(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    bool is_error_happened = false;
    for (const auto &i : hardware_diagnostic_interfaces_)
    {
      auto &interface = i.second;
      // convert error code to string
      const auto value = interface.get().get_optional().value_or(
        std::numeric_limits<double>::quiet_NaN());
      std::string error_code_str = std::to_string(value);
      const auto error_code = static_cast<float>(value);
      const auto error_code_it = error_code_map_f.find(error_code);
      if (error_code_it != error_code_map_f.end())
      {
        error_code_str = error_code_it->second;
      }
      stat.add(interface.get().get_name(), error_code_str);
      // RCLCPP_INFO(get_node()->get_logger(), "Checking interface: %s, value: %f", interface.get().get_name().c_str(), value);
      if (value != 0.0)
      {
        is_error_happened = true;
      }
    }

    if (!is_error_happened)
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Motor state OK");
    else
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Motor Error");
  }

  void DiagnosticBroadcaster::check_tty_usb_device(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    // check if /dev/ttyUSB_BODY is available
    if (access("/dev/ttyUSB_BODY", F_OK) == -1)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "USB device /dev/ttyUSB_BODY not found");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "USB device /dev/ttyUSB_BODY found");
    }
    // check if /dev/ttyUSB_MCU is available
    if (access("/dev/ttyUSB_MCU", F_OK) == -1)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "USB device /dev/ttyUSB_MCU not found");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "USB device /dev/ttyUSB_MCU found");
    }
    // check if /dev/ttyUSB_RAIDAR is available
    if (access("/dev/ttyUSB_RAIDAR", F_OK) == -1)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "USB device /dev/ttyUSB_RAIDAR not found");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "USB device /dev/ttyUSB_RAIDAR found");
    }
  }

} // namespace diagnostic_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    diagnostic_broadcaster::DiagnosticBroadcaster, controller_interface::ControllerInterface)
