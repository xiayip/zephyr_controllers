#include "battery_state_broadcaster/battery_state_broadcaster.hpp"

namespace battery_state_broadcaster
{
controller_interface::CallbackReturn BatteryStateBroadcaster::on_init()
{
  try {
    this->param_listener_ = std::make_shared<ParamListener>(this->get_node());
    this->params_ = this->param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_node()->get_logger(), "Exception thrown during init stage with message: %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State &)
{
  this->params_ = this->param_listener_->get_params();

  this->battery_state_ = std::make_unique<semantic_components::BatteryState>(
    semantic_components::BatteryState(this->params_.sensor_name, this->params_.state_interfaces));

  try {
    // register ft sensor data publisher
    this->sensor_state_publisher_ =
      get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
      "~/battery_state", rclcpp::SensorDataQoS());
    this->realtime_publisher_ = std::make_unique<StatePublisher>(this->sensor_state_publisher_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_node()->get_logger(),
      "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  this->realtime_publisher_->lock();

  this->realtime_publisher_->msg_.header.frame_id = this->params_.frame_id;
  this->realtime_publisher_->msg_.design_capacity = this->params_.design_capacity;
  this->realtime_publisher_->msg_.voltage = std::numeric_limits<float>::quiet_NaN();
  this->realtime_publisher_->msg_.temperature = std::numeric_limits<float>::quiet_NaN();
  this->realtime_publisher_->msg_.charge = std::numeric_limits<float>::quiet_NaN();
  this->realtime_publisher_->msg_.current = std::numeric_limits<float>::quiet_NaN();
  this->realtime_publisher_->msg_.capacity = std::numeric_limits<float>::quiet_NaN();
  this->realtime_publisher_->msg_.percentage = std::numeric_limits<float>::quiet_NaN();
  this->realtime_publisher_->msg_.power_supply_status = this->params_.power_supply_status;
  this->realtime_publisher_->msg_.power_supply_health = this->params_.power_supply_health;
  this->realtime_publisher_->msg_.power_supply_technology = this->params_.power_supply_technology;
  this->realtime_publisher_->msg_.present = this->params_.present;
  this->realtime_publisher_->msg_.location = this->params_.location;
  this->realtime_publisher_->msg_.serial_number = this->params_.serial_number;

  realtime_publisher_->unlock();

  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
BatteryStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = this->battery_state_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->battery_state_->assign_loaned_state_interfaces(this->state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BatteryStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  this->battery_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BatteryStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (this->realtime_publisher_ && this->realtime_publisher_->trylock()) {
    this->realtime_publisher_->msg_.header.set__stamp(time);
    this->battery_state_->get_value_as_message(this->realtime_publisher_->msg_);
    this->realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}
}  // namespace battery_state_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  battery_state_broadcaster::BatteryStateBroadcaster, controller_interface::ControllerInterface)
