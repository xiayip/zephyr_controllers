#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mit_controller/mit_controller_parameters.hpp"

namespace mit_controller {

class MitController final
    : public controller_interface::ChainableControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update_reference_from_subscribers(const rclcpp::Time &time,
                                    const rclcpp::Duration &period) override;
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override;
  bool on_set_chained_mode(bool chained_mode) override;

private:
  struct CommandIndices {
    std::size_t position;
    std::size_t velocity;
    std::size_t kp;
    std::size_t kd;
    std::size_t t_ff;
  };

  struct Command {
    double position;
    double velocity;
    double kp;
    double kd;
    double t_ff;
  };

  std::size_t find_command_interface(const std::string &full_name) const;
  std::size_t find_state_interface(const std::string &full_name) const;
  bool bind_interfaces();
  bool initialize_hold();
  bool capture_hold(std::size_t joint_index, const Command &command);
  bool enter_hold();
  bool write_hold();
  bool write_joint_command(std::size_t joint_index, double position,
                           double velocity, double kp, double kd, double t_ff);
  void set_reference(std::size_t joint_index, double position, double velocity,
                     double kp, double kd, double t_ff);

  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::vector<CommandIndices> command_indices_;
  std::vector<std::size_t> position_state_indices_;
  std::vector<std::size_t> velocity_state_indices_;
  std::vector<double> hold_positions_;
  std::vector<double> hold_kp_;
  std::vector<double> hold_kd_;
  std::vector<double> hold_t_ff_;
  std::vector<double> hold_measured_velocities_;
  std::vector<Command> last_commands_;
  bool holding_{false};
  std::vector<hardware_interface::CommandInterface::SharedPtr>
      reference_handles_;
};

} // namespace mit_controller
