#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "gravity_compensation_model/gravity_compensation_model.hpp"
#include "gravity_compensation_model/gravity_feedforward_shaper.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "mit_impedance_controller/mit_impedance_controller_parameters.hpp"

namespace mit_impedance_controller {

class MitImpedanceController final
    : public controller_interface::ChainableControllerInterface {
public:
  ~MitImpedanceController() override;

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
  struct RuntimeParameters {
    std::vector<double> stiffness;
    std::vector<double> damping;
    std::vector<double> gravity_scale;
    std::vector<double> gravity_bias;
  };

  struct ReferenceCommand {
    std::vector<double> positions;
    std::vector<double> velocities;
  };

  struct CommandIndices {
    std::size_t position;
    std::size_t velocity;
    std::size_t kp;
    std::size_t kd;
    std::size_t t_ff;
  };

  std::size_t find_command_interface(const std::string &full_name) const;
  std::size_t find_state_interface(const std::string &full_name) const;
  bool bind_interfaces();
  bool update_measured_state();
  bool update_gravity_factor(double period_seconds) noexcept;
  bool update_reference_targets(double period_seconds);
  bool compute_target_command(const RuntimeParameters &runtime_parameters);
  bool initialize_command(const RuntimeParameters &runtime_parameters);
  bool write_command(const RuntimeParameters &runtime_parameters);
  void publish_state(const rclcpp::Time &time, double period_seconds);

  using StatePublisher =
      realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>;

  std::shared_ptr<ParamListener> param_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      runtime_validation_callback_;
  Params params_;
  realtime_tools::RealtimeBuffer<RuntimeParameters> runtime_parameters_buffer_;
  realtime_tools::RealtimeBuffer<ReferenceCommand> subscriber_command_buffer_;

  std::vector<CommandIndices> command_indices_;
  std::vector<std::size_t> position_state_indices_;
  std::vector<std::size_t> velocity_state_indices_;
  std::vector<hardware_interface::CommandInterface::SharedPtr>
      reference_handles_;

  gravity_compensation_model::GravityCompensationModel gravity_model_;
  gravity_compensation_model::GravityFeedforwardShaper command_shaper_;
  Eigen::VectorXd measured_positions_;
  Eigen::VectorXd measured_velocities_;
  Eigen::VectorXd target_position_references_;
  Eigen::VectorXd active_position_references_;
  Eigen::VectorXd commanded_position_references_;
  Eigen::VectorXd velocity_references_;
  Eigen::VectorXd raw_gravity_torques_;
  Eigen::VectorXd scaled_gravity_torques_;
  Eigen::VectorXd stiffness_torques_;
  Eigen::VectorXd damping_torques_;
  Eigen::VectorXd soft_limit_torques_;
  Eigen::VectorXd target_feedforward_torques_;
  Eigen::VectorXd commanded_feedforward_torques_;

  bool gravity_compensation_enabled_{true};
  bool soft_limits_enabled_{true};
  std::atomic<double> target_gravity_factor_{1.0};
  double current_gravity_factor_{1.0};
  double gravity_factor_transition_duration_{1.0};
  double state_publish_period_{0.02};
  double state_publish_accumulator_{0.0};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr
      command_subscriber_;
  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr
      state_publisher_;
  std::unique_ptr<StatePublisher> realtime_state_publisher_;
};

} // namespace mit_impedance_controller
