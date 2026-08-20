#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "control_toolbox/low_pass_filter.hpp"
#include "controller_interface/controller_interface.hpp"
#include "gravity_compensation_model/gravity_compensation_model.hpp"
#include "gravity_compensation_model/gravity_feedforward_shaper.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "mit_damping_controller/mit_damping_controller_parameters.hpp"

namespace mit_damping_controller {

class MitDampingController final
    : public controller_interface::ControllerInterface {
public:
  ~MitDampingController() override;

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
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  struct RuntimeParameters {
    std::vector<double> damping_coefficients;
    std::vector<double> gravity_scale;
    std::vector<double> gravity_bias;
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
  bool reset_velocity_filters();
  bool update_measured_state();
  bool update_gravity_factor(double period_seconds) noexcept;
  bool update_command_transition(double period_seconds) noexcept;
  bool compute_target_torque(const RuntimeParameters &runtime_parameters);
  bool initialize_command(const RuntimeParameters &runtime_parameters);
  bool write_command();
  void publish_state(const rclcpp::Time &time, double period_seconds);

  using StatePublisher =
      realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>;

  std::shared_ptr<ParamListener> param_listener_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      runtime_validation_callback_;
  Params params_;
  realtime_tools::RealtimeBuffer<RuntimeParameters> runtime_parameters_buffer_;

  std::vector<CommandIndices> command_indices_;
  std::vector<std::size_t> position_state_indices_;
  std::vector<std::size_t> velocity_state_indices_;
  std::vector<control_toolbox::LowPassFilter<double>> velocity_filters_;

  gravity_compensation_model::GravityCompensationModel gravity_model_;
  gravity_compensation_model::GravityFeedforwardShaper command_shaper_;
  Eigen::VectorXd measured_positions_;
  Eigen::VectorXd measured_velocities_;
  Eigen::VectorXd filtered_velocities_;
  Eigen::VectorXd raw_gravity_torques_;
  Eigen::VectorXd scaled_gravity_torques_;
  Eigen::VectorXd damping_torques_;
  Eigen::VectorXd soft_limit_torques_;
  Eigen::VectorXd target_torques_;
  Eigen::VectorXd commanded_torques_;
  Eigen::VectorXd transition_positions_;
  Eigen::VectorXd transition_velocities_;
  Eigen::VectorXd transition_kp_;
  Eigen::VectorXd transition_kd_;

  bool gravity_compensation_enabled_{true};
  bool velocity_filter_enabled_{true};
  bool soft_limits_enabled_{true};
  std::atomic<double> target_gravity_factor_{1.0};
  double current_gravity_factor_{1.0};
  double gravity_factor_transition_duration_{1.0};
  double command_transition_duration_{1.0};
  double command_transition_progress_{1.0};
  double velocity_filter_cutoff_frequency_{10.0};
  double state_publish_period_{0.02};
  double state_publish_accumulator_{0.0};

  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr
      state_publisher_;
  std::unique_ptr<StatePublisher> realtime_state_publisher_;
};

} // namespace mit_damping_controller
