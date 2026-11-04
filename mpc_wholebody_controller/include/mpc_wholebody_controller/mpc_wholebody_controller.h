// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Created for OCS2 Arm Controller
//

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "mpc_wholebody_controller/control/CtrlComponent.h"

#include <arms_controller_common/CtrlInterfaces.h>
#include "arms_ros2_control_msgs/srv/kinematics_service.hpp"
namespace zephyr::mpc_wholebody
{
/** Runs only OCS2. Solver work stays off the controller_manager update thread. */
class MpcExecution
{
public:
    explicit MpcExecution(std::shared_ptr<CtrlComponent> component, double controller_frequency);
    ~MpcExecution();
    void start();
    void stop();
    void pause();
    /** Starts from the current measured pose after activation or resume. */
    bool tick(const rclcpp::Time& time);
    bool consumeFailure() { return failed_.exchange(false, std::memory_order_acq_rel); }
private:
    enum class Job { Idle, Requested, Running };
    bool requestSolve();
    void workerLoop();
    std::shared_ptr<CtrlComponent> component_;
    std::thread worker_;
    std::atomic_bool running_{false};
    std::atomic_bool failed_{false};
    std::atomic<Job> job_{Job::Idle};
    bool initialized_{false}; // controller thread only
    double period_sec_{0.02};
    int sleep_ms_{2};
    rclcpp::Time last_request_time_{0, 0, RCL_ROS_TIME};
};

class TcpKinematics {
public:
    TcpKinematics(pinocchio::Model model,const std::vector<std::string>& joints,std::string tcp_frame);
    void handle(const arms_ros2_control_msgs::srv::KinematicsService::Request& request,
                arms_ros2_control_msgs::srv::KinematicsService::Response& response);
private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::string tcp_frame_;
    pinocchio::FrameIndex frame_;
    std::vector<int> indices_;
    std::mutex mutex_;
};

    using ControlMode = arms_controller_common::ControlMode;
    class mpc_wholebody_controller final : public controller_interface::ControllerInterface
    {
    public:
        mpc_wholebody_controller() = default;
        ~mpc_wholebody_controller() override = default;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        void publishTrackingStatus(bool active);
        void publishStatusMessage();
        void stopExecution();

        // Hardware parameters
        std::string command_prefix_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        // Control interfaces
        CtrlInterfaces ctrl_interfaces_;

        // Interface mapping
        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
        command_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"effort", &ctrl_interfaces_.joint_force_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_},
            {"effort", &ctrl_interfaces_.joint_force_state_interface_}
        };

        // Numeric status is retained solely for unchanged upstream visualization clients.
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tracking_status_publisher_;
        rclcpp::TimerBase::SharedPtr tracking_status_timer_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
        std::atomic_bool pause_requested_{false};
        std::atomic_bool tracking_active_{false};
        std::shared_ptr<CtrlComponent> ctrl_comp_;
        std::unique_ptr<MpcExecution> execution_;

        // Kinematics service
        rclcpp::Service<arms_ros2_control_msgs::srv::KinematicsService>::SharedPtr kinematics_service_;
        std::shared_ptr<TcpKinematics> kinematics_;

        // Service handler
        void handleKinematicsService(
            const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Request> request,
            const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Response> response);
    };
}
