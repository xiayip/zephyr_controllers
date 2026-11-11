// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Created for OCS2 Arm Controller - CtrlComponent
//
#pragma once

#include <cstdint>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <mpc_controller/runtime/RtCycleTiming.h>
#include <mpc_controller/visualization/mpc_visualizer.h>

#include <mpc_controller/control/ControlSafety.h>
#include <mpc_controller/control/TcpReferenceManager.h>
#include <mpc_controller/control/TcpTrajectoryRecorder.h>
#include <ocs2_core/Types.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <mpc_controller/control/JointInterfaces.h>
#include <pinocchio/multibody/data.hpp>

#include <ocs2_mpc/SystemObservation.h>

namespace zephyr::mpc {

    using namespace ocs2;
    using namespace ocs2::mobile_manipulator;
    // Local ros2_control interface storage
    using CtrlInterfaces = JointInterfaces;

    class CtrlComponent
    {
    public:
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces);
        ~CtrlComponent();

        /** Commit only complete, finite feedback. False means hold outputs this cycle. */
        bool updateObservation(const rclcpp::Time& time);
        const std::array<double, 7>& measuredTcpPose() const { return measured_tcp_pose_; }
        /** False is a latched execution fault; outputs are already held. */
        bool evaluatePolicy(const rclcpp::Time& time);
        /** Command the mobile base. The default zero command is used outside OCS2 mode. */
        bool stopMobileBase() const;
        /** Reset only after the solver is idle and the reference reset is acknowledged.
         * Does not wait for the first policy; this is not a hard-RT guarantee.
         */
        void resetMpc();
        bool requestReferenceReset();
        /** True once the first MPC policy after resetMpc() is available. */
        [[nodiscard]] bool initialPolicyReceived() const;
        void advanceMpc();
        /** MPC request period; used to warn only when a solve overruns that period. */
        void setMpcPeriodSec(double period_sec);
        /** Hold joints at last commanded positions (used while waiting for initial policy). */
        bool holdLastSentPositions() const;
        /** Lifecycle only, after update/solver output has stopped. Drains the base
         * publisher with a bounded wait and confirms the final zero publication. */
        bool holdForDeactivation() const;

        /**
         * Prediction, self-collision markers and recording on an independent periodic worker
         * (not RT update). RT consumes a timestamped collision result, not a display flag.
         * Start on controller activate, stop on deactivate. Request updates via
         * maybeRequestVisualizationUpdate() from the common control path (the active lifetime).
         */
        void startVisualizationThread();
        /** Blocking stop: clear running + join (destructor / deactivate). */
        void stopVisualizationThread();
        void requestVisualizationUpdate();
        /** Supply the newest measured observation on every control cycle. */
        void maybeRequestVisualizationUpdate();

        void beginRtCycle() { rt_timing_.beginCycle(); }
        void endRtCycle(const std::string& status) { rt_timing_.endCycle(status, node_->get_logger()); }
        runtime::RtCycleTiming::Scope rtScopeObs() { return rt_timing_.scope(&rt_timing_.obs_us); }
        runtime::RtCycleTiming::Scope rtScopeViz() { return rt_timing_.scope(&rt_timing_.viz_us); }
        void applyWorkerThreadScheduling(const char* thread_name) const;
        /** Pin the 500 Hz controller_manager thread away from the MPC core (once). */
        void applyRtLoopSchedulingOnce();

        // Visualization management
        void clearTrajectoryVisualization();

        // Torque calculation for force control
        vector_t calculateStaticTorques() const;

        CollisionStatus collisionStatus() const;

        // Get node reference
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> getNode() const { return node_; }

        // OCS2 interface (public access)
        std::shared_ptr<MobileManipulatorInterface> interface_;

        // MPC components
        std::unique_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        /** True after first successful updatePolicy() swap; cleared in resetMpc(). */
        bool policy_active_{false};
        std::shared_ptr<TcpReferenceManager> pose_reference_manager_;
        TcpTrajectoryRecorder recorder_;

        // Observation state
        SystemObservation observation_;

    private:
        bool holdJointPositions() const;
        void receiveBaseOdometry(const nav_msgs::msg::Odometry& odometry);
        bool publishMobileBaseCommand(const geometry_msgs::msg::Twist& command) const;
        void applyBaseCommandDeadband(geometry_msgs::msg::Twist& command);

        /** Requires a launch-injected, expanded planning URDF (planning_urdf_path). */
        std::string resolvePlanningUrdfPath() const;
        /** Split isolcpus: RT / MPC / DDP+viz, and pin DDP ThreadPool before GaussNewtonDDP_MPC construction. */
        void prepareThreadIsolation();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        CtrlInterfaces& ctrl_interfaces_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_command_publisher_;
        std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>
            realtime_base_command_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr base_odometry_subscription_;

        struct BaseOdometryState
        {
            double x{0.0};
            double y{0.0};
            double yaw{0.0};
            bool valid{false};
            std::chrono::steady_clock::time_point received_at{};
        };
        realtime_tools::RealtimeBuffer<BaseOdometryState> base_odometry_buffer_;
        std::string base_odom_frame_;
        std::atomic_bool base_odometry_received_{false};
        vector_t observation_candidate_;
        bool observation_valid_{false}; // RT thread only
        double base_linear_start_threshold_{0.03};
        double base_linear_stop_threshold_{0.015};
        double base_angular_start_threshold_{0.04};
        double base_angular_stop_threshold_{0.02};
        bool base_translation_active_{false};
        bool base_rotation_active_{false};
        std::string base_command_profile_{"omni"};
        double base_max_linear_speed_{0.0};
        double base_max_angular_speed_{0.0};

        // Visualization component
        std::unique_ptr<zephyr::mpc_visualization::mpc_visualizer> visualizer_;

        // Independent visualization worker; full solver export never runs on the RT path.
        struct VisualizationSnapshot
        {
            SystemObservation observation;
            int64_t observed_at_us{0};
            /** Command-overlay q for FCL + collision markers (obs base + last_sent joints). */
            vector_t collision_state;
            bool has_pred{false};
            double pred_time{0.0};
            vector_t pred_state;
            std::shared_ptr<const zephyr::mpc_visualization::TimedPrediction> prediction;
            uint64_t epoch{0};
        };

        void visualizationThreadLoop();
        void runVisualizationOnce(const VisualizationSnapshot& snap);
        vector_t buildCollisionState() const;
        [[nodiscard]] bool shouldPublishDds(const rclcpp::Time& time);

        std::thread visualization_thread_;
        std::atomic_bool visualization_running_{false};
        std::condition_variable visualization_wakeup_;
        std::mutex visualization_wait_mutex_;
        std::atomic_bool trajectory_clear_requested_{false};
        std::atomic<uint64_t> visualization_epoch_{0};
        bool mock_debug_enabled_{false}; // Initialization-only opt-in from mock launch/config.
        bool prediction_visualization_enabled_{false}; // initialization only
        double prediction_ghost_interval_{0.2};
        int prediction_ghost_count_{5};
        double prediction_max_age_{0.5};
        double visualization_period_sec_{0.05};
        SystemObservation measured_observation_; // RT only, never overwritten by command caching.
        std::array<double, 7> measured_tcp_pose_{0, 0, 0, 0, 0, 0, 1};
        std::shared_ptr<const zephyr::mpc_visualization::TimedPrediction> solver_prediction_; // atomic worker -> RT
        std::shared_ptr<const zephyr::mpc_visualization::TimedPrediction> active_prediction_; // RT only
        double last_recorded_observation_time_{-1.0}; // viz worker only
        CollisionWatchdog collision_watchdog_;
        int64_t collision_started_us_{0};
        double collision_check_timeout_{0.25};
        /** Published via atomic_store/load only (RT → viz handoff, no mutex). */
        std::shared_ptr<VisualizationSnapshot> visualization_snapshot_;
        /** RT-only staging for pred samples; folded into snapshot in requestVisualizationUpdate(). */
        bool pending_viz_has_pred_{false};
        double pending_viz_pred_time_{0.0};
        vector_t pending_viz_pred_state_;

        // Configuration
        std::string robot_name_;
        std::string planning_urdf_path_; // Expanded URDF file path
        std::vector<std::string> joint_names_;
        std::vector<int> arm_torque_indices_;
        vector_t arm_torques_;
        vector_t reset_tcp_pose_;
        bool mobile_base_enabled_{false};
        std::size_t base_state_offset_{0};
        double future_time_offset_; // Number of control cycles ahead to sample.
        double policy_max_age_{0.5};
        bool policy_valid_{false};
        std::chrono::steady_clock::time_point policy_received_at_{};
        std::string traj_record_dir_{"/tmp/traj_record"};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_parameter_callback_handle_;
        std::unordered_map<std::string, rclcpp::ParameterValue> initialization_parameters_;
        void freezeInitializationParameters();
        void applyParameterChanges(const std::vector<rclcpp::Parameter>& parameters);
        rcl_interfaces::msg::SetParametersResult on_parameter_change(
            const std::vector<rclcpp::Parameter>& parameters);

        runtime::RtCycleTiming rt_timing_;
        int thread_priority_{0};
        std::vector<int> cpu_affinity_;
        std::vector<int> rt_loop_cpus_;
        std::vector<int> mpc_thread_cpus_;
        bool rt_loop_scheduling_applied_{false};
        std::atomic<int64_t> mpc_period_us_{10000};
        double dds_publish_period_sec_{0.02};
        rclcpp::Time last_dds_publish_time_{0, 0, RCL_ROS_TIME};
        mutable std::optional<pinocchio::Data> rnea_data_;
    };
}
