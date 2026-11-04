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

#include <ocs2_controller_common/rt/RtCycleTiming.h>
#include <mpc_wholebody_controller/visualization/mpc_visualizer.h>

#include <mpc_wholebody_controller/control/StagedMotion.h>
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

#include <arms_controller_common/CtrlInterfaces.h>
#include <array>
#include <pinocchio/multibody/data.hpp>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <arms_ros2_control_msgs/srv/execute_circle.hpp>
#include <mpc_wholebody_controller/srv/execute_tcp_path.hpp>

// TCP target interfaces and reference trajectories.
namespace zephyr::mpc_wholebody {
/** One real TCP, with no arm selector or ROS name translation. */
class TcpReferenceManager final : public ocs2::ReferenceManagerDecorator {
public:
    TcpReferenceManager(std::shared_ptr<ocs2::ReferenceManagerInterface> wrapped,
                        std::string base_frame, std::string tcp_frame, size_t input_dim);
    void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
    void setAcceptingTargets(bool accept);
    void setCurrentObservation(const ocs2::SystemObservation& observation);
    void setCurrentTcpPose(const ocs2::vector_t& pose, bool update = false);
    void resetTargetStateCache();
    void publishCurrentTargetsFromCache();
private:
    bool transform(const geometry_msgs::msg::PoseStamped& pose, ocs2::vector_t& result) const;
    bool transformPath(const nav_msgs::msg::Path& path, ocs2::vector_array_t& goals) const;
    void acceptPose(const geometry_msgs::msg::PoseStamped& pose, bool interpolate);
    void acceptRelative(const geometry_msgs::msg::TwistStamped& delta);
    void acceptPath(const nav_msgs::msg::Path& path);
    void plan(const std::vector<ocs2::vector_t>& poses, double duration);
    ocs2::vector_t sample(double time) const;
    void publish() const;
    void commitImmediate();
    void circle(const arms_ros2_control_msgs::srv::ExecuteCircle::Request& request,
                arms_ros2_control_msgs::srv::ExecuteCircle::Response& response);
    mutable std::mutex mutex_;
    std::atomic_bool accepting_{false};
    std::string base_frame_, tcp_frame_;
    size_t input_dim_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    ocs2::vector_t target_, current_;
    double time_{0.0}, last_time_{-1.0};
    ocs2::scalar_array_t times_;
    ocs2::vector_array_t poses_;
    geometry_msgs::msg::Twist twist_;
    std::chrono::steady_clock::time_point twist_received_{};
    bool twist_active_{false};
    double duration_{2.0}, interval_{0.04}, max_linear_{0.3}, max_angular_{1.0};
    double max_linear_acc_{1.0}, max_angular_acc_{2.0}, max_linear_jerk_{2.0}, max_angular_jerk_{4.0};
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr stamped_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr relative_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Service<arms_ros2_control_msgs::srv::ExecuteCircle>::SharedPtr circle_service_;
    rclcpp::Service<::mpc_wholebody_controller::srv::ExecuteTcpPath>::SharedPtr path_service_;
};
struct TcpTrajectorySample {
    double stamp_sec{0.0};
    std::array<double,3> position{};
    std::array<double,4> quat_xyzw{0,0,0,1};
};
class TcpTrajectoryRecorder {
public:
    ~TcpTrajectoryRecorder();
    void setEnabled(bool enabled,const std::string& directory);
    bool enabled() const { return enabled_.load(); }
    void appendActual(const TcpTrajectorySample& sample);
    void appendPredicted(const TcpTrajectorySample& sample);
private:
    std::atomic_bool enabled_{false};
    std::mutex mutex_;
    std::string directory_;
    std::vector<TcpTrajectorySample> actual_,predicted_;
};

    using namespace ocs2;
    using namespace ocs2::mobile_manipulator;
    // Use CtrlInterfaces from arms_controller_common
    using CtrlInterfaces = arms_controller_common::CtrlInterfaces;

    class CtrlComponent
    {
    public:
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces);
        ~CtrlComponent();

        /** Commit only complete, finite feedback. False means hold outputs this cycle. */
        bool updateObservation(const rclcpp::Time& time);
        void evaluatePolicy(const rclcpp::Time& time);
        /** Command the mobile base. The default zero command is used outside OCS2 mode. */
        void stopMobileBase() const;
        /** Reset observation/targets and MPC node. Does NOT block for the first policy (RT-safe). */
        void resetMpc();
        /** True once the first MPC policy after resetMpc() is available. */
        [[nodiscard]] bool initialPolicyReceived() const;
        void advanceMpc();
        /** MPC request period; used to warn only when a solve overruns that period. */
        void setMpcPeriodSec(double period_sec);
        /** Hold joints at last commanded positions (used while waiting for initial policy). */
        void holdLastSentPositions() const;
        /** Publish TCP current_target from cache (call on OCS2 enter for marker sync). */
        void publishCachedCurrentTargets() const;

        /**
         * Prediction, self-collision markers and recording on an independent periodic worker
         * (not RT update). FCL judgment runs on viz thread; RT only reads collision_active_.
         * Start on controller activate, stop on deactivate. Request updates via
         * maybeRequestVisualizationUpdate() from the common control path (the active lifetime).
         */
        void startVisualizationThread();
        /** Blocking stop: clear running + join (destructor / deactivate). */
        void stopVisualizationThread();
        void requestVisualizationUpdate();
        /** Supply the newest measured observation on every control cycle. */
        void maybeRequestVisualizationUpdate(const rclcpp::Time& time);

        void beginRtCycle() { rt_timing_.beginCycle(); }
        void endRtCycle(const std::string& status) { rt_timing_.endCycle(status, node_->get_logger()); }
        controller_common::RtCycleTiming::Scope rtScopeObs() { return rt_timing_.scope(&rt_timing_.obs_us); }
        controller_common::RtCycleTiming::Scope rtScopeViz() { return rt_timing_.scope(&rt_timing_.viz_us); }
        void applyWorkerThreadScheduling(const char* thread_name) const;
        /** Pin the 500 Hz controller_manager thread away from the MPC core (once). */
        void applyRtLoopSchedulingOnce();

        // Visualization management
        void clearTrajectoryVisualization();

        // Torque calculation for force control
        vector_t calculateStaticTorques() const;

        /**
         * RT-safe: viz thread already compared FCL min distance to configured threshold.
         * The controller only consumes this flag (no threshold re-judgment).
         */
        bool isSelfCollisionActive() const
        {
            return collision_active_.load(std::memory_order_acquire);
        }

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
        void receiveBaseOdometry(const nav_msgs::msg::Odometry& odometry);
        void publishMobileBaseCommand(const geometry_msgs::msg::Twist& command) const;
        void applyBaseCommandDeadband(geometry_msgs::msg::Twist& command);

        /** Requires launch-injected xacro planning URDF (planning_urdf_path). */
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

        struct StagedPolicy
        {
            StagedMotionState stage;
            uint64_t epoch{0};
        };
        std::shared_ptr<StagedReferenceManager> staged_reference_;
        std::shared_ptr<const StagedPolicy> staged_policy_; // atomic solver -> RT handoff
        MotionPhase applied_motion_phase_{MotionPhase::Arm}; // RT only
        vector_t held_arm_positions_; // Held during base approach, settling, and unreachable-height phases.
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motion_phase_publisher_;

        // Visualization component
        std::unique_ptr<zephyr::mpc_visualization::mpc_visualizer> visualizer_;

        // Independent visualization worker; full solver export never runs on the RT path.
        struct VisualizationSnapshot
        {
            SystemObservation observation;
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
        bool prediction_visualization_enabled_{true}; // initialization only
        double prediction_ghost_interval_{0.2};
        int prediction_ghost_count_{5};
        double prediction_max_age_{0.5};
        double visualization_period_sec_{0.05};
        SystemObservation measured_observation_; // RT only, never overwritten by command caching.
        std::shared_ptr<const zephyr::mpc_visualization::TimedPrediction> solver_prediction_; // atomic worker -> RT
        std::shared_ptr<const zephyr::mpc_visualization::TimedPrediction> active_prediction_; // RT only
        double last_recorded_observation_time_{-1.0}; // viz worker only
        /** Viz-thread judgment (valid FCL && min_dist <= configured thr); RT only loads. */
        std::atomic_bool collision_active_{false};
        /** Published via atomic_store/load only (RT → viz handoff, no mutex). */
        std::shared_ptr<VisualizationSnapshot> visualization_snapshot_;
        /** RT-only staging for pred samples; folded into snapshot in requestVisualizationUpdate(). */
        bool pending_viz_has_pred_{false};
        double pending_viz_pred_time_{0.0};
        vector_t pending_viz_pred_state_;

        // Configuration
        std::string robot_name_;
        std::string planning_urdf_variant_; // must be "xacro"
        std::string planning_urdf_path_; // xacro-generated URDF cache path
        std::vector<std::string> joint_names_;
        bool mobile_base_enabled_{false};
        std::size_t base_state_offset_{0};
        double future_time_offset_; // Number of control cycles ahead to sample.
        /// cached MPC observation state (which is different from real observation on purpose)
        bool cached_ob_state_;
        rclcpp::Time last_execute_time_;
        vector_t cached_last_action_;
        std::atomic<double> hardware_latency_{0.2};
        std::string traj_record_dir_{"/tmp/traj_record"};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_parameter_callback_handle_;
        std::unordered_map<std::string, rclcpp::ParameterValue> initialization_parameters_;
        void freezeInitializationParameters();
        void applyParameterChanges(const std::vector<rclcpp::Parameter>& parameters);
        rcl_interfaces::msg::SetParametersResult on_parameter_change(
            const std::vector<rclcpp::Parameter>& parameters);

        controller_common::RtCycleTiming rt_timing_;
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
