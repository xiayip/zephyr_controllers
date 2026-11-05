// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Created for OCS2 Arm Controller - CtrlComponent
//

#include "mpc_controller/control/CtrlComponent.h"

#include <mpc_controller/runtime/ThreadScheduling.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <exception>
#include <cstdlib>
#include <type_traits>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <chrono>
#include <optional>

#include <Eigen/Geometry>
#include <stdexcept>

namespace zephyr::mpc
{
static int64_t steadyMicroseconds()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Adapt an omni MPC command to the chassis backend after applying deadbands.
// Positive limits bound speed; zero leaves that axis unbounded.
static void constrainBaseCommand(geometry_msgs::msg::Twist& command,
                                 bool ranger, double max_linear, double max_angular)
{
    if (ranger)
    {
        // Legacy Ranger backend profile: crab translation and yaw are exclusive.
        // A full swerve/holonomic backend must use the omni profile instead.
        if (std::abs(command.linear.y) > 0.001)
        {
            command.angular.z = 0.0;
        }
        else
        {
            command.linear.y = 0.0;
        }
    }
    const double speed = std::hypot(command.linear.x, command.linear.y);
    if (max_linear > 0.0 && speed > max_linear)
    {
        const double scale = max_linear / speed;
        command.linear.x *= scale;
        command.linear.y *= scale;
    }
    if (max_angular > 0.0)
    {
        command.angular.z = std::clamp(command.angular.z, -max_angular, max_angular);
    }
}

    using namespace ocs2;
    using namespace ocs2::mobile_manipulator;
    /** Writable directory for OCS2 CppAD generated libraries (must not live under read-only share/). */
    static std::string defaultOcs2LibraryFolder(const std::string& robot_pkg)
    {
        if (const char* xdg = std::getenv("XDG_CACHE_HOME"); xdg && xdg[0] != '\0')
        {
            return std::string(xdg) + "/ocs2_ros2/" + robot_pkg;
        }
        if (const char* home = std::getenv("HOME"); home && home[0] != '\0')
        {
            return std::string(home) + "/.ros/ocs2_cache/" + robot_pkg;
        }
        return std::string("/tmp/ocs2_ros2/") + robot_pkg;
    }

    static FrameOverrides loadInfoFrameDefaults(const std::string& task_file)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(task_file, pt);
        FrameOverrides frames;
        loadData::loadPtreeValue<std::string>(pt, frames.baseFrame, "model_information.baseFrame", false);
        loadData::loadPtreeValue<std::string>(pt, frames.eeFrame, "model_information.eeFrame", false);
        return frames;
    }

    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces)
        : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        // Match ControllerInterface::auto_declare, including predeclared overrides.
        const auto auto_declare = [this](const std::string& name, const auto& value) {
            using Value = std::decay_t<decltype(value)>;
            return node_->has_parameter(name) ? node_->get_parameter(name).get_value<Value>()
                                             : node_->declare_parameter<Value>(name, value);
        };
        // Keep explicit migration errors for old YAML, never silently run the
        // former command-as-observation substitution or invent a delay model.
        if (auto_declare("cached_ob_state", false) || auto_declare("hardware_latency", 0.0) != 0.0)
        {
            throw std::runtime_error("Command observation caching was removed: set cached_ob_state=false and hardware_latency=0.0");
        }
        policy_max_age_ = auto_declare("policy_max_age", 0.5);
        collision_check_timeout_ = auto_declare("collision_check_timeout", 0.25);
        if (!std::isfinite(policy_max_age_) || policy_max_age_ <= 0.0 ||
            !std::isfinite(collision_check_timeout_) || collision_check_timeout_ <= 0.0)
            throw std::runtime_error("Policy and collision timeouts must be finite and positive");
        // Production defaults must not allocate prediction meshes or record CSV.
        // Only hardware-checked mock launch profiles opt into these diagnostics.
        mock_debug_enabled_ = auto_declare("mock_debug_enabled", false);
        prediction_visualization_enabled_ = auto_declare("prediction_visualization_enabled", false);
        // Declare dir before enabled so both can be applied together from yaml/callback.
        traj_record_dir_ = auto_declare("traj_record_dir", std::string("/tmp/traj_record"));
        const bool traj_record_enabled = auto_declare("traj_record_enabled", false);
        if (!mock_debug_enabled_ && (prediction_visualization_enabled_ || traj_record_enabled))
        {
            throw std::runtime_error("Prediction visualization and trajectory recording are mock-only; keep them disabled outside the mock launch/profile");
        }
        if (traj_record_dir_.empty())
        {
            throw std::runtime_error("traj_record_dir must be nonempty");
        }
        recorder_.setEnabled(traj_record_enabled, traj_record_dir_);

        rt_timing_.enabled.store(auto_declare("rt_timing_enabled", false), std::memory_order_relaxed);
        thread_priority_ = auto_declare("thread_priority", 0);
        {
            const auto affinity_i64 = auto_declare("cpu_affinity", std::vector<int64_t>{});
            cpu_affinity_.clear();
            cpu_affinity_.reserve(affinity_i64.size());
            for (const auto cpu : affinity_i64)
            {
                cpu_affinity_.push_back(static_cast<int>(cpu));
            }
        }
        const double dds_publish_hz = auto_declare("dds_publish_hz", 50.0);
        if (!std::isfinite(dds_publish_hz) || dds_publish_hz < 0.0)
        {
            throw std::runtime_error("dds_publish_hz must be finite and nonnegative");
        }
        dds_publish_period_sec_ = dds_publish_hz > 0.0 ? 1.0 / dds_publish_hz : 0.0;
        const double prediction_publish_hz = auto_declare("prediction_publish_hz", 20.0);
        prediction_ghost_interval_ = auto_declare("prediction_ghost_interval", 0.2);
        prediction_ghost_count_ = auto_declare("prediction_ghost_count", 5);
        prediction_max_age_ = auto_declare("prediction_max_age", 0.5);
        if (!std::isfinite(prediction_publish_hz) || prediction_publish_hz < 1.0 || prediction_publish_hz > 60.0 ||
            !std::isfinite(prediction_ghost_interval_) || prediction_ghost_interval_ <= 0.0 ||
            prediction_ghost_count_ < 1 ||
            !std::isfinite(prediction_max_age_) || prediction_max_age_ <= 0.0)
        {
            throw std::runtime_error("Prediction rate must be 1..60 Hz; interval, count and max age must be positive");
        }
        visualization_period_sec_ = 1.0 / prediction_publish_hz;

        robot_name_ = auto_declare("robot_name", std::string("cr5"));
        mobile_base_enabled_ = auto_declare("mobile_base_enabled", false);
        StagedMotionConfig staged_config;
        staged_config.enabled = auto_declare("staged_motion_enabled", false);
        staged_config.arm_base_frame = auto_declare("arm_workspace_frame", std::string("base_link"));
        staged_config.reach_radius = auto_declare("arm_reach_radius", 0.0);
        staged_config.reach_switch_margin = auto_declare("reach_switch_margin", 0.02);
        staged_config.parking_radius_ratio = auto_declare("base_parking_radius_ratio", 0.65);
        staged_config.parking_tolerance = auto_declare("base_parking_tolerance", 0.025);
        staged_config.heading_tolerance = auto_declare("base_heading_tolerance", 0.08);
        staged_config.approach_keep_heading = auto_declare("base_approach_keep_heading", false);
        staged_config.tracking_stall_seconds = auto_declare("tracking_stall_seconds", 2.0);
        staged_config.tracking_position_tolerance = auto_declare("tracking_position_tolerance", 0.02);
        staged_config.tracking_orientation_tolerance = auto_declare("tracking_orientation_tolerance", 0.12);
        staged_config.tracking_progress_threshold = auto_declare("tracking_progress_threshold", 0.003);
        staged_config.assist_no_progress_seconds = auto_declare("assist_no_progress_seconds", 4.0);
        staged_config.settle_seconds = auto_declare("base_settle_seconds", 0.25);
        if ((staged_config.enabled && !mobile_base_enabled_) ||
            !std::isfinite(staged_config.reach_radius) || staged_config.reach_radius < 0.0 ||
            !std::isfinite(staged_config.reach_switch_margin) || staged_config.reach_switch_margin < 0.0 ||
            !std::isfinite(staged_config.parking_radius_ratio) || staged_config.parking_radius_ratio <= 0.0 || staged_config.parking_radius_ratio >= 1.0 ||
            !std::isfinite(staged_config.parking_tolerance) || staged_config.parking_tolerance <= 0.0 ||
            !std::isfinite(staged_config.heading_tolerance) || staged_config.heading_tolerance <= 0.0 || staged_config.heading_tolerance >= std::acos(-1.0) ||
            !std::isfinite(staged_config.tracking_stall_seconds) || staged_config.tracking_stall_seconds <= 0.0 ||
            !std::isfinite(staged_config.tracking_position_tolerance) || staged_config.tracking_position_tolerance <= 0.0 ||
            !std::isfinite(staged_config.tracking_orientation_tolerance) || staged_config.tracking_orientation_tolerance <= 0.0 ||
            !std::isfinite(staged_config.tracking_progress_threshold) || staged_config.tracking_progress_threshold <= 0.0 ||
            !std::isfinite(staged_config.assist_no_progress_seconds) || staged_config.assist_no_progress_seconds <= 0.0 ||
            !std::isfinite(staged_config.settle_seconds) || staged_config.settle_seconds <= 0.0)
            throw std::runtime_error("Invalid staged motion configuration");
        const std::string base_odom_frame = auto_declare("base_odom_frame", std::string("odom"));
        base_odom_frame_ = base_odom_frame;
        const std::string base_odom_topic = auto_declare(
            "base_odom_topic", std::string("/ranger_mini_controller/odom"));
        const std::string base_cmd_topic = auto_declare("base_cmd_topic", std::string("/cmd_vel"));
        base_linear_start_threshold_ = auto_declare("base_linear_start_threshold", 0.03);
        base_linear_stop_threshold_ = auto_declare("base_linear_stop_threshold", 0.015);
        base_angular_start_threshold_ = auto_declare("base_angular_start_threshold", 0.04);
        base_angular_stop_threshold_ = auto_declare("base_angular_stop_threshold", 0.02);
        base_command_profile_ = auto_declare("base_command_profile", std::string("omni"));
        staged_config.ranger_base = base_command_profile_ == "ranger";
        base_max_linear_speed_ = auto_declare("base_max_linear_speed", 0.0);
        staged_config.approach_max_linear_velocity = base_max_linear_speed_ > 0.0 ? base_max_linear_speed_ : 0.15;
        base_max_angular_speed_ = auto_declare("base_max_angular_speed", 0.0);
        staged_config.heading_max_velocity = base_max_angular_speed_ > 0.0 ? base_max_angular_speed_ : 0.3;
        if ((base_command_profile_ != "omni" && base_command_profile_ != "ranger") ||
            !std::isfinite(base_max_linear_speed_) || base_max_linear_speed_ < 0.0 ||
            !std::isfinite(base_max_angular_speed_) || base_max_angular_speed_ < 0.0)
        {
            throw std::runtime_error("Invalid base command profile or speed limits");
        }
        if (!std::isfinite(base_linear_start_threshold_) ||
            !std::isfinite(base_linear_stop_threshold_) ||
            !std::isfinite(base_angular_start_threshold_) ||
            !std::isfinite(base_angular_stop_threshold_) ||
            base_linear_stop_threshold_ < 0.0 ||
            base_linear_start_threshold_ <= base_linear_stop_threshold_ ||
            base_angular_stop_threshold_ < 0.0 ||
            base_angular_start_threshold_ <= base_angular_stop_threshold_)
        {
            throw std::runtime_error(
                "Base command start thresholds must be greater than nonnegative stop thresholds");
        }
        planning_urdf_path_ = auto_declare("planning_urdf_path", std::string(""));
        future_time_offset_ = auto_declare("future_time_offset", 1.0);
        if (!std::isfinite(future_time_offset_) || future_time_offset_ < 0.0)
        {
            throw std::runtime_error("Future offset must be finite and nonnegative");
        }
        const std::string info_file_name = auto_declare("info_file_name", std::string("task"));
        joint_names_ = node_->get_parameter("joints").as_string_array();
        const std::string robot_pkg = robot_name_ + "_description";
        const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);

        const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
        const std::string lib_folder =
            auto_declare("ocs2_library_folder", defaultOcs2LibraryFolder(robot_pkg));
        const std::string urdf_file = resolvePlanningUrdfPath();

        // Frame params: YAML overrides task.info defaults; applied in-memory into Interface.
        const auto info_frames = loadInfoFrameDefaults(task_file);
        const std::string base_frame =
            auto_declare("base_frame", info_frames.baseFrame);
        const std::string tcp_ee_frame =
            auto_declare("tcp_ee_frame", info_frames.eeFrame);
        FrameOverrides frame_overrides;
        frame_overrides.baseFrame = base_frame;
        frame_overrides.eeFrame = tcp_ee_frame;

        interface_ = std::make_shared<MobileManipulatorInterface>(
            task_file, lib_folder, urdf_file, frame_overrides);

        if (interface_->getManipulatorModelInfo().manipulatorModelType ==
            ManipulatorModelType::OmniWheelBasedMobileManipulator)
        {
            base_state_offset_ = 3;
        }
        if (mobile_base_enabled_ && base_state_offset_ != 3)
        {
            throw std::runtime_error(
                "mobile_base_enabled requires manipulatorModelType=4 "
                "(OmniWheelBasedMobileManipulator) in task.info");
        }
        if (mobile_base_enabled_)
        {
            base_odometry_buffer_.writeFromNonRT(BaseOdometryState{});
            base_odometry_subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
                base_odom_topic, rclcpp::SystemDefaultsQoS(),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    if (msg) receiveBaseOdometry(*msg);
                });
            base_command_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
                base_cmd_topic, rclcpp::SystemDefaultsQoS());
            realtime_base_command_publisher_ =
                std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Twist>>(
                    base_command_publisher_);
            RCLCPP_INFO(node_->get_logger(), "Mobile base MPC enabled: odom=%s cmd_vel=%s",
                        base_odom_topic.c_str(), base_cmd_topic.c_str());
        }

        if (interface_->dual_arm_)
        {
            throw std::runtime_error("mpc_controller supports one TCP; disable dualArmMode");
        }

        // The state/input model is scalar arm joints with an optional planar
        // base prefix. Fail configuration rather than silently permuting DoFs.
        const auto& model = interface_->getPinocchioInterface().getModel();
        const auto& info = interface_->getManipulatorModelInfo();
        if (model.nq != static_cast<int>(joint_names_.size() + base_state_offset_) ||
            model.nv != model.nq || info.stateDim != static_cast<size_t>(model.nq) ||
            info.inputDim != static_cast<size_t>(model.nv))
            throw std::runtime_error("MPC requires scalar arm joints and matching state/input model dimensions");
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const auto id = model.getJointId(joint_names_[i]);
            if (id == 0 || id >= model.joints.size())
                throw std::runtime_error("Arm joint missing from MPC model: " + joint_names_[i]);
            const auto& joint = model.joints[id];
            if (joint.nq() != 1 || joint.nv() != 1 || joint.idx_q() != static_cast<int>(i + base_state_offset_) ||
                joint.idx_v() != joint.idx_q())
                throw std::runtime_error("Configured joints must follow the MPC model DoF order: " + joint_names_[i]);
            arm_torque_indices_.push_back(joint.idx_v());
        }
        arm_torques_ = vector_t::Zero(joint_names_.size());
        if (interface_->isSelfCollisionEnabled() && collision_check_timeout_ < 2.0 * visualization_period_sec_)
            throw std::runtime_error("collision_check_timeout must cover at least two collision worker periods (prediction_publish_hz)");
        if (interface_->isSelfCollisionEnabled() && !std::isfinite(interface_->getSelfCollisionMinimumDistance()))
            throw std::runtime_error("Self-collision distance threshold must be finite");

        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);

        {
            zephyr::mpc_visualization::mpc_visualizer_config viz_cfg;
            viz_cfg.urdf_file = urdf_file;
            const auto& minfo = interface_->getManipulatorModelInfo();
            viz_cfg.base_frame = minfo.baseFrame;
            viz_cfg.prediction_visualization_enabled = prediction_visualization_enabled_;
            viz_cfg.ghost_interval = prediction_ghost_interval_;
            viz_cfg.ghost_count = static_cast<size_t>(prediction_ghost_count_);
            viz_cfg.marker_lifetime = std::max(0.15, 3.0 * visualization_period_sec_);
            viz_cfg.control_period = 1.0 / std::max<int64_t>(1, node_->get_parameter("update_rate").as_int());
            // The mobile-manipulator state contains odometry x/y/yaw, so Pinocchio
            // FK produces world/odom coordinates rather than base-local coordinates.
            viz_cfg.visualization_frame = mobile_base_enabled_ ? base_odom_frame : minfo.baseFrame;
            viz_cfg.tcp_ee_frame = minfo.eeFrame;
            viz_cfg.self_collision_enabled = interface_->isSelfCollisionEnabled();
            if (auto geo = interface_->getPinocchioGeometryInterface())
            {
                viz_cfg.pinocchio_geometry = std::move(*geo);
                viz_cfg.self_collision_activation_distance = interface_->getSelfCollisionActivationDistance();
            }
            visualizer_ = std::make_unique<zephyr::mpc_visualization::mpc_visualizer>(
                node_, interface_->getPinocchioInterface(), std::move(viz_cfg));
        }
        visualizer_->initialize();
        visualizer_->setPosePublishPeriod(dds_publish_period_sec_);
        RCLCPP_INFO(node_->get_logger(), "Future sample offset: %.2f control cycles", future_time_offset_);

        auto_declare("movel_duration", 2.0);
        auto_declare("movel_sample_interval", 0.04);
        auto_declare("movel_max_linear_velocity", 0.3);
        auto_declare("movel_max_linear_acceleration", 1.0);
        auto_declare("movel_max_linear_jerk", 2.0);
        auto_declare("movel_max_angular_velocity", 1.0);
        auto_declare("movel_max_angular_acceleration", 2.0);
        auto_declare("movel_max_angular_jerk", 4.0);

        pose_reference_manager_ = std::make_shared<TcpReferenceManager>(
            interface_->getReferenceManagerPtr(),
            mobile_base_enabled_ ? base_odom_frame : interface_->getManipulatorModelInfo().baseFrame,
            interface_->getManipulatorModelInfo().eeFrame,
            interface_->getManipulatorModelInfo().inputDim);
        pose_reference_manager_->subscribe(node_);
        // Extend a copy of the upstream problem through public APIs only.
        auto problem = interface_->getOptimalControlProblem();
        if (staged_config.enabled)
        {
            staged_reference_ = std::make_shared<StagedReferenceManager>(pose_reference_manager_,
                interface_->getPinocchioInterface(), interface_->getManipulatorModelInfo().eeFrame,
                staged_config, node_->get_logger());
            staged_reference_->augmentProblem(problem);
            motion_phase_publisher_ = node_->create_publisher<std_msgs::msg::String>(
                "~/motion_phase", rclcpp::QoS(1).transient_local());
        }
        prepareThreadIsolation();
        mpc_ = std::make_unique<GaussNewtonDDP_MPC>(
            interface_->mpcSettings(),
            interface_->ddpSettings(),
            interface_->getRollout(),
            problem,
            interface_->getInitializer());
        ocs2::ThreadPool::setLaunchWorkerCpuAffinity({});

        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

        if (staged_reference_) mpc_->getSolverPtr()->setReferenceManager(staged_reference_);
        else mpc_->getSolverPtr()->setReferenceManager(pose_reference_manager_);

        observation_.state = interface_->getInitialState();
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
        observation_.time = 0.0;
        observation_candidate_ = observation_.state;
        measured_observation_ = observation_;

        freezeInitializationParameters();
        parameter_callback_handle_ = node_->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                return on_parameter_change(parameters);
            });
        post_parameter_callback_handle_ = node_->add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& parameters) {
                applyParameterChanges(parameters);
            });
    }

    CtrlComponent::~CtrlComponent() { stopVisualizationThread(); }

    void CtrlComponent::freezeInitializationParameters()
    {
        // These values configure models, resources or cached control settings once.
        // Reject later changes rather than reporting success with unchanged behavior.
        for (const auto* name : {
                 "joints", "command_prefix", "command_interfaces", "state_interfaces",
                 "kinematics.group", "kinematics.solver", "kinematics.max_iterations",
                 "kinematics.tolerance", "kinematics.damping", "kinematics.timeout", "kinematics.random_restart",
                 "pd_gains", "mpc_frequency", "update_rate",
                 "cached_ob_state", "hardware_latency", "policy_max_age", "collision_check_timeout", "thread_priority", "cpu_affinity",
                 "dds_publish_hz", "robot_name", "mobile_base_enabled", "base_odom_frame",
                 "base_odom_topic", "base_cmd_topic", "base_linear_start_threshold",
                 "base_linear_stop_threshold", "base_angular_start_threshold",
                 "base_angular_stop_threshold", "base_command_profile", "base_max_linear_speed",
                 "base_max_angular_speed", "planning_urdf_path",
                 "future_time_offset", "info_file_name", "ocs2_library_folder", "base_frame",
                 "tcp_ee_frame", "mock_debug_enabled", "prediction_visualization_enabled", "prediction_publish_hz",
                 "prediction_ghost_interval", "prediction_ghost_count", "prediction_max_age",
                 "staged_motion_enabled", "arm_workspace_frame", "arm_reach_radius",
                 "reach_switch_margin", "base_settle_seconds",
                 "base_parking_radius_ratio", "base_parking_tolerance", "base_heading_tolerance",
                 "base_approach_keep_heading", "tracking_stall_seconds",
                 "tracking_position_tolerance", "tracking_orientation_tolerance",
                 "tracking_progress_threshold", "assist_no_progress_seconds",
                 "movel_duration", "movel_sample_interval", "movel_max_linear_velocity",
                 "movel_max_linear_acceleration", "movel_max_linear_jerk",
                 "movel_max_angular_velocity", "movel_max_angular_acceleration", "movel_max_angular_jerk"})
        {
            if (node_->has_parameter(name))
            {
                initialization_parameters_.emplace(name, node_->get_parameter(name).get_parameter_value());
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult CtrlComponent::on_parameter_change(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool recording_after_update = recorder_.enabled();
        for (const auto& param : parameters)
        {
            if (param.get_name() == "traj_record_enabled" &&
                param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                recording_after_update = param.as_bool();
            }
        }
        for (const auto& param : parameters)
        {
            const auto& name = param.get_name();
            const auto frozen = initialization_parameters_.find(name);
            if (frozen != initialization_parameters_.end() &&
                param.get_parameter_value() != frozen->second)
            {
                result.reason = name + " is initialization-only; change the launch/YAML configuration and reload the controller";
            }
            else if ((name == "traj_record_enabled" || name == "rt_timing_enabled") &&
                     param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                result.reason = name + " must be a bool";
            }
            else if (name == "traj_record_enabled" && param.as_bool() && !mock_debug_enabled_)
            {
                result.reason = "Trajectory recording is only available in the mock launch/profile";
            }
            else if (name == "traj_record_dir")
            {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING || param.as_string().empty())
                {
                    result.reason = "traj_record_dir must be a nonempty string";
                }
                else if (recorder_.enabled() && recording_after_update &&
                         param.as_string() != traj_record_dir_)
                {
                    result.reason = "Stop trajectory recording before changing traj_record_dir";
                }
            }
            if (!result.reason.empty())
            {
                result.successful = false;
                break;
            }
        }
        // Validation has no side effects: another callback can still reject this batch.
        return result;
    }

    void CtrlComponent::applyParameterChanges(const std::vector<rclcpp::Parameter>& parameters)
    {

        // Apply dir before enabled so a same-batch update uses the new directory.
        std::optional<bool> traj_record_enabled_update;
        for (const auto& param : parameters)
        {
            if (param.get_name() == "traj_record_dir")
            {
                traj_record_dir_ = param.as_string();
            }
            else if (param.get_name() == "traj_record_enabled")
            {
                traj_record_enabled_update = param.as_bool();
            }
            else if (param.get_name() == "rt_timing_enabled")
            {
                rt_timing_.enabled.store(param.as_bool(), std::memory_order_relaxed);
                RCLCPP_INFO(node_->get_logger(),
                            "rt_timing_enabled=%s; collect [rt_timing] and control-loop overrun logs.",
                            param.as_bool() ? "true" : "false");
            }
        }
        if (traj_record_enabled_update.has_value())
        {
            const auto error = recorder_.setEnabled(*traj_record_enabled_update, traj_record_dir_);
            if (!error.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "Trajectory recording stopped but CSV save failed: %s", error.c_str());
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "traj_record_enabled=%d, dir=%s",
                            *traj_record_enabled_update, traj_record_dir_.c_str());
            }
        }
    }

    void CtrlComponent::receiveBaseOdometry(const nav_msgs::msg::Odometry& odometry)
    {
        BaseOdometryState state;
        state.x = odometry.pose.pose.position.x;
        state.y = odometry.pose.pose.position.y;
        const auto& q = odometry.pose.pose.orientation;
        const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
        state.valid = odometry.header.frame_id == base_odom_frame_ &&
            std::isfinite(state.x) && std::isfinite(state.y) &&
            std::isfinite(odometry.pose.pose.position.z) &&
            std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
            std::isfinite(norm) && norm > 1e-12;
        if (state.valid)
        {
            const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
            state.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }
        else
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                "Rejected invalid odometry or frame '%s' (expected '%s')",
                                odometry.header.frame_id.c_str(), base_odom_frame_.c_str());
        }
        state.received_at = std::chrono::steady_clock::now();
        base_odometry_buffer_.writeFromNonRT(state);
    }

    void CtrlComponent::stopMobileBase() const
    {
        if (mobile_base_enabled_ && base_command_publisher_)
        {
            geometry_msgs::msg::Twist command;
            publishMobileBaseCommand(command);
        }
    }

    void CtrlComponent::publishMobileBaseCommand(const geometry_msgs::msg::Twist& command) const
    {
        if (!realtime_base_command_publisher_ || !realtime_base_command_publisher_->trylock())
        {
            return;
        }
        realtime_base_command_publisher_->msg_ = command;
        realtime_base_command_publisher_->unlockAndPublish();
    }

    void CtrlComponent::applyBaseCommandDeadband(geometry_msgs::msg::Twist& command)
    {
        // Both axes use the same start/stop hysteresis, with independent state.
        const auto allow_motion = [](double speed, double start, double stop, bool& active) {
            const bool suppress = active ? speed <= stop : speed < start;
            active = !suppress;
            return active;
        };
        if (!allow_motion(std::hypot(command.linear.x, command.linear.y),
                          base_linear_start_threshold_, base_linear_stop_threshold_, base_translation_active_))
        {
            command.linear.x = command.linear.y = 0.0;
        }
        if (!allow_motion(std::abs(command.angular.z),
                          base_angular_start_threshold_, base_angular_stop_threshold_, base_rotation_active_))
        {
            command.angular.z = 0.0;
        }
    }

    bool CtrlComponent::updateObservation(const rclcpp::Time& time)
    {
        observation_valid_ = false;
        if (!mpc_mrt_interface_)
        {
            return false;
        }
        // Stage feedback first. Never commit a mixture of fresh and failed reads.
        if (mobile_base_enabled_)
        {
            const auto* base = base_odometry_buffer_.readFromRT();
            const bool fresh = base && base->valid &&
                (std::chrono::steady_clock::now() - base->received_at) < std::chrono::milliseconds(500);
            base_odometry_received_.store(fresh, std::memory_order_release);
            if (fresh)
            {
                observation_candidate_(0) = base->x;
                observation_candidate_(1) = base->y;
                observation_candidate_(2) = base->yaw;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                     "Missing, invalid or stale mobile base odometry; holding outputs");
                return false;
            }
        }
        if (ctrl_interfaces_.joint_position_state_interface_.size() != joint_names_.size())
        {
            return false;
        }
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
            if (!value || !std::isfinite(*value))
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                    "Invalid position feedback for %s; holding outputs", joint_names_[i].c_str());
                return false;
            }
            observation_candidate_[static_cast<Eigen::Index>(i + base_state_offset_)] = *value;
        }
        if (!observation_candidate_.allFinite())
        {
            return false;
        }
        observation_.state = observation_candidate_;
        observation_.time = time.seconds();
        observation_.input.setZero();
        observation_valid_ = true;
        measured_observation_ = observation_;

        // Update the native TCP reference manager observation.
        if (pose_reference_manager_)
        {
            pose_reference_manager_->setCurrentObservation(observation_);
        }

        if (visualizer_)
        {
            vector_t tcp_pose;
            {
                auto fk_scope = rt_timing_.scope(&rt_timing_.ee_fk_us);
                tcp_pose = visualizer_->computeEndEffectorPose(observation_.state);
            }
            auto pub_scope = rt_timing_.scope(&rt_timing_.ee_pub_us);
            visualizer_->publishTcpPose(time, tcp_pose);
        }
        return true;
    }

    bool CtrlComponent::evaluatePolicy(const rclcpp::Time& time)
    {
        auto eval_scope = rt_timing_.scope(&rt_timing_.eval_us);
        if (!mpc_mrt_interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "MPC MRT interface not available");
            holdLastSentPositions();
            return false;
        }
        if (!observation_valid_ ||
            (mobile_base_enabled_ && !base_odometry_received_.load(std::memory_order_acquire)))
        {
            stopMobileBase();
            holdLastSentPositions();
            return false;
        }
        {
            auto upd_scope = rt_timing_.scope(&rt_timing_.eval_upd_us);
            if (mpc_mrt_interface_->updatePolicy())
            {
                policy_active_ = true;
                const auto& policy = mpc_mrt_interface_->getPolicy();
                policy_valid_ = validPolicy(policy.timeTrajectory_, policy.stateTrajectory_,
                    policy.inputTrajectory_, observation_.state.size(), observation_.input.size());
                policy_received_at_ = std::chrono::steady_clock::now();
            }
        }
        // Always feed real feedback back into MPC, including while holding for
        // a policy/phase handoff. Commands are never substituted as observations.
        mpc_mrt_interface_->setCurrentObservation(measured_observation_);
        if (!policy_active_)
        {
            holdLastSentPositions();
            return true;
        }
        const auto& policy = mpc_mrt_interface_->getPolicy();
        const double future_time = time.seconds() + future_time_offset_ / ctrl_interfaces_.frequency_;
        const double policy_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - policy_received_at_).count();
        if (!policy_valid_ || !usablePolicy(time.seconds(), future_time, policy.timeTrajectory_.front(),
                                           policy.timeTrajectory_.back(), policy_age, policy_max_age_))
        {
            RCLCPP_ERROR(node_->get_logger(), "MPC policy invalid, expired, or outside its sampling horizon; explicit resume required");
            holdLastSentPositions();
            return false;
        }
        MotionPhase phase = MotionPhase::Arm;
        StagedMotionState approach_state;
        if (staged_reference_)
        {
            const auto plan = std::atomic_load_explicit(&staged_policy_, std::memory_order_acquire);
            const auto& times = mpc_mrt_interface_->getPolicy().timeTrajectory_;
            if (!policy_active_ || !plan || times.empty() ||
                plan->epoch != visualization_epoch_.load(std::memory_order_acquire) ||
                std::abs(plan->stage.policy_start - times.front()) > 1e-6)
            {
                mpc_mrt_interface_->setCurrentObservation(measured_observation_);
                holdLastSentPositions();
                return true; // Never apply a policy with a different phase/version.
            }
            phase = plan->stage.phase;
            approach_state = plan->stage;
            if ((phase == MotionPhase::Base || phase == MotionPhase::Settling || phase == MotionPhase::Unreachable) &&
                phase != applied_motion_phase_)
                held_arm_positions_ = measured_observation_.state.tail(joint_names_.size());
            if (phase != applied_motion_phase_)
            {
                std_msgs::msg::String message;
                message.data = motionPhaseName(phase);
                motion_phase_publisher_->publish(message);
            }
            applied_motion_phase_ = phase;
        }
        {
            auto pub_scope = rt_timing_.scope(&rt_timing_.eval_obs_pub_us);
            if (shouldPublishDds(time))
            {
                mpc_observation_publisher_->publish(ros_msg_conversions::createObservationMsg(observation_));
            }
        }

        vector_t future_state;
        vector_t future_input;
        {
            auto mrt_scope = rt_timing_.scope(&rt_timing_.eval_mrt_us);
            // Commands have always used the future policy sample below. Do not
            // also evaluate a current-time state/input that is never consumed.
            future_state = LinearInterpolation::interpolate(
                future_time, policy.timeTrajectory_, policy.stateTrajectory_);
            future_input = LinearInterpolation::interpolate(
                future_time, policy.timeTrajectory_, policy.inputTrajectory_);
            if (staged_reference_)
            {
                if (phase == MotionPhase::Arm || phase == MotionPhase::Settling || phase == MotionPhase::Unreachable)
                {
                    future_input.head<3>().setZero();
                    future_state.head<3>() = measured_observation_.state.head<3>();
                    base_translation_active_ = base_rotation_active_ = false;
                }
                if (phase == MotionPhase::Base || phase == MotionPhase::Settling || phase == MotionPhase::Unreachable)
                {
                    future_input.tail(joint_names_.size()).setZero();
                    future_state.tail(joint_names_.size()) = held_arm_positions_;
                }
            }
            if (staged_reference_ && phase == MotionPhase::Base)
            {
                const double max_angular = base_max_angular_speed_ > 0.0 ? base_max_angular_speed_ : 0.3;
                if (base_command_profile_ == "ranger")
                    future_input.head<3>() = rangerApproachCommand(measured_observation_.state.head<3>(), approach_state,
                        base_max_linear_speed_ > 0.0 ? base_max_linear_speed_ : 0.15, max_angular).input;
                else
                    future_input(2) = rangerHeadingCommand(measured_observation_.state(2),
                        approach_state.parking_yaw, max_angular).velocity;
            }
            if (recorder_.enabled())
            {
                pending_viz_has_pred_ = true;
                pending_viz_pred_time_ = future_time;
                pending_viz_pred_state_ = future_state;
            }
        }

        auto cmd_scope = rt_timing_.scope(&rt_timing_.eval_cmd_us);
        const auto mode = ctrl_interfaces_.control_mode_;
        const bool mixed = mode == ControlMode::MIX;
        if ((!mixed && mode != ControlMode::POSITION) ||
            !future_state.allFinite() || !future_input.allFinite())
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid MPC command or unsupported output mode; holding");
            holdLastSentPositions();
            return false;
        }
        // Validate all commands before writing any base or arm output.
        if (mixed)
        {
            try
            {
                if (!selectJointTorques(calculateStaticTorques(), arm_torque_indices_, arm_torques_) ||
                    ctrl_interfaces_.joint_force_command_interface_.size() != joint_names_.size() ||
                    ctrl_interfaces_.joint_velocity_command_interface_.size() != joint_names_.size())
                    throw std::runtime_error("Invalid gravity torque or mixed command interfaces");
            }
            catch (const std::exception& error)
            {
                RCLCPP_ERROR(node_->get_logger(), "Mixed output rejected: %s", error.what());
                holdLastSentPositions();
                return false;
            }
        }
        if (mobile_base_enabled_ && base_command_publisher_)
        {
            geometry_msgs::msg::Twist command;
            if (base_odometry_received_.load(std::memory_order_acquire) && future_input.size() >= 3)
            {
                command.linear.x = future_input(0);
                command.linear.y = future_input(1);
                command.angular.z = future_input(2);
                // The staged Ranger model locks lateral speed. Remove solver
                // residuals before the backend's mutually exclusive crab/yaw adapter.
                if (staged_reference_ && base_command_profile_ == "ranger") command.linear.y = 0.0;
            }
            // An active Ranger parking maneuver intentionally slows below the
            // idle deadband. Arrival/settling explicitly sends zero to stop it.
            if (!(staged_reference_ && phase == MotionPhase::Base && base_command_profile_ == "ranger"))
                applyBaseCommandDeadband(command);
            constrainBaseCommand(command, base_command_profile_ == "ranger",
                                 base_max_linear_speed_, base_max_angular_speed_);
            publishMobileBaseCommand(command);
        }
        if (mixed)
        {
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(arm_torques_(i));
            }
        }
        for (size_t i = 0; i < joint_names_.size() &&
                            i + base_state_offset_ < static_cast<size_t>(future_state.size()); ++i)
        {
            ctrl_interfaces_.setJointMotionCommand(
                i, future_state(static_cast<Eigen::Index>(i + base_state_offset_)),
                future_input(static_cast<Eigen::Index>(i + base_state_offset_)));
        }
        return true;
    }

    bool CtrlComponent::requestReferenceReset()
    {
        // Use real joints, even if the held reference differs after disturbance.
        reset_tcp_pose_ = visualizer_->computeEndEffectorPose(measured_observation_.state);
        return pose_reference_manager_->requestReset(reset_tcp_pose_, measured_observation_.time);
    }

    void CtrlComponent::resetMpc()
    {
        clearTrajectoryVisualization();
        if (staged_reference_) staged_reference_->reset();
        applied_motion_phase_ = MotionPhase::Arm;
        held_arm_positions_ = measured_observation_.state.tail(joint_names_.size());
        if (motion_phase_publisher_)
        {
            std_msgs::msg::String message;
            message.data = motionPhaseName(MotionPhase::Arm);
            motion_phase_publisher_->publish(message);
        }
        observation_ = measured_observation_;
        const auto& tcp_pose = reset_tcp_pose_;
        RCLCPP_INFO(node_->get_logger(),
                    "Initial TCP target: t=%.3f xyz=[%.3f, %.3f, %.3f]",
                    observation_.time, tcp_pose(0), tcp_pose(1), tcp_pose(2));
        const TargetTrajectories target_trajectories({observation_.time}, {tcp_pose}, {observation_.input});

        // Set initial observation and target trajectory (non-blocking).
        // Waiting for initialPolicyReceived() must happen on the MPC worker thread —
        // blocking here runs inside controller_manager::update and causes overruns.
        mpc_mrt_interface_->reset();
        policy_active_ = false;
        policy_valid_ = false;
        mpc_mrt_interface_->setCurrentObservation(observation_);
        mpc_mrt_interface_->resetMpcNode(target_trajectories);
        RCLCPP_INFO(node_->get_logger(),
                    "MPC reset complete; waiting for initial policy on MPC thread");
    }

    bool CtrlComponent::initialPolicyReceived() const
    {
        return mpc_mrt_interface_ && mpc_mrt_interface_->initialPolicyReceived();
    }

    void CtrlComponent::holdLastSentPositions() const
    {
        stopMobileBase();
        for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.last_sent_joint_positions_.size(); ++i)
        {
            ctrl_interfaces_.setJointPositionCommand(i, ctrl_interfaces_.last_sent_joint_positions_[i]);
        }
        for (auto& interface : ctrl_interfaces_.joint_velocity_command_interface_)
        {
            std::ignore = interface.get().set_value(0.0);
        }
    }

    void CtrlComponent::setMpcPeriodSec(double period_sec)
    {
        if (period_sec <= 0.0)
        {
            return;
        }
        mpc_period_us_.store(static_cast<int64_t>(period_sec * 1e6 + 0.5), std::memory_order_relaxed);
    }

    void CtrlComponent::advanceMpc()
    {
        const auto t0 = std::chrono::steady_clock::now();
        const auto epoch = visualization_epoch_.load(std::memory_order_acquire);
        mpc_mrt_interface_->advanceMpc();
        if (staged_reference_ && epoch == visualization_epoch_.load(std::memory_order_acquire))
        {
            auto plan = std::make_shared<StagedPolicy>();
            plan->stage = staged_reference_->state();
            plan->epoch = epoch;
            std::shared_ptr<const StagedPolicy> immutable = std::move(plan);
            std::atomic_store_explicit(&staged_policy_, std::move(immutable), std::memory_order_release);
        }
        if (prediction_visualization_enabled_)
        {
            // Same worker owns the solver. Export the entire horizon separately from
            // MRT's short control window; never copy the full solution in update().
            try
            {
                const double final_time = mpc_->getSolverPtr()->getFinalTime();
                const auto previous = std::atomic_load_explicit(&solver_prediction_, std::memory_order_acquire);
                if (!previous || previous->epoch != epoch ||
                    std::abs(previous->times.back() - final_time) > 1e-6)
                {
                    PrimalSolution full;
                    mpc_->getSolverPtr()->getPrimalSolution(final_time, &full);
                    auto prediction = std::make_shared<zephyr::mpc_visualization::TimedPrediction>();
                    prediction->times = std::move(full.timeTrajectory_);
                    prediction->states = std::move(full.stateTrajectory_);
                    prediction->epoch = epoch;
                    prediction->captured_at = std::chrono::steady_clock::now();
                    if (prediction->valid() && epoch == visualization_epoch_.load(std::memory_order_acquire))
                    {
                        std::shared_ptr<const zephyr::mpc_visualization::TimedPrediction> immutable = std::move(prediction);
                        std::atomic_store_explicit(&solver_prediction_, std::move(immutable), std::memory_order_release);
                    }
                }
            }
            catch (const std::exception& error)
            {
                // Optional visualization export must not interrupt robot control.
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                    "Prediction export failed: %s", error.what());
            }
        }
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
        const auto period_us = mpc_period_us_.load(std::memory_order_relaxed);
        if (period_us > 0 && us > period_us)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "MPC solve %ld us exceeds period %ld us (%.1f Hz); tracking will lag",
                                 static_cast<long>(us), static_cast<long>(period_us),
                                 1e6 / static_cast<double>(period_us));
        }
    }

    void CtrlComponent::startVisualizationThread()
    {
        if (visualization_running_.load(std::memory_order_acquire)) return;
        // A reactivated controller must not briefly redisplay its previous epoch.
        clearTrajectoryVisualization();
        std::atomic_store_explicit(&visualization_snapshot_,
            std::shared_ptr<VisualizationSnapshot>{}, std::memory_order_release);
        last_recorded_observation_time_ = -1.0;
        collision_started_us_ = steadyMicroseconds();
        collision_watchdog_.reset();
        // Production has no dynamically enabled recorder. Keep the worker only
        // when configured collision monitoring needs it; never disable that check.
        if (!mock_debug_enabled_ && !interface_->isSelfCollisionEnabled()) return;
        visualization_running_.store(true, std::memory_order_release);
        visualization_thread_ = std::thread(&CtrlComponent::visualizationThreadLoop, this);
    }

    void CtrlComponent::stopVisualizationThread()
    {
        visualization_running_.store(false, std::memory_order_release);
        visualization_wakeup_.notify_one();
        if (visualization_thread_.joinable()) visualization_thread_.join();
        collision_watchdog_.reset();
    }

    CollisionStatus CtrlComponent::collisionStatus() const
    {
        if (!interface_->isSelfCollisionEnabled()) return CollisionStatus::Safe;
        return collision_watchdog_.status(steadyMicroseconds(), collision_started_us_,
            static_cast<int64_t>(collision_check_timeout_ * 1e6));
    }

    void CtrlComponent::requestVisualizationUpdate()
    {
        const bool collision_enabled = interface_ && interface_->isSelfCollisionEnabled();
        const bool recording_enabled = recorder_.enabled();
        if (!prediction_visualization_enabled_ && !collision_enabled && !recording_enabled)
        {
            pending_viz_has_pred_ = false;
            return;
        }
        auto snap = std::make_shared<VisualizationSnapshot>();
        snap->observation = measured_observation_;
        snap->observed_at_us = steadyMicroseconds();
        snap->epoch = visualization_epoch_.load(std::memory_order_acquire);
        if (collision_enabled)
        {
            snap->collision_state = buildCollisionState();
        }
        snap->has_pred = recording_enabled && pending_viz_has_pred_;
        snap->pred_time = pending_viz_pred_time_;
        snap->pred_state = std::move(pending_viz_pred_state_);
        pending_viz_has_pred_ = false;
        pending_viz_pred_time_ = 0.0;
        {
            auto traj_scope = rt_timing_.scope(&rt_timing_.eval_traj_us);
            if (prediction_visualization_enabled_ && policy_active_ && mpc_mrt_interface_)
            {
                const auto prediction = std::atomic_load_explicit(&solver_prediction_, std::memory_order_acquire);
                const auto& active_times = mpc_mrt_interface_->getPolicy().timeTrajectory_;
                const auto matches_active = [&](const auto& candidate) {
                    return candidate && candidate->epoch == snap->epoch && !active_times.empty() &&
                        std::abs(candidate->times.front() - active_times.front()) < 1e-6;
                };
                if (matches_active(prediction)) active_prediction_ = prediction;
                // The solver may have published the next policy before MRT swaps it.
                // Keep only a full prediction belonging to the policy actually in use.
                if (matches_active(active_prediction_)) snap->prediction = active_prediction_;
            }
        }
        std::atomic_store_explicit(&visualization_snapshot_, std::move(snap), std::memory_order_release);
    }

    void CtrlComponent::maybeRequestVisualizationUpdate()
    {
        if (visualization_running_.load(std::memory_order_acquire)) requestVisualizationUpdate();
    }

    void CtrlComponent::visualizationThreadLoop()
    {
        applyWorkerThreadScheduling("viz");
        RCLCPP_INFO(node_->get_logger(),
                    "Prediction display: %.1f Hz, future pose spacing %.2f s, maximum %d poses, expiry %.2f s",
                    1.0 / visualization_period_sec_, prediction_ghost_interval_, prediction_ghost_count_, prediction_max_age_);
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(visualization_period_sec_));
        auto deadline = std::chrono::steady_clock::now();
        while (visualization_running_.load(std::memory_order_acquire))
        {
            if (trajectory_clear_requested_.exchange(false, std::memory_order_acq_rel))
                visualizer_->clearTrajectoryHistory();
            const auto snap = std::atomic_load_explicit(&visualization_snapshot_, std::memory_order_acquire);
            if (snap) runVisualizationOnce(*snap);
            // Render again even without a new solve: "future" moves with current time,
            // and stale snapshots must expire even when the producer has stopped.
            deadline += period;
            const auto now = std::chrono::steady_clock::now();
            if (deadline < now) deadline = now + period; // Skip missed frames; never build a backlog.
            std::unique_lock<std::mutex> lock(visualization_wait_mutex_);
            visualization_wakeup_.wait_until(lock, deadline, [this] {
                return !visualization_running_.load(std::memory_order_acquire) ||
                    trajectory_clear_requested_.load(std::memory_order_acquire);
            });
        }
        if (prediction_visualization_enabled_ && node_->get_node_base_interface()->get_context()->is_valid())
            visualizer_->clearTrajectoryHistory();
    }

    void CtrlComponent::runVisualizationOnce(const VisualizationSnapshot& snap)
    {
        const auto& obs = snap.observation;
        if (!visualizer_ || obs.state.size() == 0)
        {
            return;
        }

        if (interface_->isSelfCollisionEnabled())
        {
            // Check measured geometry AND commanded geometry. Failure is not
            // clearance, and re-rendering an old snapshot cannot renew its age.
            const auto check = [&](const vector_t& q, bool& collision) {
                if (!visualizer_->publishSelfCollisionVisualization(q)) return false;
                const auto distance = visualizer_->getLastMinDistance();
                if (!std::isfinite(distance)) return false;
                collision = collision || distance <= interface_->getSelfCollisionMinimumDistance();
                return true;
            };
            bool collision = false;
            bool valid = check(obs.state, collision);
            if (valid && snap.collision_state.size() > 0) valid = check(snap.collision_state, collision);
            collision_watchdog_.record(valid, collision, snap.observed_at_us);
        }

        if (prediction_visualization_enabled_)
        {
            const double render_time = node_->now().seconds();
            const bool usable = snap.epoch == visualization_epoch_.load(std::memory_order_acquire) &&
                snap.prediction && snap.prediction->epoch == snap.epoch &&
                snap.prediction->fresh(render_time, prediction_max_age_) &&
                render_time >= obs.time && render_time - obs.time <= prediction_max_age_;
            if (usable)
            {
                visualizer_->updatePrediction(obs.state, *snap.prediction, render_time);
                if (snap.epoch == visualization_epoch_.load(std::memory_order_acquire))
                    visualizer_->publishEndEffectorTrajectory(node_->now());
                else visualizer_->clearTrajectoryHistory();
            }
            else visualizer_->clearTrajectoryHistory();
        }

        auto& rec = recorder_;
        if (rec.enabled() && obs.time != last_recorded_observation_time_)
        {
            last_recorded_observation_time_ = obs.time;
            const double t = obs.time;
            const vector_t lp = visualizer_->computeRecordedEndEffectorPose(obs.state);
            TcpTrajectorySample ls;
            ls.stamp_sec = t;
            ls.position = {lp(0), lp(1), lp(2)};
            ls.quat_xyzw = {lp(3), lp(4), lp(5), lp(6)};
            rec.appendActual(ls);
            if (snap.has_pred && snap.pred_state.size() > 0)
            {
                TcpTrajectorySample lps;
                lps.stamp_sec = snap.pred_time;
                const vector_t lpe = visualizer_->computeRecordedEndEffectorPose(snap.pred_state);
                lps.position = {lpe(0), lpe(1), lpe(2)};
                lps.quat_xyzw = {lpe(3), lpe(4), lpe(5), lpe(6)};
                rec.appendPredicted(lps);
            }
        }
    }

    void CtrlComponent::clearTrajectoryVisualization()
    {
        policy_active_ = false;
        pending_viz_has_pred_ = false;
        visualization_epoch_.fetch_add(1, std::memory_order_acq_rel);
        active_prediction_.reset();
        std::atomic_store_explicit(&staged_policy_, std::shared_ptr<const StagedPolicy>{}, std::memory_order_release);
        std::atomic_store_explicit(&solver_prediction_,
            std::shared_ptr<const zephyr::mpc_visualization::TimedPrediction>{}, std::memory_order_release);
        if (prediction_visualization_enabled_)
        {
            trajectory_clear_requested_.store(true, std::memory_order_release);
            visualization_wakeup_.notify_one();
        }
    }

    std::string CtrlComponent::resolvePlanningUrdfPath() const
    {
        if (planning_urdf_path_.empty())
        {
            throw std::runtime_error(
                "planning_urdf_path must point to the expanded planning URDF provided by launch.");
        }
        if (!std::filesystem::is_regular_file(planning_urdf_path_))
        {
            throw std::runtime_error(
                "planning_urdf_path is not a regular file: " + planning_urdf_path_);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Using expanded planning URDF: %s",
                    planning_urdf_path_.c_str());
        return planning_urdf_path_;
    }

    vector_t CtrlComponent::calculateStaticTorques() const
    {
        const auto& pinocchio_model = interface_->getPinocchioInterface().getModel();
        if (!rnea_data_)
        {
            rnea_data_.emplace(pinocchio_model);
        }
        Eigen::VectorXd q = observation_.state;
        return pinocchio::rnea(pinocchio_model, *rnea_data_, q,
                               Eigen::VectorXd::Zero(pinocchio_model.nv),
                               Eigen::VectorXd::Zero(pinocchio_model.nv));
    }

    bool CtrlComponent::shouldPublishDds(const rclcpp::Time& time)
    {
        if (dds_publish_period_sec_ <= 0.0)
        {
            return true;
        }
        if (last_dds_publish_time_.nanoseconds() != 0 &&
            (time - last_dds_publish_time_).seconds() < dds_publish_period_sec_)
        {
            return false;
        }
        last_dds_publish_time_ = time;
        return true;
    }

    void CtrlComponent::applyWorkerThreadScheduling(const char* thread_name) const
    {
        const bool is_mpc = thread_name != nullptr && std::string(thread_name) == "MPC";
        const auto& cpus = mpc_thread_cpus_.empty() ? cpu_affinity_ : mpc_thread_cpus_;
        // Viz uses CFS on the MPC core; it must not preempt DDP workers.
        runtime::applyThisThreadSchedulingLogged(
            is_mpc ? thread_priority_ : 0, cpus, node_->get_logger(), thread_name);
    }

    void CtrlComponent::applyRtLoopSchedulingOnce()
    {
        if (rt_loop_scheduling_applied_)
        {
            return;
        }
        rt_loop_scheduling_applied_ = true;
        if (rt_loop_cpus_.empty())
        {
            return;
        }
        runtime::applyThisThreadCpuAffinity(rt_loop_cpus_);
        RCLCPP_INFO(node_->get_logger(), "Pinned RT control loop to CPU [%s]",
                    runtime::formatCpuList(rt_loop_cpus_).c_str());
    }

    void CtrlComponent::prepareThreadIsolation()
    {
        const auto plan = runtime::planIsolatedCpus(cpu_affinity_);
        mpc_thread_cpus_ = plan.mpc;
        rt_loop_cpus_ = plan.rt;
        if (plan.isolated)
        {
            ocs2::ThreadPool::setLaunchWorkerCpuAffinity(plan.workers);
            RCLCPP_INFO(node_->get_logger(),
                        "CPU isolation: RT=[%s] MPC=[%s] DDP=[%s] (viz shares MPC core at CFS)",
                        runtime::formatCpuList(plan.rt).c_str(),
                        runtime::formatCpuList(plan.mpc).c_str(),
                        runtime::formatCpuList(plan.workers).c_str());
        }
    }

    vector_t CtrlComponent::buildCollisionState() const
    {
        vector_t q = observation_.state;
        const auto& cmd = ctrl_interfaces_.last_sent_joint_positions_;
        for (size_t i = 0; i < joint_names_.size() && i < cmd.size() &&
                            i + base_state_offset_ < static_cast<size_t>(q.size()); ++i)
        {
            q[static_cast<Eigen::Index>(i + base_state_offset_)] = cmd[i];
        }
        return q;
    }

} // namespace zephyr::mpc
