// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Created for OCS2 Arm Controller - CtrlComponent
//

#include "mpc_wholebody_controller/control/CtrlComponent.h"

#include <ocs2_controller_common/rt/ThreadScheduling.h>
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
#include <fstream>
#include <iomanip>
#include <chrono>
#include <optional>

#include <Eigen/Geometry>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace zephyr::mpc_wholebody
{
// Adapt an omni MPC command to the chassis backend after applying deadbands.
// Positive limits bound speed; zero leaves that axis unbounded.
static void constrainBaseCommand(geometry_msgs::msg::Twist& command,
                                 bool ranger, double max_linear, double max_angular)
{
    if (ranger)
    {
        // Ranger supports crab translation or yaw, never lateral motion plus yaw.
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
        cached_ob_state_ = auto_declare("cached_ob_state", true);
        const double joint_speed_threshold = auto_declare("joint_speed_threshold", 0.1);
        hardware_latency_ = auto_declare("hardware_latency", 0.2);
        if (!std::isfinite(hardware_latency_.load()) || hardware_latency_.load() < 0.0)
        {
            throw std::runtime_error("hardware_latency must be finite and nonnegative");
        }
        // Declare dir before enabled so both can be applied together from yaml/callback.
        traj_record_dir_ = auto_declare("traj_record_dir", std::string("/tmp/traj_record"));
        const bool traj_record_enabled = auto_declare("traj_record_enabled", false);
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
        prediction_visualization_enabled_ = auto_declare("prediction_visualization_enabled", true);
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
        planning_urdf_variant_ = auto_declare("planning_urdf_variant", std::string(""));
        planning_urdf_path_ = auto_declare("planning_urdf_path", std::string(""));
        future_time_offset_ = auto_declare("future_time_offset", 1.0);
        if (!std::isfinite(future_time_offset_) || future_time_offset_ < 0.0 ||
            !std::isfinite(joint_speed_threshold) || joint_speed_threshold < 0.0)
        {
            throw std::runtime_error("Future offset and joint speed threshold must be finite and nonnegative");
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
            throw std::runtime_error("mpc_wholebody_controller supports one TCP; disable dualArmMode");
        }

        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);

        {
            zephyr::mpc_visualization::mpc_visualizer_config viz_cfg;
            viz_cfg.urdf_file = urdf_file;
            const auto& minfo = interface_->getManipulatorModelInfo();
            viz_cfg.base_frame = minfo.baseFrame;
            viz_cfg.mobile_base_enabled = mobile_base_enabled_;
            viz_cfg.prediction_visualization_enabled = prediction_visualization_enabled_;
            viz_cfg.ghost_interval = prediction_ghost_interval_;
            viz_cfg.ghost_count = static_cast<size_t>(prediction_ghost_count_);
            viz_cfg.marker_lifetime = std::max(0.15, 3.0 * visualization_period_sec_);
            viz_cfg.control_period = 1.0 / std::max<int64_t>(1, node_->get_parameter("update_rate").as_int());
            // The mobile-manipulator state contains odometry x/y/yaw, so Pinocchio
            // FK produces world/odom coordinates rather than base-local coordinates.
            viz_cfg.visualization_frame = mobile_base_enabled_ ? base_odom_frame : minfo.baseFrame;
            viz_cfg.tcp_ee_frame = minfo.eeFrame;
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

        auto_declare("movel_trajectory_duration", 2.0);
        auto_declare("movel_duration", 2.0);
        auto_declare("movel_sample_interval", 0.04);
        auto_declare("movel_max_linear_velocity", 0.3);
        auto_declare("movel_max_linear_acceleration", 1.0);
        auto_declare("movel_max_linear_jerk", 2.0);
        auto_declare("movel_max_angular_velocity", 1.0);
        auto_declare("movel_max_angular_acceleration", 2.0);
        auto_declare("movel_max_angular_jerk", 4.0);
        auto_declare("movel_auto_extend_duration", true);

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

        // Initialize cached state
        last_execute_time_ = node_->now();
        cached_last_action_ = observation_.state;

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
                 "pd_gains", "mpc_frequency", "update_rate",
                 "cached_ob_state", "joint_speed_threshold", "thread_priority", "cpu_affinity",
                 "dds_publish_hz", "robot_name", "mobile_base_enabled", "base_odom_frame",
                 "base_odom_topic", "base_cmd_topic", "base_linear_start_threshold",
                 "base_linear_stop_threshold", "base_angular_start_threshold",
                 "base_angular_stop_threshold", "base_command_profile", "base_max_linear_speed",
                 "base_max_angular_speed", "planning_urdf_variant", "planning_urdf_path",
                 "future_time_offset", "info_file_name", "ocs2_library_folder", "base_frame",
                 "tcp_ee_frame", "prediction_visualization_enabled", "prediction_publish_hz",
                 "prediction_ghost_interval", "prediction_ghost_count", "prediction_max_age",
                 "staged_motion_enabled", "arm_workspace_frame", "arm_reach_radius",
                 "reach_switch_margin", "base_settle_seconds",
                 "base_parking_radius_ratio", "base_parking_tolerance", "base_heading_tolerance", "tracking_stall_seconds",
                 "tracking_position_tolerance", "tracking_orientation_tolerance",
                 "tracking_progress_threshold", "assist_no_progress_seconds"})
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
            else if (name == "hardware_latency" &&
                     (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                      !std::isfinite(param.as_double()) || param.as_double() < 0.0))
            {
                result.reason = "hardware_latency must be a finite nonnegative double (seconds)";
            }
            else if ((name == "traj_record_enabled" || name == "rt_timing_enabled") &&
                     param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
            {
                result.reason = name + " must be a bool";
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
            if (param.get_name() == "hardware_latency")
            {
                hardware_latency_.store(param.as_double(), std::memory_order_release);
            }
            else if (param.get_name() == "traj_record_dir")
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
            recorder_.setEnabled(*traj_record_enabled_update, traj_record_dir_);
            RCLCPP_INFO(node_->get_logger(), "traj_record_enabled=%d, dir=%s",
                        *traj_record_enabled_update, traj_record_dir_.c_str());
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
            zephyr::mpc_visualization::mpc_visualizer::EndEffectorPoses poses;
            {
                auto fk_scope = rt_timing_.scope(&rt_timing_.ee_fk_us);
                poses = visualizer_->computeEndEffectorPoses(observation_.state);
            }
            auto pub_scope = rt_timing_.scope(&rt_timing_.ee_pub_us);
            visualizer_->publishEndEffectorPoses(time, poses);
        }
        return true;
    }

    void CtrlComponent::evaluatePolicy(const rclcpp::Time& time)
    {
        auto eval_scope = rt_timing_.scope(&rt_timing_.eval_us);
        if (!mpc_mrt_interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "MPC MRT interface not available");
            return;
        }
        if (!observation_valid_ ||
            (mobile_base_enabled_ && !base_odometry_received_.load(std::memory_order_acquire)))
        {
            stopMobileBase();
            holdLastSentPositions();
            return;
        }
        {
            auto upd_scope = rt_timing_.scope(&rt_timing_.eval_upd_us);
            if (mpc_mrt_interface_->updatePolicy())
            {
                policy_active_ = true;
            }
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
                return; // Never apply a policy with a different phase/version.
            }
            phase = plan->stage.phase;
            approach_state = plan->stage;
            if ((phase == MotionPhase::Base || phase == MotionPhase::Settling || phase == MotionPhase::Unreachable) &&
                phase != applied_motion_phase_)
                held_arm_positions_ = measured_observation_.state.tail(joint_names_.size());
            if (phase != applied_motion_phase_)
            {
                cached_last_action_ = measured_observation_.state;
                std_msgs::msg::String message;
                message.data = motionPhaseName(phase);
                motion_phase_publisher_->publish(message);
            }
            applied_motion_phase_ = phase;
        }
        bool trigger_cached_state = (time - last_execute_time_).seconds() <
                                    hardware_latency_.load(std::memory_order_acquire);
        // Staged reachability/progress must use measured joints, not cached commands.
        if (trigger_cached_state && cached_ob_state_ && !staged_reference_)
        {
            if (base_state_offset_ == 0)
            {
                observation_.state = cached_last_action_;
            }
            else if (cached_last_action_.size() == observation_.state.size())
            {
                const auto arm_state_size =
                    observation_.state.size() - static_cast<Eigen::Index>(base_state_offset_);
                observation_.state.tail(arm_state_size) = cached_last_action_.tail(arm_state_size);
            }
        }

        mpc_mrt_interface_->setCurrentObservation(observation_);
        {
            auto pub_scope = rt_timing_.scope(&rt_timing_.eval_obs_pub_us);
            if (shouldPublishDds(time))
            {
                mpc_observation_publisher_->publish(ros_msg_conversions::createObservationMsg(observation_));
            }
        }

        if (!policy_active_)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Policy not ready (no active MRT policy yet), holding");
            holdLastSentPositions();
            return;
        }

        vector_t future_state;
        vector_t future_input;
        {
            auto mrt_scope = rt_timing_.scope(&rt_timing_.eval_mrt_us);
            const auto& policy = mpc_mrt_interface_->getPolicy();
            if (time.seconds() > policy.timeTrajectory_.back())
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                    "Current time is beyond the received MPC policy");
            }
            // Commands have always used the future policy sample below. Do not
            // also evaluate a current-time state/input that is never consumed.
            const double future_time = observation_.time + future_time_offset_ / ctrl_interfaces_.frequency_;
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
        const auto mode = ctrl_interfaces_.control_mode_;
        const bool mixed = mode == arms_controller_common::ControlMode::MIX;
        if (!mixed && mode != arms_controller_common::ControlMode::POSITION)
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown control output mode");
            return;
        }
        if (mixed)
        {
            const vector_t static_torques = calculateStaticTorques();
            for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(static_torques.size()); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques(i));
            }
        }
        for (size_t i = 0; i < joint_names_.size() &&
                            i + base_state_offset_ < static_cast<size_t>(future_state.size()); ++i)
        {
            ctrl_interfaces_.setJointPositionCommand(
                i, future_state(static_cast<Eigen::Index>(i + base_state_offset_)));
        }
        if (mixed)
        {
            for (size_t i = 0; i < joint_names_.size() &&
                                i + base_state_offset_ < static_cast<size_t>(future_input.size()); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().
                    set_value(future_input(static_cast<Eigen::Index>(i + base_state_offset_)));
            }
        }
        else
        {
            cached_last_action_ = future_state;
            last_execute_time_ = time;
        }
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
        // Update observation time to current time
        observation_.time = node_->now().seconds();

        // Use last sent joint positions for initial state (avoids jumps when entering OCS2 state)
        for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.last_sent_joint_positions_.size(); ++i)
        {
            observation_.state[static_cast<Eigen::Index>(i + base_state_offset_)] =
                ctrl_interfaces_.last_sent_joint_positions_[i];
        }

        const vector_t tcp_pose = visualizer_->computeEndEffectorPose(observation_.state);
        RCLCPP_INFO(node_->get_logger(),
                    "Initial TCP target: t=%.3f xyz=[%.3f, %.3f, %.3f]",
                    observation_.time, tcp_pose(0), tcp_pose(1), tcp_pose(2));
        if (pose_reference_manager_)
        {
            pose_reference_manager_->setCurrentObservation(observation_);
            pose_reference_manager_->setCurrentTcpPose(tcp_pose);
            pose_reference_manager_->publishCurrentTargetsFromCache();
        }
        const TargetTrajectories target_trajectories({observation_.time}, {tcp_pose}, {observation_.input});

        // Set initial observation and target trajectory (non-blocking).
        // Waiting for initialPolicyReceived() must happen on the MPC worker thread —
        // blocking here runs inside controller_manager::update and causes overruns.
        mpc_mrt_interface_->reset();
        policy_active_ = false;
        collision_active_.store(false, std::memory_order_release);
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

    void CtrlComponent::publishCachedCurrentTargets() const
    {
        if (pose_reference_manager_)
        {
            pose_reference_manager_->publishCurrentTargetsFromCache();
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
        visualization_running_.store(true, std::memory_order_release);
        visualization_thread_ = std::thread(&CtrlComponent::visualizationThreadLoop, this);
    }

    void CtrlComponent::stopVisualizationThread()
    {
        visualization_running_.store(false, std::memory_order_release);
        visualization_wakeup_.notify_one();
        if (visualization_thread_.joinable()) visualization_thread_.join();
        collision_active_.store(false, std::memory_order_release);
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

    void CtrlComponent::maybeRequestVisualizationUpdate(const rclcpp::Time& /*time*/)
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

        // One FCL per viz tick on command-overlay q: updates markers + collision_active_ for RT.
        const vector_t& collision_q =
            snap.collision_state.size() > 0 ? snap.collision_state : obs.state;
        bool active = false;
        if (visualizer_->publishSelfCollisionVisualization(collision_q))
        {
            const scalar_t thr = interface_->getSelfCollisionMinimumDistance();
            active = visualizer_->getLastMinDistance() <= thr;
        }
        collision_active_.store(active, std::memory_order_release);

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
        if (planning_urdf_variant_ != "xacro" || planning_urdf_path_.empty())
        {
            throw std::runtime_error(
                "OCS2 planning URDF must come from xacro (planning_urdf_variant:=xacro, "
                "planning_urdf_path set by robot_common_launch). Static urdf/ lookup was removed.");
        }
        if (!std::filesystem::exists(planning_urdf_path_))
        {
            throw std::runtime_error(
                "planning_urdf_path does not exist: " + planning_urdf_path_);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Using xacro-generated planning URDF: %s",
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
        ocs2::controller_common::applyThisThreadSchedulingLogged(
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
        ocs2::controller_common::applyThisThreadCpuAffinity(rt_loop_cpus_);
        RCLCPP_INFO(node_->get_logger(), "Pinned RT control loop to CPU [%s]",
                    ocs2::controller_common::formatCpuList(rt_loop_cpus_).c_str());
    }

    void CtrlComponent::prepareThreadIsolation()
    {
        const auto plan = ocs2::controller_common::planIsolatedCpus(cpu_affinity_);
        mpc_thread_cpus_ = plan.mpc;
        rt_loop_cpus_ = plan.rt;
        if (plan.isolated)
        {
            ocs2::ThreadPool::setLaunchWorkerCpuAffinity(plan.workers);
            RCLCPP_INFO(node_->get_logger(),
                        "CPU isolation: RT=[%s] MPC=[%s] DDP=[%s] (viz shares MPC core at CFS)",
                        ocs2::controller_common::formatCpuList(plan.rt).c_str(),
                        ocs2::controller_common::formatCpuList(plan.mpc).c_str(),
                        ocs2::controller_common::formatCpuList(plan.workers).c_str());
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

// CSV recording belongs to the component visualization worker.

TcpTrajectoryRecorder::~TcpTrajectoryRecorder() { try { setEnabled(false,directory_); } catch (...) {} }
void TcpTrajectoryRecorder::setEnabled(bool enabled,const std::string& directory) {
    std::vector<TcpTrajectorySample> actual,predicted;
    std::string output;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (enabled==enabled_) return;
        if (enabled) {
            directory_=directory; actual_.clear(); predicted_.clear();
            actual_.reserve(10000); predicted_.reserve(10000); enabled_=true; return;
        }
        enabled_=false; actual.swap(actual_); predicted.swap(predicted_); output=directory_;
    }
    std::filesystem::create_directories(output);
    const auto write=[&](const char* name,const std::vector<TcpTrajectorySample>& samples) {
        std::ofstream file(std::filesystem::path(output)/name);
        file.exceptions(std::ios::badbit|std::ios::failbit);
        file<<"stamp_sec,x,y,z,qx,qy,qz,qw\n"<<std::setprecision(17);
        for (const auto& s:samples) { file<<s.stamp_sec; for (double v:s.position) file<<','<<v; for (double v:s.quat_xyzw) file<<','<<v; file<<'\n'; }
    };
    write("real_tcp.csv",actual); write("pred_tcp.csv",predicted);
}
void TcpTrajectoryRecorder::appendActual(const TcpTrajectorySample& sample) {
    std::lock_guard<std::mutex> lock(mutex_); if (enabled_) actual_.push_back(sample);
}
void TcpTrajectoryRecorder::appendPredicted(const TcpTrajectorySample& sample) {
    std::lock_guard<std::mutex> lock(mutex_); if (enabled_) predicted_.push_back(sample);
}

// TCP target input, frame conversion, and OCS2 reference trajectories.

using namespace ocs2;
namespace {
vector_t poseVector(const geometry_msgs::msg::Pose& p) {
    vector_t v(7);
    v << p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w;
    if (!v.allFinite() || v.tail<4>().norm() < 1e-9) throw std::invalid_argument("Invalid TCP pose");
    v.tail<4>().normalize();
    return v;
}
geometry_msgs::msg::Pose rosPose(const vector_t& v) {
    geometry_msgs::msg::Pose p;
    p.position.x=v(0); p.position.y=v(1); p.position.z=v(2);
    p.orientation.x=v(3); p.orientation.y=v(4); p.orientation.z=v(5); p.orientation.w=v(6);
    return p;
}
Eigen::Quaterniond quaternion(const vector_t& p) { return Eigen::Quaterniond(p(6),p(3),p(4),p(5)); }
vector_t interpolate(const vector_t& a,const vector_t& b,double t) {
    vector_t p(7); p.head<3>()=(1-t)*a.head<3>()+t*b.head<3>();
    p.tail<4>()=quaternion(a).slerp(t,quaternion(b)).normalized().coeffs(); return p;
}
double smooth(double t) { return t*t*t*(10+t*(-15+6*t)); }
double minimumDuration(double distance,double velocity,double acceleration,double jerk) {
    return std::max({1.875*distance/velocity,std::sqrt(5.774*distance/acceleration),std::cbrt(60*distance/jerk)});
}
}
TcpReferenceManager::TcpReferenceManager(std::shared_ptr<ReferenceManagerInterface> wrapped,
    std::string base_frame,std::string tcp_frame,size_t input_dim)
    : ReferenceManagerDecorator(std::move(wrapped)),base_frame_(std::move(base_frame)),
      tcp_frame_(std::move(tcp_frame)),input_dim_(input_dim),target_(vector_t::Zero(7)),current_(vector_t::Zero(7)) {
    target_(6)=current_(6)=1.0;
}
void TcpReferenceManager::subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
    node_=node;
    const auto parameter=[&](const char* key,double fallback) {
        const double value=node->has_parameter(key)?node->get_parameter(key).as_double():node->declare_parameter(key,fallback);
        if (!std::isfinite(value)||value<=0.0) throw std::invalid_argument(std::string(key)+" must be positive");
        return value;
    };
    duration_=parameter("movel_duration",2.0); interval_=parameter("movel_sample_interval",0.04);
    max_linear_=parameter("movel_max_linear_velocity",0.3); max_angular_=parameter("movel_max_angular_velocity",1.0);
    max_linear_acc_=parameter("movel_max_linear_acceleration",1.0); max_angular_acc_=parameter("movel_max_angular_acceleration",2.0);
    max_linear_jerk_=parameter("movel_max_linear_jerk",2.0); max_angular_jerk_=parameter("movel_max_angular_jerk",4.0);
    tf_=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
    target_pub_=node->create_publisher<geometry_msgs::msg::PoseStamped>("tcp_current_target",rclcpp::QoS(1).transient_local());
    pose_sub_=node->create_subscription<geometry_msgs::msg::Pose>("tcp_target",1,
        [this](geometry_msgs::msg::Pose::ConstSharedPtr p) { geometry_msgs::msg::PoseStamped s; s.header.frame_id=base_frame_; s.pose=*p; acceptPose(s,false); });
    stamped_sub_=node->create_subscription<geometry_msgs::msg::PoseStamped>("tcp_target/stamped",1,
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) { acceptPose(*p,true); });
    relative_sub_=node->create_subscription<geometry_msgs::msg::TwistStamped>("tcp_target/relative",1,
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr p) { acceptRelative(*p); });
    twist_sub_=node->create_subscription<geometry_msgs::msg::Twist>("tcp_target/twist",1,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr p) {
            if (!accepting_) return;
            const double values[]={p->linear.x,p->linear.y,p->linear.z,p->angular.x,p->angular.y,p->angular.z};
            for (double v:values) if (!std::isfinite(v)) return;
            std::lock_guard<std::mutex> lock(mutex_);
            target_=sample(time_); twist_=*p; twist_active_=true; twist_received_=std::chrono::steady_clock::now();
        });
    path_sub_=node->create_subscription<nav_msgs::msg::Path>("target_path",1,
        [this](nav_msgs::msg::Path::ConstSharedPtr p) { acceptPath(*p); });
    path_service_=node->create_service<::mpc_wholebody_controller::srv::ExecuteTcpPath>("execute_path",
        [this](const std::shared_ptr<::mpc_wholebody_controller::srv::ExecuteTcpPath::Request> req,
               std::shared_ptr<::mpc_wholebody_controller::srv::ExecuteTcpPath::Response> res) {
            try {
                if (!accepting_||req->path.poses.empty()) throw std::invalid_argument("Tracking paused or empty TCP path");
                if (!std::isfinite(req->trajectory_duration)||req->trajectory_duration<0.0) throw std::invalid_argument("Invalid path duration");
                vector_array_t goals;
                if (!transformPath(req->path, goals)) throw std::invalid_argument("Invalid path pose");
                std::lock_guard<std::mutex> lock(mutex_);
                if (!accepting_) throw std::invalid_argument("Tracking paused");
                twist_active_=false; plan(goals,req->trajectory_duration>0.0?req->trajectory_duration:duration_); publish();
                res->success=true;res->estimated_duration=times_.back()-times_.front();res->message="TCP path accepted";
            } catch (const std::exception& e) { res->success=false;res->message=e.what(); }
        });
    circle_service_=node->create_service<arms_ros2_control_msgs::srv::ExecuteCircle>("execute_tcp_circle",
        [this](const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Request> req,
               std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Response> res) {
            try { circle(*req,*res); } catch (const std::exception& e) { res->success=false; res->message=e.what(); }
        });
}
bool TcpReferenceManager::transform(const geometry_msgs::msg::PoseStamped& p,vector_t& result) const {
    try {
        auto converted=p;
        if (!p.header.frame_id.empty()&&p.header.frame_id!=base_frame_)
            tf2::doTransform(p,converted,tf_->lookupTransform(base_frame_,p.header.frame_id,tf2::TimePointZero));
        result=poseVector(converted.pose); return true;
    } catch (const std::exception& e) { RCLCPP_WARN(node_->get_logger(),"TCP target rejected: %s",e.what()); return false; }
}
void TcpReferenceManager::acceptPose(const geometry_msgs::msg::PoseStamped& p,bool interpolate_target) {
    if (!accepting_) return;
    vector_t goal; if (!transform(p,goal)) return;
    std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return;
    twist_active_=false;
    try {
        if (interpolate_target) plan({goal},duration_);
        else { target_=goal; commitImmediate(); }
        publish();
    } catch (const std::exception& e) { RCLCPP_WARN(node_->get_logger(),"TCP target rejected: %s",e.what()); }
}
void TcpReferenceManager::acceptRelative(const geometry_msgs::msg::TwistStamped& delta) {
    if (!accepting_) return;
    try {
        Eigen::Vector3d translation(delta.twist.linear.x,delta.twist.linear.y,delta.twist.linear.z);
        Eigen::Vector3d rotation(delta.twist.angular.x,delta.twist.angular.y,delta.twist.angular.z);
        if (!translation.allFinite()||!rotation.allFinite()) throw std::invalid_argument("Invalid relative target");
        std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return;
        vector_t goal=sample(time_);
        Eigen::Quaterniond frame=Eigen::Quaterniond::Identity();
        if (delta.header.frame_id==tcp_frame_) frame=quaternion(goal);
        else if (!delta.header.frame_id.empty()&&delta.header.frame_id!=base_frame_) {
            const auto tf=tf_->lookupTransform(base_frame_,delta.header.frame_id,tf2::TimePointZero);
            frame=Eigen::Quaterniond(tf.transform.rotation.w,tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z).normalized();
        }
        goal.head<3>()+=frame*translation;
        const double angle=rotation.norm();
        if (angle>1e-12) goal.tail<4>()=(frame*Eigen::Quaterniond(Eigen::AngleAxisd(angle,rotation/angle))*frame.conjugate()*quaternion(goal)).normalized().coeffs();
        twist_active_=false; plan({goal},duration_); publish();
    } catch (const std::exception& e) { RCLCPP_WARN(node_->get_logger(),"Relative TCP target rejected: %s",e.what()); }
}
void TcpReferenceManager::acceptPath(const nav_msgs::msg::Path& path) {
    if (!accepting_||path.poses.empty()) return;
    vector_array_t goals;
    if (!transformPath(path, goals)) return;
    std::lock_guard<std::mutex> lock(mutex_); if (!accepting_) return;
    try { twist_active_=false; plan(goals,duration_); publish(); }
    catch (const std::exception& e) { RCLCPP_WARN(node_->get_logger(),"TCP path rejected: %s",e.what()); }
}
bool TcpReferenceManager::transformPath(const nav_msgs::msg::Path& path, vector_array_t& goals) const {
    goals.clear();
    goals.reserve(path.poses.size());
    for (auto pose : path.poses) {
        if (pose.header.frame_id.empty()) pose.header = path.header;
        vector_t goal;
        if (!transform(pose, goal)) return false;
        goals.push_back(std::move(goal));
    }
    return true;
}
void TcpReferenceManager::plan(const std::vector<vector_t>& goals,double requested_duration) {
    vector_t start=sample(time_);
    scalar_array_t times{time_}; vector_array_t poses{start};
    double elapsed=0.0;
    for (const auto& goal:goals) {
        const double distance=(goal.head<3>()-start.head<3>()).norm();
        const double angle=quaternion(start).angularDistance(quaternion(goal));
        const double duration=std::max({interval_,requested_duration/goals.size(),
            minimumDuration(distance,max_linear_,max_linear_acc_,max_linear_jerk_),
            minimumDuration(angle,max_angular_,max_angular_acc_,max_angular_jerk_)});
        const size_t count=std::max<size_t>(1,std::ceil(duration/interval_));
        if (count>100000 || poses.size()+count>200000) throw std::invalid_argument("TCP path too long");
        for (size_t i=1;i<=count;++i) { const double t=static_cast<double>(i)/count;
            times.push_back(time_+elapsed+t*duration); poses.push_back(interpolate(start,goal,smooth(t))); }
        elapsed+=duration; start=goal;
    }
    referenceManagerPtr_->setTargetTrajectories(TargetTrajectories(times,poses,vector_array_t(times.size(),vector_t::Zero(input_dim_))));
    times_=std::move(times); poses_=std::move(poses); target_=goals.back();
}
vector_t TcpReferenceManager::sample(double time) const {
    if (times_.empty()||time>=times_.back()) return target_;
    if (time<=times_.front()) return poses_.front();
    const auto upper=std::upper_bound(times_.begin(),times_.end(),time);
    const size_t end=upper-times_.begin(),begin=end-1;
    return interpolate(poses_[begin],poses_[end],(time-times_[begin])/(times_[end]-times_[begin]));
}
void TcpReferenceManager::commitImmediate() {
    times_.clear(); poses_.clear();
    referenceManagerPtr_->setTargetTrajectories(TargetTrajectories({time_},{target_},{vector_t::Zero(input_dim_)}));
}
void TcpReferenceManager::setCurrentObservation(const SystemObservation& observation) {
    std::lock_guard<std::mutex> lock(mutex_);
    const double dt=observation.time-last_time_; time_=observation.time; last_time_=time_;
    if (!accepting_||!twist_active_) return;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-twist_received_).count()>0.2) { twist_active_=false; return; }
    if (dt<=0.0||dt>0.5) return;
    target_.head<3>()+=dt*Eigen::Vector3d(twist_.linear.x,twist_.linear.y,twist_.linear.z);
    const Eigen::Vector3d rotation(twist_.angular.x,twist_.angular.y,twist_.angular.z);
    if (rotation.norm()>1e-12) target_.tail<4>()=(Eigen::Quaterniond(Eigen::AngleAxisd(dt*rotation.norm(),rotation.normalized()))*quaternion(target_)).normalized().coeffs();
    commitImmediate(); publish();
}
void TcpReferenceManager::setAcceptingTargets(bool accept) {
    accepting_=accept; if (!accept) { std::lock_guard<std::mutex> lock(mutex_); twist_active_=false; }
}
void TcpReferenceManager::setCurrentTcpPose(const vector_t& pose,bool update) {
    std::lock_guard<std::mutex> lock(mutex_); current_=target_=pose; times_.clear(); poses_.clear();
    if (update) commitImmediate();
}
void TcpReferenceManager::resetTargetStateCache() {
    std::lock_guard<std::mutex> lock(mutex_); twist_active_=false; last_time_=-1.0; times_.clear(); poses_.clear(); target_=current_;
}
void TcpReferenceManager::publish() const {
    if (!target_pub_) return;
    geometry_msgs::msg::PoseStamped p; p.header.frame_id=base_frame_; p.header.stamp=node_->now(); p.pose=rosPose(target_); target_pub_->publish(p);
}
void TcpReferenceManager::publishCurrentTargetsFromCache() { std::lock_guard<std::mutex> lock(mutex_); publish(); }

void TcpReferenceManager::circle(const arms_ros2_control_msgs::srv::ExecuteCircle::Request& request,
                                arms_ros2_control_msgs::srv::ExecuteCircle::Response& response) {
    const auto& parameters=request.circle_params;
    if (!accepting_) throw std::invalid_argument("TCP tracking is paused");
    if (!parameters.arm_name.empty()&&parameters.arm_name!="tcp"&&parameters.arm_name!=tcp_frame_)
        throw std::invalid_argument("Select the configured TCP frame");
    std::lock_guard<std::mutex> lock(mutex_);
    const vector_t start=sample(time_);
    Eigen::Vector3d center,axis; double angle;
    vector_t end=start;
    if (parameters.use_three_point_method) {
        geometry_msgs::msg::PoseStamped p; p.header.frame_id=parameters.frame_id; p.pose=parameters.midpoint;
        vector_t middle; if (!transform(p,middle)) throw std::invalid_argument("Invalid circle midpoint");
        p.pose=parameters.endpoint; if (!transform(p,end)) throw std::invalid_argument("Invalid circle endpoint");
        const Eigen::Vector3d a=middle.head<3>()-start.head<3>(),b=end.head<3>()-start.head<3>(),cross=a.cross(b);
        if (cross.squaredNorm()<1e-12) throw std::invalid_argument("Circle points are collinear");
        center=start.head<3>()+(b.squaredNorm()*cross.cross(a)+a.squaredNorm()*b.cross(cross))/(2*cross.squaredNorm());
        axis=cross.normalized();
        const Eigen::Vector3d radius=start.head<3>()-center;
        const auto arc=[&](const Eigen::Vector3d& point) { const auto v=(point-center).eval();
            double t=std::atan2(axis.dot(radius.cross(v)),radius.dot(v)); if (t<0) t+=2*std::acos(-1.0); return t; };
        angle=arc(end.head<3>()); if (arc(middle.head<3>())>angle) angle-=2*std::acos(-1.0);
    } else {
        center={parameters.center.x,parameters.center.y,parameters.center.z}; axis={parameters.axis.x,parameters.axis.y,parameters.axis.z}; angle=parameters.rotate_angle;
        if (!center.allFinite()||!axis.allFinite()||axis.norm()<1e-9||!std::isfinite(angle)) throw std::invalid_argument("Invalid circle geometry");
        if (!parameters.frame_id.empty()&&parameters.frame_id!=base_frame_) {
            const auto t=tf_->lookupTransform(base_frame_,parameters.frame_id,tf2::TimePointZero);
            Eigen::Quaterniond r(t.transform.rotation.w,t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z);
            center=r*center+Eigen::Vector3d(t.transform.translation.x,t.transform.translation.y,t.transform.translation.z); axis=r*axis;
        }
        axis.normalize();
    }
    if (std::abs(angle)>20*std::acos(-1.0)) throw std::invalid_argument("Circle exceeds ten revolutions");
    const auto limit=[](double requested,double configured) {
        if (!std::isfinite(requested)||requested<0) throw std::invalid_argument("Invalid circle limit");
        return requested>0?std::min(requested,configured):configured;
    };
    const double velocity=limit(parameters.max_linear_velocity,max_linear_);
    const double acceleration=limit(parameters.max_linear_acceleration,max_linear_acc_);
    const double jerk=limit(parameters.max_linear_jerk,max_linear_jerk_);
    const double angular_velocity=limit(parameters.max_angular_velocity,max_angular_);
    const double angular_acceleration=limit(parameters.max_angular_acceleration,max_angular_acc_);
    const double angular_jerk=limit(parameters.max_angular_jerk,max_angular_jerk_);
    if (!std::isfinite(parameters.duration)||parameters.duration<0) throw std::invalid_argument("Invalid circle duration");
    const Eigen::Vector3d radial=start.head<3>()-center;
    const double radius=radial.cross(axis).norm(),sweep=std::abs(angle);
    const double duration=std::max({interval_,parameters.duration>0?parameters.duration:duration_,
        minimumDuration(radius*sweep,velocity,acceleration,jerk),
        minimumDuration(quaternion(start).angularDistance(quaternion(end)),angular_velocity,angular_acceleration,angular_jerk),
        std::sqrt(radius*(5.774*sweep+1.875*1.875*sweep*sweep)/acceleration),
        std::cbrt(radius*(60*sweep+3*1.875*5.774*sweep*sweep+std::pow(1.875*sweep,3))/jerk)});
    const size_t count=std::max<size_t>(2,std::ceil(duration/interval_));
    if (count>200000) throw std::invalid_argument("TCP circle too long");
    scalar_array_t times; vector_array_t poses; times.reserve(count+1);poses.reserve(count+1);
    for (size_t i=0;i<=count;++i) { const double fraction=static_cast<double>(i)/count,progress=smooth(fraction);
        vector_t p=interpolate(start,end,progress);
        p.head<3>()=center+Eigen::AngleAxisd(angle*progress,axis)*radial;
        times.push_back(time_+duration*fraction);poses.push_back(std::move(p)); }
    referenceManagerPtr_->setTargetTrajectories(TargetTrajectories(times,poses,vector_array_t(times.size(),vector_t::Zero(input_dim_))));
    twist_active_=false;times_=std::move(times);poses_=std::move(poses);target_=poses_.back();publish();
    response.success=true; response.estimated_duration=times_.back()-times_.front(); response.message="TCP circle accepted";
}
} // namespace zephyr::mpc_wholebody
