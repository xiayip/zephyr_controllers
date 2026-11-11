// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Created for OCS2 Arm Controller
//

#include "mpc_controller/mpc_controller.h"
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <chrono>
#include <stdexcept>

#include <mpc_controller/runtime/MpcExecutionParams.hpp>
#include <pinocchio/algorithm/model.hpp>

namespace zephyr::mpc
{
    using namespace ocs2;
    using namespace ocs2::mobile_manipulator;
    namespace
    {
        // The upstream arm IK/FK helper accepts arm joints only. Lock chassis
        // coordinates in its own model and express its result in baseFrame.
        // The MPC and gravity models retain their original state dimensions.
        pinocchio::Model makeTcpKinematicsModel(
            const pinocchio::Model& full_model,
            const std::vector<std::string>& arm_joints, const std::string& base_frame)
        {
            std::vector<pinocchio::JointIndex> locked;
            for (pinocchio::JointIndex i = 1; i < full_model.joints.size(); ++i)
            {
                if (std::find(arm_joints.begin(), arm_joints.end(), full_model.names[i]) == arm_joints.end())
                {
                    locked.push_back(i);
                }
            }
            auto source = full_model;
            // A fixed camera joint may share its name with its BODY frame.
            // Pinocchio's reduction looks up frame names without a type filter.
            // Give only ambiguous non-BODY frames private names in this copy.
            for (pinocchio::FrameIndex i = 0; i < source.frames.size(); ++i)
            {
                auto& frame = source.frames[i];
                if (frame.type == pinocchio::BODY)
                {
                    continue;
                }
                for (pinocchio::FrameIndex j = 0; j < source.frames.size(); ++j)
                {
                    if (j != i && source.frames[j].name == frame.name)
                    {
                        frame.name = "__tcp_frame_" + std::to_string(i) + "_" + frame.name;
                        break;
                    }
                }
            }
            auto model = locked.empty() ? source :
                pinocchio::buildReducedModel(source, locked, pinocchio::neutral(source));
            if (model.nq != static_cast<int>(arm_joints.size()))
            {
                throw std::runtime_error("TCP kinematics model does not match configured arm joints");
            }
            const auto frame_id = model.getFrameId(base_frame);
            if (frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes))
            {
                throw std::runtime_error("TCP kinematics base frame is absent from model");
            }
            pinocchio::Data data(model);
            pinocchio::framesForwardKinematics(model, data, pinocchio::neutral(model));
            const pinocchio::SE3 to_base = data.oMf[frame_id].inverse();
            for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i)
            {
                if (model.parents[i] == 0)
                {
                    model.jointPlacements[i] = to_base * model.jointPlacements[i];
                }
            }
            for (auto& frame : model.frames)
            {
                if (frame.parentJoint == 0)
                {
                    frame.placement = to_base * frame.placement;
                }
            }
            return model;
        }
    }
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration mpc_controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        const std::string prefix = command_prefix_.empty() ? "" : command_prefix_ + "/";
        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                conf.names.push_back(prefix + joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration mpc_controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type mpc_controller::update(
        const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
    {
        ctrl_comp_->beginRtCycle();
        ctrl_comp_->applyRtLoopSchedulingOnce();
        bool valid;
        {
            auto scope = ctrl_comp_->rtScopeObs();
            valid = ctrl_comp_->updateObservation(time);
        }
        if (execution_->consumeFailure()) {
            pause_requested_ = true;
            motion_fault_ = MotionStatus::SOLVER_FAILED;
        }
        if (ctrl_comp_->pose_reference_manager_->consumeFailure()) {
            pause_requested_ = true;
            motion_fault_ = MotionStatus::REFERENCE_FAILED;
        }
        const auto collision = ctrl_comp_->collisionStatus();
        if (valid && !pause_requested_.load(std::memory_order_acquire) &&
            collision != CollisionStatus::Safe && collision != CollisionStatus::Pending)
        {
            pause_requested_ = true;
            motion_fault_ = MotionStatus::COLLISION_BLOCKED;
            const char* reason = collision == CollisionStatus::Collision ? "collision" :
                collision == CollisionStatus::Invalid ? "collision check failed" : "collision check stale";
            RCLCPP_ERROR(get_node()->get_logger(), "MPC paused: %s; resolve before explicit ~/pause resume", reason);
        }
        const auto stop_token = motion_runtime_->stop_request.load();
        const bool action_stop = stop_token != motion_runtime_->stop_release.load();
        bool held = false;
        if (!valid || collision == CollisionStatus::Pending || pause_requested_.load(std::memory_order_acquire) || action_stop)
        {
            held = execution_->pause();
            publishTrackingStatus(false);
        }
        else
        {
            const bool tracking = execution_->tick(time);
            // Output validation faults hold immediately and latch in this cycle.
            if (execution_->consumeFailure()) {
                pause_requested_ = true;
                motion_fault_ = MotionStatus::POLICY_FAILED;
            }
            publishTrackingStatus(tracking && !pause_requested_.load(std::memory_order_acquire));
        }
        if (valid)
        {
            auto scope = ctrl_comp_->rtScopeViz();
            ctrl_comp_->maybeRequestVisualizationUpdate();
        }
        MotionSample sample;
        sample.pose = ctrl_comp_->measuredTcpPose();
        sample.stamp_ns = time.nanoseconds();
        sample.reference_time = time.seconds();
        sample.received_ns = motionSteadyNs();
        sample.valid = valid;
        sample.stop_ack = action_stop && held ? stop_token : 0;
        sample.reason = motion_fault_.load();
        if (action_stop) sample.state = MotionStatus::STOPPING;
        else if (!valid) {
            sample.state = MotionStatus::FAULT;
            sample.reason = MotionStatus::FEEDBACK_UNAVAILABLE;
        } else if (sample.reason != MotionStatus::NONE) sample.state = MotionStatus::FAULT;
        else if (pause_requested_) {
            sample.state = MotionStatus::PAUSED;
            sample.reason = MotionStatus::PAUSE_REQUESTED;
        } else sample.state = tracking_active_ && ctrl_comp_->policy_active_ ? MotionStatus::IDLE : MotionStatus::INITIALIZING;
        motion_runtime_->samples.tryWrite(sample);
        ctrl_comp_->endRtCycle(tracking_active_ ? "ocs2" : "paused");
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn mpc_controller::on_init()
    {
        try
        {
            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
            ctrl_interfaces_.pd_gains_ = auto_declare<std::vector<double>>("pd_gains", ctrl_interfaces_.pd_gains_);
            auto_declare<int>("mpc_frequency", 0);
            KinematicsOptions ik_options;
            ik_options.group = auto_declare<std::string>("kinematics.group", "arm");
            ik_options.solver = auto_declare<std::string>("kinematics.solver", "auto");
            ik_options.max_iterations = auto_declare<int>("kinematics.max_iterations", 300);
            ik_options.tolerance = auto_declare<double>("kinematics.tolerance", 1e-4);
            ik_options.damping = auto_declare<double>("kinematics.damping", 0.01);
            ik_options.timeout = auto_declare<double>("kinematics.timeout", 0.1);
            ik_options.random_restart = auto_declare<bool>("kinematics.random_restart", false);
            ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);
            const auto& info = ctrl_comp_->interface_->getManipulatorModelInfo();
            const auto model = makeTcpKinematicsModel(
                ctrl_comp_->interface_->getPinocchioInterface().getModel(), joint_names_, info.baseFrame);
            kinematics_ = std::make_shared<TcpKinematics>(model, joint_names_, info.eeFrame, info.baseFrame, ik_options);
            execution_ = std::make_unique<MpcExecution>(ctrl_comp_, ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Direct OCS2 controller initialized; TCP=%s", info.eeFrame.c_str());
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "MPC initialization failed: %s", error.what());
            return CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn mpc_controller::on_configure(const rclcpp_lifecycle::State&)
    {
        if (!motion_server_) motion_server_ = std::make_unique<MotionActionServer>(
            get_node(), ctrl_comp_->pose_reference_manager_, motion_runtime_);
        tracking_status_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32>(
            "/fsm_state", rclcpp::QoS(1).transient_local());
        // Periodic status also supports clients using volatile QoS.
        tracking_status_timer_ = get_node()->create_wall_timer(std::chrono::seconds(1),
            [this] { publishStatusMessage(); });
        pause_service_ = get_node()->create_service<std_srvs::srv::SetBool>("~/pause",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                pause_requested_.store(request->data, std::memory_order_release);
                if (!request->data) motion_fault_ = MotionStatus::NONE;
                response->success = true;
                response->message = request->data ? "Pause requested" : "Resume requested";
            });
        fk_service_ = get_node()->create_service<moveit_msgs::srv::GetPositionFK>("~/compute_fk",
            [this](moveit_msgs::srv::GetPositionFK::Request::SharedPtr request,
                   moveit_msgs::srv::GetPositionFK::Response::SharedPtr response) {
                try { kinematics_->fk(*request, *response); }
                catch (const std::exception& error) {
                    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
                    response->error_code.message = error.what();
                }
            });
        ik_service_ = get_node()->create_service<moveit_msgs::srv::GetPositionIK>("~/compute_ik",
            [this](moveit_msgs::srv::GetPositionIK::Request::SharedPtr request,
                   moveit_msgs::srv::GetPositionIK::Response::SharedPtr response) {
                try { kinematics_->ik(*request, *response); }
                catch (const std::exception& error) {
                    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
                    response->error_code.message = error.what();
                }
            });
        ctrl_comp_->pose_reference_manager_->setAcceptingTargets(false);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn mpc_controller::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();
        ctrl_comp_->stopMobileBase();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                interface_name = interface_name.substr(pos + 1);
            }
            command_interface_map_.at(interface_name)->push_back(interface);
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            state_interface_map_.at(interface.get_interface_name())->push_back(interface);
        }

        // Auto-detect control mode based on available interfaces
        ctrl_interfaces_.detectAndSetControlMode();

        // Log detected control mode
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Mixed control mode enabled - detected position, velocity, effort interfaces"
                        " (kp/kd optional)");
        }
        else
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Position control mode enabled - standard position control interfaces detected");
        }

        std::vector<double> initial_positions;
        initial_positions.reserve(joint_names_.size());
        if (ctrl_interfaces_.joint_position_state_interface_.size() != joint_names_.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate: joint position feedback is incomplete");
            return CallbackReturn::ERROR;
        }
        if (ctrl_interfaces_.joint_position_command_interface_.size() != joint_names_.size() ||
            (!ctrl_interfaces_.joint_velocity_command_interface_.empty() &&
             ctrl_interfaces_.joint_velocity_command_interface_.size() != joint_names_.size()))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate: incomplete position/velocity commands");
            return CallbackReturn::ERROR;
        }
        for (const auto& interface : ctrl_interfaces_.joint_position_state_interface_)
        {
            const auto value = interface.get().get_optional();
            if (!value || !std::isfinite(*value))
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Cannot activate: joint position feedback is invalid");
                return CallbackReturn::ERROR;
            }
            initial_positions.push_back(*value);
        }
        ctrl_interfaces_.last_sent_joint_positions_ = std::move(initial_positions);
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && ctrl_interfaces_.pd_gains_.size() >= 2)
        {
            for (auto& interface : ctrl_interfaces_.joint_kp_command_interface_)
                std::ignore = interface.get().set_value(ctrl_interfaces_.pd_gains_[0]);
            for (auto& interface : ctrl_interfaces_.joint_kd_command_interface_)
                std::ignore = interface.get().set_value(ctrl_interfaces_.pd_gains_[1]);
        }
        pause_requested_ = false;
        motion_fault_ = MotionStatus::NONE;
        tracking_active_ = false;
        execution_->start();
        motion_runtime_->activate();
        publishStatusMessage();
        RCLCPP_INFO(get_node()->get_logger(), "OCS2 starts automatically once feedback and the first policy are ready");
        return CallbackReturn::SUCCESS;
    }

    void mpc_controller::stopExecution()
    {
        const auto activation_epoch = motion_runtime_->activation_epoch.load();
        pause_requested_ = true;
        if (ctrl_comp_)
        {
            ctrl_comp_->pose_reference_manager_->setAcceptingTargets(false);
            ctrl_comp_->holdLastSentPositions();
        }
        if (execution_) execution_->stop();
        if (ctrl_comp_)
        {
            ctrl_comp_->stopVisualizationThread();
            ctrl_comp_->pose_reference_manager_->setAcceptingTargets(false);
            if (ctrl_comp_->holdForDeactivation())
                motion_runtime_->stopped_epoch.store(activation_epoch);
            else if (activation_epoch != 0 && motion_runtime_->stopped_epoch.load() != activation_epoch)
                RCLCPP_ERROR(get_node()->get_logger(),
                    "Lifecycle hold was not confirmed; pending motion ownership remains locked");
        }
        tracking_active_ = false;
        motion_runtime_->active = false;
    }
    controller_interface::CallbackReturn mpc_controller::on_deactivate(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        ctrl_comp_->holdLastSentPositions();
        publishStatusMessage();
        release_interfaces();
        ctrl_interfaces_.clear();
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn mpc_controller::on_cleanup(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        tracking_status_timer_.reset();
        pause_service_.reset();
        fk_service_.reset();
        ik_service_.reset();
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn mpc_controller::on_error(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn mpc_controller::on_shutdown(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        return CallbackReturn::SUCCESS;
    }

    void mpc_controller::publishTrackingStatus(bool active)
    {
        const bool changed = tracking_active_.exchange(active, std::memory_order_acq_rel) != active;
        if (changed)
        {
            ctrl_comp_->pose_reference_manager_->setAcceptingTargets(active);
            publishStatusMessage();
        }
    }
    void mpc_controller::publishStatusMessage()
    {
        if (!tracking_status_publisher_ || !get_node()->get_node_base_interface()->get_context()->is_valid()) return;
        std_msgs::msg::Int32 status;
        status.data = tracking_active_.load(std::memory_order_acquire) ? 3 : 2;
        tracking_status_publisher_->publish(status);
    }

// MPC worker scheduling belongs to this controller.

MpcExecution::MpcExecution(std::shared_ptr<CtrlComponent> component, double frequency)
    : component_(std::move(component))
{
    const auto node = component_->getNode();
    const auto params = runtime::computeMpcExecutionParams(
        frequency, node->get_parameter("mpc_frequency").as_int(), node->get_logger());
    period_sec_ = params.mpc_period_sec;
    sleep_ms_ = params.thread_sleep_ms;
    component_->setMpcPeriodSec(period_sec_);
}
MpcExecution::~MpcExecution() { stop(); }
void MpcExecution::start()
{
    stop();
    initialized_ = false;
    reference_reset_requested_ = false;
    failed_ = false;
    job_ = Job::Idle;
    running_ = true;
    worker_ = std::thread(&MpcExecution::workerLoop, this);
    component_->startVisualizationThread();
}
void MpcExecution::stop()
{
    running_ = false;
    if (worker_.joinable()) worker_.join();
    job_ = Job::Idle;
    initialized_ = false;
}
bool MpcExecution::pause()
{
    auto expected = Job::Requested;
    job_.compare_exchange_strong(expected, Job::Idle, std::memory_order_acq_rel);
    if (initialized_) component_->clearTrajectoryVisualization();
    initialized_ = false;
    reference_reset_requested_ = false;
    return component_->holdLastSentPositions();
}
bool MpcExecution::requestSolve()
{
    auto expected = Job::Idle;
    return job_.compare_exchange_strong(expected, Job::Requested, std::memory_order_acq_rel);
}
bool MpcExecution::tick(const rclcpp::Time& time)
{
    if (!initialized_)
    {
        // An in-flight solve must finish before reset touches the solver.
        if (job_.load(std::memory_order_acquire) != Job::Idle)
        {
            component_->holdLastSentPositions();
            return false;
        }
        if (!reference_reset_requested_)
        {
            reference_reset_requested_ = component_->requestReferenceReset();
            component_->holdLastSentPositions();
            return false;
        }
        if (!component_->pose_reference_manager_->resetReady())
        {
            component_->holdLastSentPositions();
            return false;
        }
        reference_reset_requested_ = false;
        component_->resetMpc();
        component_->requestVisualizationUpdate();
        last_request_time_ = time;
        initialized_ = true;
        requestSolve();
    }
    if (!component_->initialPolicyReceived())
    {
        requestSolve();
        component_->holdLastSentPositions();
        return true;
    }
    if ((time - last_request_time_).seconds() >= period_sec_ && requestSolve())
        last_request_time_ = time;
    if (!component_->evaluatePolicy(time))
    {
        pause();
        failed_.store(true, std::memory_order_release);
        return false;
    }
    return true;
}
void MpcExecution::workerLoop()
{
    component_->applyWorkerThreadScheduling("MPC");
    while (running_.load(std::memory_order_acquire))
    {
        auto expected = Job::Requested;
        if (!job_.compare_exchange_strong(expected, Job::Running, std::memory_order_acq_rel))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms_));
            continue;
        }
        try { component_->advanceMpc(); }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(component_->getNode()->get_logger(), "MPC solve failed: %s", error.what());
            failed_.store(true, std::memory_order_release);
        }
        job_.store(Job::Idle, std::memory_order_release);
    }
}
}

PLUGINLIB_EXPORT_CLASS(zephyr::mpc::mpc_controller, controller_interface::ControllerInterface)
