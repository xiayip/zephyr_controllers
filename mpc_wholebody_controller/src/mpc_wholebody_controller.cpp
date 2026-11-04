// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Created for OCS2 Arm Controller
//

#include "mpc_wholebody_controller/mpc_wholebody_controller.h"
#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/Cholesky>
#include <chrono>
#include <random>
#include <stdexcept>
#include <cctype>
#include <limits>

#include <ocs2_controller_common/mpc/MpcExecutionParams.hpp>
#include <pinocchio/algorithm/model.hpp>

namespace zephyr::mpc_wholebody
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

    controller_interface::InterfaceConfiguration mpc_wholebody_controller::command_interface_configuration() const
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

    controller_interface::InterfaceConfiguration mpc_wholebody_controller::state_interface_configuration() const
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

    controller_interface::return_type mpc_wholebody_controller::update(
        const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        ctrl_comp_->beginRtCycle();
        ctrl_comp_->applyRtLoopSchedulingOnce();
        bool valid;
        {
            auto scope = ctrl_comp_->rtScopeObs();
            valid = ctrl_comp_->updateObservation(time);
        }
        if (execution_->consumeFailure()) pause_requested_ = true;
        if (valid && !pause_requested_.load(std::memory_order_acquire) &&
            ctrl_comp_->interface_->isSelfCollisionEnabled() && ctrl_comp_->isSelfCollisionActive())
        {
            pause_requested_ = true;
            RCLCPP_ERROR(get_node()->get_logger(), "MPC paused by self-collision check; resume with ~/pause after resolving it");
        }
        if (!valid || pause_requested_.load(std::memory_order_acquire))
        {
            execution_->pause();
            publishTrackingStatus(false);
        }
        else publishTrackingStatus(execution_->tick(time));
        if (valid)
        {
            auto scope = ctrl_comp_->rtScopeViz();
            ctrl_comp_->maybeRequestVisualizationUpdate(time);
        }
        ctrl_comp_->endRtCycle(tracking_active_ ? "ocs2" : "paused");
        ctrl_interfaces_.recordCommandMotion(period.seconds());
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn mpc_wholebody_controller::on_init()
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
            ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);
            const auto& info = ctrl_comp_->interface_->getManipulatorModelInfo();
            const auto model = makeTcpKinematicsModel(
                ctrl_comp_->interface_->getPinocchioInterface().getModel(), joint_names_, info.baseFrame);
            kinematics_ = std::make_shared<TcpKinematics>(model, joint_names_, info.eeFrame);
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

    controller_interface::CallbackReturn mpc_wholebody_controller::on_configure(const rclcpp_lifecycle::State&)
    {
        tracking_status_publisher_ = get_node()->create_publisher<std_msgs::msg::Int32>(
            "/fsm_state", rclcpp::QoS(1).transient_local());
        // Periodic status also supports clients using volatile QoS.
        tracking_status_timer_ = get_node()->create_wall_timer(std::chrono::seconds(1),
            [this] { publishStatusMessage(); });
        pause_service_ = get_node()->create_service<std_srvs::srv::SetBool>("~/pause",
            [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
                pause_requested_.store(request->data, std::memory_order_release);
                response->success = true;
                response->message = request->data ? "Pause requested" : "Resume requested";
            });
        kinematics_service_ = get_node()->create_service<arms_ros2_control_msgs::srv::KinematicsService>(
            "kinematics_service", std::bind(&mpc_wholebody_controller::handleKinematicsService, this,
                                            std::placeholders::_1, std::placeholders::_2));
        ctrl_comp_->pose_reference_manager_->setAcceptingTargets(false);
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn mpc_wholebody_controller::on_activate(
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
        ctrl_interfaces_.recordCommandMotion(0.0);
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && ctrl_interfaces_.pd_gains_.size() >= 2)
        {
            for (auto& interface : ctrl_interfaces_.joint_kp_command_interface_)
                std::ignore = interface.get().set_value(ctrl_interfaces_.pd_gains_[0]);
            for (auto& interface : ctrl_interfaces_.joint_kd_command_interface_)
                std::ignore = interface.get().set_value(ctrl_interfaces_.pd_gains_[1]);
        }
        pause_requested_ = false;
        tracking_active_ = false;
        execution_->start();
        publishStatusMessage();
        RCLCPP_INFO(get_node()->get_logger(), "OCS2 starts automatically once feedback and the first policy are ready");
        return CallbackReturn::SUCCESS;
    }

    void mpc_wholebody_controller::stopExecution()
    {
        pause_requested_ = true;
        if (execution_) execution_->stop();
        if (ctrl_comp_)
        {
            ctrl_comp_->stopVisualizationThread();
            ctrl_comp_->pose_reference_manager_->setAcceptingTargets(false);
            ctrl_comp_->stopMobileBase();
        }
        tracking_active_ = false;
    }
    controller_interface::CallbackReturn mpc_wholebody_controller::on_deactivate(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        ctrl_comp_->holdLastSentPositions();
        publishStatusMessage();
        release_interfaces();
        ctrl_interfaces_.clear();
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn mpc_wholebody_controller::on_cleanup(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        tracking_status_timer_.reset();
        pause_service_.reset();
        kinematics_service_.reset();
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn mpc_wholebody_controller::on_error(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn mpc_wholebody_controller::on_shutdown(const rclcpp_lifecycle::State&)
    {
        stopExecution();
        return CallbackReturn::SUCCESS;
    }

    void mpc_wholebody_controller::handleKinematicsService(
        const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Request> request,
        const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Response> response)
    {
        if (!kinematics_)
        {
            response->success = false;
            response->message = "Kinematics not initialized";
            return;
        }
        try
        {
            kinematics_->handle(*request, *response);
        }
        catch (const std::exception& error)
        {
            response->success = false;
            response->message = std::string("TCP kinematics failed: ") + error.what();
        }
    }

    void mpc_wholebody_controller::publishTrackingStatus(bool active)
    {
        const bool changed = tracking_active_.exchange(active, std::memory_order_acq_rel) != active;
        if (changed)
        {
            ctrl_comp_->pose_reference_manager_->setAcceptingTargets(active);
            publishStatusMessage();
        }
    }
    void mpc_wholebody_controller::publishStatusMessage()
    {
        if (!tracking_status_publisher_ || !get_node()->get_node_base_interface()->get_context()->is_valid()) return;
        std_msgs::msg::Int32 status;
        status.data = tracking_active_.load(std::memory_order_acquire) ? 3 : 2;
        tracking_status_publisher_->publish(status);
    }

// Single-TCP FK/IK service implementation.

TcpKinematics::TcpKinematics(pinocchio::Model model,const std::vector<std::string>& joints,std::string tcp_frame)
    : model_(std::move(model)),data_(model_),tcp_frame_(std::move(tcp_frame)),frame_(model_.getFrameId(tcp_frame_)) {
    if (frame_>=model_.frames.size()||model_.nq!=model_.nv) throw std::invalid_argument("Invalid TCP kinematics model");
    for (const auto& name:joints) {
        const auto id=model_.getJointId(name);
        if (id>=model_.joints.size()||model_.joints[id].nq()!=1) throw std::invalid_argument("Invalid TCP joint: "+name);
        indices_.push_back(model_.joints[id].idx_q());
    }
}
void TcpKinematics::handle(const arms_ros2_control_msgs::srv::KinematicsService::Request& request,
                           arms_ros2_control_msgs::srv::KinematicsService::Response& response) {
    const auto began=std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    response.success=false;
    if (!request.arm_type.empty()&&request.arm_type!="tcp"&&request.arm_type!=tcp_frame_)
        throw std::invalid_argument("Select arm_type="+tcp_frame_);
    if (request.operation_type!="fk"&&request.operation_type!="ik") throw std::invalid_argument("operation_type must be fk or ik");
    if (request.joint_angles.size()!=indices_.size()) throw std::invalid_argument("Provide one angle per configured arm joint");
    Eigen::VectorXd q=pinocchio::neutral(model_);
    for (size_t i=0;i<indices_.size();++i) {
        if (!std::isfinite(request.joint_angles[i])) throw std::invalid_argument("Nonfinite joint angle");
        q(indices_[i])=request.joint_angles[i];
    }
    const auto clamp=[&](Eigen::VectorXd value) { return value.cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit).eval(); };
    const auto writePose=[&](const pinocchio::SE3& transform) {
        geometry_msgs::msg::Pose p; const Eigen::Quaterniond rotation(transform.rotation());
        p.position.x=transform.translation().x(); p.position.y=transform.translation().y(); p.position.z=transform.translation().z();
        p.orientation.x=rotation.x(); p.orientation.y=rotation.y(); p.orientation.z=rotation.z(); p.orientation.w=rotation.w();
        response.result_poses={p};
    };
    if (request.operation_type=="fk") {
        pinocchio::framesForwardKinematics(model_,data_,q); writePose(data_.oMf[frame_]);
        response.is_out_of_range=(q.array()<model_.lowerPositionLimit.array()).any()||(q.array()>model_.upperPositionLimit.array()).any();
        response.success=true; response.used_solver="FK"; response.message="TCP FK complete";
    } else {
        if (request.target_poses.size()!=1) throw std::invalid_argument("Provide exactly one TCP target pose");
        const auto& p=request.target_poses.front();
        Eigen::Quaterniond orientation(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
        const Eigen::Vector3d position(p.position.x,p.position.y,p.position.z);
        if (!position.allFinite()||!orientation.coeffs().allFinite()||orientation.norm()<1e-9) throw std::invalid_argument("Invalid IK target");
        const pinocchio::SE3 target(orientation.normalized().toRotationMatrix(),position);
        std::string solver=request.solver_type.empty()?"auto":request.solver_type;
        std::transform(solver.begin(),solver.end(),solver.begin(),[](unsigned char c){return std::tolower(c);});
        if (solver!="dls"&&solver!="bfgs"&&solver!="auto") throw std::invalid_argument("solver_type must be dls, bfgs or auto");
        const int iterations=request.max_iterations>0?std::min(request.max_iterations,10000):300;
        const double tolerance=request.tolerance>0?request.tolerance:1e-4;
        const double damping=request.dls_damping>0?request.dls_damping:0.01;
        if (!std::isfinite(tolerance)||!std::isfinite(damping)) throw std::invalid_argument("Invalid IK parameters");
        Eigen::VectorXd best=clamp(q); double best_error=std::numeric_limits<double>::infinity();
        const auto errorAt=[&](const Eigen::VectorXd& value) {
            pinocchio::framesForwardKinematics(model_,data_,value);
            return pinocchio::log6(data_.oMf[frame_].actInv(target)).toVector().eval();
        };
        std::mt19937 generator(0);
        const int attempts=request.enable_random_restart?4:1;
        for (int attempt=0;attempt<attempts&&!response.success;++attempt) {
            q=best;
            if (attempt) for (int i:indices_) q(i)=std::uniform_real_distribution<double>(model_.lowerPositionLimit(i),model_.upperPositionLimit(i))(generator);
            for (const auto& method: solver=="auto"?std::vector<std::string>{"dls","bfgs"}:std::vector<std::string>{solver}) {
                Eigen::MatrixXd inverse_hessian=Eigen::MatrixXd::Identity(model_.nv,model_.nv);
                Eigen::VectorXd old_gradient,old_q;
                for (int iteration=0;iteration<iterations;++iteration) {
                    const Eigen::VectorXd error=errorAt(q); const double norm=error.norm();
                    if (norm<best_error) { best_error=norm; best=q; }
                    ++response.iterations; response.used_solver=method;
                    if (norm<=tolerance) { response.success=true; break; }
                    Eigen::Matrix<double,6,Eigen::Dynamic> jacobian(6,model_.nv); jacobian.setZero();
                    pinocchio::computeFrameJacobian(model_,data_,q,frame_,pinocchio::LOCAL,jacobian);
                    Eigen::Matrix<double,6,6> log_jacobian;
                    pinocchio::Jlog6(data_.oMf[frame_].actInv(target).inverse(),log_jacobian);
                    jacobian=-log_jacobian*jacobian;
                    const Eigen::VectorXd gradient=jacobian.transpose()*error;
                    Eigen::VectorXd step;
                    if (method=="dls") {
                        Eigen::Matrix<double,6,6> normal=jacobian*jacobian.transpose(); normal.diagonal().array()+=damping*damping;
                        step=-jacobian.transpose()*normal.ldlt().solve(error);
                    } else {
                        if (old_gradient.size()) {
                            const Eigen::VectorXd s=q-old_q,y=gradient-old_gradient; const double sy=s.dot(y);
                            if (sy>1e-12) {
                                const Eigen::MatrixXd v=Eigen::MatrixXd::Identity(model_.nv,model_.nv)-s*y.transpose()/sy;
                                inverse_hessian=v*inverse_hessian*v.transpose()+s*s.transpose()/sy;
                            }
                        }
                        step=-inverse_hessian*gradient;
                        if (step.dot(gradient)>=0.0) { inverse_hessian.setIdentity(); step=-gradient; }
                        old_q=q; old_gradient=gradient;
                    }
                    if (!step.allFinite()) break;
                    const double maximum=step.cwiseAbs().maxCoeff(); if (maximum>0.25) step*=0.25/maximum;
                    bool improved=false;
                    for (double scale=1.0;scale>=1e-5;scale*=0.5) {
                        Eigen::VectorXd candidate=clamp(q+scale*step);
                        if (errorAt(candidate).squaredNorm()<error.squaredNorm()) { q=std::move(candidate); improved=true; break; }
                    }
                    if (!improved) break;
                }
                if (response.success) break;
                q=best;
            }
        }
        q=best; response.final_error=best_error;
        for (int i:indices_) response.result_joint_angles.push_back(q(i));
        pinocchio::framesForwardKinematics(model_,data_,q); writePose(data_.oMf[frame_]);
        response.message=response.success?"TCP IK converged":"TCP IK did not converge within joint limits";
        response.additional_info="TCP="+tcp_frame_+"; failure does not prove geometric unreachability";
    }
    response.computation_time_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
}

// MPC worker scheduling belongs to this controller.

MpcExecution::MpcExecution(std::shared_ptr<CtrlComponent> component, double frequency)
    : component_(std::move(component))
{
    const auto node = component_->getNode();
    const auto params = ocs2::controller_common::computeMpcExecutionParams(
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
void MpcExecution::pause()
{
    auto expected = Job::Requested;
    job_.compare_exchange_strong(expected, Job::Idle, std::memory_order_acq_rel);
    if (initialized_) component_->clearTrajectoryVisualization();
    initialized_ = false;
    component_->holdLastSentPositions();
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
        component_->pose_reference_manager_->resetTargetStateCache();
        component_->resetMpc();
        component_->publishCachedCurrentTargets();
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
    component_->evaluatePolicy(time);
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

PLUGINLIB_EXPORT_CLASS(zephyr::mpc_wholebody::mpc_wholebody_controller, controller_interface::ControllerInterface)
