// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Created by fiveages on 8/21/25.
//

#include "mpc_controller/visualization/mpc_visualizer.h"
#include <fstream>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>
#include <std_msgs/msg/string.hpp>

namespace zephyr::mpc_visualization {
using namespace ocs2;
    mpc_visualizer::mpc_visualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                                     PinocchioInterface pinocchio_interface, mpc_visualizer_config config)
        : node_(node),
          pinocchio_interface_(std::move(pinocchio_interface)),
          config_(std::move(config)),
          rt_data_(pinocchio_interface_.getModel()),
          viz_data_(pinocchio_interface_.getModel())
    {
        if (config_.visualization_frame.empty()) config_.visualization_frame = config_.base_frame;
        const auto& model = pinocchio_interface_.getModel();
        tcp_ee_frame_id_ = model.getFrameId(config_.tcp_ee_frame);
        tcp_ee_frame_valid_ = tcp_ee_frame_id_ < static_cast<pinocchio::FrameIndex>(model.nframes);
    }

    void mpc_visualizer::initialize()
    {
        // Create publisher
        if (config_.prediction_visualization_enabled)
        {
            trajectory_marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                "trajectory_markers", 1);
        }

        // Create end effector pose publisher
        tcp_end_effector_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "tcp_current_pose", 1);

        // Create robot description publisher and publish URDF
        if (!config_.urdf_file.empty())
        {
            robot_description_publisher_ = node_->create_publisher<std_msgs::msg::String>(
                "/ocs2_robot_description", rclcpp::QoS(1).transient_local());

            // Read and publish URDF file (ifstream does not throw on open failure by default).
            std::ifstream urdf_stream(config_.urdf_file);
            if (urdf_stream.is_open())
            {
                std::string urdf_content((std::istreambuf_iterator(urdf_stream)),
                                        std::istreambuf_iterator<char>());
                urdf_stream.close();

                std_msgs::msg::String urdf_msg;
                urdf_msg.data = urdf_content;
                robot_description_publisher_->publish(urdf_msg);

                RCLCPP_INFO(node_->get_logger(), "Published OCS2 robot description from: %s", config_.urdf_file.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Failed to open URDF file: %s", config_.urdf_file.c_str());
            }
        }

        try
        {
            auto geometry_opt = std::move(config_.pinocchio_geometry);
            if (geometry_opt)
            {
                if (config_.self_collision_enabled && geometry_opt->getNumCollisionPairs() == 0)
                    throw std::runtime_error("Self-collision monitoring is enabled but no collision pairs are configured");
                const scalar_t activationDistance = config_.self_collision_activation_distance;
                geometry_visualization_ = std::make_unique<GeometryInterfaceVisualization>(
                    pinocchio_interface_, std::move(*geometry_opt), config_.base_frame, activationDistance);
                RCLCPP_INFO(node_->get_logger(),
                            "Self-collision visualization initialized (activation distance: %.3f m)", activationDistance);
            }
            else
            {
                if (config_.self_collision_enabled)
                    throw std::runtime_error("Self-collision monitoring is enabled but geometry is unavailable");
                RCLCPP_INFO(node_->get_logger(),
                            "No Pinocchio geometry provided, self-collision visualization disabled");
            }
        }
        catch (const std::exception& e)
        {
            geometry_visualization_.reset();
            if (config_.self_collision_enabled) throw;
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to initialize self-collision visualization: %s, self-collision visualization disabled",
                        e.what());
        }

        if (config_.prediction_visualization_enabled && !config_.urdf_file.empty())
        {
            try
            {
                pinocchio::urdf::buildGeom(
                    pinocchio_interface_.getModel(), config_.urdf_file, pinocchio::VISUAL,
                    visual_geometry_model_, pinocchio::rosPaths());
                if (!visual_geometry_model_.geometryObjects.empty())
                {
                    visual_geometry_data_ = std::make_unique<pinocchio::GeometryData>(
                        visual_geometry_model_);
                    RCLCPP_INFO(node_->get_logger(),
                                "Loaded %zu visual geometries for MPC predicted robot ghosts",
                                visual_geometry_model_.geometryObjects.size());
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "URDF has no visual geometries; MPC robot ghosts are disabled");
                }
            }
            catch (const std::exception& e)
            {
                visual_geometry_model_.geometryObjects.clear();
                visual_geometry_data_.reset();
                RCLCPP_WARN(node_->get_logger(),
                            "Failed to load URDF visual geometries for MPC robot ghosts: %s",
                            e.what());
            }
        }
        RCLCPP_INFO(node_->get_logger(), "mpc_visualizer initialization completed");
        RCLCPP_INFO(node_->get_logger(), "Self-collision visualization: %s",
                    geometry_visualization_ ? "enabled" : "disabled");
    }

    void mpc_visualizer::updatePrediction(
        const vector_t& measured_state, const TimedPrediction& prediction, double render_time)
    {
        tcp_arm_trajectory_history_.clear();
        tcp_arm_next_step_.clear();
        predicted_robot_states_.clear();
        if (!tcp_ee_frame_valid_ || prediction.states.front().size() != measured_state.size()) return;

        forwardKinematicsInto(viz_data_, measured_state);
        const vector_t current_pose = extractFramePose7(viz_data_, tcp_ee_frame_id_);
        tcp_arm_trajectory_history_.push_back(current_pose);
        tcp_arm_next_step_.push_back(current_pose);

        // Uniform absolute-time sampling bounds FK/rendering cost and drops ALL past
        // knots. No nearest-position heuristic and no extrapolated tail to the target.
        constexpr size_t path_samples = 64;
        const double remaining = prediction.times.back() - render_time;
        vector_t state;
        for (size_t i = 1; i <= path_samples; ++i)
        {
            const double sample_time = render_time + remaining * static_cast<double>(i) / path_samples;
            if (prediction.sample(sample_time, state))
            {
                forwardKinematicsInto(viz_data_, state);
                tcp_arm_trajectory_history_.push_back(extractFramePose7(viz_data_, tcp_ee_frame_id_));
            }
        }
        const double next_time = std::min(render_time + config_.control_period, prediction.times.back());
        if (prediction.sample(next_time, state))
        {
            forwardKinematicsInto(viz_data_, state);
            tcp_arm_next_step_.push_back(extractFramePose7(viz_data_, tcp_ee_frame_id_));
        }

        // Pose spacing is independent of the publishing rate. Every ghost refers
        // to now + k * interval, never to the origin of an old MPC solve.
        for (size_t ghost = 1; ghost <= config_.ghost_count; ++ghost)
        {
            const double sample_time = render_time + static_cast<double>(ghost) * config_.ghost_interval;
            if (!prediction.sample(sample_time, state)) break;
            // Preserve the user's removal of motion thresholds; suppress roundoff only.
            if ((state - measured_state).cwiseAbs().maxCoeff() > 1e-9)
                predicted_robot_states_.push_back(state);
        }
    }

    void mpc_visualizer::publishEndEffectorTrajectory(const rclcpp::Time& time)
    {
        if (!config_.prediction_visualization_enabled || !trajectory_marker_publisher_)
        {
            return;
        }
        visualization_msgs::msg::MarkerArray marker_array;

        // Create TCP trajectory marker
        if (!tcp_arm_trajectory_history_.empty())
        {
            // Full MPC horizon and the immediate next step use distinct colors.
            marker_array.markers.push_back(createTrajectoryLineMarker(
                tcp_arm_trajectory_history_, {{0.0, 0.4470, 0.7410}}, trajectory_line_width_, "MPC Full Horizon"));
            marker_array.markers.push_back(createTrajectoryLineMarker(
                tcp_arm_next_step_, {{1.0, 0.35, 0.0}}, trajectory_line_width_ * 2.0, "MPC Next Step"));
        }
        else
        {
            for (const auto* ns : {"MPC Full Horizon", "MPC Next Step"})
            {
                visualization_msgs::msg::Marker marker;
                marker.ns = ns;
                marker.id = 0;
                marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_array.markers.push_back(std::move(marker));
            }
        }
        appendPredictedRobotMarkers(marker_array);

        // Set marker header information
        for (auto& marker : marker_array.markers)
        {
            marker.header.stamp = time;
            marker.header.frame_id = config_.visualization_frame;
            if (marker.action == visualization_msgs::msg::Marker::ADD)
                marker.lifetime = rclcpp::Duration::from_seconds(config_.marker_lifetime);
        }

        // Keep only the newest display frame in the publisher queue.
        trajectory_marker_publisher_->publish(marker_array);
    }

    void mpc_visualizer::appendPredictedRobotMarkers(
        visualization_msgs::msg::MarkerArray& marker_array)
    {
        if (!visual_geometry_data_)
        {
            return;
        }
        const auto& model = pinocchio_interface_.getModel();
        const auto& geometry_objects = visual_geometry_model_.geometryObjects;
        // MarkerArray ADD updates existing IDs but does not remove IDs that are
        // no longer emitted. Delete trailing ghosts explicitly when a new
        // prediction contains fewer poses, otherwise they remain frozen in RViz.
        for (size_t ghost = predicted_robot_states_.size();
             ghost < published_ghost_pose_count_; ++ghost)
        {
            for (size_t geometry_index = 0; geometry_index < geometry_objects.size(); ++geometry_index)
            {
                if (geometry_objects[geometry_index].meshPath.empty())
                {
                    continue;
                }
                visualization_msgs::msg::Marker marker;
                marker.ns = "MPC Predicted Robot";
                marker.id = static_cast<int>(ghost * geometry_objects.size() + geometry_index);
                marker.action = visualization_msgs::msg::Marker::DELETE;
                marker_array.markers.push_back(std::move(marker));
            }
        }

        for (size_t ghost = 0; ghost < predicted_robot_states_.size(); ++ghost)
        {
            forwardKinematicsInto(viz_data_, predicted_robot_states_[ghost]);
            pinocchio::updateGeometryPlacements(
                model, viz_data_, visual_geometry_model_, *visual_geometry_data_);

            for (size_t geometry_index = 0; geometry_index < geometry_objects.size(); ++geometry_index)
            {
                const auto& geometry = geometry_objects[geometry_index];
                // Mesh visuals preserve the robot's link silhouettes while allowing
                // RViz to tint each future pose with a translucent overlay.
                if (geometry.meshPath.empty())
                {
                    continue;
                }

                const auto& placement = visual_geometry_data_->oMg[geometry_index];
                const Eigen::Quaterniond orientation(placement.rotation());

                visualization_msgs::msg::Marker marker;
                marker.ns = "MPC Predicted Robot";
                marker.id = static_cast<int>(ghost * geometry_objects.size() + geometry_index);
                marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                // Pinocchio resolves package meshes to absolute filesystem paths.
                // RViz's resource retriever requires a URI, even for local files.
                marker.mesh_resource = geometry.meshPath.front() == '/'
                                           ? "file://" + geometry.meshPath
                                           : geometry.meshPath;
                marker.mesh_use_embedded_materials = false;
                marker.pose.position.x = placement.translation().x();
                marker.pose.position.y = placement.translation().y();
                marker.pose.position.z = placement.translation().z();
                marker.pose.orientation.x = orientation.x();
                marker.pose.orientation.y = orientation.y();
                marker.pose.orientation.z = orientation.z();
                marker.pose.orientation.w = orientation.w();
                marker.scale.x = geometry.meshScale.x();
                marker.scale.y = geometry.meshScale.y();
                marker.scale.z = geometry.meshScale.z();
                marker.color.r = 0.12f;
                marker.color.g = 0.78f;
                marker.color.b = 1.0f;
                marker.color.a = static_cast<float>(0.12 + 0.018 * ghost);
                marker_array.markers.push_back(std::move(marker));
            }
        }
        published_ghost_pose_count_ = predicted_robot_states_.size();
    }

    bool mpc_visualizer::publishSelfCollisionVisualization(const vector_t& state) const
    {
        if (!geometry_visualization_)
        {
            return false;
        }
        const auto& model = pinocchio_interface_.getModel();
        if (static_cast<size_t>(state.size()) != static_cast<size_t>(model.nq))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Self-collision viz skipped: state size %ld != model.nq %u (check controller joints vs OCS2)",
                                 static_cast<long>(state.size()), model.nq);
            return false;
        }
        try
        {
            std::lock_guard<std::mutex> lock(self_collision_mutex_);
            geometry_visualization_->publishDistances(state);
            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Self-collision check failed; fix the URDF/collision configuration before resuming: %s",
                                 e.what());
            return false;
        }
    }

    scalar_t mpc_visualizer::getLastMinDistance() const
    {
        if (!geometry_visualization_)
        {
            return std::numeric_limits<scalar_t>::max();
        }
        std::lock_guard<std::mutex> lock(self_collision_mutex_);
        return geometry_visualization_->getLastMinDistance();
    }

    void mpc_visualizer::forwardKinematicsInto(pinocchio::Data& data, const vector_t& state) const
    {
        const auto& model = pinocchio_interface_.getModel();
        pinocchio::forwardKinematics(model, data, state);
        pinocchio::updateFramePlacements(model, data);
    }

    vector_t mpc_visualizer::extractFramePose7(const pinocchio::Data& data,
                                                        const pinocchio::FrameIndex frame_id) const
    {
        vector_t pose = vector_t::Zero(7);
        pose(6) = 1.0;
        if (frame_id >= static_cast<pinocchio::FrameIndex>(data.oMf.size()))
        {
            return pose;
        }
        const auto& frame_placement = data.oMf[frame_id];
        pose.head<3>() = frame_placement.translation();
        const Eigen::Quaterniond quat(frame_placement.rotation());
        pose(3) = quat.x();
        pose(4) = quat.y();
        pose(5) = quat.z();
        pose(6) = quat.w();
        return pose;
    }

    void mpc_visualizer::fillPoseStamped(geometry_msgs::msg::PoseStamped& msg, const rclcpp::Time& time,
                                                  const std::string& frame_id, const vector_t& pose7)
    {
        msg.header.stamp = time;
        msg.header.frame_id = frame_id;
        msg.pose.position.x = pose7(0);
        msg.pose.position.y = pose7(1);
        msg.pose.position.z = pose7(2);
        msg.pose.orientation.x = pose7(3);
        msg.pose.orientation.y = pose7(4);
        msg.pose.orientation.z = pose7(5);
        msg.pose.orientation.w = pose7(6);
    }

    void mpc_visualizer::setPosePublishPeriod(const double period_sec)
    {
        pose_publish_period_sec_ = std::max(0.0, period_sec);
    }

    vector_t mpc_visualizer::computeEndEffectorPose(const vector_t& state) const
    {
        if (tcp_ee_frame_valid_)
        {
            forwardKinematicsInto(rt_data_, state);
            return extractFramePose7(rt_data_, tcp_ee_frame_id_);
        }

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "TCP frame '%s' not in model (nframes=%d)",
                             config_.tcp_ee_frame.c_str(), pinocchio_interface_.getModel().nframes);
        vector_t pose = vector_t::Zero(7);
        pose(6) = 1.0;
        return pose;
    }

    void mpc_visualizer::publishTcpPose(const rclcpp::Time& time, const vector_t& pose) const
    {
        if (!tcp_end_effector_pose_publisher_) return;
        if (pose_publish_period_sec_ > 0.0 && last_pose_publish_time_.nanoseconds() != 0 &&
            (time - last_pose_publish_time_).seconds() < pose_publish_period_sec_)
        {
            return;
        }
        last_pose_publish_time_ = time;
        geometry_msgs::msg::PoseStamped msg;
        fillPoseStamped(msg, time, config_.visualization_frame, pose);
        tcp_end_effector_pose_publisher_->publish(msg);
    }

    vector_t mpc_visualizer::computeRecordedEndEffectorPose(const vector_t& state) const
    {
        if (!tcp_ee_frame_valid_)
        {
            vector_t pose = vector_t::Zero(7);
            pose(6) = 1.0;
            return pose;
        }
        forwardKinematicsInto(viz_data_, state);
        return extractFramePose7(viz_data_, tcp_ee_frame_id_);
    }

    visualization_msgs::msg::Marker mpc_visualizer::createTrajectoryLineMarker(
        const std::vector<vector_t>& poses,
        const std::array<double, 3>& color,
        const double line_width,
        const std::string& namespace_name) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = namespace_name;
        marker.scale.x = line_width;
        marker.color = ros_msg_helpers::getColor(color, 0.8);
        marker.points.reserve(poses.size());
        for (const auto& pose : poses)
        {
            geometry_msgs::msg::Point point;
            point.x = pose(0);
            point.y = pose(1);
            point.z = pose(2);
            marker.points.push_back(point);
        }
        return marker;
    }

    void mpc_visualizer::clearTrajectoryHistory()
    {
        if (tcp_arm_trajectory_history_.empty() && tcp_arm_next_step_.empty() &&
            predicted_robot_states_.empty() && published_ghost_pose_count_ == 0) return;
        tcp_arm_trajectory_history_.clear();
        tcp_arm_next_step_.clear();
        predicted_robot_states_.clear();

        // Publish empty trajectory visualization to clear previous display
        publishEndEffectorTrajectory(node_->now());
    }
}
