// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
//
// Pinocchio + optional self-collision visualization for in-process OCS2 controllers.
//
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <pinocchio/multibody/geometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace zephyr::mpc_visualization {
/** Immutable, worker-owned export of the full solver horizon, including its ROS times. */
struct TimedPrediction {
    ocs2::scalar_array_t times;
    ocs2::vector_array_t states;
    uint64_t epoch{0};
    std::chrono::steady_clock::time_point captured_at;

    bool valid() const {
        if (times.size() < 2 || times.size() != states.size() ||
            states.front().size() == 0 || !(times.back() > times.front())) return false;
        for (size_t i = 0; i < times.size(); ++i) {
            if (!std::isfinite(times[i]) || (i && times[i] < times[i - 1]) ||
                states[i].size() != states.front().size() || !states[i].allFinite()) return false;
        }
        return true;
    }

    /** Interpolate absolute time. Never extrapolate beyond the optimized horizon.
     * Call only on a validated snapshot; upper_bound handles repeated event times.
     */
    bool sample(double time, ocs2::vector_t& state) const {
        if (!std::isfinite(time) || times.empty() || time < times.front() || time > times.back()) return false;
        const auto upper = std::upper_bound(times.begin(), times.end(), time);
        if (upper == times.end()) { state = states.back(); return true; }
        const size_t right = static_cast<size_t>(upper - times.begin());
        const size_t left = right - 1;
        const double weight = (time - times[left]) / (times[right] - times[left]);
        state = (1.0 - weight) * states[left] + weight * states[right];
        return true;
    }

    bool fresh(double now, double max_age) const {
        const double wall_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - captured_at).count();
        return !times.empty() && std::isfinite(now) && now >= times.front() &&
            now < times.back() && now - times.front() <= max_age && wall_age <= max_age;
    }
};
using namespace ocs2;

/** Frame names and optional geometry — no MobileManipulatorInterface dependency. */
struct mpc_visualizer_config {
    std::string urdf_file;
    std::string base_frame;
    /** Frame in which FK poses are expressed (odom for an OCS2 mobile-base state). */
    std::string visualization_frame;
    /** The first three state entries are mobile-base x, y, and yaw. */
    bool mobile_base_enabled{false};
    bool prediction_visualization_enabled{true};
    double ghost_interval{0.2};
    size_t ghost_count{5};
    double marker_lifetime{0.15};
    double control_period{0.02};
    std::string tcp_ee_frame;
    /// Body/torso frame for reference FK (wheel humanoid). Empty: callers use identity pose for body reference block.
    std::string body_frame;
    /// Optional head frame; publish current pose only when present in the model.
    std::string head_frame;
    std::optional<PinocchioGeometryInterface> pinocchio_geometry;
    scalar_t self_collision_activation_distance{0};
};

class mpc_visualizer {
public:
    mpc_visualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                            PinocchioInterface pinocchio_interface, mpc_visualizer_config config);

    void initialize();

    /** Crop to the future and anchor paths at measured TCP; called only on viz worker. */
    void updatePrediction(const vector_t& measured_state, const TimedPrediction& prediction, double render_time);
    void publishEndEffectorTrajectory(const rclcpp::Time& time);

    /**
     * Self-collision markers via GeometryInterfaceVisualization::publishDistances.
     * Used by the async viz thread only. Returns true if FCL ran (lastMinDistance updated).
     */
    bool publishSelfCollisionVisualization(const vector_t& state) const;
    /** Min distance from the last successful publishDistances under the mutex. */
    scalar_t getLastMinDistance() const;

    /** One FK into RT Data; extract TCP / body / head (7-vectors). No ROS publish. */
    struct EndEffectorPoses {
        vector_t tcp = vector_t::Zero(7);
        vector_t body = vector_t::Zero(7);
        vector_t head = vector_t::Zero(7);
    };
    EndEffectorPoses computeEndEffectorPoses(const vector_t& state) const;

    /** Throttled PoseStamped publish (default 50 Hz). Control FK must use computeEndEffectorPoses. */
    void publishEndEffectorPoses(const rclcpp::Time& time, const EndEffectorPoses& poses) const;
    void publishEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;
    void setPosePublishPeriod(double period_sec);

    vector_t computeEndEffectorPose(const vector_t& state) const;
    /** Worker thread only; never touches the RT FK cache. */
    vector_t computeRecordedEndEffectorPose(const vector_t& state) const;
    /** 7-dim pose (x,y,z, qx,qy,qz,qw) of body_frame in base; identity at origin if body_frame empty or missing. */
    vector_t computeBodyFramePose(const vector_t& state) const;

    void clearTrajectoryHistory();

private:
    void forwardKinematicsInto(pinocchio::Data& data, const vector_t& state) const;
    vector_t extractFramePose7(const pinocchio::Data& data, pinocchio::FrameIndex frame_id) const;
    static void fillPoseStamped(geometry_msgs::msg::PoseStamped& msg, const rclcpp::Time& time,
                                const std::string& frame_id, const vector_t& pose7);
    void publishPose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
                     const rclcpp::Time& time, const vector_t& pose7) const;
    void appendPredictedRobotMarkers(visualization_msgs::msg::MarkerArray& marker_array);

    visualization_msgs::msg::Marker createTrajectoryLineMarker(const std::vector<vector_t>& poses,
                                                                 const std::array<double, 3>& color, double line_width,
                                                                 const std::string& namespace_name) const;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    PinocchioInterface pinocchio_interface_;
    mpc_visualizer_config config_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tcp_end_effector_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr body_frame_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr head_frame_pose_publisher_;

    std::unique_ptr<GeometryInterfaceVisualization> geometry_visualization_;
    /** Guards geometry_visualization_ (viz-thread publishDistances / getLastMinDistance). */
    mutable std::mutex self_collision_mutex_;

    std::vector<vector_t> tcp_arm_trajectory_history_;
    std::vector<vector_t> tcp_arm_next_step_;
    std::vector<vector_t> predicted_robot_states_;
    size_t published_ghost_pose_count_{0};

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_publisher_;

    double trajectory_line_width_{0.005};

    /** Dedicated Data so RT and viz threads never share a pinocchio cache. */
    mutable pinocchio::Data rt_data_;
    mutable pinocchio::Data viz_data_;
    pinocchio::GeometryModel visual_geometry_model_;
    std::unique_ptr<pinocchio::GeometryData> visual_geometry_data_;
    pinocchio::FrameIndex tcp_ee_frame_id_{0};
    pinocchio::FrameIndex body_frame_id_{0};
    pinocchio::FrameIndex head_frame_id_{0};
    bool tcp_ee_frame_valid_{false};
    bool body_frame_valid_{false};
    bool head_frame_valid_{false};

    double pose_publish_period_sec_{0.02};
    mutable rclcpp::Time last_pose_publish_time_{0, 0, RCL_ROS_TIME};
};

} // namespace zephyr::mpc_visualization
