// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mpc_controller/visualization/mpc_visualizer.h"

using namespace zephyr::mpc_visualization;
using namespace std::chrono_literals;

namespace {
// Analytic one-joint fixture: no robot meshes, solver or hardware required.
class TcpVisualizer : public testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override {
        node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
            "tcp_visualizer", "/mpc_visualizer_tests", rclcpp::NodeOptions().use_global_arguments(false));
        pinocchio::Model model;
        const auto joint = model.addJoint(0, pinocchio::JointModelRZ(), pinocchio::SE3::Identity(), "joint1");
        model.addFrame(pinocchio::Frame("tcp", joint, 0,
            pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 0, 0)), pinocchio::OP_FRAME));
        mpc_visualizer_config config;
        config.base_frame = "base";
        config.visualization_frame = "world";
        config.tcp_ee_frame = "tcp";
        config.prediction_visualization_enabled = false;
        visualizer = std::make_unique<mpc_visualizer>(node, ocs2::PinocchioInterface(model), config);
        visualizer->initialize();
        node->configure();
        node->activate();
        executor.add_node(node->get_node_base_interface());
    }
    void TearDown() override {
        executor.remove_node(node->get_node_base_interface());
        visualizer.reset();
        node.reset();
    }
    template<class Predicate>
    bool spinUntil(Predicate done) {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(1ms);
        }
        return done();
    }
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::unique_ptr<mpc_visualizer> visualizer;
};
}

TEST_F(TcpVisualizer, TcpFkMatchesAnalyticPoseAndIndependentRecordingFk) {
    ocs2::vector_t state(1);
    state << std::acos(-1.0) / 2;
    const auto pose = visualizer->computeEndEffectorPose(state);
    ASSERT_EQ(pose.size(), 7);
    EXPECT_NEAR(pose(0), 0, 1e-12);
    EXPECT_NEAR(pose(1), 1, 1e-12);
    EXPECT_NEAR(pose(2), 0, 1e-12);
    EXPECT_NEAR(pose(3), 0, 1e-12);
    EXPECT_NEAR(pose(4), 0, 1e-12);
    EXPECT_NEAR(pose(5), std::sqrt(0.5), 1e-12);
    EXPECT_NEAR(pose(6), std::sqrt(0.5), 1e-12);
    const auto recorded = visualizer->computeRecordedEndEffectorPose(ocs2::vector_t::Zero(1));
    EXPECT_NEAR(recorded(0), 1, 1e-12);
    EXPECT_NEAR(recorded(1), 0, 1e-12);
    EXPECT_TRUE(visualizer->computeEndEffectorPose(state).isApprox(pose, 1e-12));
}

TEST_F(TcpVisualizer, PublishesOnlyTcpPoseWithFrameAndThrottlePreserved) {
    const std::string topic = "/mpc_visualizer_tests/tcp_current_pose";
    std::vector<geometry_msgs::msg::PoseStamped> received;
    auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 10, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose) { received.push_back(*pose); });
    ASSERT_TRUE(spinUntil([&] { return subscription->get_publisher_count() == 1; }));
    const auto topics = node->get_node_graph_interface()->get_publisher_names_and_types_by_node(
        node->get_name(), node->get_namespace());
    EXPECT_EQ(topics.count(topic), 1u);
    EXPECT_EQ(topics.count("/mpc_visualizer_tests/body_current_pose"), 0u);
    EXPECT_EQ(topics.count("/mpc_visualizer_tests/head_current_pose"), 0u);

    const auto pose = visualizer->computeEndEffectorPose(ocs2::vector_t::Zero(1));
    visualizer->setPosePublishPeriod(0.02);
    visualizer->publishTcpPose(rclcpp::Time(1, 0, RCL_ROS_TIME), pose);
    ASSERT_TRUE(spinUntil([&] { return !received.empty(); }));
    EXPECT_EQ(received.front().header.frame_id, "world");
    EXPECT_DOUBLE_EQ(received.front().pose.position.x, 1);
    EXPECT_DOUBLE_EQ(received.front().pose.orientation.w, 1);
    visualizer->publishTcpPose(rclcpp::Time(1, 10000000, RCL_ROS_TIME), pose);  // suppressed at 10 ms
    visualizer->publishTcpPose(rclcpp::Time(1, 30000000, RCL_ROS_TIME), pose);  // published at 30 ms
    ASSERT_TRUE(spinUntil([&] { return received.size() >= 2; }));
    ASSERT_EQ(received.size(), 2u);
    EXPECT_EQ(received.back().header.stamp.nanosec, 30000000u);
}
