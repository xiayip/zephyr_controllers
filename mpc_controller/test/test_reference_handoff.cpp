// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <condition_variable>
#include <future>
#include <limits>
#include <thread>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "mpc_controller/control/TcpReferenceManager.h"

using namespace zephyr::mpc;
using namespace std::chrono_literals;
namespace {
// Simulate a slow non-RT reference commit while TcpReferenceManager holds its
// trajectory mutex. No controller, solver or hardware is started by these tests.
class BlockingReference final : public ocs2::ReferenceManager {
public:
    void setTargetTrajectories(ocs2::TargetTrajectories&& target) override {
        {
            std::unique_lock<std::mutex> lock(mutex);
            if (block) {
                entered = true;
                condition.notify_all();
                condition.wait(lock, [&] { return !block; });
            }
        }
        ReferenceManager::setTargetTrajectories(std::move(target));
    }
    std::mutex mutex;
    std::condition_variable condition;
    bool block{false}, entered{false};
};

class ReferenceHandoff : public testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
    void SetUp() override {
        node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
            "reference", "/mpc_control_tests", rclcpp::NodeOptions().use_global_arguments(false));
        wrapped = std::make_shared<BlockingReference>();
        reference = std::make_unique<TcpReferenceManager>(wrapped, "base", "tcp", 6);
        reference->subscribe(node);
        executor.add_node(node->get_node_base_interface());
        pose = ocs2::vector_t::Zero(7);
        pose << 0.1, 0.2, 0.3, 0, 0, 0, 1;
    }
    void TearDown() override {
        executor.remove_node(node->get_node_base_interface());
        reference.reset();
        node.reset();
    }
    bool spinUntilReset() {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (!reference->resetReady() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(1ms);
        }
        return reference->resetReady();
    }
    void spinFor(std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(1ms);
        }
    }
    ocs2::vector_t committedTarget() {
        wrapped->preSolverRun(10, 11, ocs2::vector_t::Zero(6));
        return wrapped->getTargetTrajectories().stateTrajectory.front();
    }
    template<class Message>
    void send(const char* topic, const Message& message) {
        auto pub = node->create_publisher<Message>(topic, 1);
        pub->on_activate();
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (!pub->get_subscription_count() && std::chrono::steady_clock::now() < deadline) {
            executor.spin_some();
            std::this_thread::sleep_for(1ms);
        }
        ASSERT_GT(pub->get_subscription_count(), 0u);
        pub->publish(message);
        // Include rejected/paused requests: they deliberately produce no ack.
        const auto delivered = std::chrono::steady_clock::now() + 100ms;
        while (std::chrono::steady_clock::now() < delivered) {
            executor.spin_some();
            std::this_thread::sleep_for(1ms);
        }
        wrapped->preSolverRun(10, 11, ocs2::vector_t::Zero(6));
    }
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::shared_ptr<BlockingReference> wrapped;
    std::unique_ptr<TcpReferenceManager> reference;
    ocs2::vector_t pose;
};
}

TEST_F(ReferenceHandoff, ResetNeedsExecutorAcknowledgementAndReplacesTarget) {
    EXPECT_FALSE(reference->resetReady());
    ASSERT_TRUE(reference->requestReset(pose, 10));
    EXPECT_FALSE(reference->resetReady());
    ASSERT_TRUE(spinUntilReset());
    EXPECT_EQ(committedTarget(), pose);
    pose(2) += 0.2;
    ASSERT_TRUE(reference->requestReset(pose, 11));
    EXPECT_FALSE(reference->resetReady());
    ASSERT_TRUE(spinUntilReset());
    EXPECT_EQ(committedTarget(), pose);
}

TEST_F(ReferenceHandoff, ObservationUpdateDoesNotCommitTargetsOnControlThread) {
    ASSERT_TRUE(reference->requestReset(pose, 10));
    ASSERT_TRUE(spinUntilReset());
    ocs2::SystemObservation observation;
    observation.time = 11;
    observation.state = ocs2::vector_t::Constant(6, 42);
    reference->setCurrentObservation(observation);
    EXPECT_EQ(committedTarget(), pose);
    EXPECT_TRUE((observation.state.array() == 42).all());
}

TEST_F(ReferenceHandoff, ReferenceSettingsContainNoLegacyNoOpParameters) {
    EXPECT_TRUE(node->has_parameter("movel_duration"));
    EXPECT_TRUE(node->has_parameter("movel_max_linear_velocity"));
    EXPECT_TRUE(node->has_parameter("movel_max_linear_acceleration"));
    EXPECT_TRUE(node->has_parameter("movel_max_linear_jerk"));
    for (const auto* name : {"joint_speed_threshold", "movel_trajectory_duration",
                             "movel_auto_extend_duration"}) {
        EXPECT_FALSE(node->has_parameter(name)) << name;
    }
}

TEST_F(ReferenceHandoff, BusyReferenceCallbackCannotBlockControlOrPause) {
    wrapped->block = true;
    ASSERT_TRUE(reference->requestReset(pose, 10));
    std::thread worker([&] { executor.spin(); });
    bool entered;
    {
        std::unique_lock<std::mutex> lock(wrapped->mutex);
        entered = wrapped->condition.wait_for(lock, 2s, [&] { return wrapped->entered; });
    }
    EXPECT_TRUE(entered);
    auto control = std::async(std::launch::async, [&] {
        ocs2::SystemObservation observation;
        observation.time = 11;
        reference->setCurrentObservation(observation);
        reference->setAcceptingTargets(false);
        return reference->requestReset(pose, 11);
    });
    const auto result = control.wait_for(200ms);
    // Always release and join, even if the nonblocking assertion fails.
    {
        std::lock_guard<std::mutex> lock(wrapped->mutex);
        wrapped->block = false;
    }
    wrapped->condition.notify_all();
    executor.cancel();
    worker.join();
    EXPECT_EQ(result, std::future_status::ready);
    EXPECT_TRUE(control.get());
    EXPECT_FALSE(reference->consumeFailure());
    ASSERT_TRUE(spinUntilReset());
    EXPECT_EQ(committedTarget(), pose);
}

TEST_F(ReferenceHandoff, StandardPathTopicInterpolatesAndIgnoresTargetsWhenPaused) {
    ASSERT_TRUE(reference->requestReset(pose, 10));
    ASSERT_TRUE(spinUntilReset());
    reference->setAcceptingTargets(true);
    nav_msgs::msg::Path path;
    path.header.frame_id = "base";
    geometry_msgs::msg::PoseStamped goal;
    goal.pose.position.x = 0.1;
    goal.pose.position.y = 0.2;
    goal.pose.position.z = 0.31;
    goal.pose.orientation.w = 1;
    path.poses.push_back(goal);
    send("target_path", path);
    const auto& target = wrapped->getTargetTrajectories();
    ASSERT_GT(target.stateTrajectory.size(), 1u);
    EXPECT_NEAR(target.stateTrajectory.back()(2), 0.31, 1e-12);
    EXPECT_GE(target.timeTrajectory.back() - target.timeTrajectory.front(), 2.0);
    reference->setAcceptingTargets(false);
    path.poses.front().pose.position.z = 0.32;
    send("target_path", path);
    EXPECT_NEAR(wrapped->getTargetTrajectories().stateTrajectory.back()(2), 0.31, 1e-12);
}

TEST_F(ReferenceHandoff, StandardPathRejectsEmptyInvalidAndUntransformablePoses) {
    ASSERT_TRUE(reference->requestReset(pose, 10));
    ASSERT_TRUE(spinUntilReset());
    reference->setAcceptingTargets(true);
    nav_msgs::msg::Path path;
    path.header.frame_id = "base";
    send("target_path", path);
    EXPECT_EQ(committedTarget(), pose);
    path.poses.emplace_back();  // Invalid zero quaternion.
    send("target_path", path);
    EXPECT_EQ(committedTarget(), pose);
    path.poses.front().pose.orientation.w = 1;
    path.poses.front().pose.position.z = std::numeric_limits<double>::quiet_NaN();
    send("target_path", path);
    EXPECT_EQ(committedTarget(), pose);
    path.poses.front().pose.position.z = 0.5;
    path.poses.front().header.frame_id = "missing_frame";  // Overrides path frame.
    send("target_path", path);
    EXPECT_EQ(committedTarget(), pose);
}

TEST_F(ReferenceHandoff, PoseStreamUsedByRvizIsImmediateAndSessionGated) {
    ASSERT_TRUE(reference->requestReset(pose, 10));
    ASSERT_TRUE(spinUntilReset());
    reference->setAcceptingTargets(true);
    geometry_msgs::msg::Pose target;
    target.position.x = 0.1;
    target.position.y = 0.2;
    target.position.z = 0.31;
    target.orientation.w = 2;  // Normalized by the reference manager.
    send("tcp_target", target);
    EXPECT_DOUBLE_EQ(committedTarget()(2), 0.31);
    EXPECT_DOUBLE_EQ(committedTarget()(6), 1.0);
    EXPECT_EQ(wrapped->getTargetTrajectories().stateTrajectory.size(), 1u);
    reference->setAcceptingTargets(false);
    target.position.z = 0.5;
    send("tcp_target", target);
    EXPECT_DOUBLE_EQ(committedTarget()(2), 0.31);
}

TEST_F(ReferenceHandoff, DoesNotAdvertiseCustomMotionServices) {
    for (const auto& [name, types] : node->get_service_names_and_types()) {
        EXPECT_NE(name, "/mpc_control_tests/execute_path");
        EXPECT_NE(name, "/mpc_control_tests/execute_tcp_circle");
        for (const auto& type : types)
            EXPECT_EQ(type.find("zephyr_msgs/"), std::string::npos);
    }
}

TEST_F(ReferenceHandoff, ProfileHoldReplacesTrajectoryAndBlocksEveryInputAcrossPauseResume) {
    ocs2::SystemObservation observation;
    observation.time = 10;
    reference->setCurrentObservation(observation);
    ASSERT_TRUE(reference->requestReset(pose, 10));
    ASSERT_TRUE(spinUntilReset());
    reference->setAcceptingTargets(true);
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "base";
    target.pose.position.x = 0.1;
    target.pose.position.y = 0.2;
    target.pose.position.z = 0.9;
    target.pose.orientation.w = 1;
    send("tcp_target/stamped", target);
    ASSERT_GT(wrapped->getTargetTrajectories().stateTrajectory.size(), 1u);

    auto measured = pose;
    measured(2) = 0.32;
    ASSERT_TRUE(reference->beginProfileChange(measured));
    EXPECT_EQ(committedTarget(), measured);
    EXPECT_EQ(wrapped->getTargetTrajectories().stateTrajectory.size(), 1u);
    EXPECT_FALSE(reference->claimMotion());
    reference->setAcceptingTargets(false);
    reference->setAcceptingTargets(true);  // Lifecycle resume cannot release the profile gate.
    EXPECT_FALSE(reference->claimMotion());

    send("tcp_target", target.pose);
    EXPECT_EQ(committedTarget(), measured);
    send("tcp_target/stamped", target);
    EXPECT_EQ(committedTarget(), measured);
    geometry_msgs::msg::TwistStamped delta;
    delta.twist.linear.z = 0.2;
    send("tcp_target/relative", delta);
    EXPECT_EQ(committedTarget(), measured);
    send("tcp_target/twist", delta.twist);
    observation.time = 10.05;
    reference->setCurrentObservation(observation);
    spinFor(30ms);
    EXPECT_EQ(committedTarget(), measured);
    nav_msgs::msg::Path path;
    path.header.frame_id = "base";
    path.poses.push_back(target);
    send("target_path", path);
    EXPECT_EQ(committedTarget(), measured);

    measured(2) = 0.33;  // Retrying an unacknowledged change refreshes its measured hold.
    ASSERT_TRUE(reference->beginProfileChange(measured));
    EXPECT_EQ(committedTarget(), measured);
    EXPECT_FALSE(reference->claimMotion());
}

TEST_F(ReferenceHandoff, ProfileChangeCannotStealMotionAndInvalidHoldDoesNotReserveInputs) {
    ASSERT_TRUE(reference->requestReset(pose, 10));
    ASSERT_TRUE(spinUntilReset());
    reference->setAcceptingTargets(true);
    ASSERT_TRUE(reference->claimMotion());
    EXPECT_FALSE(reference->beginProfileChange(pose));
    EXPECT_TRUE(reference->motionSessionValid());
    reference->releaseMotion();

    auto invalid = pose;
    invalid.tail<4>().setZero();
    EXPECT_FALSE(reference->beginProfileChange(invalid));
    invalid = pose;
    invalid(0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(reference->beginProfileChange(invalid));
    EXPECT_FALSE(reference->beginProfileChange(ocs2::vector_t::Zero(6)));
    ASSERT_TRUE(reference->claimMotion());
    reference->releaseMotion();
    ASSERT_TRUE(reference->beginProfileChange(pose));
    EXPECT_FALSE(reference->claimMotion());
    reference->endProfileChange();
    EXPECT_TRUE(reference->claimMotion());
    reference->releaseMotion();
}

TEST_F(ReferenceHandoff, ProfileReleaseNeverReplaysCachedTwistAndAcceptsFreshTargets) {
    ocs2::SystemObservation observation;
    observation.time = 10;
    reference->setCurrentObservation(observation);
    ASSERT_TRUE(reference->requestReset(pose, 10));
    ASSERT_TRUE(spinUntilReset());
    reference->setAcceptingTargets(true);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.1;
    send("tcp_target/twist", twist);
    observation.time = 10.05;
    reference->setCurrentObservation(observation);
    spinFor(20ms);
    ASSERT_GT(committedTarget()(0), pose(0));  // The cached stream was active before the handoff.

    auto measured = pose;
    measured(2) = 0.34;
    ASSERT_TRUE(reference->beginProfileChange(measured));
    reference->endProfileChange();
    observation.time = 10.10;
    reference->setCurrentObservation(observation);
    spinFor(20ms);  // Less than the 200 ms twist watchdog: cancellation, not expiry, must hold it.
    EXPECT_EQ(committedTarget(), measured);

    geometry_msgs::msg::Pose fresh;
    fresh.position.x = 0.1;
    fresh.position.y = 0.2;
    fresh.position.z = 0.36;
    fresh.orientation.w = 1;
    send("tcp_target", fresh);
    EXPECT_DOUBLE_EQ(committedTarget()(2), 0.36);
    EXPECT_TRUE(reference->claimMotion());
    reference->releaseMotion();
}
