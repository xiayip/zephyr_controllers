// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <condition_variable>
#include <future>
#include <thread>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include "mpc_controller/control/CtrlComponent.h"

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
    ocs2::vector_t committedTarget() {
        wrapped->preSolverRun(10, 11, ocs2::vector_t::Zero(6));
        return wrapped->getTargetTrajectories().stateTrajectory.front();
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
