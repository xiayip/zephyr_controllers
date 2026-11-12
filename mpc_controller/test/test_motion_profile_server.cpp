// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <mpc_controller/control/MotionProfileServer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <limits>
#include <thread>

using namespace zephyr::mpc;
using namespace std::chrono_literals;

namespace {
class MotionProfileServerTest : public testing::Test {
 protected:
  using SetProfile = zephyr_msgs::srv::SetMotionProfile;
  using ProfileStatus = zephyr_msgs::msg::MotionProfileStatus;
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "controller", "/profile_test", rclcpp::NodeOptions().use_global_arguments(false));
    observer = std::make_shared<rclcpp::Node>("observer", "/profile_test",
        rclcpp::NodeOptions().use_global_arguments(false));
    wrapped = std::make_shared<ocs2::ReferenceManager>();
    reference = std::make_shared<TcpReferenceManager>(wrapped, "base", "tcp", 6);
    reference->subscribe(node);
    profile = std::make_shared<RuntimeMotionProfile>();
    reference->setRuntimeProfile(profile);
    runtime = std::make_shared<MotionRuntime>();
    runtime->activate();
    sample.valid = true;
    sample.state = MotionStatus::IDLE;
    sample.reference_time = 10;
    sample.profile_revision = 1;
    sample.pose = {0.1, 0.2, 0.3, 0, 0, 0, 1};
    executor.add_node(node->get_node_base_interface());
    executor.add_node(observer);
    ASSERT_TRUE(reference->requestReset(measuredPose(), 10));
    pump(50ms);
    ASSERT_TRUE(reference->resetReady());
    reference->setAcceptingTargets(true);
    server = std::make_unique<MotionProfileServer>(node, reference, profile, runtime);
    node->configure();
    node->activate();
    client = observer->create_client<SetProfile>("/profile_test/controller/set_motion_profile");
    status_subscription = observer->create_subscription<ProfileStatus>(
        "/profile_test/controller/motion_profile_status", 1,
        [this](const ProfileStatus& status) { latest_status = status; });
    ASSERT_TRUE(client->wait_for_service(2s));
    pump(100ms);
  }
  void TearDown() override {
    runtime->active = false;
    pump(60ms);
    client.reset();
    server.reset();
    status_subscription.reset();
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(observer);
    reference.reset();
    node.reset();
    observer.reset();
  }
  ocs2::vector_t measuredPose() const {
    return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(sample.pose.data());
  }
  void refresh() {
    sample.activation_epoch = runtime->activation_epoch.load();
    sample.received_ns = motionSteadyNs();
    sample.stamp_ns = node->now().nanoseconds();
    runtime->samples.tryWrite(sample);
    ocs2::SystemObservation observation;
    observation.time = sample.reference_time;
    reference->setCurrentObservation(observation);
  }
  void pump(std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    do {
      if (emit_feedback) refresh();
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
  }
  std::shared_ptr<SetProfile::Response> request(uint8_t mode = RuntimeProfileConfig::LOCKED,
                                               double tcp_scale = 1.0, double input_scale = 1.0) {
    auto req = std::make_shared<SetProfile::Request>();
    req->profile.base_mode = mode;
    req->profile.tcp_cost_scale = tcp_scale;
    req->profile.input_cost_scale = input_scale;
    auto future = client->async_send_request(req);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (future.wait_for(0ms) != std::future_status::ready && std::chrono::steady_clock::now() < deadline)
      pump(5ms);
    if (future.wait_for(0ms) != std::future_status::ready) {
      ADD_FAILURE() << "Profile response timed out";
      return std::make_shared<SetProfile::Response>();
    }
    return future.get();
  }
  void matchingPolicy(uint64_t revision) {
    const auto requested = profile->requestedConfig();
    sample.profile_revision = revision;
    sample.profile_base_mode = requested.base_mode;
    sample.profile_tcp_cost_scale = requested.tcp_cost_scale;
    sample.profile_input_cost_scale = requested.input_cost_scale;
  }
  void solveBoundary() {
    const auto state = ocs2::vector_t::Zero(6).eval();
    reference->preSolverRun(10, 11, state);
    profile->preSolverRun(10, 11, state, *reference);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::Node::SharedPtr observer;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<ocs2::ReferenceManager> wrapped;
  std::shared_ptr<TcpReferenceManager> reference;
  std::shared_ptr<RuntimeMotionProfile> profile;
  std::shared_ptr<MotionRuntime> runtime;
  std::unique_ptr<MotionProfileServer> server;
  rclcpp::Client<SetProfile>::SharedPtr client;
  rclcpp::Subscription<ProfileStatus>::SharedPtr status_subscription;
  MotionSample sample;
  ProfileStatus latest_status;
  bool emit_feedback{true};
};

TEST_F(MotionProfileServerTest, AcceptanceWaitsForMatchingPolicyBeforeReleasingTargets) {
  const auto response = request(RuntimeProfileConfig::LOCKED, 2.0, 3.0);
  ASSERT_TRUE(response->accepted) << response->message;
  EXPECT_EQ(response->revision, 2U);
  EXPECT_EQ(profile->requestedRevision(), 2U);
  EXPECT_FALSE(runtime->profile_output_allowed);
  EXPECT_FALSE(reference->claimMotion());
  pump(100ms);
  EXPECT_FALSE(latest_status.applied);
  EXPECT_TRUE(latest_status.transition_pending);
  matchingPolicy(99);
  pump(100ms);
  EXPECT_FALSE(runtime->profile_output_allowed);
  matchingPolicy(response->revision);
  pump(100ms);
  EXPECT_TRUE(runtime->profile_output_allowed);
  EXPECT_TRUE(latest_status.applied);
  EXPECT_FALSE(latest_status.transition_pending);
  EXPECT_EQ(latest_status.active_revision, 2U);
  EXPECT_DOUBLE_EQ(latest_status.active.tcp_cost_scale, 2.0);
  EXPECT_DOUBLE_EQ(latest_status.active.input_cost_scale, 3.0);
  EXPECT_TRUE(reference->claimMotion());
  reference->releaseMotion();
}

TEST_F(MotionProfileServerTest, OnlyActiveFreshIdleControllerAcceptsNewProfile) {
  runtime->active = false;
  EXPECT_FALSE(request()->accepted);
  runtime->active = true;
  sample.state = MotionStatus::INITIALIZING;
  EXPECT_FALSE(request()->accepted);
  sample.state = MotionStatus::STOPPING;
  EXPECT_FALSE(request()->accepted);
  sample.state = MotionStatus::FAULT;
  sample.reason = MotionStatus::SOLVER_FAILED;
  EXPECT_FALSE(request()->accepted);
  sample.state = MotionStatus::IDLE;
  sample.reason = MotionStatus::NONE;
  emit_feedback = false;
  pump(300ms);
  EXPECT_FALSE(request()->accepted);
  EXPECT_EQ(profile->requestedRevision(), 1U);
  emit_feedback = true;
  EXPECT_TRUE(request()->accepted);
}

TEST_F(MotionProfileServerTest, PriorActivationFeedbackCannotReleaseProfileGate) {
  refresh();
  emit_feedback = false;
  runtime->active = false;
  runtime->activate();
  pump(60ms);
  EXPECT_FALSE(latest_status.applied);
  EXPECT_FALSE(runtime->profile_output_allowed);
  EXPECT_FALSE(request()->accepted);
  emit_feedback = true;
  pump(60ms);
  EXPECT_TRUE(latest_status.applied);
}

TEST_F(MotionProfileServerTest, StatusNeverClaimsAppliedWhileFaultedOrStopping) {
  pump(100ms);
  ASSERT_TRUE(latest_status.applied);
  sample.state = MotionStatus::FAULT;
  sample.reason = MotionStatus::POLICY_FAILED;
  pump(100ms);
  EXPECT_FALSE(latest_status.applied);
  EXPECT_NE(latest_status.message, "MPC profile applied");
  runtime->stop_request = 1;
  pump(100ms);
  EXPECT_FALSE(latest_status.applied);
  EXPECT_EQ(latest_status.message, "waiting for stop confirmation");
}

TEST_F(MotionProfileServerTest, InvalidProfilesLeaveRevisionAndOwnershipUnchanged) {
  for (const double scale : {0.0, -1.0, 10.1, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_FALSE(request(RuntimeProfileConfig::LOCKED, scale, 1.0)->accepted);
    EXPECT_FALSE(request(RuntimeProfileConfig::LOCKED, 1.0, scale)->accepted);
  }
  EXPECT_FALSE(request(99)->accepted);
  EXPECT_EQ(profile->requestedRevision(), 1U);
  EXPECT_TRUE(runtime->profile_output_allowed);
  EXPECT_TRUE(reference->claimMotion());
  reference->releaseMotion();
}

TEST_F(MotionProfileServerTest, ExistingMotionOwnershipCannotBeStolenByProfileChange) {
  ASSERT_TRUE(reference->claimMotion());
  const auto response = request();
  EXPECT_FALSE(response->accepted);
  EXPECT_EQ(profile->requestedRevision(), 1U);
  EXPECT_TRUE(reference->motionSessionValid());
  EXPECT_TRUE(runtime->profile_output_allowed);
  reference->releaseMotion();
  EXPECT_TRUE(request()->accepted);
}

TEST_F(MotionProfileServerTest, NewProfileIsSubmittedWithMeasuredHoldInsteadOfPreviousTarget) {
  ASSERT_TRUE(reference->claimMotion());
  zephyr_msgs::action::ExecuteCartesianMotion::Goal goal;
  goal.target.header.frame_id = "base";
  goal.target.pose.position.x = 0.9;
  goal.target.pose.position.y = 0.2;
  goal.target.pose.position.z = 0.3;
  goal.target.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped target;
  double end = 0;
  std::string reason;
  ASSERT_TRUE(reference->startMotion(goal, measuredPose(), target, end, reason)) << reason;
  reference->releaseMotion();
  solveBoundary();
  EXPECT_NEAR(wrapped->getTargetTrajectories().stateTrajectory.back()(0), 0.9, 1e-12);
  sample.pose[0] = 0.15;
  const auto response = request();
  ASSERT_TRUE(response->accepted) << response->message;
  EXPECT_EQ(profile->requestedRevision(), response->revision);
  EXPECT_EQ(profile->solverConfig().revision, 1U);
  // Public SolverBase order: reference manager, synchronized modules, solve.
  solveBoundary();
  EXPECT_EQ(profile->solverConfig().revision, response->revision);
  const auto& trajectory = wrapped->getTargetTrajectories();
  ASSERT_EQ(trajectory.stateTrajectory.size(), 1U);
  EXPECT_TRUE(trajectory.stateTrajectory.front().isApprox(measuredPose()));
  EXPECT_FALSE(runtime->profile_output_allowed);
}

TEST_F(MotionProfileServerTest, FaultAndPendingStopCannotApplyOrReleaseReferenceGate) {
  const auto response = request();
  ASSERT_TRUE(response->accepted);
  matchingPolicy(response->revision);
  sample.state = MotionStatus::FAULT;
  sample.reason = MotionStatus::SOLVER_FAILED;
  pump(100ms);
  EXPECT_FALSE(runtime->profile_output_allowed);
  EXPECT_FALSE(latest_status.applied);
  EXPECT_FALSE(reference->claimMotion());
  sample.state = MotionStatus::INITIALIZING;
  sample.reason = MotionStatus::NONE;
  runtime->stop_request = 1;
  pump(100ms);
  EXPECT_FALSE(runtime->profile_output_allowed);
  EXPECT_FALSE(latest_status.applied);
  runtime->stop_release = 1;
  pump(100ms);
  EXPECT_TRUE(runtime->profile_output_allowed);
}

TEST_F(MotionProfileServerTest, ReactivationReappliesSelectedProfileOnlyAfterFreshPolicy) {
  const auto response = request();
  ASSERT_TRUE(response->accepted);
  runtime->active = false;
  pump(100ms);
  EXPECT_FALSE(runtime->profile_output_allowed);
  runtime->activate();
  sample.state = MotionStatus::INITIALIZING;
  matchingPolicy(0);  // No policy has been activated in this new lifecycle.
  pump(100ms);
  EXPECT_FALSE(runtime->profile_output_allowed);
  EXPECT_FALSE(latest_status.applied);
  EXPECT_TRUE(latest_status.transition_pending);
  EXPECT_FALSE(reference->claimMotion());
  matchingPolicy(response->revision);
  pump(100ms);
  EXPECT_TRUE(runtime->profile_output_allowed);
  EXPECT_TRUE(latest_status.applied);
  EXPECT_FALSE(latest_status.transition_pending);
  EXPECT_EQ(latest_status.requested_revision, response->revision);
}

TEST_F(MotionProfileServerTest, MatchingRevisionCannotApplyUsingStaleControlFeedback) {
  const auto response = request();
  ASSERT_TRUE(response->accepted);
  emit_feedback = false;
  pump(300ms);
  matchingPolicy(response->revision);
  sample.received_ns = motionSteadyNs() - 300000000;
  ASSERT_TRUE(runtime->samples.tryWrite(sample));
  pump(100ms);
  EXPECT_FALSE(runtime->profile_output_allowed);
  EXPECT_FALSE(latest_status.applied);
  EXPECT_FALSE(reference->claimMotion());
  emit_feedback = true;
  pump(100ms);
  EXPECT_TRUE(runtime->profile_output_allowed);
}

}  // namespace
