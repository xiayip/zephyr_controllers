// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <mpc_controller/control/MotionActionServer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <limits>
#include <thread>

using namespace zephyr::mpc;
using namespace std::chrono_literals;
namespace {
class MotionActionTest : public testing::Test {
protected:
  using Action = MotionActionServer::Action;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
        "controller", "/motion_test", rclcpp::NodeOptions().use_global_arguments(false));
    wrapped = std::make_shared<ocs2::ReferenceManager>();
    reference = std::make_shared<TcpReferenceManager>(wrapped, "base", "tcp", 9);
    reference->configureWholeBody(true,{"joint1","joint2","joint3","joint4","joint5","joint6"},
        ocs2::vector_t::Constant(6,-3), ocs2::vector_t::Constant(6,3));
    reference->subscribe(node);
    runtime = std::make_shared<MotionRuntime>();
    runtime->activate();
    sample.valid = true; sample.state = MotionStatus::IDLE; sample.reference_time = 10;
    sample.speeds.valid = true; // Fixture explicitly supplies fresh measured standstill.
    sample.pose = {0.1, 0.2, 0.3, 0, 0, 0, 1};
    sample.whole_body_size = 9;
    executor.add_node(node->get_node_base_interface());
    ocs2::vector_t p = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(sample.pose.data());
    ASSERT_TRUE(reference->requestReset(p, 10));
    pump(50ms);
    ASSERT_TRUE(reference->resetReady());
    reference->setAcceptingTargets(true);
    server = std::make_unique<MotionActionServer>(node, reference, runtime);
    node->configure(); node->activate();
    client = rclcpp_action::create_client<Action>(node, "/motion_test/controller/execute_motion");
    ASSERT_TRUE(client->wait_for_action_server(2s));
    stop_client = node->create_client<std_srvs::srv::Trigger>("/motion_test/controller/stop");
    status_sub = node->create_subscription<MotionStatus>("/motion_test/controller/motion_status", 1,
        [this](const MotionStatus& status) { latest_status = status; });
    ASSERT_TRUE(stop_client->wait_for_service(2s));
  }
  void TearDown() override {
    runtime->active = false;
    pump(100ms);
    status_sub.reset(); stop_client.reset(); client.reset(); server.reset();
    executor.remove_node(node->get_node_base_interface());
    reference.reset(); node.reset();
  }
  void refresh() {
    sample.activation_epoch = runtime->activation_epoch.load();
    sample.received_ns = motionSteadyNs(); sample.stamp_ns = node->now().nanoseconds();
    runtime->samples.tryWrite(sample);
    ocs2::SystemObservation observation; observation.time = sample.reference_time;
    reference->setCurrentObservation(observation);
  }
  void pump(std::chrono::milliseconds duration, bool feedback = true) {
    const auto end = std::chrono::steady_clock::now() + duration;
    do {
      if (feedback) refresh();
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < end);
  }
  template<class Future> bool wait(Future& future) {
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (future.wait_for(0ms) != std::future_status::ready && std::chrono::steady_clock::now() < end) pump(5ms);
    return future.wait_for(0ms) == std::future_status::ready;
  }
  Action::Goal goal() {
    Action::Goal g;
    g.target.header.frame_id = "base";
    g.target.pose.position.x = 0.1; g.target.pose.position.y = 0.2; g.target.pose.position.z = 0.31;
    g.target.pose.orientation.w = 1;
    g.timeout = 10; g.settle_time = 0.1;
    return g;
  }
  Handle::SharedPtr send(const Action::Goal& g) {
    auto future = client->async_send_goal(g);
    if (!wait(future)) { ADD_FAILURE() << "No action acknowledgement"; return {}; }
    return future.get();
  }
  void commit() { wrapped->preSolverRun(10, 11, ocs2::vector_t::Zero(6)); }
  void stopAck() {
    sample.stop_ack = runtime->stop_request;
    sample.state = MotionStatus::STOPPING;
    reference->setAcceptingTargets(false);
  }
  std_srvs::srv::Trigger::Response::SharedPtr requestStop() {
    auto future = stop_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (!wait(future)) { ADD_FAILURE() << "No stop response"; return {}; }
    return future.get();
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<ocs2::ReferenceManager> wrapped;
  std::shared_ptr<TcpReferenceManager> reference;
  std::shared_ptr<MotionRuntime> runtime;
  std::unique_ptr<MotionActionServer> server;
  rclcpp_action::Client<Action>::SharedPtr client;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client;
  rclcpp::Subscription<MotionStatus>::SharedPtr status_sub;
  MotionStatus latest_status;
  MotionSample sample;
};
}

TEST_F(MotionActionTest, ArrivalRequiresActualFeedbackAndSettlingAfterTrajectoryEnd) {
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  sample.reference_time = 100;
  pump(200ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready); // Reference ended, arm has not arrived.
  sample.pose[2] = 0.31;
  pump(55ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.get().result->success);
}

TEST_F(MotionActionTest, TcpArrivalCannotCompleteBeforeRequestedBaseYawAndPosition) {
  auto g=goal();
  g.base_position_weight=5; g.base_yaw_weight=5;
  g.whole_body_references.resize(1);
  auto& base=g.whole_body_references[0].base_pose;
  base.header.frame_id="base"; base.pose.position.x=.1;
  base.pose.orientation.z=std::sin(.1); base.pose.orientation.w=std::cos(.1);
  auto h=send(g); ASSERT_TRUE(h); auto result=client->async_get_result(h);
  sample.reference_time=100; sample.pose[2]=.31;
  pump(250ms); EXPECT_NE(result.wait_for(0ms),std::future_status::ready);
  sample.whole_body_state[0]=.1;
  pump(250ms); EXPECT_NE(result.wait_for(0ms),std::future_status::ready);
  sample.whole_body_state[2]=.2;
  ASSERT_TRUE(wait(result)); EXPECT_TRUE(result.get().result->success);
  EXPECT_NEAR(result.get().result->base_yaw_error,0,1e-10);
}

TEST_F(MotionActionTest, PostureIsSoftAndMalformedWholeBodyGoalIsRejected) {
  auto g=goal(); g.posture_weight=.1; g.whole_body_references.resize(1);
  auto& posture=g.whole_body_references[0].posture;
  posture.name={"joint1"}; posture.position={4};
  EXPECT_FALSE(send(g));
  posture.position={.2};
  auto h=send(g); ASSERT_TRUE(h); auto result=client->async_get_result(h);
  sample.reference_time=100; sample.pose[2]=.31;
  ASSERT_TRUE(wait(result)); EXPECT_TRUE(result.get().result->success);
}

TEST_F(MotionActionTest, MotionRejectsConcurrentGoalsAndAllLegacyTargetInputs) {
  auto h = send(goal()); ASSERT_TRUE(h);
  EXPECT_FALSE(send(goal()));
  commit();
  const auto endpoint = wrapped->getTargetTrajectories().stateTrajectory.back();
  auto pose_pub = node->create_publisher<geometry_msgs::msg::Pose>("tcp_target", 1); pose_pub->on_activate();
  auto stamped_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("tcp_target/stamped", 1); stamped_pub->on_activate();
  auto relative_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("tcp_target/relative", 1); relative_pub->on_activate();
  auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("tcp_target/twist", 1); twist_pub->on_activate();
  auto path_pub = node->create_publisher<nav_msgs::msg::Path>("target_path", 1); path_pub->on_activate();
  auto continuous_pub = node->create_publisher<nav_msgs::msg::Path>("target_path/continuous", 1); continuous_pub->on_activate();
  pump(100ms);
  auto p = goal().target; p.pose.position.z = 0.9;
  geometry_msgs::msg::TwistStamped d; d.twist.linear.z = 0.9;
  nav_msgs::msg::Path path; path.poses = {p};
  pose_pub->publish(p.pose); stamped_pub->publish(p); relative_pub->publish(d); twist_pub->publish(d.twist); path_pub->publish(path);
  continuous_pub->publish(path);
  pump(100ms); commit();
  EXPECT_EQ(wrapped->getTargetTrajectories().stateTrajectory.back(), endpoint);
}

TEST_F(MotionActionTest, CancelWaitsForControlLoopStopAcknowledgement) {
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  auto cancel = client->async_cancel_goal(h); ASSERT_TRUE(wait(cancel));
  pump(100ms);
  EXPECT_TRUE(runtime->stopRequested());
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  EXPECT_FALSE(send(goal()));
  stopAck(); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.get().result->reason, MotionStatus::CANCELED);
  EXPECT_FALSE(runtime->stopRequested());
}

TEST_F(MotionActionTest, TimeoutStopsBeforeReturningFailure) {
  auto g = goal(); g.timeout = 0.25;
  auto h = send(g); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  pump(400ms);
  EXPECT_TRUE(runtime->stopRequested());
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  stopAck(); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->reason, MotionStatus::TIMED_OUT);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(MotionActionTest, AutomaticDeadlineAcceptsLongReferenceAndStillRequiresActualArrival) {
  auto g = goal(); g.timeout = 0;
  g.mode = Action::Goal::CONTINUOUS_PATH;
  auto p = g.target; p.pose.position.z = 3.0; g.waypoints.push_back(p);
  p.pose.position.z = 10.0; g.waypoints.push_back(p);
  auto h = send(g); ASSERT_TRUE(h); commit();
  EXPECT_GT(wrapped->getTargetTrajectories().timeTrajectory.back() - 10.0, 40.0);
  auto result = client->async_get_result(h);
  pump(250ms);
  EXPECT_FALSE(runtime->stopRequested());
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  sample.reference_time = 1000;
  pump(200ms); EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  sample.pose[2] = 10.0;
  ASSERT_TRUE(wait(result)); EXPECT_TRUE(result.get().result->success);
}

TEST_F(MotionActionTest, AutomaticTimeoutDoesNotAcceptNegativeOrNonfiniteValues) {
  for (double timeout : {-1.0, std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::quiet_NaN()}) {
    auto g = goal(); g.timeout = timeout;
    EXPECT_FALSE(send(g));
  }
  EXPECT_TRUE(send(goal()));
}

TEST_F(MotionActionTest, InactiveWithoutConfirmedLifecycleHoldKeepsOwnership) {
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  runtime->active = false;  // Lifecycle stopped, but its final output write failed.
  pump(150ms);
  EXPECT_TRUE(runtime->stopRequested());
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  EXPECT_FALSE(reference->claimMotion());
  runtime->stopped_epoch = runtime->activation_epoch.load();
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->reason, MotionStatus::CONTROLLER_INACTIVE);
  EXPECT_FALSE(runtime->stopRequested());
}

TEST_F(MotionActionTest, PreviousActivationHoldCannotConfirmCurrentMotionStopped) {
  runtime->active = false;
  runtime->stopped_epoch = runtime->activation_epoch.load();
  runtime->activate();
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  runtime->active = false;
  pump(150ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  EXPECT_FALSE(reference->claimMotion());
  runtime->stopped_epoch = runtime->activation_epoch.load();
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->reason, MotionStatus::CONTROLLER_INACTIVE);
}

TEST_F(MotionActionTest, StaleFeedbackCannotCompleteOrContinueMotion) {
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  pump(350ms, false);
  EXPECT_TRUE(runtime->stopRequested());
  stopAck(); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->reason, MotionStatus::FEEDBACK_UNAVAILABLE);
}

TEST_F(MotionActionTest, RejectsInvalidAndInactiveGoalsAndReportsBadTransform) {
  auto g = goal(); g.speed_scale = 0; EXPECT_FALSE(send(g));
  g = goal(); g.target.pose.orientation.w = 0; EXPECT_FALSE(send(g));
  g = goal(); g.mode = Action::Goal::PATH; EXPECT_FALSE(send(g));
  runtime->active = false; EXPECT_FALSE(send(goal())); runtime->active = true;
  sample.state = MotionStatus::INITIALIZING; EXPECT_FALSE(send(goal())); sample.state = MotionStatus::IDLE;
  g = goal(); g.target.header.frame_id = "absent";
  auto h = send(g); ASSERT_TRUE(h);
  auto result = client->async_get_result(h); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->reason, MotionStatus::INVALID_GOAL);
  EXPECT_FALSE(runtime->stopRequested());
}

TEST_F(MotionActionTest, RelativeUsesMeasuredTcpAxesAndPerGoalScaling) {
  // Measured TCP is rotated +90 degrees around world Z: local X is world Y.
  sample.pose[5] = sample.pose[6] = std::sqrt(0.5);
  auto g = goal(); g.mode = Action::Goal::RELATIVE; g.relative.header.frame_id = "tcp";
  g.relative.twist.linear.x = 0.02; g.speed_scale = 0.1;
  auto h = send(g); ASSERT_TRUE(h); commit();
  const auto& trajectory = wrapped->getTargetTrajectories();
  EXPECT_NEAR(trajectory.stateTrajectory.front()(1), 0.2, 1e-12);
  EXPECT_NEAR(trajectory.stateTrajectory.back()(1), 0.22, 1e-12);
  EXPECT_NEAR(trajectory.stateTrajectory.back()(0), 0.1, 1e-12);
  EXPECT_GE(trajectory.timeTrajectory.back() - trajectory.timeTrajectory.front(), 1.875 * 0.02 / 0.03);
}

TEST_F(MotionActionTest, ControllerFaultHasStructuredReasonAndNeedsStopAck) {
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  sample.state = MotionStatus::FAULT; sample.reason = MotionStatus::SOLVER_FAILED;
  pump(100ms); EXPECT_TRUE(runtime->stopRequested());
  stopAck(); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->reason, MotionStatus::SOLVER_FAILED);
}

TEST_F(MotionActionTest, InvalidatedReferenceSessionCannotResumeOldTask) {
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  reference->setAcceptingTargets(false);
  reference->setAcceptingTargets(true);
  pump(100ms);
  EXPECT_TRUE(runtime->stopRequested());
  stopAck(); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->reason, MotionStatus::REFERENCE_FAILED);
}

TEST_F(MotionActionTest, TopicStopIsAcknowledgedBeforeAcceptingFreshTargetsWithoutResume) {
  auto pub = node->create_publisher<geometry_msgs::msg::Pose>("tcp_target", 1);
  pub->on_activate(); pump(80ms);
  auto target = goal().target.pose; target.position.z = 0.5;
  pub->publish(target); pump(50ms); commit();
  EXPECT_DOUBLE_EQ(wrapped->getTargetTrajectories().stateTrajectory.back()(2), 0.5);
  auto response = requestStop(); ASSERT_TRUE(response); EXPECT_TRUE(response->success);
  const auto token = runtime->stop_request.load();
  response = requestStop(); ASSERT_TRUE(response); EXPECT_TRUE(response->success);
  EXPECT_EQ(runtime->stop_request.load(), token);  // Repeated stop is idempotent.
  reference->setAcceptingTargets(false);  // Next control cycle closes inputs before hold acknowledgement.
  target.position.z = 0.9;
  pub->publish(target); pump(100ms); commit();
  EXPECT_DOUBLE_EQ(wrapped->getTargetTrajectories().stateTrajectory.back()(2), 0.5);
  EXPECT_TRUE(runtime->stopRequested());
  EXPECT_EQ(latest_status.state, MotionStatus::STOPPING);
  EXPECT_FALSE(send(goal()));
  stopAck(); pump(100ms);
  EXPECT_FALSE(runtime->stopRequested());
  // Simulate the normal RT reinitialization: measured hold and a fresh policy.
  ASSERT_TRUE(reference->requestReset(Eigen::Map<const Eigen::Matrix<double, 7, 1>>(sample.pose.data()), 10));
  pump(50ms); ASSERT_TRUE(reference->resetReady());
  sample.state = MotionStatus::IDLE;
  reference->setAcceptingTargets(true);
  pub->publish(target); pump(80ms); commit();
  EXPECT_DOUBLE_EQ(wrapped->getTargetTrajectories().stateTrajectory.back()(2), 0.9);
  EXPECT_EQ(latest_status.state, MotionStatus::IDLE);
  EXPECT_TRUE(send(goal()));
}

TEST_F(MotionActionTest, StopServiceAbortsOwnedActionAndDoesNotClearFault) {
  auto h = send(goal()); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  auto response = requestStop(); ASSERT_TRUE(response); EXPECT_TRUE(response->success);
  pump(80ms); EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  sample.reason = MotionStatus::SOLVER_FAILED;
  stopAck(); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  sample.state = MotionStatus::FAULT; pump(100ms);
  EXPECT_EQ(latest_status.reason, MotionStatus::SOLVER_FAILED);
  EXPECT_FALSE(send(goal()));
  runtime->active = false;
  response = requestStop(); ASSERT_TRUE(response); EXPECT_FALSE(response->success);
}

TEST_F(MotionActionTest, PriorActivationFeedbackCannotAdmitNewMotion) {
  refresh();
  runtime->active = false;
  runtime->activate();  // Recent timestamp, but feedback belongs to the old activation.
  pump(60ms, false);
  EXPECT_FALSE(latest_status.feedback_valid);
  auto future = client->async_send_goal(goal());
  pump(60ms, false);
  ASSERT_EQ(future.wait_for(0ms), std::future_status::ready);
  EXPECT_FALSE(future.get());
  EXPECT_TRUE(send(goal()));  // Fresh current-activation feedback restores admission.
}

TEST_F(MotionActionTest, FullPathIsInterpolatedAndZeroDistanceStillRequiresFreshSettling) {
  auto g = goal(); g.mode = Action::Goal::PATH;
  g.waypoints = {g.target, g.target}; g.waypoints.back().pose.position.z = 0.32;
  auto h = send(g); ASSERT_TRUE(h); commit();
  const auto& t = wrapped->getTargetTrajectories();
  EXPECT_GT(t.stateTrajectory.size(), 2u);
  EXPECT_NEAR(t.stateTrajectory.front()(2), 0.3, 1e-12);
  EXPECT_NEAR(t.stateTrajectory.back()(2), 0.32, 1e-12);
  auto result = client->async_get_result(h);
  sample.pose[2] = 0.32; sample.reference_time = 100;
  ASSERT_TRUE(wait(result));
  EXPECT_TRUE(result.get().result->success);
}

TEST_F(MotionActionTest, ContinuousPathUsesBufferedReferenceAndPassesIntermediateFrames) {
  commit();
  const auto old = wrapped->getTargetTrajectories().stateTrajectory.back();
  auto g = goal(); g.mode = Action::Goal::CONTINUOUS_PATH;
  for (int i = 1; i <= 6; ++i) {
    auto p = g.target; p.pose.position.z = 0.3 + 0.01 * i;
    g.waypoints.push_back(p);
  }
  auto h = send(g); ASSERT_TRUE(h);
  EXPECT_EQ(wrapped->getTargetTrajectories().stateTrajectory.back(), old);
  commit();  // Only OCS2's iteration boundary activates the new complete path.
  const auto& t = wrapped->getTargetTrajectories();
  size_t middle = 0;
  for (size_t i = 0; i < t.stateTrajectory.size(); ++i)
    if (std::abs(t.stateTrajectory[i][2] - 0.33) < 1e-12) middle = i;
  ASSERT_GT(middle, 0u); ASSERT_LT(middle + 1, t.stateTrajectory.size());
  EXPECT_GT((t.stateTrajectory[middle + 1][2] - t.stateTrajectory[middle - 1][2]) /
            (t.timeTrajectory[middle + 1] - t.timeTrajectory[middle - 1]), 0.01);
  auto result = client->async_get_result(h);
  sample.pose[2] = 0.36; sample.reference_time = 100;
  ASSERT_TRUE(wait(result)); EXPECT_TRUE(result.get().result->success);
}

TEST_F(MotionActionTest, InvalidContinuousGeometryLeavesPreviousReferenceUnchanged) {
  commit(); const auto old = wrapped->getTargetTrajectories().stateTrajectory.back();
  auto g = goal(); g.mode = Action::Goal::CONTINUOUS_PATH;
  auto p = g.target; p.pose.position.x += 0.2;
  g.waypoints.push_back(p); p.pose.position.y += 0.2;
  p.pose.orientation.z = 1; p.pose.orientation.w = 0; // Ambiguous >3 rad chart, even with no deviation bound.
  g.waypoints.push_back(p);
  auto h = send(g); ASSERT_TRUE(h);
  auto result = client->async_get_result(h); ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->reason, MotionStatus::INVALID_GOAL);
  commit(); EXPECT_EQ(wrapped->getTargetTrajectories().stateTrajectory.back(), old);
  EXPECT_FALSE(runtime->stopRequested());
  EXPECT_TRUE(send(goal()));
}

TEST_F(MotionActionTest, AtTargetStillWaitsForEveryMeasuredSpeedAndValidEstimate) {
  auto g = goal(); g.settle_time = 0.2;
  auto h = send(g); ASSERT_TRUE(h);
  auto result = client->async_get_result(h);
  sample.pose[2] = 0.31; sample.reference_time = 100;
  for (size_t axis = 0; axis < MotionSpeeds::Count; ++axis) {
    sample.speeds.values.fill(0); sample.speeds.values[axis] = 0.1;
    pump(250ms);
    EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
    EXPECT_EQ(latest_status.state, MotionStatus::SETTLING);
  }
  sample.speeds.values.fill(0); sample.speeds.valid = false;
  pump(250ms); EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  sample.speeds.valid = true;
  sample.speeds.values[0] = std::numeric_limits<double>::quiet_NaN();
  pump(250ms); EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  sample.speeds.values.fill(0);
  pump(110ms); EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  // A new motion burst resets the entire settling dwell, not just one tick.
  sample.speeds.values[MotionSpeeds::BaseAngular] = 0.1; pump(75ms);
  sample.speeds.values.fill(0); pump(110ms);
  EXPECT_NE(result.wait_for(0ms), std::future_status::ready);
  ASSERT_TRUE(wait(result)); EXPECT_TRUE(result.get().result->success);
}

TEST_F(MotionActionTest, SettlingThresholdsCannotBeSilentlyChangedOnline) {
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("settle_tcp_linear_velocity", 0.2)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("settle_joint_velocity", "invalid")).successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("settle_tcp_linear_velocity").as_double(), 0.005);
}
