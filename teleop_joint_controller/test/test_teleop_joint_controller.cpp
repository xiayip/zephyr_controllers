#include <gtest/gtest.h>
#include <thread>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "teleop_joint_controller/teleop_joint_controller.hpp"
using teleop_joint_controller::TeleopJointController;
using CB = controller_interface::CallbackReturn;
class ControllerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
      rclcpp::Parameter("downstream_controller", "position_adapter"),
      rclcpp::Parameter("servo_status_topic", "/test_teleop/servo_status"),
      rclcpp::Parameter("max_velocity", std::vector<double>{4}),
      rclcpp::Parameter("max_acceleration", std::vector<double>{24}),
      rclcpp::Parameter("command_timeout", command_timeout), rclcpp::Parameter("startup_timeout", .06)});
    const std::string urdf = R"(<robot name="test"><link name="base"/><link name="tip"/><joint name="joint1" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="4" effort="20"/></joint></robot>)";
    ASSERT_EQ(controller.init("test_teleop", urdf, 200, "", opts), controller_interface::return_type::OK);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CB::SUCCESS);
    p = std::make_shared<hardware_interface::CommandInterface>("position_adapter/joint1", "position", "double");
    v = std::make_shared<hardware_interface::CommandInterface>("position_adapter/joint1", "velocity", "double");
    q = std::make_shared<hardware_interface::StateInterface>("joint1", "position", "double");
    ASSERT_TRUE(q->set_value(.3));
    std::vector<hardware_interface::LoanedCommandInterface> commands; commands.emplace_back(v, nullptr); commands.emplace_back(p, nullptr);
    std::vector<hardware_interface::LoanedStateInterface> states; states.emplace_back(q);
    controller.assign_interfaces(std::move(commands), std::move(states));
    node = std::make_shared<rclcpp::Node>("teleop_test_sender");
    publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/test_teleop/joint_trajectory", rclcpp::QoS(1));
    status_publisher = node->create_publisher<moveit_msgs::msg::ServoStatus>("/test_teleop/servo_status", rclcpp::QoS(10));
    diagnostics = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/test_teleop/status", rclcpp::QoS(1).transient_local(),
        [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) { last_diagnostic = *msg; });
    executor.add_node(node); executor.add_node(controller.get_node()->get_node_base_interface());
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while ((publisher->get_subscription_count() == 0 || status_publisher->get_subscription_count() == 0) &&
           std::chrono::steady_clock::now() < until) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(publisher->get_subscription_count(), 0u);
    ASSERT_GT(status_publisher->get_subscription_count(), 0u);
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  }
  void TearDown() override { controller.on_deactivate(rclcpp_lifecycle::State()); controller.release_interfaces(); }
  trajectory_msgs::msg::JointTrajectory message(std::vector<double> values) {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {"joint1"}; msg.header.stamp = node->now();
    msg.points.resize(1); msg.points.back().positions = std::move(values);
    return msg;
  }
  void send_message(const trajectory_msgs::msg::JointTrajectory& msg) {
    publisher->publish(msg);
    for (int k = 0; k < 10; ++k) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  }
  void send(std::vector<double> values) { send_message(message(std::move(values))); }
  void servo_status(int8_t code) {
    moveit_msgs::msg::ServoStatus msg; msg.code = code; status_publisher->publish(msg);
    for (int k = 0; k < 10; ++k) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  }
  void update(double dt=.005) { EXPECT_EQ(controller.update(node->now(), rclcpp::Duration::from_seconds(dt)), controller_interface::return_type::OK); }
  double position() { return *p->get_optional(); }
  double velocity() { return *v->get_optional(); }
  double command_timeout{.04};
  diagnostic_msgs::msg::DiagnosticArray last_diagnostic;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics;
  TeleopJointController controller;
  std::shared_ptr<hardware_interface::CommandInterface> p,v;
  std::shared_ptr<hardware_interface::StateInterface> q;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
  rclcpp::Publisher<moveit_msgs::msg::ServoStatus>::SharedPtr status_publisher;
  rclcpp::executors::SingleThreadedExecutor executor;
};
TEST_F(ControllerTest, StartsAtMeasuredPositionAndRejectsOldSession) {
  EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.32}); update(); EXPECT_GT(position(), .3);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  send({.4}); ASSERT_TRUE(q->set_value(.5));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS); update();
  EXPECT_DOUBLE_EQ(position(), .5); EXPECT_DOUBLE_EQ(velocity(), 0);
}
TEST_F(ControllerTest, TimeoutHoldsAndFreshNearbyPacketsResume) {
  send({.34}); update(); const double hold = position();
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.32}); update(); EXPECT_GT(position(), hold); EXPECT_GT(velocity(), 0);
  EXPECT_LE(velocity(), 24.0 * .005 + 1e-9);
}
TEST_F(ControllerTest, TimeoutRejectsDistantWindowUntilNearFeedback) {
  send({.34}); update(); const double hold = position();
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); update();
  send({.6}); update(); EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  auto window = message({.6});
  auto end = window.points.front(); end.positions = {.32}; end.time_from_start.nanosec = 1000000;
  window.points.push_back(end); send_message(window); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.32}); update(); EXPECT_GT(position(), hold);
}
TEST_F(ControllerTest, PacketBeforeTimeoutUpdateStillRequiresProximity) {
  send({.34}); update(); const double hold = position();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  send({.6}); update(); EXPECT_DOUBLE_EQ(position(), hold);
  send({.32}); update(); EXPECT_GT(position(), hold);
}
TEST_F(ControllerTest, RecoveryChecksMeasuredJointsAsWellAsHeldReference) {
  send({.34}); update(); const double hold = position();
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); update();
  ASSERT_TRUE(q->set_value(.2));
  // Close to the reference, but too far from the actual joint position.
  send({.44}); update(); EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.31}); update(); EXPECT_GT(position(), hold);
}
TEST_F(ControllerTest, TimeoutCannotResumeFromDuplicateOrExpiredQueuedPacket) {
  const auto old = message({.32}); send_message(old); update(); const double hold = position();
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); update();
  // No new command: the previous reference must stay stopped indefinitely.
  update(); EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.33});
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.32}); update(); EXPECT_GT(position(), hold);
}
TEST_F(ControllerTest, InvalidPacketCannotBeHiddenByFollowingValidPacket) {
  send({NAN}); send({.32}); update(); EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
}
TEST_F(ControllerTest, WrongSizeLatches) {
  send({.31,.32}); update(); send({.31}); update(); EXPECT_DOUBLE_EQ(position(), .3);
}
TEST_F(ControllerTest, JumpAndLongUpdateLatch) {
  send({.7}); update(); EXPECT_DOUBLE_EQ(position(), .3);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  send({.32}); update(.06); EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
}
TEST_F(ControllerTest, NoFirstCommandTimesOut) {
  std::this_thread::sleep_for(std::chrono::milliseconds(70)); update();
  send({.32}); update(); EXPECT_DOUBLE_EQ(position(), .3);
}
TEST_F(ControllerTest, PositionLimitsRejectInput) {
  send({3.1}); update(); EXPECT_DOUBLE_EQ(position(), .3);
}
TEST_F(ControllerTest, HardwareTrackingFaultLatches) {
  ASSERT_TRUE(q->set_value(.6)); update();
  send({.32}); update(); EXPECT_DOUBLE_EQ(position(), .3);
}
TEST_F(ControllerTest, CompressedWindowCannotForwardSplineOrVelocitySpike) {
  auto msg = message({.32});
  msg.points.front().velocities = {9.0};
  auto last = msg.points.front(); last.positions = {.33};
  last.time_from_start.nanosec = 640961;  // Real-trial compressed interval.
  msg.points.push_back(last);
  send_message(msg); update();
  EXPECT_GT(position(), .3);
  EXPECT_LE(velocity(), 24.0 * .005 + 1e-9);
  EXPECT_NEAR(position() - .3, velocity() * .005, 1e-12);
}
TEST_F(ControllerTest, SamplesControllerTimeAndHoldsWindowEndpoint) {
  auto msg = message({.3});
  auto last = msg.points.front(); last.positions = {.303};
  last.time_from_start.nanosec = 30000000;
  msg.points.push_back(last);
  send_message(msg);
  const rclcpp::Time begin(msg.header.stamp);
  const auto tick = [&](double seconds) {
    EXPECT_EQ(controller.update(begin + rclcpp::Duration::from_seconds(seconds),
        rclcpp::Duration::from_seconds(.005)), controller_interface::return_type::OK);
  };
  tick(0); EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
  tick(.015);
  // Midpoint is .3015, rather than the future endpoint .303.
  EXPECT_NEAR(velocity(), .1, 1e-9); EXPECT_NEAR(position(), .3005, 1e-9);
  for (int k = 0; k < 60; ++k) tick(.030 + .005 * k);
  EXPECT_NEAR(position(), .303, 1e-9);
  EXPECT_NEAR(velocity(), 0, 1e-7);
}
TEST_F(ControllerTest, ReorderedWindowCannotReplaceAcceptedReference) {
  auto old = message({.29});
  auto msg = message({.3});
  auto last = msg.points.front(); last.positions = {.303};
  last.time_from_start.nanosec = 30000000; msg.points.push_back(last);
  send_message(msg);
  const rclcpp::Time begin(msg.header.stamp);
  EXPECT_EQ(controller.update(begin + rclcpp::Duration::from_seconds(.015),
      rclcpp::Duration::from_seconds(.005)), controller_interface::return_type::OK);
  const double before = position();
  send_message(old);
  EXPECT_EQ(controller.update(begin + rclcpp::Duration::from_seconds(.020),
      rclcpp::Duration::from_seconds(.005)), controller_interface::return_type::OK);
  EXPECT_GT(position(), before); EXPECT_GT(velocity(), 0);
}
TEST_F(ControllerTest, InvalidIntermediatePositionAndUnorderedTimesLatch) {
  for (const int variant : {0, 1, 2}) {
    ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
    auto msg = message({.31}); auto last = msg.points.front();
    last.positions = {.32}; last.time_from_start.nanosec = 1000000;
    msg.points.push_back(last);
    if (variant == 0) msg.points.front().positions = {NAN};
    else if (variant == 1) msg.points.front().positions = {3.1};
    else msg.points.front().time_from_start.nanosec = 1000000;
    send_message(msg); update(); send({.32}); update();
    EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
  }
}
TEST_F(ControllerTest, EmptyAndWrongJointNamesLatch) {
  auto msg = message({.32}); msg.points.clear(); send_message(msg); update();
  send({.32}); update(); EXPECT_DOUBLE_EQ(position(), .3);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  msg = message({.32}); msg.joint_names = {"wrong_joint"}; send_message(msg); update();
  send({.32}); update(); EXPECT_DOUBLE_EQ(position(), .3);
}
TEST_F(ControllerTest, StaleAndFarFutureSourceLatch) {
  auto msg = message({.32}); msg.header.stamp = node->now() - rclcpp::Duration::from_seconds(.2);
  send_message(msg); update(); EXPECT_DOUBLE_EQ(position(), .3);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  msg = message({.32}); msg.points.back().time_from_start.sec = 1;
  send_message(msg); update(); EXPECT_DOUBLE_EQ(position(), .3);
}
TEST_F(ControllerTest, RepeatedSourceDoesNotRefreshWatchdog) {
  auto msg = message({.32});
  // A short look-ahead is valid for Servo. Repeat it while its age remains valid.
  msg.points.back().time_from_start.nanosec = 16000000;
  send_message(msg); update(); const double hold = position();
  std::this_thread::sleep_for(std::chrono::milliseconds(25));
  send_message(msg); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
}
TEST_F(ControllerTest, CollisionStopLatchesDespiteFreshTrajectoryAndNormalStatus) {
  send({.34}); update(); const double hold = position();
  EXPECT_GT(velocity(), 0);
  servo_status(moveit_msgs::msg::ServoStatus::HALT_FOR_COLLISION);
  servo_status(moveit_msgs::msg::ServoStatus::NO_WARNING);
  send({.36}); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  send({.32}); update(); EXPECT_GT(position(), .3);
}
TEST_F(ControllerTest, DecelerationStatusKeepsTracking) {
  for (const int8_t code : {moveit_msgs::msg::ServoStatus::DECELERATE_FOR_APPROACHING_SINGULARITY,
       moveit_msgs::msg::ServoStatus::DECELERATE_FOR_LEAVING_SINGULARITY,
       moveit_msgs::msg::ServoStatus::DECELERATE_FOR_COLLISION}) {
    const double before = position(); servo_status(code); send({.34}); update();
    EXPECT_GT(position(), before); EXPECT_GT(velocity(), 0);
  }
}
TEST_F(ControllerTest, CollisionAndInvalidStopsLatch) {
  for (const int8_t code : {moveit_msgs::msg::ServoStatus::HALT_FOR_COLLISION,
       moveit_msgs::msg::ServoStatus::INVALID}) {
    ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
    servo_status(code); send({.34}); update();
    EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
  }
}
TEST_F(ControllerTest, InactiveServoStatusDoesNotPoisonNewActivation) {
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  servo_status(moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  send({.32}); update(); EXPECT_GT(position(), .3); EXPECT_GT(velocity(), 0);
}

// Recovery tests deliberately separate status/trajectory delivery with executor
// spins. Give that sequence room; dedicated tests retain the 40 ms watchdog.
class JointBoundRecoveryTest : public ControllerTest {
protected:
  void SetUp() override { command_timeout = .2; ControllerTest::SetUp(); }
};
TEST_F(JointBoundRecoveryTest, JointBoundHoldsUntilClearStatusAndFreshCommandThenReverses) {
  using S = moveit_msgs::msg::ServoStatus;
  send({.34}); update(); const double hold = position();
  servo_status(S::JOINT_BOUND); send({.35}); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  // New trajectories alone must not override a still-active bound.
  send({.29}); update(); EXPECT_DOUBLE_EQ(position(), hold);
  servo_status(S::JOINT_BOUND); update();
  // Clearing the status alone must not replay the pre-bound window.
  servo_status(S::NO_WARNING); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.29}); update();
  EXPECT_LT(position(), hold); EXPECT_LT(velocity(), 0);
}
TEST_F(JointBoundRecoveryTest, BriefJointBoundIsNotLostBeforeUpdate) {
  using S = moveit_msgs::msg::ServoStatus;
  send({.34}); update(); const double hold = position();
  servo_status(S::JOINT_BOUND); servo_status(S::NO_WARNING); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.29}); update(); EXPECT_LT(position(), hold);
}
TEST_F(ControllerTest, JointBoundStillEnforcesCommandTimeout) {
  using S = moveit_msgs::msg::ServoStatus;
  send({.34}); update(); const double hold = position();
  servo_status(S::JOINT_BOUND); update();
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); update();
  servo_status(S::NO_WARNING); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.29}); update(); EXPECT_LT(position(), hold); EXPECT_LT(velocity(), 0);
}
TEST_F(ControllerTest, JointBoundCannotClearFatalServoStop) {
  using S = moveit_msgs::msg::ServoStatus;
  servo_status(S::HALT_FOR_COLLISION); servo_status(S::JOINT_BOUND); update();
  servo_status(S::NO_WARNING); send({.29}); update();
  EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
}
TEST_F(ControllerTest, JointBoundStillRejectsOutOfRangeCommand) {
  using S = moveit_msgs::msg::ServoStatus;
  servo_status(S::JOINT_BOUND); send({3.1}); update();
  servo_status(S::NO_WARNING); send({.29}); update();
  EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
}
TEST_F(ControllerTest, JointBoundDoesNotPoisonNewActivation) {
  using S = moveit_msgs::msg::ServoStatus;
  servo_status(S::JOINT_BOUND); update();
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), CB::SUCCESS);
  servo_status(S::JOINT_BOUND);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CB::SUCCESS);
  send({.32}); update(); EXPECT_GT(position(), .3); EXPECT_GT(velocity(), 0);
}

TEST_F(JointBoundRecoveryTest, GradualTargetsDuringHoldDoNotBecomeARecoveryDiscontinuity) {
  using S = moveit_msgs::msg::ServoStatus;
  send({.32}); update(); const double hold = position();
  servo_status(S::JOINT_BOUND);
  // Servo can continue updating other joints while its aggregate status is
  // JOINT_BOUND. The cumulative change exceeds the single-update step guard.
  for (const double target : {.36, .40, .44, .48, .52, .56}) {
    send({target}); update();
    EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  }
  servo_status(S::NO_WARNING); send({.54}); update();
  EXPECT_GT(position(), hold);
  EXPECT_LE(velocity(), 24.0 * .005 + 1e-9);
  EXPECT_NEAR(position() - hold, velocity() * .005, 1e-12);
}
TEST_F(JointBoundRecoveryTest, ActualTargetDiscontinuityDuringHoldStillLatches) {
  using S = moveit_msgs::msg::ServoStatus;
  servo_status(S::JOINT_BOUND); send({.32}); update();
  send({.6}); update();
  servo_status(S::NO_WARNING); send({.31}); update();
  EXPECT_DOUBLE_EQ(position(), .3); EXPECT_DOUBLE_EQ(velocity(), 0);
}

TEST_F(JointBoundRecoveryTest, SingularityHoldsAndRecoversOnLeavingStatusWithFreshCommand) {
  using S = moveit_msgs::msg::ServoStatus;
  send({.34}); update(); const double hold = position();
  servo_status(S::HALT_FOR_SINGULARITY); send({.35}); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  servo_status(S::HALT_FOR_SINGULARITY);
  servo_status(S::DECELERATE_FOR_LEAVING_SINGULARITY); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.29}); update(); EXPECT_LT(position(), hold); EXPECT_LT(velocity(), 0);
}
TEST_F(ControllerTest, SingularityDoesNotBypassTimeout) {
  using S = moveit_msgs::msg::ServoStatus;
  send({.34}); update(); const double hold = position();
  servo_status(S::HALT_FOR_SINGULARITY); update();
  std::this_thread::sleep_for(std::chrono::milliseconds(50)); update();
  servo_status(S::NO_WARNING); update();
  EXPECT_DOUBLE_EQ(position(), hold); EXPECT_DOUBLE_EQ(velocity(), 0);
  send({.29}); update(); EXPECT_LT(position(), hold); EXPECT_LT(velocity(), 0);
}

TEST_F(ControllerTest, TrackingFaultPublishesOriginalJointAndValues) {
  ASSERT_TRUE(q->set_value(.6)); update();
  ASSERT_TRUE(q->set_value(.7)); update();
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (last_diagnostic.status.empty() && std::chrono::steady_clock::now() < until) {
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(last_diagnostic.status.empty());
  const auto& values = last_diagnostic.status.front().values;
  const auto value = [&](const std::string& key) {
    for (const auto& entry : values) if (entry.key == key) return entry.value;
    return std::string{};
  };
  EXPECT_EQ(value("state"), "FAULT"); EXPECT_EQ(value("joint"), "joint1");
  EXPECT_DOUBLE_EQ(std::stod(value("observed")), .6);
  EXPECT_DOUBLE_EQ(std::stod(value("reference")), .3);
  EXPECT_DOUBLE_EQ(std::stod(value("limit")), .15);
  EXPECT_DOUBLE_EQ(std::stod(value("joint_position")), .6);
}
