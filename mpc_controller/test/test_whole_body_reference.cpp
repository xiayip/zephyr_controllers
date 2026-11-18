// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <ocs2_core/PreComputation.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <mpc_controller/control/WholeBodyReference.h>
#include <mpc_controller/control/TcpReferenceManager.h>
#include <mpc_controller/control/ContinuousTcpPath.h>
#include <cmath>
#include <thread>

using namespace zephyr::mpc;
using namespace ocs2;

TEST(WholeBodyPath, MonotoneContinuousPassageAndVelocityBound) {
  vector_array_t points{vector_t::Zero(4), vector_t::Constant(4, 1), vector_t::Constant(4, 2)};
  auto p = makeWholeBodyTrajectory({0, 1, 2}, points, vector_t::Ones(4), -1, vector_t::Constant(4, .4));
  EXPECT_GT(p->times.back(), 2);
  EXPECT_TRUE(p->sample(-1).isApprox(points.front()));
  EXPECT_TRUE(p->sample(100).isApprox(points.back()));
  EXPECT_TRUE(p->sample(p->times[1]).isApprox(points[1]));
  EXPECT_GT(p->slopes[1][0], .01);
  EXPECT_LT((p->sample(1e-4)-p->sample(0)).norm()/1e-4,1e-6);
  EXPECT_LT((p->sample(p->times.back())-p->sample(p->times.back()-1e-4)).norm()/1e-4,1e-6);
  auto previous = p->sample(0);
  const double dt = .001;
  for (double t = dt; t <= p->times.back(); t += dt) {
    auto x = p->sample(t);
    EXPECT_GE((x-previous).minCoeff(), -1e-10);
    EXPECT_LE(((x-previous)/dt).maxCoeff(), .40001);
    EXPECT_GE(x.minCoeff(), 0); EXPECT_LE(x.maxCoeff(), 2);
    previous = x;
  }
}
TEST(WholeBodyPath, YawWrapsButBoundedJointsDoNot) {
  vector_t a(4), b(4); a << 0,0,3.1,3.1; b << 0,0,-3.1,-3.1;
  auto p = makeWholeBodyTrajectory({0,10}, {a,b}, vector_t::Ones(4), 2, vector_t::Ones(4));
  EXPECT_LT(std::abs(p->states.back()[2]-3.1), .1);
  EXPECT_DOUBLE_EQ(p->states.back()[3], -3.1);
  EXPECT_NEAR(p->sample(p->times.back()/2)[3], 0, 1e-10);
}
TEST(WholeBodyPath, RejectsMalformedInput) {
  const auto x = vector_t::Zero(3).eval(), w = vector_t::Ones(3).eval();
  EXPECT_THROW(makeWholeBodyTrajectory({0,0}, {x,x}, w, 2, w), std::invalid_argument);
  EXPECT_THROW(makeWholeBodyTrajectory({0,1}, {x}, w, 2, w), std::invalid_argument);
  EXPECT_THROW(makeWholeBodyTrajectory({0,1}, {x,x}, -w, 2, w), std::invalid_argument);
  EXPECT_THROW(makeWholeBodyTrajectory({0,1}, {x,x}, w, 3, w), std::invalid_argument);
}
TEST(WholeBodyCost, AnalyticGradientHessianAndDisabledCoordinates) {
  auto ref = std::make_shared<WholeBodyReference>();
  WholeBodyStateCost cost(ref);
  EXPECT_FALSE(cost.isActive(0));
  vector_t target(4), state(4), weights(4);
  target << .2,-.1,3.1,.4; state << .5,4,-3.05,.6; weights << 5,0,3,.2;
  ref->solver = makeWholeBodyTrajectory({0,1}, {target,target}, weights, 2, vector_t::Ones(4));
  const TargetTrajectories tcp; const PreComputation pre;
  const auto a = cost.getQuadraticApproximation(.5,state,tcp,pre);
  EXPECT_DOUBLE_EQ(a.f, cost.getValue(.5,state,tcp,pre));
  EXPECT_EQ(a.dfdx[1], 0); EXPECT_EQ(a.dfdxx(1,1), 0);
  const double eps=1e-5;
  for (int i=0;i<4;++i) {
    auto plus=state, minus=state; plus[i]+=eps; minus[i]-=eps;
    EXPECT_NEAR(a.dfdx[i], (cost.getValue(.5,plus,tcp,pre)-cost.getValue(.5,minus,tcp,pre))/(2*eps), 1e-8);
    EXPECT_TRUE(((cost.getQuadraticApproximation(.5,plus,tcp,pre).dfdx-
        cost.getQuadraticApproximation(.5,minus,tcp,pre).dfdx)/(2*eps)).isApprox(a.dfdxx.col(i),1e-7));
  }
  std::unique_ptr<WholeBodyStateCost> clone(cost.clone());
  EXPECT_EQ(clone->getValue(.5,state,tcp,pre), a.f);
  ref->solver.reset(); EXPECT_FALSE(clone->isActive(0));
}
TEST(WholeBodyPath, TcpCanStayFixedWhileBodyPassesThroughKeyframes) {
  vector_t tcp(7); tcp << 0,0,.5,0,0,0,1;
  auto path = makeContinuousTcpPath({tcp,tcp,tcp}, TcpPathLimits{}, 0, {1,1});
  ASSERT_EQ(path.waypoint_times.size(), 3U);
  EXPECT_GT(path.waypoint_times[1],0);
  EXPECT_LT(path.waypoint_times[1],path.times.back());
  for (const auto& p : path.poses) EXPECT_TRUE(p.isApprox(tcp));
}
TEST(WholeBodyPath, LinearBaseAndTcpUseTheSameProgressNotIndependentTimeSplines) {
  vector_t p(7); p << 0,0,.5,0,0,0,1;
  vector_array_t poses{p}; p[0]=.25; poses.push_back(p); p[0]=.6; poses.push_back(p);
  auto tcp=makeContinuousTcpPath(poses,TcpPathLimits{},0,{.25,.35});
  vector_array_t states{vector_t::Zero(3), vector_t::Zero(3),vector_t::Zero(3)};
  states[1][0]=.25; states[2][0]=.6;
  auto body=makeWholeBodyTrajectory(tcp.waypoint_times,states,vector_t::Ones(3),2,vector_t::Constant(3,100));
  for (size_t i=0;i<tcp.times.size();++i)
    EXPECT_NEAR(body->sample(tcp.times[i])[0],tcp.poses[i][0],1e-10);
}

class WholeBodyHandoff : public testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0,nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("whole_body_test");
    wrapped = std::make_shared<ReferenceManager>();
    ref = std::make_unique<TcpReferenceManager>(wrapped,"odom","tcp",5);
    ref->configureWholeBody(true,{"j1","j2"},vector_t::Constant(2,-3),vector_t::Constant(2,3));
    ref->subscribe(node);
    executor.add_node(node->get_node_base_interface());
    pose.resize(7); pose << .2,0,.5,0,0,0,1;
    ASSERT_TRUE(ref->requestReset(pose,10));
    for(int i=0;i<100 && !ref->resetReady();++i) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    ASSERT_TRUE(ref->resetReady());
    ref->preSolverRun(10,15,vector_t::Zero(5));
    ref->setAcceptingTargets(true);
  }
  void TearDown() override { executor.remove_node(node->get_node_base_interface()); }
  zephyr_msgs::action::ExecuteCartesianMotion::Goal goal() {
    zephyr_msgs::action::ExecuteCartesianMotion::Goal g;
    g.target.header.frame_id="odom"; g.target.pose.position.x=.3; g.target.pose.position.z=.5; g.target.pose.orientation.w=1;
    g.base_yaw_weight=5; g.posture_weight=.1;
    g.whole_body_references.resize(1);
    auto& r=g.whole_body_references.front(); r.base_pose.header.frame_id="odom";
    r.base_pose.pose.orientation.z=std::sin(.1); r.base_pose.pose.orientation.w=std::cos(.1);
    r.posture.name={"j2","j1"}; r.posture.position={.2,.1};
    return g;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<ReferenceManager> wrapped;
  std::unique_ptr<TcpReferenceManager> ref;
  vector_t pose;
};
TEST_F(WholeBodyHandoff, ValidatesModelNamesFramesWeightsAndCount) {
  std::string error; auto g=goal(); EXPECT_TRUE(ref->validateWholeBodyGoal(g,error));
  g.whole_body_references.front().posture.name={"j1","j1"}; EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
  g=goal(); g.whole_body_references.front().posture.position[0]=4; EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
  g=goal(); g.whole_body_references.front().base_pose.header.frame_id="base_link"; EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
  g=goal(); g.whole_body_references.front().base_pose.pose.position.z=.1; EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
  g=goal(); g.whole_body_references.clear(); EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
  g=goal(); g.base_yaw_weight=-1; EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
  g=goal(); g.mode=g.PATH; EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
}
TEST_F(WholeBodyHandoff, TcpAndBodySwapTogetherAndResetClearsBoth) {
  auto g=goal(); std::string error; double end=0; geometry_msgs::msg::PoseStamped target;
  ASSERT_TRUE(ref->claimMotion());
  ASSERT_TRUE(ref->startMotion(g,pose,target,end,error,vector_t::Zero(5))) << error;
  EXPECT_FALSE(ref->wholeBodyReference()->solver);
  EXPECT_TRUE(wrapped->getTargetTrajectories().stateTrajectory.back().isApprox(pose));
  ref->preSolverRun(10,15,vector_t::Zero(5));
  auto body=ref->wholeBodyReference()->solver; ASSERT_TRUE(body);
  EXPECT_NEAR(body->times.back(),wrapped->getTargetTrajectories().timeTrajectory.back(),1e-10);
  EXPECT_NEAR(body->states.back()[2],.2,1e-10);
  EXPECT_NEAR(body->states.back()[3],.1,1e-10);
  EXPECT_NEAR(body->states.back()[4],.2,1e-10);
  EXPECT_NEAR(ref->wholeBodyErrors(vector_t::Zero(5))[1],.2,1e-10);
  ref->releaseMotion(); ASSERT_TRUE(ref->beginProfileChange(pose));
  EXPECT_EQ(ref->wholeBodyReference()->solver,body);
  ref->preSolverRun(10,15,vector_t::Zero(5));
  EXPECT_FALSE(ref->wholeBodyReference()->solver);
}
TEST_F(WholeBodyHandoff, FixedArmAcceptsPostureButRejectsBase) {
  ref.reset();
  ref = std::make_unique<TcpReferenceManager>(wrapped,"odom","tcp",2);
  ref->configureWholeBody(false,{"j1","j2"},vector_t::Constant(2,-3),vector_t::Constant(2,3));
  ref->subscribe(node);
  std::string error; auto g=goal(); EXPECT_FALSE(ref->validateWholeBodyGoal(g,error));
  g.base_yaw_weight=0; g.whole_body_references.front().base_pose.header.frame_id.clear();
  EXPECT_TRUE(ref->validateWholeBodyGoal(g,error));
  ref->setAcceptingTargets(true); ASSERT_TRUE(ref->claimMotion());
  geometry_msgs::msg::PoseStamped target; double end;
  ASSERT_TRUE(ref->startMotion(g,pose,target,end,error,vector_t::Zero(2))) << error;
  ref->preSolverRun(0,5,vector_t::Zero(2));
  ASSERT_TRUE(ref->wholeBodyReference()->solver);
  EXPECT_EQ(ref->wholeBodyReference()->solver->weights.size(),2);
  EXPECT_NEAR(ref->wholeBodyReference()->solver->states.back()[0],.1,1e-10);
}
