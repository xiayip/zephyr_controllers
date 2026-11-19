// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <mpc_controller/control/Gripper.h>
#include <mpc_controller/control/UrdfMimic.h>
#include <mpc_controller/control/MeasuredMotion.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

using namespace zephyr::mpc;
namespace {
const std::string xml = R"(<robot name="gripper">
<link name="base"/><link name="control"/><link name="finger"/><link name="tip"/>
<joint name="closure" type="revolute"><parent link="base"/><child link="control"/>
<axis xyz="0 0 1"/><limit lower="0" upper="1" effort="1" velocity="0.5"/></joint>
<joint name="finger_joint" type="revolute"><parent link="base"/><child link="finger"/>
<axis xyz="0 0 1"/><limit lower="-0.7" upper="0" effort="1" velocity="1"/>
<mimic joint="closure" multiplier="0.7" offset="-0.7"/></joint>
<joint name="tip_joint" type="revolute"><parent link="finger"/><child link="tip"/>
<origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
<mimic joint="finger_joint" multiplier="-1.5" offset="0.1"/></joint></robot>)";
}
TEST(Gripper, BoundsRateAndCapUnits) {
  GripperConfig c; c.joint="closure"; c.command_prefix="gripper";
  EXPECT_NO_THROW(c.validate());
  EXPECT_NEAR(c.step(.5, 1, .01), .505, 1e-12);
  EXPECT_NEAR(c.step(.5, -1, .01), .495, 1e-12);
  EXPECT_EQ(c.step(0, -1, .01), 0);
  EXPECT_EQ(c.step(1, 2, .01), 1);
  EXPECT_THROW(c.step(NAN,.5,.01), std::invalid_argument);
  c.speed_limit=-.2; EXPECT_THROW(c.validate(),std::invalid_argument);
}
TEST(GripperModel, OneIndependentDofAndCorrectChainedMimic) {
  auto independent = makeMimicGeometry(xml,false,{"finger_joint", "tip_joint"});
  auto model = makeMimicGeometry(xml,false,{});
  const auto relations = readMimicRelations(xml);
  const auto expansion = mimicExpansion(independent.getModel(),model.getModel(),relations);
  const auto& m=model.getModel(); auto& data=model.getData();
  ASSERT_EQ(independent.getModel().nq,1);
  ASSERT_EQ(m.nq,3); ASSERT_EQ(m.nv,3); // Geometry coordinates, NOT solver DoFs.
  const auto f=m.getFrameId("tip");
  for(double c : {0.,.3,.5,1.}) {
    const Eigen::VectorXd q=expansion.matrix*Eigen::VectorXd::Constant(1,c)+expansion.offset;
    pinocchio::framesForwardKinematics(m,data,q);
    const double angle=.7*c-.7;
    EXPECT_NEAR(data.oMf[f].translation().x(),std::cos(angle),1e-12);
    EXPECT_NEAR(data.oMf[f].translation().y(),std::sin(angle),1e-12);
    EXPECT_NEAR(std::atan2(data.oMf[f].rotation()(1,0),data.oMf[f].rotation()(0,0)), -.5*angle+.1,1e-12);
    Eigen::Matrix<double,6,Eigen::Dynamic> jac(6,m.nv); jac.setZero();
    pinocchio::computeFrameJacobian(m,data,q,f,pinocchio::LOCAL_WORLD_ALIGNED,jac);
    const auto reduced_jac=(jac*expansion.matrix).eval();
    EXPECT_NEAR(reduced_jac(0,0),-.7*std::sin(angle),1e-10);
    EXPECT_NEAR(reduced_jac(1,0), .7*std::cos(angle),1e-10);
  }
  auto mobile=makeMimicGeometry(xml,true,{});
  auto reduced=makeMimicGeometry(xml,true,{"finger_joint","tip_joint"});
  EXPECT_EQ(mobile.getModel().nq,6); EXPECT_EQ(reduced.getModel().nq,4);
  const auto map=mimicExpansion(reduced.getModel(),mobile.getModel(),relations);
  EXPECT_TRUE(map.matrix.topLeftCorner(3,3).isIdentity());
  EXPECT_THROW(mimicExpansion(makeMimicGeometry(xml,false,{"closure"}).getModel(),m,relations),std::invalid_argument);
}
TEST(GripperModel, UrdfOnlyHalfClosureAndInvalidMimic) {
  const auto relations=readMimicRelations(xml);
  ASSERT_EQ(relations.size(),2U);
  EXPECT_EQ(relations[0].source,"closure"); EXPECT_EQ(relations[1].source,"closure");
  EXPECT_NEAR(relations[0].multiplier*.5+relations[0].offset,-.35,1e-12);
  EXPECT_NEAR(relations[1].multiplier*.5+relations[1].offset,.625,1e-12);
  auto cycle=xml;
  cycle.replace(cycle.find("mimic joint=\"closure\""),21,"mimic joint=\"tip_joint\"");
  EXPECT_THROW(readMimicRelations(cycle),std::invalid_argument);
}
TEST(Gripper, MeasuredSpeedDoesNotUseArmRadians) {
  MeasuredMotion m; m.configure(1,false,true);
  Eigen::VectorXd q(2); q<<0,0;
  const std::array<double,7> pose{0,0,0,0,0,0,1};
  m.update(1,q,pose); q[1]=.03; m.update(1.06,q,pose);
  ASSERT_TRUE(m.speeds().valid);
  EXPECT_EQ(m.speeds().values[MotionSpeeds::ArmJoint],0);
  EXPECT_NEAR(m.gripperSpeed(),.5,1e-12);
}
