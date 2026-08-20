#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "gravity_compensation_model/gravity_compensation_model.hpp"
#include "gravity_compensation_model/gravity_feedforward_shaper.hpp"

namespace {

constexpr char kPendulumUrdf[] = R"(
<robot name="pendulum">
  <link name="base_link"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2"/>
  </joint>
</robot>)";

TEST(GravityCompensationModelTest, ComputesPendulumGravity) {
  gravity_compensation_model::GravityCompensationModel model;
  std::string error;
  ASSERT_TRUE(model.configure(kPendulumUrdf, {"joint1"},
                              Eigen::Vector3d(0.0, 0.0, -9.80665), error))
      << error;
  ASSERT_EQ(model.lower_position_limits().size(), 1);
  ASSERT_EQ(model.upper_position_limits().size(), 1);
  EXPECT_NEAR(model.lower_position_limits()[0], -3.14, 1e-12);
  EXPECT_NEAR(model.upper_position_limits()[0], 3.14, 1e-12);

  Eigen::VectorXd q(1);
  Eigen::VectorXd torque(1);
  q[0] = 0.0;
  ASSERT_TRUE(model.compute(q, torque));
  EXPECT_NEAR(torque[0], -9.80665, 1e-6);

  q[0] = M_PI_2;
  ASSERT_TRUE(model.compute(q, torque));
  EXPECT_NEAR(torque[0], 0.0, 1e-6);
}

TEST(GravityCompensationModelTest, RejectsInvalidControlledJoint) {
  gravity_compensation_model::GravityCompensationModel model;
  std::string error;
  EXPECT_FALSE(model.configure(kPendulumUrdf, {"missing_joint"},
                               Eigen::Vector3d(0.0, 0.0, -9.80665), error));
  EXPECT_NE(error.find("missing_joint"), std::string::npos);
}

TEST(GravityFeedforwardShaperTest, BlendsAndLimitsGravityTorque) {
  gravity_compensation_model::GravityFeedforwardShaper shaper;
  gravity_compensation_model::GravityFeedforwardShaperConfig config;
  config.scale = Eigen::VectorXd::Constant(1, 2.0);
  config.torque_limit = Eigen::VectorXd::Constant(1, 10.0);
  config.torque_rate_limit = Eigen::VectorXd::Constant(1, 100.0);
  config.transition_duration = 1.0;
  std::string error;
  ASSERT_TRUE(shaper.configure(config, error)) << error;

  Eigen::VectorXd initial = Eigen::VectorXd::Constant(1, 1.0);
  Eigen::VectorXd gravity = Eigen::VectorXd::Constant(1, 4.0);
  Eigen::VectorXd output(1);
  ASSERT_TRUE(shaper.reset(initial));
  ASSERT_TRUE(shaper.update(gravity, 0.5, output));
  EXPECT_DOUBLE_EQ(output[0], 4.5);
  ASSERT_TRUE(shaper.update(gravity, 0.5, output));
  EXPECT_DOUBLE_EQ(output[0], 8.0);

  gravity[0] = 20.0;
  ASSERT_TRUE(shaper.update(gravity, 0.1, output));
  EXPECT_DOUBLE_EQ(output[0], 10.0);
}

TEST(GravityFeedforwardShaperTest, AppliesTorqueRateLimit) {
  gravity_compensation_model::GravityFeedforwardShaper shaper;
  gravity_compensation_model::GravityFeedforwardShaperConfig config;
  config.scale = Eigen::VectorXd::Ones(1);
  config.torque_limit = Eigen::VectorXd::Constant(1, 20.0);
  config.torque_rate_limit = Eigen::VectorXd::Constant(1, 2.0);
  config.transition_duration = 0.0;
  std::string error;
  ASSERT_TRUE(shaper.configure(config, error)) << error;

  Eigen::VectorXd initial = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd gravity = Eigen::VectorXd::Constant(1, 10.0);
  Eigen::VectorXd output(1);
  ASSERT_TRUE(shaper.reset(initial));
  ASSERT_TRUE(shaper.update(gravity, 0.25, output));
  EXPECT_DOUBLE_EQ(output[0], 0.5);
}

} // namespace
