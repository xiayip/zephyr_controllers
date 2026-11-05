#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "gtest/gtest.h"

namespace swerve_drive_controller
{

class SwerveDriveKinematicsTest : public ::testing::Test
{
protected:
  SwerveDriveKinematicsTest()
  : kinematics_({{{0.25, 0.19}, {0.25, -0.19}, {-0.25, 0.19}, {-0.25, -0.19}}})
  {
    double wheel_radius = 0.09;
    std::array<std::pair<double, double>, 4> wheel_positions = {{
      {0.25, 0.19}, {0.25, -0.19}, {-0.25, 0.19}, {-0.25, -0.19}}};
    kinematics_.setWheelParams(wheel_positions, wheel_radius);
  }

  SwerveDriveKinematics kinematics_;
  const std::array<double, 4> centered_steering_{{0.0, 0.0, 0.0, 0.0}};
};

TEST_F(SwerveDriveKinematicsTest, ComputesStraightDrive)
{
  const auto commands =
    kinematics_.compute_wheel_commands(0.9, 0.0, 0.0, centered_steering_);

  for (const auto & command : commands) {
    EXPECT_NEAR(command.drive_velocity, 10.0, 1.0e-9);
    EXPECT_NEAR(command.steering_angle, 0.0, 1.0e-9);
  }
}

TEST_F(SwerveDriveKinematicsTest, ComputesCoordinatedTurn)
{
  const auto commands =
    kinematics_.compute_wheel_commands(0.6, 0.0, 0.8, centered_steering_);

  EXPECT_LT(commands[0].drive_velocity, commands[1].drive_velocity);
  EXPECT_LT(commands[2].drive_velocity, commands[3].drive_velocity);
  EXPECT_GT(commands[0].steering_angle, commands[1].steering_angle);
  EXPECT_GT(commands[0].steering_angle, 0.0);
  EXPECT_GT(commands[1].steering_angle, 0.0);
  EXPECT_NEAR(commands[2].steering_angle, -commands[0].steering_angle, 1.0e-9);
  EXPECT_NEAR(commands[3].steering_angle, -commands[1].steering_angle, 1.0e-9);
}

TEST_F(SwerveDriveKinematicsTest, ComputesOutwardPatternForInPlaceRotation)
{
  const auto commands =
    kinematics_.compute_wheel_commands(0.0, 0.0, 0.8, centered_steering_);

  EXPECT_LT(commands[0].steering_angle, 0.0);
  EXPECT_GT(commands[1].steering_angle, 0.0);
  EXPECT_GT(commands[2].steering_angle, 0.0);
  EXPECT_LT(commands[3].steering_angle, 0.0);
  EXPECT_LT(commands[0].drive_velocity, 0.0);
  EXPECT_GT(commands[1].drive_velocity, 0.0);
  EXPECT_LT(commands[2].drive_velocity, 0.0);
  EXPECT_GT(commands[3].drive_velocity, 0.0);
}

TEST_F(SwerveDriveKinematicsTest, ComputesCrabMotion)
{
  const auto commands =
    kinematics_.compute_wheel_commands(0.5, 0.2, 0.0, centered_steering_);
  const double expected_steering = std::atan2(0.2, 0.5);

  for (const auto & command : commands) {
    EXPECT_NEAR(command.steering_angle, expected_steering, 1.0e-9);
    EXPECT_NEAR(command.drive_velocity, std::hypot(0.5, 0.2) / 0.09, 1.0e-9);
  }
}

TEST_F(SwerveDriveKinematicsTest, RecoversCommandedBaseVelocity)
{
  const auto commands =
    kinematics_.compute_wheel_commands(0.6, 0.1, 0.8, centered_steering_);
  std::array<double, 4> wheel_velocities{};
  std::array<double, 4> steering_positions{};
  for (std::size_t index = 0; index < commands.size(); ++index) {
    wheel_velocities[index] = commands[index].drive_velocity;
    steering_positions[index] = commands[index].steering_angle;
  }

  const auto velocity = kinematics_.estimate_velocity(wheel_velocities, steering_positions);

  EXPECT_NEAR(velocity.linear_x, 0.6, 1.0e-9);
  EXPECT_NEAR(velocity.linear_y, 0.1, 1.0e-9);
  EXPECT_NEAR(velocity.angular_z, 0.8, 1.0e-9);
}

TEST_F(SwerveDriveKinematicsTest, ReverseAndRearDiagonalUseNegativeWheelVelocity)
{
  for (const double vy : {-0.2, 0.0, 0.2}) {
    const auto commands = kinematics_.compute_wheel_commands(-0.5, vy, 0.0, centered_steering_);
    for (const auto & command : commands) {
      EXPECT_LT(command.drive_velocity, 0.0);
      EXPECT_NEAR(command.steering_angle, std::atan2(-vy, 0.5), 1.0e-9);
    }
  }
}

TEST_F(SwerveDriveKinematicsTest, LateralNoiseKeepsNearestSteeringBranchWithinLimits)
{
  // Either rolling direction at either steering boundary remains stable even
  // when tiny longitudinal corrections cross vx=0 repeatedly.
  for (const double boundary : {-M_PI_2, M_PI_2}) {
    for (const double vy : {-0.15, 0.15}) {
      std::array<double, 4> steering;
      steering.fill(boundary);
      for (const double vx : {-0.001, 0.001, -0.001, 0.0}) {
        const auto commands = kinematics_.compute_wheel_commands(
          vx, vy, 0.0, steering, M_PI_2, 0.05, 0.1);
        for (std::size_t i = 0; i < commands.size(); ++i) {
          const auto & command = commands[i];
          EXPECT_LE(std::abs(command.steering_angle), M_PI_2);
          EXPECT_LT(std::abs(command.steering_angle - boundary), 0.01);
          EXPECT_GT(command.drive_velocity * std::sin(boundary) * vy, 0.0);
          EXPECT_LE(std::abs(command.drive_velocity), std::hypot(vx, vy) / 0.09);
          steering[i] = command.steering_angle;
        }
      }
    }
  }
}

TEST_F(SwerveDriveKinematicsTest, HysteresisDoesNotBlockAnIntentionalDirectionChange)
{
  std::array<double, 4> steering;
  steering.fill(M_PI_2);
  const auto commands = kinematics_.compute_wheel_commands(
    -0.15, 0.15, 0.0, steering, M_PI_2, 0.05, 0.1);
  for (const auto & command : commands) {
    EXPECT_NEAR(command.steering_angle, -M_PI_4, 1.0e-9);
    EXPECT_NEAR(command.drive_velocity, 0.0, 1.0e-9);
  }
}

TEST_F(SwerveDriveKinematicsTest, AllWheelsWaitForLateralAlignment)
{
  for (const double direction : {-1.0, 1.0}) {
    for (const double fraction : {0.0, 0.5, 1.0}) {
      std::array<double, 4> steering;
      steering.fill(direction * fraction * M_PI_2);
      const auto commands = kinematics_.compute_wheel_commands(
        0.0, direction * 0.15, 0.0, steering, M_PI_2, 0.05, 0.1);
      for (const auto & command : commands) {
        EXPECT_NEAR(command.drive_velocity,
          fraction == 1.0 ? 0.15 / 0.09 : 0.0, 1.0e-9);
      }
    }
  }
  const std::array<double, 4> one_unaligned{{M_PI_2, M_PI_2, 0.0, M_PI_2}};
  const auto commands = kinematics_.compute_wheel_commands(
    0.0, 0.15, 0.0, one_unaligned, M_PI_2, 0.05, 0.1);
  for (const auto & command : commands) {
    EXPECT_EQ(command.drive_velocity, 0.0);
    EXPECT_NEAR(command.steering_angle, M_PI_2, 1.0e-9);
  }
}

TEST_F(SwerveDriveKinematicsTest, RejectsInvalidHysteresis)
{
  for (const double band : {-0.1, 0.3, std::nan("")}) {
    EXPECT_THROW(kinematics_.compute_wheel_commands(
      0.0, 0.1, 0.0, centered_steering_, M_PI_2, band), std::invalid_argument);
  }
  for (const double tolerance : {-0.1, M_PI, std::nan("")}) {
    EXPECT_THROW(kinematics_.compute_wheel_commands(
      0.0, 0.1, 0.0, centered_steering_, M_PI_2, 0.05, tolerance), std::invalid_argument);
  }
}

TEST_F(SwerveDriveKinematicsTest, LateralFeedbackWithFiniteSteeringRateMakesProgress)
{
  // Reproduce the mock's 200 Hz loop and steering limit. Longitudinal feedback
  // must not cause repeated +/-90 degree reversals during lateral travel.
  std::array<double, 4> steering{};
  OdometryState odom{};
  constexpr double dt = 0.005;
  for (int step = 0; step < 600; ++step) {
    const auto commands = kinematics_.compute_wheel_commands(
      -3.0 * odom.x, 0.15, 0.0, steering, M_PI_2, 0.05, 0.1);
    std::array<double, 4> velocity{};
    for (std::size_t i = 0; i < commands.size(); ++i) {
      steering[i] += std::clamp(commands[i].steering_angle - steering[i], -6.28 * dt, 6.28 * dt);
      velocity[i] = commands[i].drive_velocity;
    }
    odom = kinematics_.update_odometry(velocity, steering, dt);
  }
  EXPECT_GT(odom.y, 0.40);
  EXPECT_LT(std::abs(odom.x), 0.001);
  EXPECT_NEAR(odom.theta, 0.0, 1.0e-12);
}

}  // namespace swerve_drive_controller
