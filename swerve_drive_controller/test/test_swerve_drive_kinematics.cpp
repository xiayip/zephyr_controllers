#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

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

}  // namespace swerve_drive_controller