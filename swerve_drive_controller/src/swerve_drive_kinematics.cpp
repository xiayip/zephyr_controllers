// Copyright 2025 ros2_control development team
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

namespace swerve_drive_controller
{

SwerveDriveKinematics::SwerveDriveKinematics(
  const std::array<std::pair<double, double>, 4> & wheel_positions)
: wheel_positions_(wheel_positions), odometry_{0.0, 0.0, 0.0}
{
}

std::array<WheelCommand, 4> SwerveDriveKinematics::compute_wheel_commands(
  double linear_velocity_x, double linear_velocity_y, double angular_velocity_z, std::array<double, 4> steering_angles)
{
  std::array<WheelCommand, 4> wheel_commands;

  // wx = W/2, wy = L/2

  for (std::size_t i = 0; i < 4; i++) {
    const auto & [wx, wy] = wheel_positions_[i];

    double vx = linear_velocity_x - angular_velocity_z * wy;
    double vy = linear_velocity_y + angular_velocity_z * wx;

    double v_i = std::hypot(vx, vy);
    double theta_i = std::atan2(vy, vx);
    // TODO: read limitAngle from urdf
    auto [theta, v] = limitAngle(theta_i, -170 * M_PI / 180, 170 * M_PI / 180, v_i);

    double target = theta;
    double actual = steering_angles[i];
    double error = target - actual;
  
    if (abs(error)> M_2_PI){
      theta += M_PI;
      v = -v;
    }
    wheel_commands[i].drive_velocity = v / wheel_radius_;
    wheel_commands[i].steering_angle = theta;

   
  }
  return wheel_commands;
}

OdometryState SwerveDriveKinematics::update_odometry(
  const std::array<double, 4> & wheel_velocities, const std::array<double, 4> & steering_angles,
  double dt)
{
  // Compute robot-centric velocity (assuming perfect wheel control)
  double vx_sum = 0.0, vy_sum = 0.0, wz_sum = 0.0;
  for (std::size_t i = 0; i < 4; i++) {
    double vx = (wheel_velocities[i] * wheel_radius_) * std::cos(steering_angles[i]);
    double vy = (wheel_velocities[i] * wheel_radius_) * std::sin(steering_angles[i]);

    // Accumulate contributions to linear and angular velocities
    vx_sum += vx;
    vy_sum += vy;

    wz_sum += (vy * wheel_positions_[i].first - vx * wheel_positions_[i].second);
  }

  double vx_robot = vx_sum / 4.0;
  double vy_robot = vy_sum / 4.0;

  double wz_denominator = 0.0;
  for (std::size_t i = 0; i < 4; i++) {
    wz_denominator +=
      (wheel_positions_[i].first * wheel_positions_[i].first +
      wheel_positions_[i].second * wheel_positions_[i].second);
  }
  double wz_robot = wz_sum / wz_denominator;

  // Transform velocities to global frame
  double cos_theta = std::cos(odometry_.theta);
  double sin_theta = std::sin(odometry_.theta);

  double vx_global = vx_robot * cos_theta - vy_robot * sin_theta;
  double vy_global = vx_robot * sin_theta + vy_robot * cos_theta;

  // Integrate to compute new position and orientation
  odometry_.x += vx_global * dt;
  odometry_.y += vy_global * dt;
  odometry_.theta = normalize_angle(odometry_.theta + wz_robot * dt);
  return odometry_;
}

void SwerveDriveKinematics::setWheelParams(std::array<std::pair<double, double>, 4> & wheel_params, double &wheel_radius)
{
  wheel_positions_ = wheel_params;
  wheel_radius_ = wheel_radius;
}

double SwerveDriveKinematics::normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::pair<double, double> SwerveDriveKinematics::limitAngle(double angle, double min_angle, double max_angle, double speed)
{
  if (angle < min_angle) {
    angle += M_PI;
    speed = -speed;
  } else if(angle > max_angle) {
    angle -= M_PI;
    speed = -speed;
  } else {
  }
  return {angle, speed};
}

}  // namespace swerve_drive_controller
