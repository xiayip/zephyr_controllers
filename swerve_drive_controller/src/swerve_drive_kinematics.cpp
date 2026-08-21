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

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace swerve_drive_controller
{

SwerveDriveKinematics::SwerveDriveKinematics(
  const std::array<std::pair<double, double>, 4> & wheel_positions)
: wheel_positions_(wheel_positions), odometry_{0.0, 0.0, 0.0}
{
}

std::array<WheelCommand, 4> SwerveDriveKinematics::compute_wheel_commands(
  double linear_velocity_x, double linear_velocity_y, double angular_velocity_z,
  const std::array<double, 4> & steering_angles, double steering_limit) const
{
  if (steering_limit <= 0.0 || steering_limit > M_PI) {
    throw std::invalid_argument("steering_limit must be in the range (0, pi]");
  }

  std::array<WheelCommand, 4> wheel_commands{};
  constexpr double velocity_epsilon = 1.0e-6;

  for (std::size_t index = 0; index < wheel_commands.size(); ++index) {
    const auto & [wheel_x, wheel_y] = wheel_positions_[index];
    const double wheel_linear_x = linear_velocity_x - angular_velocity_z * wheel_y;
    const double wheel_linear_y = linear_velocity_y + angular_velocity_z * wheel_x;
    const double wheel_speed = std::hypot(wheel_linear_x, wheel_linear_y);

    if (wheel_speed < velocity_epsilon) {
      continue;
    }

    const double direct_angle = normalize_angle(std::atan2(wheel_linear_y, wheel_linear_x));
    const double reverse_angle = normalize_angle(direct_angle + M_PI);
    const bool direct_is_valid = std::abs(direct_angle) <= steering_limit;
    const bool reverse_is_valid = std::abs(reverse_angle) <= steering_limit;
    const double direct_error = std::abs(normalize_angle(direct_angle - steering_angles[index]));
    const double reverse_error = std::abs(normalize_angle(reverse_angle - steering_angles[index]));

    bool use_reverse = false;
    if (!direct_is_valid && reverse_is_valid) {
      use_reverse = true;
    } else if (direct_is_valid && reverse_is_valid) {
      use_reverse = reverse_error < direct_error;
    } else if (!direct_is_valid && !reverse_is_valid) {
      use_reverse = reverse_error < direct_error;
    }

    const double selected_angle = use_reverse ? reverse_angle : direct_angle;
    wheel_commands[index].steering_angle =
      std::clamp(selected_angle, -steering_limit, steering_limit);
    wheel_commands[index].drive_velocity =
      (use_reverse ? -wheel_speed : wheel_speed) / wheel_radius_;
  }
  return wheel_commands;
}

BaseVelocity SwerveDriveKinematics::estimate_velocity(
  const std::array<double, 4> & wheel_velocities,
  const std::array<double, 4> & steering_angles) const
{
  BaseVelocity velocity;
  double angular_numerator = 0.0;
  double angular_denominator = 0.0;

  for (std::size_t index = 0; index < wheel_velocities.size(); ++index) {
    const auto & [wheel_x, wheel_y] = wheel_positions_[index];
    const double wheel_speed = wheel_velocities[index] * wheel_radius_;
    const double wheel_linear_x = wheel_speed * std::cos(steering_angles[index]);
    const double wheel_linear_y = wheel_speed * std::sin(steering_angles[index]);

    velocity.linear_x += wheel_linear_x;
    velocity.linear_y += wheel_linear_y;
    angular_numerator += -wheel_y * wheel_linear_x + wheel_x * wheel_linear_y;
    angular_denominator += wheel_x * wheel_x + wheel_y * wheel_y;
  }

  velocity.linear_x /= static_cast<double>(wheel_velocities.size());
  velocity.linear_y /= static_cast<double>(wheel_velocities.size());
  if (angular_denominator > std::numeric_limits<double>::epsilon()) {
    velocity.angular_z = angular_numerator / angular_denominator;
  }
  return velocity;
}

OdometryState SwerveDriveKinematics::update_odometry(
  const std::array<double, 4> & wheel_velocities, const std::array<double, 4> & steering_angles,
  double dt)
{
  const BaseVelocity velocity = estimate_velocity(wheel_velocities, steering_angles);

  const double cos_theta = std::cos(odometry_.theta);
  const double sin_theta = std::sin(odometry_.theta);

  const double vx_global = velocity.linear_x * cos_theta - velocity.linear_y * sin_theta;
  const double vy_global = velocity.linear_x * sin_theta + velocity.linear_y * cos_theta;

  odometry_.x += vx_global * dt;
  odometry_.y += vy_global * dt;
  odometry_.theta = normalize_angle(odometry_.theta + velocity.angular_z * dt);
  return odometry_;
}

void SwerveDriveKinematics::setWheelParams(
  std::array<std::pair<double, double>, 4> & wheel_params, double & wheel_radius)
{
  if (wheel_radius <= 0.0) {
    throw std::invalid_argument("wheel_radius must be positive");
  }
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

}  // namespace swerve_drive_controller
