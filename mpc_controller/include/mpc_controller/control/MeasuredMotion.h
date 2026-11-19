// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace zephyr::mpc {
struct MotionSpeeds {
  enum Axis { TcpLinear, TcpAngular, ArmJoint, BaseLinear, BaseAngular, Count };
  // m/s, rad/s, rad/s, m/s, rad/s. Invalid is NOT a zero-speed observation.
  std::array<double, Count> values{};
  bool valid{false};
};

/** Measured FK/joint/base total variation over 50 ms, not commanded velocity.
 * Accumulate absolute motion each cycle so an out-and-back is not called rest.
 * Configure outside RT; update/reset allocate nothing. This is an arrival gate,
 * not a safety-rated standstill detector or a replacement for hardware freshness.
 */
class MeasuredMotion {
public:
  void configure(size_t joints, bool mobile, bool gripper = false) {
    if (!joints) throw std::invalid_argument("MeasuredMotion needs arm joints");
    offset_ = mobile ? 3 : 0;
    arm_end_ = joints + offset_;
    gripper_ = gripper;
    previous_state_.setZero(arm_end_ + (gripper ? 1 : 0));
    reset();
  }
  void reset() { initialized_ = false; speeds_ = {}; distance_.fill(0); gripper_distance_ = gripper_speed_ = 0; }
  double gripperSpeed() const { return gripper_speed_; }
  const MotionSpeeds& speeds() const { return speeds_; }
  void update(double time, const Eigen::Ref<const Eigen::VectorXd>& state,
              const std::array<double, 7>& pose) {
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> p(pose.data());
    if (state.size() != previous_state_.size() || state.size() <= offset_ ||
        !state.allFinite() || !p.allFinite() || !std::isfinite(time) ||
        !std::isfinite(p.tail<4>().norm()) || p.tail<4>().norm() < 1e-9) {
      reset(); return;
    }
    const Eigen::Quaterniond q = Eigen::Quaterniond(p[6], p[3], p[4], p[5]).normalized();
    if (initialized_ && time > previous_time_ && time - previous_time_ <= 0.1) {
      distance_[MotionSpeeds::TcpLinear] += (p.head<3>() - previous_position_).norm();
      distance_[MotionSpeeds::TcpAngular] += previous_rotation_.angularDistance(q);
      double joint_step = 0;
      for (Eigen::Index i = offset_; i < arm_end_; ++i)
        joint_step = std::max(joint_step, std::abs(state[i] - previous_state_[i]));
      distance_[MotionSpeeds::ArmJoint] += joint_step;
      if (gripper_) gripper_distance_ += std::abs(state[arm_end_] - previous_state_[arm_end_]);
      if (offset_) {
        distance_[MotionSpeeds::BaseLinear] += (state.head<2>() - previous_state_.head<2>()).norm();
        distance_[MotionSpeeds::BaseAngular] += std::abs(std::remainder(state[2] - previous_state_[2], 2 * M_PI));
      }
      if (time - window_start_ >= 0.05) {
        speeds_.valid = true;
        for (size_t i = 0; i < distance_.size(); ++i) {
          speeds_.values[i] = distance_[i] / (time - window_start_);
          speeds_.valid = speeds_.valid && std::isfinite(speeds_.values[i]);
        }
        gripper_speed_ = gripper_distance_ / (time - window_start_);
        gripper_distance_ = 0;
        distance_.fill(0);
        window_start_ = time;
      }
    } else {
      reset(); initialized_ = true; window_start_ = time;
    }
    previous_state_ = state;
    previous_position_ = p.head<3>();
    previous_rotation_ = q;
    previous_time_ = time;
  }
private:
  Eigen::VectorXd previous_state_;
  Eigen::Vector3d previous_position_;
  Eigen::Quaterniond previous_rotation_;
  Eigen::Index offset_{0};
  Eigen::Index arm_end_{0};
  bool gripper_{false};
  double gripper_distance_{0}, gripper_speed_{0};
  double previous_time_{0}, window_start_{0};
  bool initialized_{false};
  MotionSpeeds speeds_;
  std::array<double, MotionSpeeds::Count> distance_{};
};
}  // namespace zephyr::mpc
