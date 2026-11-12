// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <mutex>
#include <string>
#include <zephyr_msgs/msg/controller_motion_status.hpp>

namespace zephyr::mpc_rviz {
// A view of the controller's status, not a second state machine or enable switch.
class CommandStatus {
 public:
  using Clock = std::chrono::steady_clock;
  using Message = zephyr_msgs::msg::ControllerMotionStatus;
  struct View { bool ready; std::string message; };
  void update(const Message& status, Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    received_ = now;
    have_status_ = true;
    ready_ = status.state == Message::IDLE && status.reason == Message::NONE &&
        status.feedback_valid && status.goal_id.empty();
    message_ = status.message;
  }
  View view(Clock::time_point now = Clock::now()) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_status_) return {false, "Waiting for controller motion_status"};
    if (now - received_ > std::chrono::seconds(1))
      return {false, "Controller status timeout; sending disabled"};
    return {ready_, message_};
  }
 private:
  mutable std::mutex mutex_;
  Clock::time_point received_{};
  bool have_status_{false}, ready_{false};
  std::string message_;
};
}  // namespace zephyr::mpc_rviz
