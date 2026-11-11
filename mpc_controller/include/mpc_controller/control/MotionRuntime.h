// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <mpc_controller/control/ControlSafety.h>
#include <zephyr_msgs/msg/controller_motion_status.hpp>

namespace zephyr::mpc {
using MotionStatus = zephyr_msgs::msg::ControllerMotionStatus;
inline int64_t motionSteadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
// One bounded snapshot from the control loop. No ROS publishing or Action work in RT.
struct MotionSample {
  std::array<double, 7> pose{0, 0, 0, 0, 0, 0, 1};
  int64_t stamp_ns{0}, received_ns{0};
  double reference_time{0};
  uint64_t stop_ack{0};
  uint8_t state{MotionStatus::INACTIVE};
  uint16_t reason{MotionStatus::NONE};
  bool valid{false};
};
struct MotionRuntime {
  TryWriteMailbox<MotionSample> samples;
  std::atomic<uint64_t> stop_request{0}, stop_release{0};
  // Inactivity alone is not evidence that the final hold command was delivered.
  std::atomic_bool active{false};
  std::atomic<uint64_t> activation_epoch{0}, stopped_epoch{0};
  void activate() {
    activation_epoch.fetch_add(1);
    active.store(true);
  }
  bool stopRequested() const { return stop_request.load() != stop_release.load(); }
};
}  // namespace zephyr::mpc
