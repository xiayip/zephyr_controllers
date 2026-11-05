// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
#pragma once

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

namespace zephyr::mpc {
struct TcpTrajectorySample {
  double stamp_sec{0.0};
  std::array<double, 3> position{};
  std::array<double, 4> quat_xyzw{0, 0, 0, 1};
};
class TcpTrajectoryRecorder {
public:
  ~TcpTrajectoryRecorder();
  void setEnabled(bool enabled, const std::string &directory);
  bool enabled() const { return enabled_.load(); }
  void appendActual(const TcpTrajectorySample &sample);
  void appendPredicted(const TcpTrajectorySample &sample);

private:
  std::atomic_bool enabled_{false};
  std::mutex mutex_;
  std::string directory_;
  std::vector<TcpTrajectorySample> actual_, predicted_;
};
} // namespace zephyr::mpc
