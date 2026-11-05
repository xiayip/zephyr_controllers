// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.
#include "mpc_controller/control/TcpTrajectoryRecorder.h"

#include <filesystem>
#include <fstream>
#include <iomanip>

namespace zephyr::mpc {
// CSV recording belongs to the component visualization worker.

TcpTrajectoryRecorder::~TcpTrajectoryRecorder() {
  try {
    setEnabled(false, directory_);
  } catch (...) {
  }
}
std::string TcpTrajectoryRecorder::setEnabled(bool enabled,
                                             const std::string &directory) {
  std::vector<TcpTrajectorySample> actual, predicted;
  std::string output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled == enabled_)
      return {};
    if (enabled) {
      directory_ = directory;
      actual_.clear();
      predicted_.clear();
      actual_.reserve(10000);
      predicted_.reserve(10000);
      enabled_ = true;
      return {};
    }
    enabled_ = false;
    actual.swap(actual_);
    predicted.swap(predicted_);
    output = directory_;
  }
  try {
    std::filesystem::create_directories(output);
    const auto write = [&](const char *name,
                           const std::vector<TcpTrajectorySample> &samples) {
      std::ofstream file(std::filesystem::path(output) / name);
      file.exceptions(std::ios::badbit | std::ios::failbit);
      file << "stamp_sec,x,y,z,qx,qy,qz,qw\n" << std::setprecision(17);
      for (const auto &s : samples) {
        file << s.stamp_sec;
        for (double v : s.position)
          file << ',' << v;
        for (double v : s.quat_xyzw)
          file << ',' << v;
        file << '\n';
      }
      file.close(); // Detect buffered write failures before reporting success.
    };
    write("real_tcp.csv", actual);
    write("pred_tcp.csv", predicted);
    return {};
  } catch (const std::exception &error) {
    return error.what();
  }
}
void TcpTrajectoryRecorder::appendActual(const TcpTrajectorySample &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_)
    actual_.push_back(sample);
}
void TcpTrajectoryRecorder::appendPredicted(const TcpTrajectorySample &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_)
    predicted_.push_back(sample);
}
} // namespace zephyr::mpc
