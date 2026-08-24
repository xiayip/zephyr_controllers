#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace mit_impedance_controller {

class BlockedMotionDetector {
public:
  bool configure(const std::vector<double> &position_error_thresholds,
                 const std::vector<double> &stationary_velocity_thresholds,
                 double blocking_duration, std::string &error);

  bool update(const std::vector<double> &position_errors,
              const std::vector<double> &velocities, double period_seconds);

  void reset() noexcept;
  std::size_t last_triggered_joint() const noexcept;

private:
  std::vector<double> position_error_thresholds_;
  std::vector<double> stationary_velocity_thresholds_;
  std::vector<double> blocked_durations_;
  std::vector<bool> latched_;
  double blocking_duration_{0.0};
  std::size_t last_triggered_joint_{std::numeric_limits<std::size_t>::max()};
  bool configured_{false};
};

} // namespace mit_impedance_controller
