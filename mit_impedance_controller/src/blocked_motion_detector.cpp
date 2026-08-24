#include "mit_impedance_controller/blocked_motion_detector.hpp"

#include <algorithm>
#include <cmath>

namespace mit_impedance_controller {
namespace {

bool finite_positive(const double value) {
  return std::isfinite(value) && value > 0.0;
}

bool finite_nonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

} // namespace

bool BlockedMotionDetector::configure(
    const std::vector<double> &position_error_thresholds,
    const std::vector<double> &stationary_velocity_thresholds,
    const double blocking_duration, std::string &error) {
  configured_ = false;
  if (position_error_thresholds.empty() ||
      position_error_thresholds.size() !=
          stationary_velocity_thresholds.size() ||
      !std::all_of(position_error_thresholds.begin(),
                   position_error_thresholds.end(), finite_positive) ||
      !std::all_of(stationary_velocity_thresholds.begin(),
                   stationary_velocity_thresholds.end(), finite_nonnegative) ||
      !finite_positive(blocking_duration)) {
    error = "blocked-motion thresholds must be finite, valid, and have one "
            "value per joint";
    return false;
  }

  position_error_thresholds_ = position_error_thresholds;
  stationary_velocity_thresholds_ = stationary_velocity_thresholds;
  blocked_durations_.assign(position_error_thresholds.size(), 0.0);
  latched_.assign(position_error_thresholds.size(), false);
  blocking_duration_ = blocking_duration;
  last_triggered_joint_ = std::numeric_limits<std::size_t>::max();
  configured_ = true;
  error.clear();
  return true;
}

bool BlockedMotionDetector::update(const std::vector<double> &position_errors,
                                   const std::vector<double> &velocities,
                                   const double period_seconds) {
  last_triggered_joint_ = std::numeric_limits<std::size_t>::max();
  if (!configured_ || position_errors.size() != blocked_durations_.size() ||
      velocities.size() != blocked_durations_.size() ||
      !finite_nonnegative(period_seconds)) {
    return false;
  }

  for (std::size_t index = 0; index < blocked_durations_.size(); ++index) {
    if (!std::isfinite(position_errors[index]) ||
        !std::isfinite(velocities[index])) {
      blocked_durations_[index] = 0.0;
      latched_[index] = false;
      continue;
    }

    const bool blocked =
        std::abs(position_errors[index]) >= position_error_thresholds_[index] &&
        std::abs(velocities[index]) <= stationary_velocity_thresholds_[index];
    if (!blocked) {
      blocked_durations_[index] = 0.0;
      latched_[index] = false;
      continue;
    }

    blocked_durations_[index] += period_seconds;
    if (!latched_[index] && blocked_durations_[index] >= blocking_duration_) {
      latched_[index] = true;
      last_triggered_joint_ = index;
      return true;
    }
  }
  return false;
}

void BlockedMotionDetector::reset() noexcept {
  std::fill(blocked_durations_.begin(), blocked_durations_.end(), 0.0);
  std::fill(latched_.begin(), latched_.end(), false);
  last_triggered_joint_ = std::numeric_limits<std::size_t>::max();
}

std::size_t BlockedMotionDetector::last_triggered_joint() const noexcept {
  return last_triggered_joint_;
}

} // namespace mit_impedance_controller
