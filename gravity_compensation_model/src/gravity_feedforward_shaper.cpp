#include "gravity_compensation_model/gravity_feedforward_shaper.hpp"

#include <algorithm>
#include <cmath>

namespace gravity_compensation_model {
namespace {

bool finite_positive_vector(const Eigen::VectorXd &values) {
  return values.size() > 0 && values.allFinite() &&
         (values.array() > 0.0).all();
}

} // namespace

bool GravityFeedforwardShaper::configure(
    const GravityFeedforwardShaperConfig &config, std::string &error) {
  configured_ = false;
  reset_ = false;

  if (config.scale.size() == 0 || !config.scale.allFinite() ||
      (config.scale.array() < 0.0).any()) {
    error = "scale must contain finite non-negative values";
    return false;
  }
  if (config.torque_limit.size() != config.scale.size() ||
      !finite_positive_vector(config.torque_limit)) {
    error = "torque_limit must contain one finite positive value per joint";
    return false;
  }
  if (config.torque_rate_limit.size() != config.scale.size() ||
      !finite_positive_vector(config.torque_rate_limit)) {
    error =
        "torque_rate_limit must contain one finite positive value per joint";
    return false;
  }
  if (!std::isfinite(config.transition_duration) ||
      config.transition_duration < 0.0) {
    error = "transition_duration must be finite and non-negative";
    return false;
  }

  scale_ = config.scale;
  torque_limit_ = config.torque_limit;
  torque_rate_limit_ = config.torque_rate_limit;
  transition_source_.setZero(config.scale.size());
  target_.setZero(config.scale.size());
  current_.setZero(config.scale.size());
  transition_duration_ = config.transition_duration;
  transition_progress_ = 0.0;
  configured_ = true;
  error.clear();
  return true;
}

bool GravityFeedforwardShaper::reset(
    const Eigen::Ref<const Eigen::VectorXd> &initial_torque) noexcept {
  if (!configured_ || initial_torque.size() != scale_.size() ||
      !initial_torque.allFinite()) {
    return false;
  }
  for (Eigen::Index index = 0; index < initial_torque.size(); ++index) {
    const double limit = torque_limit_[index];
    current_[index] = std::clamp(initial_torque[index], -limit, limit);
  }
  transition_source_ = current_;
  transition_progress_ = transition_duration_ == 0.0 ? 1.0 : 0.0;
  reset_ = true;
  return true;
}

bool GravityFeedforwardShaper::update(
    const Eigen::Ref<const Eigen::VectorXd> &gravity_torque,
    const double period_seconds,
    Eigen::Ref<Eigen::VectorXd> shaped_torque) noexcept {
  if (!configured_ || !reset_ || gravity_torque.size() != scale_.size() ||
      shaped_torque.size() != scale_.size() || !gravity_torque.allFinite() ||
      !std::isfinite(period_seconds) || period_seconds < 0.0) {
    return false;
  }

  if (transition_duration_ == 0.0) {
    transition_progress_ = 1.0;
  } else {
    transition_progress_ = std::min(
        1.0, transition_progress_ + period_seconds / transition_duration_);
  }

  for (Eigen::Index index = 0; index < gravity_torque.size(); ++index) {
    const double limit = torque_limit_[index];
    const double gravity_target =
        std::clamp(scale_[index] * gravity_torque[index], -limit, limit);
    target_[index] = (1.0 - transition_progress_) * transition_source_[index] +
                     transition_progress_ * gravity_target;

    const double maximum_delta = torque_rate_limit_[index] * period_seconds;
    const double delta = std::clamp(target_[index] - current_[index],
                                    -maximum_delta, maximum_delta);
    current_[index] += delta;
    shaped_torque[index] = current_[index];
  }
  return true;
}

std::size_t GravityFeedforwardShaper::dof() const noexcept {
  return static_cast<std::size_t>(scale_.size());
}

} // namespace gravity_compensation_model
