#pragma once

#include <cstddef>
#include <string>

#include <Eigen/Core>

namespace gravity_compensation_model {

struct GravityFeedforwardShaperConfig {
  Eigen::VectorXd scale;
  Eigen::VectorXd torque_limit;
  Eigen::VectorXd torque_rate_limit;
  double transition_duration{1.0};
};

class GravityFeedforwardShaper {
public:
  bool configure(const GravityFeedforwardShaperConfig &config,
                 std::string &error);

  bool reset(const Eigen::Ref<const Eigen::VectorXd> &initial_torque) noexcept;

  bool update(const Eigen::Ref<const Eigen::VectorXd> &gravity_torque,
              double period_seconds,
              Eigen::Ref<Eigen::VectorXd> shaped_torque) noexcept;

  std::size_t dof() const noexcept;

private:
  Eigen::VectorXd scale_;
  Eigen::VectorXd torque_limit_;
  Eigen::VectorXd torque_rate_limit_;
  Eigen::VectorXd transition_source_;
  Eigen::VectorXd target_;
  Eigen::VectorXd current_;
  double transition_duration_{1.0};
  double transition_progress_{0.0};
  bool configured_{false};
  bool reset_{false};
};

} // namespace gravity_compensation_model
