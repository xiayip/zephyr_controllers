#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace gravity_compensation_model {

class GravityCompensationModel {
public:
  bool configure(const std::string &robot_description,
                 const std::vector<std::string> &controlled_joints,
                 const Eigen::Vector3d &gravity, std::string &error);

  bool compute(const Eigen::Ref<const Eigen::VectorXd> &joint_positions,
               Eigen::Ref<Eigen::VectorXd> gravity_torques) noexcept;

  std::size_t dof() const noexcept;
  bool configured() const noexcept;
  const Eigen::VectorXd &lower_position_limits() const noexcept;
  const Eigen::VectorXd &upper_position_limits() const noexcept;

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::VectorXd model_positions_;
  Eigen::VectorXd lower_position_limits_;
  Eigen::VectorXd upper_position_limits_;
  std::vector<Eigen::Index> position_indices_;
  std::vector<Eigen::Index> velocity_indices_;
};

} // namespace gravity_compensation_model
