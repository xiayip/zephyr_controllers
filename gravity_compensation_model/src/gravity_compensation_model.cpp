#include "gravity_compensation_model/gravity_compensation_model.hpp"

#include <exception>
#include <unordered_set>
#include <utility>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace gravity_compensation_model {

bool GravityCompensationModel::configure(
    const std::string &robot_description,
    const std::vector<std::string> &controlled_joints,
    const Eigen::Vector3d &gravity, std::string &error) {
  data_.reset();
  model_ = pinocchio::Model{};
  model_positions_.resize(0);
  lower_position_limits_.resize(0);
  upper_position_limits_.resize(0);
  position_indices_.clear();
  velocity_indices_.clear();

  if (robot_description.empty()) {
    error = "robot_description is empty";
    return false;
  }
  if (controlled_joints.empty()) {
    error = "controlled_joints is empty";
    return false;
  }
  if (!gravity.allFinite()) {
    error = "gravity vector contains a non-finite value";
    return false;
  }

  try {
    pinocchio::Model parsed_model;
    pinocchio::urdf::buildModelFromXML(robot_description, parsed_model);
    parsed_model.gravity.setZero();
    parsed_model.gravity.linear() = gravity;

    std::unordered_set<std::string> unique_joints;
    std::vector<Eigen::Index> position_indices;
    std::vector<Eigen::Index> velocity_indices;
    Eigen::VectorXd controlled_lower_limits(
        static_cast<Eigen::Index>(controlled_joints.size()));
    Eigen::VectorXd controlled_upper_limits(
        static_cast<Eigen::Index>(controlled_joints.size()));
    position_indices.reserve(controlled_joints.size());
    velocity_indices.reserve(controlled_joints.size());

    for (std::size_t index = 0; index < controlled_joints.size(); ++index) {
      const auto &joint_name = controlled_joints[index];
      if (!unique_joints.insert(joint_name).second) {
        error = "controlled joint is duplicated: " + joint_name;
        return false;
      }
      if (!parsed_model.existJointName(joint_name)) {
        error = "controlled joint is missing from the Pinocchio model: " +
                joint_name;
        return false;
      }
      const auto joint_id = parsed_model.getJointId(joint_name);
      const auto &joint = parsed_model.joints[joint_id];
      if (joint.nq() != 1 || joint.nv() != 1) {
        error = "controlled joint must have nq=1 and nv=1: " + joint_name;
        return false;
      }
      position_indices.push_back(static_cast<Eigen::Index>(joint.idx_q()));
      velocity_indices.push_back(static_cast<Eigen::Index>(joint.idx_v()));
      controlled_lower_limits[static_cast<Eigen::Index>(index)] =
          parsed_model
              .lowerPositionLimit[static_cast<Eigen::Index>(joint.idx_q())];
      controlled_upper_limits[static_cast<Eigen::Index>(index)] =
          parsed_model
              .upperPositionLimit[static_cast<Eigen::Index>(joint.idx_q())];
    }

    model_ = std::move(parsed_model);
    data_ = std::make_unique<pinocchio::Data>(model_);
    model_positions_ = pinocchio::neutral(model_);
    lower_position_limits_ = std::move(controlled_lower_limits);
    upper_position_limits_ = std::move(controlled_upper_limits);
    position_indices_ = std::move(position_indices);
    velocity_indices_ = std::move(velocity_indices);
  } catch (const std::exception &exception) {
    error = std::string("failed to build Pinocchio model: ") + exception.what();
    data_.reset();
    position_indices_.clear();
    velocity_indices_.clear();
    return false;
  }

  error.clear();
  return true;
}

bool GravityCompensationModel::compute(
    const Eigen::Ref<const Eigen::VectorXd> &joint_positions,
    Eigen::Ref<Eigen::VectorXd> gravity_torques) noexcept {
  if (!data_ ||
      joint_positions.size() !=
          static_cast<Eigen::Index>(position_indices_.size()) ||
      gravity_torques.size() !=
          static_cast<Eigen::Index>(velocity_indices_.size()) ||
      !joint_positions.allFinite()) {
    return false;
  }

  for (Eigen::Index index = 0; index < joint_positions.size(); ++index) {
    model_positions_[position_indices_[static_cast<std::size_t>(index)]] =
        joint_positions[index];
  }

  const auto &model_gravity =
      pinocchio::computeGeneralizedGravity(model_, *data_, model_positions_);
  for (Eigen::Index index = 0; index < gravity_torques.size(); ++index) {
    gravity_torques[index] =
        model_gravity[velocity_indices_[static_cast<std::size_t>(index)]];
  }
  return gravity_torques.allFinite();
}

std::size_t GravityCompensationModel::dof() const noexcept {
  return position_indices_.size();
}

bool GravityCompensationModel::configured() const noexcept {
  return static_cast<bool>(data_);
}

const Eigen::VectorXd &
GravityCompensationModel::lower_position_limits() const noexcept {
  return lower_position_limits_;
}

const Eigen::VectorXd &
GravityCompensationModel::upper_position_limits() const noexcept {
  return upper_position_limits_;
}

} // namespace gravity_compensation_model
