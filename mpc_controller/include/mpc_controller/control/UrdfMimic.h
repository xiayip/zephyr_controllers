// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <pinocchio/multibody/model.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <string>
#include <vector>

namespace zephyr::mpc {
// URDF is the sole source of coupling: q_joint = multiplier*q_source + offset.
// Nested mimic references are flattened to an independent source, without IK.
struct MimicRelation {
  std::string joint, source;
  double multiplier, offset;
};
std::vector<MimicRelation> readMimicRelations(const std::string& xml);

// Display/FK model only. Ordinary joints, no native Pinocchio mimic support and
// no injection into OCS2. The optimizer retains its upstream independent model.
ocs2::PinocchioInterface makeMimicGeometry(const std::string& xml, bool mobile,
    const std::vector<std::string>& fixed_joints);
struct ConfigurationExpansion {
  Eigen::MatrixXd matrix;
  Eigen::VectorXd offset;
};
// q_geometry = A*q_independent + b. Also gives the future chain-rule factor A;
// this helper does not enable collision costs or claim collision validation.
ConfigurationExpansion mimicExpansion(const pinocchio::Model& independent,
    const pinocchio::Model& geometry, const std::vector<MimicRelation>& relations);
}  // namespace zephyr::mpc
