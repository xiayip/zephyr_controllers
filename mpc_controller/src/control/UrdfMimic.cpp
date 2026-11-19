// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/UrdfMimic.h"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace zephyr::mpc {
std::vector<MimicRelation> readMimicRelations(const std::string& xml) {
  const auto tree = urdf::parseURDF(xml);
  if (!tree) throw std::invalid_argument("Invalid URDF for mimic mapping");
  std::vector<MimicRelation> result;
  for (const auto& [name, joint] : tree->joints_) {
    if (!joint->mimic) continue;
    std::set<std::string> visited{name};
    auto source = joint->mimic->joint_name;
    double a = joint->mimic->multiplier, b = joint->mimic->offset;
    while (true) {
      if (!visited.insert(source).second || !tree->joints_.count(source))
        throw std::invalid_argument("Invalid/cyclic mimic dependency: " + name);
      const auto& parent = tree->joints_.at(source);
      if (!parent->mimic) break;
      b += a * parent->mimic->offset;
      a *= parent->mimic->multiplier;
      source = parent->mimic->joint_name;
    }
    if (!std::isfinite(a) || !std::isfinite(b))
      throw std::invalid_argument("Nonfinite mimic mapping: " + name);
    result.push_back({name, source, a, b});
  }
  return result;
}

ocs2::PinocchioInterface makeMimicGeometry(const std::string& xml, bool mobile,
    const std::vector<std::string>& fixed_joints) {
  auto tree = urdf::parseURDF(xml);
  if (!tree) throw std::invalid_argument("Invalid geometry URDF");
  for (const auto& [name, joint] : tree->joints_) {
    if (std::find(fixed_joints.begin(), fixed_joints.end(), name) != fixed_joints.end())
      joint->type = urdf::Joint::FIXED;
    // Evaluated explicitly from the original URDF by readMimicRelations().
    joint->mimic.reset();
  }
  pinocchio::Model model;
  if (mobile) {
    pinocchio::JointModelComposite root(3);
    root.addJoint(pinocchio::JointModelPX());
    root.addJoint(pinocchio::JointModelPY());
    root.addJoint(pinocchio::JointModelRZ());
    pinocchio::urdf::buildModel(tree, root, model);
  } else {
    pinocchio::urdf::buildModel(tree, model);
  }
  return ocs2::PinocchioInterface(model, tree, xml);
}

ConfigurationExpansion mimicExpansion(const pinocchio::Model& independent,
    const pinocchio::Model& geometry, const std::vector<MimicRelation>& relations) {
  ConfigurationExpansion out{Eigen::MatrixXd::Zero(geometry.nq, independent.nq),
                             Eigen::VectorXd::Zero(geometry.nq)};
  for (pinocchio::JointIndex id = 1; id < geometry.joints.size(); ++id) {
    const auto& joint = geometry.joints[id];
    const auto relation = std::find_if(relations.begin(), relations.end(),
        [&](const auto& r) { return r.joint == geometry.names[id]; });
    const auto source = independent.getJointId(relation == relations.end() ? geometry.names[id] : relation->source);
    if (source == 0 || source >= independent.joints.size())
      throw std::invalid_argument("Unmapped geometry joint: " + geometry.names[id]);
    const auto& driver = independent.joints[source];
    if (relation != relations.end()) {
      if (joint.nq() != 1 || driver.nq() != 1)
        throw std::invalid_argument("Mimic requires scalar joints");
      out.matrix(joint.idx_q(), driver.idx_q()) = relation->multiplier;
      out.offset[joint.idx_q()] = relation->offset;
    } else {
      if (joint.nq() != driver.nq()) throw std::invalid_argument("Geometry coordinate mismatch");
      out.matrix.block(joint.idx_q(), driver.idx_q(), joint.nq(), driver.nq()).setIdentity();
    }
  }
  return out;
}
}  // namespace zephyr::mpc
