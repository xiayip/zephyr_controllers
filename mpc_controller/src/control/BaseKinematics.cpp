// SPDX-License-Identifier: Apache-2.0
#include "mpc_controller/control/BaseKinematics.h"
#include "mpc_controller/control/RuntimeMotionProfile.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zephyr::mpc {
using Mode = RuntimeProfileConfig;

BaseKinematics::Backend BaseKinematics::parseBackend(const std::string& name) {
    if (name == "omni") return Backend::Omni;
    if (name == "differential") return Backend::Differential;
    if (name == "ranger") return Backend::Ranger;
    throw std::invalid_argument("base_command_profile must be omni, differential or ranger");
}

void BaseKinematics::validate() const {
    if (!std::isfinite(wheel_base) || wheel_base <= 0 ||
        !std::isfinite(max_steering_angle) || max_steering_angle <= 0 || max_steering_angle >= M_PI_2 ||
        !std::isfinite(command_deadzone) || command_deadzone < 0 ||
        !std::isfinite(ackermann_penalty) || ackermann_penalty <= 0)
        throw std::invalid_argument("Invalid base kinematics geometry, deadzone or penalty");
    if (!std::isfinite(maxCurvature())) throw std::invalid_argument("Nonfinite base curvature");
    if (backend == Backend::Ranger && (max_steering_angle > 0.698 || command_deadzone < 0.001))
        throw std::invalid_argument("Ranger adapter requires steering <=0.698 and command deadzone >=0.001");
}

bool BaseKinematics::supports(uint8_t mode) const {
    if (mode > Mode::OMNI) return false;
    if (mode == Mode::COORDINATED || mode == Mode::LOCKED) return true;
    if (!mobile) return false;
    if (backend == Backend::Omni) return true;
    if (backend == Backend::Ranger)
        return mode == Mode::CRAB || mode == Mode::SPIN ||
               mode == Mode::ACKERMANN_FORWARD || mode == Mode::ACKERMANN_REVERSE;
    return mode == Mode::DIFFERENTIAL || mode == Mode::SPIN ||
           mode == Mode::ACKERMANN_FORWARD || mode == Mode::ACKERMANN_REVERSE;
}

uint8_t BaseKinematics::resolvedMode(uint8_t mode) const {
    if (mode != Mode::COORDINATED) return mode;
    // Backward-compatible Ranger default. It is NOT an Ackermann curvature
    // model; explicit directional modes are required for the curvature envelope.
    return backend == Backend::Omni ? Mode::OMNI : Mode::DIFFERENTIAL;
}

std::array<bool, 3> BaseKinematics::zeroAxes(uint8_t mode) const {
    if (!supports(mode)) throw std::invalid_argument("Base mode unsupported by configured backend");
    switch (resolvedMode(mode)) {
        case Mode::LOCKED: return {true, true, true};
        case Mode::CRAB: return {false, false, true};
        case Mode::SPIN: return {true, true, false};
        case Mode::DIFFERENTIAL:
        case Mode::ACKERMANN_FORWARD:
        case Mode::ACKERMANN_REVERSE: return {false, true, false};
        default: return {false, false, false};
    }
}

double BaseKinematics::maxCurvature() const {
    // Ranger's symmetric front/rear Ackermann command adapter uses L / 2.
    // This is not the single-steered-axle bicycle formula tan(delta) / L.
    return 2.0 * std::tan(max_steering_angle) / wheel_base;
}

Eigen::Vector3d BaseKinematics::project(uint8_t mode, Eigen::Vector3d velocity) const {
    if (!mobile || !velocity.allFinite() || !supports(mode)) return Eigen::Vector3d::Zero();
    const auto zero = zeroAxes(mode);
    for (size_t i = 0; i < 3; ++i) if (zero[i]) velocity[i] = 0;
    if (mode == Mode::ACKERMANN_FORWARD || mode == Mode::ACKERMANN_REVERSE) {
        const double sign = mode == Mode::ACKERMANN_FORWARD ? 1.0 : -1.0;
        velocity.x() = sign * std::max(0.0, sign * velocity.x());
        const double yaw_limit = std::abs(velocity.x()) <= command_deadzone ? 0.0 :
                                 maxCurvature() * std::abs(velocity.x());
        velocity.z() = std::clamp(velocity.z(), -yaw_limit, yaw_limit);
    }
    return velocity;
}
}  // namespace zephyr::mpc
