// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <Eigen/Core>

namespace zephyr::mpc {

// Command-space models, not wheel dynamics. The wheel controller still owns
// steering allocation, CAN mode acknowledgements and hardware interlocks.
struct BaseKinematics {
    enum class Backend { Omni, Differential, Ranger };
    Backend backend{Backend::Omni};
    bool mobile{true};
    double wheel_base{0.494};
    double max_steering_angle{0.698};
    double command_deadzone{0.001};
    double ackermann_penalty{1000.0};

    static Backend parseBackend(const std::string& name);
    void validate() const;
    bool supports(uint8_t mode) const;
    uint8_t resolvedMode(uint8_t mode) const;
    // True entries are hard zero-velocity equalities in [vx, vy, wz].
    std::array<bool, 3> zeroAxes(uint8_t mode) const;
    double maxCurvature() const;
    // Last-mile safety projection, including after speed limits/deadbands.
    // No allocation; nonfinite or unsupported commands fail closed.
    Eigen::Vector3d project(uint8_t mode, Eigen::Vector3d velocity) const;
};

} // namespace zephyr::mpc
