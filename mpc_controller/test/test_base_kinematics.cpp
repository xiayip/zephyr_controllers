// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "mpc_controller/control/BaseKinematics.h"
#include "mpc_controller/control/RuntimeMotionProfile.h"
#include <zephyr_msgs/msg/motion_profile.hpp>
#include <limits>

using namespace zephyr::mpc;
using Mode = RuntimeProfileConfig;

TEST(BaseKinematics, WireConstantsAndBackendCapabilities) {
    using Msg = zephyr_msgs::msg::MotionProfile;
    EXPECT_EQ(Mode::CRAB, Msg::CRAB);
    EXPECT_EQ(Mode::SPIN, Msg::SPIN);
    EXPECT_EQ(Mode::ACKERMANN_FORWARD, Msg::ACKERMANN_FORWARD);
    EXPECT_EQ(Mode::ACKERMANN_REVERSE, Msg::ACKERMANN_REVERSE);
    EXPECT_EQ(Mode::DIFFERENTIAL, Msg::DIFFERENTIAL);
    EXPECT_EQ(Mode::OMNI, Msg::OMNI);
    BaseKinematics k;
    for (uint8_t mode = 0; mode <= Mode::OMNI; ++mode) EXPECT_TRUE(k.supports(mode));
    k.backend = BaseKinematics::Backend::Ranger;
    EXPECT_TRUE(k.supports(Mode::CRAB));
    EXPECT_TRUE(k.supports(Mode::SPIN));
    EXPECT_FALSE(k.supports(Mode::OMNI));
    EXPECT_FALSE(k.supports(Mode::DIFFERENTIAL));
    k.backend = BaseKinematics::Backend::Differential;
    EXPECT_FALSE(k.supports(Mode::CRAB));
    EXPECT_TRUE(k.supports(Mode::DIFFERENTIAL));
    EXPECT_EQ(k.resolvedMode(Mode::COORDINATED), Mode::DIFFERENTIAL);
    k.mobile = false;
    EXPECT_FALSE(k.supports(Mode::SPIN));
    EXPECT_TRUE(k.supports(Mode::LOCKED));
}

TEST(BaseKinematics, ProjectionPreservesSupportedDirectionsAndFailsClosed) {
    BaseKinematics k;
    const Eigen::Vector3d u(-0.2, 0.3, -0.4);
    EXPECT_TRUE(k.project(Mode::OMNI, u).isApprox(u));
    EXPECT_TRUE(k.project(Mode::CRAB, u).isApprox(Eigen::Vector3d(-0.2, 0.3, 0)));
    EXPECT_TRUE(k.project(Mode::SPIN, u).isApprox(Eigen::Vector3d(0, 0, -0.4)));
    EXPECT_TRUE(k.project(Mode::DIFFERENTIAL, u).isApprox(Eigen::Vector3d(-0.2, 0, -0.4)));
    EXPECT_TRUE(k.project(Mode::LOCKED, u).isZero());
    EXPECT_TRUE(k.project(255, u).isZero());
    EXPECT_TRUE(k.project(Mode::OMNI, {NAN, 1, 2}).isZero());
    k.backend = BaseKinematics::Backend::Ranger;
    EXPECT_TRUE(k.project(Mode::OMNI, u).isZero());
    EXPECT_TRUE(k.project(Mode::COORDINATED, u).isApprox(Eigen::Vector3d(-0.2, 0, -0.4)));
}

TEST(BaseKinematics, AckermannConeSurvivesRestReverseAndSpeedClipping) {
    BaseKinematics k;
    for (const auto mode : {Mode::ACKERMANN_FORWARD, Mode::ACKERMANN_REVERSE}) {
        const double sign = mode == Mode::ACKERMANN_FORWARD ? 1 : -1;
        for (double vx : {-1.0, -0.001, 0.0, 0.001, 0.2, 1.0}) {
            for (double wz : {-10.0, -0.3, 0.0, 0.3, 10.0}) {
                const auto output = k.project(mode, {vx, 0.7, wz});
                EXPECT_GE(sign * output.x(), 0);
                EXPECT_DOUBLE_EQ(output.y(), 0);
                EXPECT_LE(std::abs(output.z()), k.maxCurvature() * std::abs(output.x()) + 1e-12);
                if (std::abs(output.x()) <= k.command_deadzone) { EXPECT_DOUBLE_EQ(output.z(), 0); }
                if (output.x() != 0) {
                    // Exactly the formula used by Ranger command_adapter.cpp.
                    const double steering = std::atan(k.wheel_base * output.z() / (2 * output.x()));
                    EXPECT_LE(std::abs(steering), k.max_steering_angle + 1e-12);
                }
            }
        }
        auto output = k.project(mode, {sign * 0.5, 0, 1});
        output.x() = sign * 0.01;  // Independent final speed limiter.
        output = k.project(mode, output);
        EXPECT_LE(std::abs(output.z()), k.maxCurvature() * 0.01 + 1e-12);
    }
}

TEST(BaseKinematics, InvalidGeometryAndBackendReject) {
    EXPECT_THROW(BaseKinematics::parseBackend("ackerman"), std::invalid_argument);
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        BaseKinematics k;
        k.wheel_base = bad;
        EXPECT_THROW(k.validate(), std::invalid_argument);
        k = {};
        k.max_steering_angle = bad;
        EXPECT_THROW(k.validate(), std::invalid_argument);
    }
    BaseKinematics k;
    k.max_steering_angle = M_PI_2;
    EXPECT_THROW(k.validate(), std::invalid_argument);
    k = {};
    k.backend = BaseKinematics::Backend::Ranger;
    k.max_steering_angle = .7;
    EXPECT_THROW(k.validate(), std::invalid_argument);
    k = {};
    k.backend = BaseKinematics::Backend::Ranger;
    k.command_deadzone = 0;
    EXPECT_THROW(k.validate(), std::invalid_argument);
}
