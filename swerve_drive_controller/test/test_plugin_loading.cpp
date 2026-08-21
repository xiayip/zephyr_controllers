#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "gtest/gtest.h"
#include "pluginlib/class_loader.hpp"

namespace
{

TEST(SwerveDrivePlugin, LoadsCanonicalName)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");

  EXPECT_NO_THROW({
    auto controller = loader.createSharedInstance("swerve_drive_controller/SwerveController");
    EXPECT_NE(controller, nullptr);
  });
}

}  // namespace