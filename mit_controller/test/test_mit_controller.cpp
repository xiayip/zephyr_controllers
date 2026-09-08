#include <algorithm>
#include <array>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "mit_controller/mit_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

namespace {

hardware_interface::CommandInterface::SharedPtr
make_command_interface(const std::string &prefix,
                       const std::string &interface_name, const double value) {
  auto interface = std::make_shared<hardware_interface::CommandInterface>(
      prefix, interface_name, "double");
  EXPECT_TRUE(interface->set_value(value));
  return interface;
}

hardware_interface::StateInterface::SharedPtr
make_state_interface(const std::string &prefix,
                     const std::string &interface_name, const double value) {
  auto interface = std::make_shared<hardware_interface::StateInterface>(
      prefix, interface_name, "double");
  EXPECT_TRUE(interface->set_value(value));
  return interface;
}

TEST(MitControllerTest, PreservesStartupCommandAndForwardsCompleteTuple) {
  mit_controller::MitController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
       rclcpp::Parameter("max_position_step", 1.0)});
  ASSERT_EQ(controller.init("test_mit_controller", "", 200, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  auto references = controller.export_reference_interfaces();
  ASSERT_EQ(references.size(), 5U);
  EXPECT_EQ(references[0]->get_name(), "test_mit_controller/joint1/position");

  std::vector<hardware_interface::CommandInterface::SharedPtr> commands = {
      make_command_interface("joint1", "position", 1.25),
      make_command_interface("joint1", "velocity", 0.0),
      make_command_interface("joint1", "kp", 50.0),
      make_command_interface("joint1", "kd", 3.0),
      make_command_interface("joint1", "t_ff", 0.0)};
  std::vector<hardware_interface::StateInterface::SharedPtr> states = {
      make_state_interface("joint1", "position", 1.20),
      make_state_interface("joint1", "velocity", 0.1)};
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  for (const auto &command : commands) {
    loaned_commands.emplace_back(command, nullptr);
  }
  for (const auto &state : states) {
    loaned_states.emplace_back(state);
  }
  controller.assign_interfaces(std::move(loaned_commands),
                               std::move(loaned_states));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(references[0]->get_optional().has_value());
  EXPECT_DOUBLE_EQ(*references[0]->get_optional(), 1.25);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 1.25);
  ASSERT_TRUE(controller.set_chained_mode(true));

  ASSERT_TRUE(references[0]->set_value(1.5));
  ASSERT_TRUE(references[1]->set_value(0.2));
  ASSERT_TRUE(references[2]->set_value(40.0));
  ASSERT_TRUE(references[3]->set_value(2.0));
  ASSERT_TRUE(references[4]->set_value(1.0));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 1.5);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.2);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 40.0);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 2.0);
  EXPECT_DOUBLE_EQ(*commands[4]->get_optional(), 1.0);

  ASSERT_TRUE(
      references[2]->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 1.20);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.0);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 50.0);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 3.0);
  EXPECT_NEAR(*commands[4]->get_optional(), 13.5, 1e-12);

  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  controller.release_interfaces();
}

class MitHoldTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
         rclcpp::Parameter("max_position_step", step)});
    ASSERT_EQ(controller.init("hold_test", "", 200, "", options),
              controller_interface::return_type::OK);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    references = controller.export_reference_interfaces();
    commands = {make_command_interface("joint1", "position", 0.8),
                make_command_interface("joint1", "velocity", 0.2),
                make_command_interface("joint1", "kp", 50.0),
                make_command_interface("joint1", "kd", 3.0),
                make_command_interface("joint1", "t_ff", 2.0)};
    states = {make_state_interface("joint1", "position", 0.75),
              make_state_interface("joint1", "velocity", -0.1)};
    std::vector<hardware_interface::LoanedCommandInterface> c;
    std::vector<hardware_interface::LoanedStateInterface> s;
    for (const auto &handle : commands)
      c.emplace_back(handle, nullptr);
    for (const auto &handle : states)
      s.emplace_back(handle);
    controller.assign_interfaces(std::move(c), std::move(s));
  }
  void TearDown() override { controller.release_interfaces(); }
  void activate() {
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(controller.set_chained_mode(true));
  }
  double torque() const {
    const double q = *states[0]->get_optional(), v = *states[1]->get_optional();
    return *commands[2]->get_optional() * (*commands[0]->get_optional() - q) +
           *commands[3]->get_optional() * (*commands[1]->get_optional() - v) +
           *commands[4]->get_optional();
  }
  void write_moving_command() {
    for (std::size_t i = 0; i < 5; ++i)
      ASSERT_TRUE(references[i]->set_value(
          std::array<double, 5>{0.9, 0.4, 40, 2, 1.5}[i]));
    ASSERT_EQ(controller.update_and_write_commands(
                  rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
              controller_interface::return_type::OK);
  }
  mit_controller::MitController controller;
  double step = 1.0;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands,
      references;
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
};

TEST_F(MitHoldTest, RequiresPositionAndVelocityFeedback) {
  EXPECT_EQ(controller.state_interface_configuration().names,
            (std::vector<std::string>{"joint1/position", "joint1/velocity"}));
}

TEST_F(MitHoldTest, ActivationPreservesFeedforwardAndVelocity) {
  const double before = torque();
  activate();
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 0.8);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.2);
  EXPECT_DOUBLE_EQ(*commands[4]->get_optional(), 2.0);
  EXPECT_NEAR(torque(), before, 1e-12);
}

TEST_F(MitHoldTest, DeactivationStopsAtCurrentPositionWithoutTorqueStep) {
  activate();
  write_moving_command();
  ASSERT_TRUE(states[0]->set_value(0.78));
  ASSERT_TRUE(states[1]->set_value(0.12));
  const double before = torque();
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 0.78);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.0);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 50.0);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 3.0);
  EXPECT_NEAR(torque(), before, 1e-12);
  EXPECT_NEAR(*commands[4]->get_optional(), 7.22, 1e-12);
}

TEST_F(MitHoldTest,
       UnchainingLatchesAStationaryHoldAndReactivationPreservesIt) {
  activate();
  write_moving_command();
  const double before = torque();
  ASSERT_TRUE(controller.set_chained_mode(false));
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 0.75);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.0);
  EXPECT_NEAR(torque(), before, 1e-12);
  const double hold_ff = *commands[4]->get_optional();
  ASSERT_TRUE(states[0]->set_value(0.77));
  for (int tick = 0; tick < 10; ++tick)
    ASSERT_EQ(controller.update(rclcpp::Time(0),
                                rclcpp::Duration::from_seconds(.005)),
              controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 0.75);
  EXPECT_DOUBLE_EQ(*commands[4]->get_optional(), hold_ff);
  const double holding_torque = torque();
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_NEAR(torque(), holding_torque, 1e-12);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_NEAR(torque(), holding_torque, 1e-12);
}

TEST_F(MitHoldTest, InvalidFeedbackOnExitRetainsLastFiniteHold) {
  activate();
  write_moving_command();
  const double before = torque();
  ASSERT_TRUE(states[0]->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(states[0]->set_value(0.75));
  EXPECT_NEAR(torque(), before, 1e-12);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.0);
}

TEST_F(MitHoldTest, RejectsUnknownVelocityBeforeActivationWrites) {
  ASSERT_TRUE(states[1]->set_value(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 0.8);
  EXPECT_DOUBLE_EQ(*commands[4]->get_optional(), 2.0);
}

TEST_F(MitHoldTest, RejectsUnknownFeedforwardBeforeActivationWrites) {
  ASSERT_TRUE(commands[4]->set_value(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 0.8);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.2);
}

class MitStepTest : public MitHoldTest {
protected:
  void SetUp() override {
    step = .02;
    MitHoldTest::SetUp();
  }
};

TEST_F(MitStepTest, PreloadTransferIsTorqueContinuousThroughHardwareStepLimit) {
  activate();
  const double before = torque();
  // Equivalent torque with actual-q reference, zero velocity and new gains.
  const std::array<double, 5> target{.75, 0, 40, 2, before - .2};
  for (std::size_t i = 0; i < target.size(); ++i)
    ASSERT_TRUE(references[i]->set_value(target[i]));
  for (int tick = 0; tick < 3; ++tick) {
    const double previous = *commands[0]->get_optional();
    ASSERT_EQ(controller.update_and_write_commands(
                  rclcpp::Time(0), rclcpp::Duration::from_seconds(.005)),
              controller_interface::return_type::OK);
    const double position = *commands[0]->get_optional();
    EXPECT_DOUBLE_EQ(std::clamp(position, previous - .02, previous + .02),
                     position);
    EXPECT_NEAR(torque(), before, 1e-12);
  }
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), .75);
  EXPECT_NEAR(*commands[4]->get_optional(), target[4], 1e-12);
}

TEST_F(MitStepTest, LargeMotionInterpolatesTorqueAndCachesTheEffectiveCommand) {
  activate();
  const double before = torque();
  write_moving_command();
  // p: .8 -> .9, limited to .82. Endpoint torque is 8.5 Nm at q=.75,dq=-.1.
  const double expected = before + .2 * (8.5 - before);
  EXPECT_NEAR(*commands[0]->get_optional(), .82, 1e-12);
  EXPECT_NEAR(torque(), expected, 1e-12);
  ASSERT_TRUE(
      references[4]->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(.005)),
            controller_interface::return_type::OK);
  // Fallback must use the last applied torque, not the unachieved 8.5 Nm goal.
  EXPECT_NEAR(torque(), expected, 1e-12);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.0);
}

TEST_F(MitStepTest, SingleShutdownWriteStopsVelocityWithoutATorqueStep) {
  activate();
  const double before = torque();
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_NEAR(*commands[0]->get_optional(), .78, 1e-12);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.0);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 50.0);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 3.0);
  EXPECT_NEAR(torque(), before, 1e-12);
}

TEST_F(MitStepTest, InvalidFeedbackShutdownUsesFiniteCachedTorqueAndStep) {
  activate();
  const double before = torque();
  ASSERT_TRUE(states[0]->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_TRUE(states[1]->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(states[0]->set_value(.75));
  ASSERT_TRUE(states[1]->set_value(-.1));
  EXPECT_NEAR(*commands[0]->get_optional(), .78, 1e-12);
  EXPECT_NEAR(torque(), before, 1e-12);
}

TEST_F(MitStepTest,
       ConsecutiveUnchainAndDeactivateDoNotStepTwiceBeforeHardwareWrite) {
  activate();
  const double before = torque();
  ASSERT_TRUE(controller.set_chained_mode(false));
  std::array<double, 5> hold;
  for (std::size_t i = 0; i < hold.size(); ++i)
    hold[i] = *commands[i]->get_optional();
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  for (std::size_t i = 0; i < hold.size(); ++i)
    EXPECT_DOUBLE_EQ(*commands[i]->get_optional(), hold[i]);
  EXPECT_NEAR(hold[0], .78, 1e-12);
  EXPECT_NEAR(torque(), before, 1e-12);
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
