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
      {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"})});
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
      make_state_interface("joint1", "position", 1.20)};
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
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 1.5);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.0);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 50.0);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 3.0);
  EXPECT_DOUBLE_EQ(*commands[4]->get_optional(), 0.0);

  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  controller.release_interfaces();
}

} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
