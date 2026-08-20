#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "mit_position_adapter_controller/mit_position_adapter_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

namespace {

constexpr char kPendulumUrdf[] = R"(
<robot name="pendulum">
  <link name="base_link"/>
  <link name="link1">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2"/>
  </joint>
</robot>)";

hardware_interface::CommandInterface::SharedPtr
make_command_interface(const std::string &prefix,
                       const std::string &interface_name, const double value) {
  auto interface = std::make_shared<hardware_interface::CommandInterface>(
      prefix, interface_name, "double");
  EXPECT_TRUE(interface->set_value(value));
  return interface;
}

TEST(MitPositionAdapterControllerTest,
     CompletesPositionAndVelocityReferencesIntoMitTuple) {
  mit_position_adapter_controller::MitPositionAdapterController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
       rclcpp::Parameter("downstream_controller", "mit_output"),
       rclcpp::Parameter("position_kp", std::vector<double>{50.0}),
       rclcpp::Parameter("position_kd", std::vector<double>{3.0})});
  ASSERT_EQ(controller.init("test_position_adapter", "", 200, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  auto references = controller.export_reference_interfaces();
  ASSERT_EQ(references.size(), 2U);
  EXPECT_EQ(references[0]->get_name(), "test_position_adapter/joint1/position");
  EXPECT_EQ(references[1]->get_name(), "test_position_adapter/joint1/velocity");

  std::vector<hardware_interface::CommandInterface::SharedPtr> commands = {
      make_command_interface("mit_output", "joint1/position", 0.8),
      make_command_interface("mit_output", "joint1/velocity", 0.0),
      make_command_interface("mit_output", "joint1/kp", 50.0),
      make_command_interface("mit_output", "joint1/kd", 3.0),
      make_command_interface("mit_output", "joint1/t_ff", 0.0)};
  auto position_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "position", "double");
  ASSERT_TRUE(position_state->set_value(0.75));
  auto velocity_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "velocity", "double");
  ASSERT_TRUE(velocity_state->set_value(0.0));
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  for (const auto &command : commands) {
    loaned_commands.emplace_back(command, nullptr);
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  loaned_states.emplace_back(position_state);
  loaned_states.emplace_back(velocity_state);
  controller.assign_interfaces(std::move(loaned_commands),
                               std::move(loaned_states));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  EXPECT_DOUBLE_EQ(*references[0]->get_optional(), 0.8);
  EXPECT_DOUBLE_EQ(*references[1]->get_optional(), 0.0);
  ASSERT_TRUE(references[0]->set_value(1.1));
  ASSERT_TRUE(references[1]->set_value(0.4));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 1.1);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.4);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 50.0);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 3.0);
  EXPECT_DOUBLE_EQ(*commands[4]->get_optional(), 0.0);

  const auto gains_result = controller.get_node()->set_parameters_atomically(
      {rclcpp::Parameter("position_kp", std::vector<double>{40.0}),
       rclcpp::Parameter("position_kd", std::vector<double>{2.0})});
  ASSERT_TRUE(gains_result.successful) << gains_result.reason;
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 40.0);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 2.0);

  const auto invalid_gains_result = controller.get_node()->set_parameter(
      rclcpp::Parameter("position_kd", std::vector<double>{2.0, 3.0}));
  EXPECT_FALSE(invalid_gains_result.successful);

  ASSERT_TRUE(
      references[0]->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_TRUE(
      references[1]->set_value(std::numeric_limits<double>::quiet_NaN()));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), 1.1);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), 0.4);

  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  controller.release_interfaces();
}

TEST(MitPositionAdapterControllerTest, PreservesEquivalentTorqueOnActivation) {
  mit_position_adapter_controller::MitPositionAdapterController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
       rclcpp::Parameter("downstream_controller", "mit_output"),
       rclcpp::Parameter("position_kp", std::vector<double>{100.0}),
       rclcpp::Parameter("position_kd", std::vector<double>{5.0})});
  ASSERT_EQ(controller.init("test_torque_continuity", "", 200, "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::CommandInterface::SharedPtr> commands = {
      make_command_interface("mit_output", "joint1/position", 0.8),
      make_command_interface("mit_output", "joint1/velocity", 0.0),
      make_command_interface("mit_output", "joint1/kp", 50.0),
      make_command_interface("mit_output", "joint1/kd", 3.0),
      make_command_interface("mit_output", "joint1/t_ff", 2.0)};
  auto position_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "position", "double");
  auto velocity_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "velocity", "double");
  ASSERT_TRUE(position_state->set_value(0.75));
  ASSERT_TRUE(velocity_state->set_value(0.0));

  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  for (const auto &command : commands) {
    loaned_commands.emplace_back(command, nullptr);
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  loaned_states.emplace_back(position_state);
  loaned_states.emplace_back(velocity_state);
  controller.assign_interfaces(std::move(loaned_commands),
                               std::move(loaned_states));

  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_NEAR(*commands[4]->get_optional(), -0.5, 1e-12);

  const double previous_torque = 50.0 * (0.8 - 0.75) + 2.0;
  const double new_torque = 100.0 * (*commands[0]->get_optional() - 0.75) +
                            *commands[4]->get_optional();
  EXPECT_NEAR(new_torque, previous_torque, 1e-12);

  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  controller.release_interfaces();
}

TEST(MitPositionAdapterControllerTest, WritesPinocchioGravityFeedforward) {
  mit_position_adapter_controller::MitPositionAdapterController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
       rclcpp::Parameter("downstream_controller", "mit_output"),
       rclcpp::Parameter("position_kp", std::vector<double>{50.0}),
       rclcpp::Parameter("position_kd", std::vector<double>{3.0}),
       rclcpp::Parameter("gravity_compensation.enabled", true),
       rclcpp::Parameter("gravity_compensation.factor", 1.0),
       rclcpp::Parameter("gravity_compensation.factor_transition_duration",
                         0.0),
       rclcpp::Parameter("gravity_compensation.scale",
                         std::vector<double>{1.0}),
       rclcpp::Parameter("gravity_compensation.bias", std::vector<double>{2.0}),
       rclcpp::Parameter("gravity_compensation.torque_limit",
                         std::vector<double>{20.0}),
       rclcpp::Parameter("gravity_compensation.torque_rate_limit",
                         std::vector<double>{10000.0}),
       rclcpp::Parameter("gravity_compensation.transition_duration", 0.0)});
  ASSERT_EQ(
      controller.init("test_gravity_adapter", kPendulumUrdf, 200, "", options),
      controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::CommandInterface::SharedPtr> commands = {
      make_command_interface("mit_output", "joint1/position", 0.0),
      make_command_interface("mit_output", "joint1/velocity", 0.0),
      make_command_interface("mit_output", "joint1/kp", 50.0),
      make_command_interface("mit_output", "joint1/kd", 3.0),
      make_command_interface("mit_output", "joint1/t_ff", 0.0)};
  auto position_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "position", "double");
  auto velocity_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "velocity", "double");
  ASSERT_TRUE(position_state->set_value(0.0));
  ASSERT_TRUE(velocity_state->set_value(0.0));

  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  for (const auto &command : commands) {
    loaned_commands.emplace_back(command, nullptr);
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  loaned_states.emplace_back(position_state);
  loaned_states.emplace_back(velocity_state);
  controller.assign_interfaces(std::move(loaned_commands),
                               std::move(loaned_states));

  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*commands[4]->get_optional(), -7.80665, 1e-6);

  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  controller.release_interfaces();
}

TEST(MitPositionAdapterControllerTest, UpdatesGravityFactorAtRuntime) {
  mit_position_adapter_controller::MitPositionAdapterController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
       rclcpp::Parameter("downstream_controller", "mit_output"),
       rclcpp::Parameter("position_kp", std::vector<double>{50.0}),
       rclcpp::Parameter("position_kd", std::vector<double>{3.0}),
       rclcpp::Parameter("gravity_compensation.enabled", true),
       rclcpp::Parameter("gravity_compensation.factor", 0.0),
       rclcpp::Parameter("gravity_compensation.factor_transition_duration",
                         1.0),
       rclcpp::Parameter("gravity_compensation.scale",
                         std::vector<double>{1.0}),
       rclcpp::Parameter("gravity_compensation.bias", std::vector<double>{0.0}),
       rclcpp::Parameter("gravity_compensation.torque_limit",
                         std::vector<double>{20.0}),
       rclcpp::Parameter("gravity_compensation.torque_rate_limit",
                         std::vector<double>{10000.0}),
       rclcpp::Parameter("gravity_compensation.transition_duration", 0.0)});
  ASSERT_EQ(controller.init("test_dynamic_gravity_factor", kPendulumUrdf, 200,
                            "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::CommandInterface::SharedPtr> commands = {
      make_command_interface("mit_output", "joint1/position", 0.0),
      make_command_interface("mit_output", "joint1/velocity", 0.0),
      make_command_interface("mit_output", "joint1/kp", 50.0),
      make_command_interface("mit_output", "joint1/kd", 3.0),
      make_command_interface("mit_output", "joint1/t_ff", 0.0)};
  auto position_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "position", "double");
  auto velocity_state = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "velocity", "double");
  ASSERT_TRUE(position_state->set_value(0.0));
  ASSERT_TRUE(velocity_state->set_value(0.0));

  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  for (const auto &command : commands) {
    loaned_commands.emplace_back(command, nullptr);
  }
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  loaned_states.emplace_back(position_state);
  loaned_states.emplace_back(velocity_state);
  controller.assign_interfaces(std::move(loaned_commands),
                               std::move(loaned_states));

  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(*commands[4]->get_optional(), 0.0);

  const auto set_result = controller.get_node()->set_parameter(
      rclcpp::Parameter("gravity_compensation.factor", 1.0));
  ASSERT_TRUE(set_result.successful) << set_result.reason;
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.25)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*commands[4]->get_optional(), -0.25 * 9.80665, 1e-6);

  const auto scale_result = controller.get_node()->set_parameters_atomically(
      {rclcpp::Parameter("gravity_compensation.scale",
                         std::vector<double>{2.0}),
       rclcpp::Parameter("gravity_compensation.bias",
                         std::vector<double>{1.0})});
  ASSERT_TRUE(scale_result.successful) << scale_result.reason;
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.25)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*commands[4]->get_optional(), -9.30665, 1e-6);

  const auto invalid_scale_result =
      controller.get_node()->set_parameter(rclcpp::Parameter(
          "gravity_compensation.scale", std::vector<double>{1.0, 1.0}));
  EXPECT_FALSE(invalid_scale_result.successful);

  const auto invalid_bias_result = controller.get_node()->set_parameter(
      rclcpp::Parameter("gravity_compensation.bias", std::vector<double>{6.0}));
  EXPECT_FALSE(invalid_bias_result.successful);

  const auto invalid_result = controller.get_node()->set_parameter(
      rclcpp::Parameter("gravity_compensation.factor", 1.1));
  EXPECT_FALSE(invalid_result.successful);

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
