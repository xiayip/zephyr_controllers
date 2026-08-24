#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "mit_impedance_controller/mit_impedance_controller.hpp"
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
make_command(const std::string &interface_name, const double value) {
  auto interface = std::make_shared<hardware_interface::CommandInterface>(
      "mit_output", "joint1/" + interface_name, "double");
  EXPECT_TRUE(interface->set_value(value));
  return interface;
}

rclcpp::NodeOptions base_options(const bool gravity_enabled,
                                 const bool soft_limits_enabled,
                                 const double transition_duration,
                                 const double gravity_bias = 0.0) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
      rclcpp::Parameter("downstream_controller", "mit_output"),
      rclcpp::Parameter("stiffness", std::vector<double>{10.0}),
      rclcpp::Parameter("damping_coefficients", std::vector<double>{2.0}),
      rclcpp::Parameter("feedback.stiffness_torque_limit",
                        std::vector<double>{20.0}),
      rclcpp::Parameter("reference.position_rate_limit",
                        std::vector<double>{10.0}),
      rclcpp::Parameter("reference.velocity_limit", std::vector<double>{2.0}),
      rclcpp::Parameter("gravity_compensation.enabled", gravity_enabled),
      rclcpp::Parameter("gravity_compensation.factor", 1.0),
      rclcpp::Parameter("gravity_compensation.scale", std::vector<double>{1.0}),
      rclcpp::Parameter("gravity_compensation.bias",
                        std::vector<double>{gravity_bias}),
      rclcpp::Parameter("friction_compensation.enabled", false),
      rclcpp::Parameter("friction_compensation.coulomb",
                        std::vector<double>{0.0}),
      rclcpp::Parameter("friction_compensation.viscous",
                        std::vector<double>{0.0}),
      rclcpp::Parameter("friction_compensation.velocity_smoothing",
                        std::vector<double>{0.04}),
      rclcpp::Parameter("command_shaping.torque_limit",
                        std::vector<double>{20.0}),
      rclcpp::Parameter("command_shaping.torque_rate_limit",
                        std::vector<double>{1000.0}),
      rclcpp::Parameter("command_shaping.transition_duration",
                        transition_duration),
      rclcpp::Parameter("soft_limits.enabled", soft_limits_enabled),
      rclcpp::Parameter("soft_limits.margin", std::vector<double>{0.2}),
      rclcpp::Parameter("soft_limits.stiffness", std::vector<double>{10.0}),
      rclcpp::Parameter("soft_limits.torque_limit", std::vector<double>{2.0}),
  });
  return options;
}

struct FixtureInterfaces {
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands;
  hardware_interface::StateInterface::SharedPtr position;
  hardware_interface::StateInterface::SharedPtr velocity;
};

FixtureInterfaces
assign_interfaces(mit_impedance_controller::MitImpedanceController &controller,
                  const double position, const double velocity,
                  const double old_position, const double old_velocity,
                  const double old_kp, const double old_kd,
                  const double old_t_ff) {
  FixtureInterfaces fixture;
  fixture.commands = {make_command("position", old_position),
                      make_command("velocity", old_velocity),
                      make_command("kp", old_kp), make_command("kd", old_kd),
                      make_command("t_ff", old_t_ff)};
  fixture.position = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "position", "double");
  fixture.velocity = std::make_shared<hardware_interface::StateInterface>(
      "joint1", "velocity", "double");
  EXPECT_TRUE(fixture.position->set_value(position));
  EXPECT_TRUE(fixture.velocity->set_value(velocity));
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  for (const auto &interface : fixture.commands) {
    commands.emplace_back(interface, nullptr);
  }
  std::vector<hardware_interface::LoanedStateInterface> states;
  states.emplace_back(fixture.position);
  states.emplace_back(fixture.velocity);
  controller.assign_interfaces(std::move(commands), std::move(states));
  return fixture;
}

TEST(MitImpedanceControllerTest, CommandsMotorSideMitImpedance) {
  mit_impedance_controller::MitImpedanceController controller;
  ASSERT_EQ(controller.init("test_impedance", kPendulumUrdf, 200, "",
                            base_options(false, false, 0.0)),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  auto references = controller.export_reference_interfaces();
  ASSERT_EQ(references.size(), 2U);
  auto fixture =
      assign_interfaces(controller, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(references[0]->set_value(0.2));
  ASSERT_TRUE(references[1]->set_value(0.0));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*fixture.commands[0]->get_optional(), 0.2, 1e-12);
  EXPECT_DOUBLE_EQ(*fixture.commands[1]->get_optional(), 0.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[2]->get_optional(), 10.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[3]->get_optional(), 2.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[4]->get_optional(), 0.0);

  const auto stiffness_result = controller.get_node()->set_parameter(
      rclcpp::Parameter("stiffness", std::vector<double>{20.0}));
  ASSERT_TRUE(stiffness_result.successful) << stiffness_result.reason;
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*fixture.commands[2]->get_optional(), 20.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[4]->get_optional(), 0.0);

  const auto damping_result = controller.get_node()->set_parameter(
      rclcpp::Parameter("damping_coefficients", std::vector<double>{3.0}));
  ASSERT_TRUE(damping_result.successful) << damping_result.reason;
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*fixture.commands[3]->get_optional(), 3.0);

  const auto invalid_result = controller.get_node()->set_parameter(
      rclcpp::Parameter("stiffness", std::vector<double>{10.0, 10.0}));
  EXPECT_FALSE(invalid_result.successful);
  EXPECT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  controller.release_interfaces();
}

TEST(MitImpedanceControllerTest, PreservesEquivalentActivationTorque) {
  mit_impedance_controller::MitImpedanceController controller;
  ASSERT_EQ(controller.init("test_impedance_transition", kPendulumUrdf, 200, "",
                            base_options(false, false, 1.0)),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  auto fixture =
      assign_interfaces(controller, 0.0, 0.1, 0.2, 0.0, 10.0, 2.0, 3.0);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(*fixture.commands[0]->get_optional(), 0.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[1]->get_optional(), 0.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[2]->get_optional(), 10.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[3]->get_optional(), 2.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[4]->get_optional(), 5.0);

  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*fixture.commands[2]->get_optional(), 10.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[3]->get_optional(), 2.0);
  EXPECT_NEAR(*fixture.commands[4]->get_optional(), 2.5, 1e-12);

  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5)),
            controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(*fixture.commands[2]->get_optional(), 10.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[3]->get_optional(), 2.0);
  EXPECT_DOUBLE_EQ(*fixture.commands[4]->get_optional(), 0.0);
  controller.release_interfaces();
}

TEST(MitImpedanceControllerTest, LimitsMotorSideSpringTorque) {
  auto options = base_options(false, false, 0.0);
  options.parameter_overrides().emplace_back("feedback.stiffness_torque_limit",
                                             std::vector<double>{1.0});
  mit_impedance_controller::MitImpedanceController controller;
  ASSERT_EQ(controller.init("test_impedance_spring_limit", kPendulumUrdf, 200,
                            "", options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  auto references = controller.export_reference_interfaces();
  auto fixture =
      assign_interfaces(controller, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(references[0]->set_value(0.5));
  ASSERT_TRUE(references[1]->set_value(0.0));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*fixture.commands[0]->get_optional(), 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(*fixture.commands[2]->get_optional(), 10.0);
  controller.release_interfaces();
}

TEST(MitImpedanceControllerTest, AddsAffineGravityFeedforward) {
  auto options = base_options(true, false, 0.0, 2.0);
  options.parameter_overrides().emplace_back("stiffness",
                                             std::vector<double>{0.0});
  options.parameter_overrides().emplace_back("damping_coefficients",
                                             std::vector<double>{0.0});
  mit_impedance_controller::MitImpedanceController controller;
  ASSERT_EQ(controller.init("test_impedance_gravity", kPendulumUrdf, 200, "",
                            options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  auto fixture =
      assign_interfaces(controller, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*fixture.commands[4]->get_optional(), -7.80665, 1e-6);
  controller.release_interfaces();
}

TEST(MitImpedanceControllerTest, AddsSmoothFrictionFeedforward) {
  auto options = base_options(false, false, 0.0);
  options.parameter_overrides().emplace_back("stiffness",
                                             std::vector<double>{0.0});
  options.parameter_overrides().emplace_back("damping_coefficients",
                                             std::vector<double>{0.0});
  options.parameter_overrides().emplace_back("friction_compensation.enabled",
                                             true);
  options.parameter_overrides().emplace_back("friction_compensation.coulomb",
                                             std::vector<double>{0.3});
  options.parameter_overrides().emplace_back("friction_compensation.viscous",
                                             std::vector<double>{0.2});
  options.parameter_overrides().emplace_back(
      "friction_compensation.velocity_smoothing", std::vector<double>{0.05});
  mit_impedance_controller::MitImpedanceController controller;
  ASSERT_EQ(controller.init("test_impedance_friction", kPendulumUrdf, 200, "",
                            options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  auto references = controller.export_reference_interfaces();
  auto fixture =
      assign_interfaces(controller, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);

  ASSERT_TRUE(references[1]->set_value(0.2));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*fixture.commands[4]->get_optional(),
              0.3 * std::tanh(0.2 / 0.05) + 0.2 * 0.2, 1e-12);

  ASSERT_TRUE(references[1]->set_value(-0.2));
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*fixture.commands[4]->get_optional(),
              -0.3 * std::tanh(0.2 / 0.05) - 0.2 * 0.2, 1e-12);

  const auto coulomb_result =
      controller.get_node()->set_parameter(rclcpp::Parameter(
          "friction_compensation.coulomb", std::vector<double>{0.25}));
  ASSERT_TRUE(coulomb_result.successful) << coulomb_result.reason;
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*fixture.commands[4]->get_optional(),
              -0.25 * std::tanh(0.2 / 0.05) - 0.2 * 0.2, 1e-12);
  controller.release_interfaces();
}

TEST(MitImpedanceControllerTest, AppliesUrdfSoftLimitTorque) {
  auto options = base_options(false, true, 0.0);
  options.parameter_overrides().emplace_back("stiffness",
                                             std::vector<double>{0.0});
  options.parameter_overrides().emplace_back("damping_coefficients",
                                             std::vector<double>{0.0});
  mit_impedance_controller::MitImpedanceController controller;
  ASSERT_EQ(controller.init("test_impedance_soft_limit", kPendulumUrdf, 200, "",
                            options),
            controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  auto fixture =
      assign_interfaces(controller, -3.1, 0.0, -3.1, 0.0, 0.0, 0.0, 0.0);
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(controller.update_and_write_commands(
                rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1)),
            controller_interface::return_type::OK);
  EXPECT_NEAR(*fixture.commands[4]->get_optional(), 1.6, 1e-12);
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
