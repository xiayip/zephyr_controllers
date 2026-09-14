#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "mit_position_adapter_controller/mit_position_adapter_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

namespace {
constexpr char kUrdf[] = R"(
<robot name="pendulum"><link name="base"/><link name="link">
  <inertial><origin xyz="0.5 0 0"/><mass value="2"/>
  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial></link><joint name="joint1" type="revolute">
  <parent link="base"/><child link="link"/><axis xyz="0 1 0"/>
  <limit lower="-3.14" upper="3.14" effort="20" velocity="2"/>
</joint></robot>)";

class PositionFrictionTest : public ::testing::Test {
protected:
  void configure(bool enabled = false, double friction_limit = .5,
                 double rate = 10000., bool gravity = false,
                 double total_limit = 20.) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
         rclcpp::Parameter("downstream_controller", "mit"),
         rclcpp::Parameter("position_kp", std::vector<double>{120.}),
         rclcpp::Parameter("position_kd", std::vector<double>{4.}),
         rclcpp::Parameter("gravity_compensation.enabled", gravity),
         rclcpp::Parameter("gravity_compensation.factor", 1.),
         rclcpp::Parameter("gravity_compensation.factor_transition_duration",
                           0.),
         rclcpp::Parameter("gravity_compensation.transition_duration", 0.),
         rclcpp::Parameter("gravity_compensation.torque_limit",
                           std::vector<double>{total_limit}),
         rclcpp::Parameter("gravity_compensation.torque_rate_limit",
                           std::vector<double>{rate}),
         rclcpp::Parameter("friction_compensation.enabled", enabled),
         rclcpp::Parameter("friction_compensation.coulomb",
                           std::vector<double>{.4}),
         rclcpp::Parameter("friction_compensation.viscous",
                           std::vector<double>{.2}),
         rclcpp::Parameter("friction_compensation.velocity_smoothing",
                           std::vector<double>{.04}),
         rclcpp::Parameter("friction_compensation.torque_limit",
                           std::vector<double>{friction_limit})});
    ASSERT_EQ(controller.init("position_friction", kUrdf, 200, "", options),
              controller_interface::return_type::OK);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    refs = controller.export_reference_interfaces();
    std::vector<hardware_interface::LoanedCommandInterface> loans;
    const std::vector<std::string> fields = {"position", "velocity", "kp", "kd",
                                             "t_ff"};
    const std::vector<double> values = {0., 0., 120., 4., 0.};
    for (std::size_t i = 0; i < fields.size(); ++i) {
      auto c = std::make_shared<hardware_interface::CommandInterface>(
          "mit", "joint1/" + fields[i], "double");
      ASSERT_TRUE(c->set_value(values[i]));
      commands.push_back(c);
      loans.emplace_back(c, nullptr);
    }
    position = std::make_shared<hardware_interface::StateInterface>(
        "joint1", "position", "double");
    velocity = std::make_shared<hardware_interface::StateInterface>(
        "joint1", "velocity", "double");
    ASSERT_TRUE(position->set_value(0.));
    ASSERT_TRUE(velocity->set_value(0.));
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(position);
    states.emplace_back(velocity);
    controller.assign_interfaces(std::move(loans), std::move(states));
  }
  void activate() {
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
  }
  void tick(double dt = .01) {
    ASSERT_EQ(controller.update_and_write_commands(
                  rclcpp::Time(0), rclcpp::Duration::from_seconds(dt)),
              controller_interface::return_type::OK);
  }
  void set(const std::string &name, double value) {
    const auto result = controller.get_node()->set_parameter(
        rclcpp::Parameter(name, std::vector<double>{value}));
    ASSERT_TRUE(result.successful) << result.reason;
  }
  void enable(bool enabled) {
    const auto result = controller.get_node()->set_parameter(
        rclcpp::Parameter("friction_compensation.enabled", enabled));
    ASSERT_TRUE(result.successful) << result.reason;
  }
  double ff() { return *commands[4]->get_optional(); }
  void TearDown() override { controller.release_interfaces(); }
  mit_position_adapter_controller::MitPositionAdapterController controller;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands, refs;
  hardware_interface::StateInterface::SharedPtr position, velocity;
};

TEST_F(PositionFrictionTest, DisabledKeepsExistingTupleAndGravityBehavior) {
  configure(false, .5, 10000., true);
  activate();
  ASSERT_TRUE(refs[0]->set_value(.2));
  ASSERT_TRUE(refs[1]->set_value(.04));
  tick();
  EXPECT_NEAR(ff(), -9.80665, 1e-6);
  EXPECT_DOUBLE_EQ(*commands[0]->get_optional(), .2);
  EXPECT_DOUBLE_EQ(*commands[1]->get_optional(), .04);
  EXPECT_DOUBLE_EQ(*commands[2]->get_optional(), 120.);
  EXPECT_DOUBLE_EQ(*commands[3]->get_optional(), 4.);
}

TEST_F(PositionFrictionTest, UsesReferenceVelocitySmoothlyAndSymmetrically) {
  configure(true);
  activate();
  const double expected = .4 * std::tanh(1.) + .2 * .04;
  ASSERT_TRUE(refs[1]->set_value(.04));
  tick();
  EXPECT_NEAR(ff(), expected, 1e-12);
  ASSERT_TRUE(velocity->set_value(-.3));
  tick();
  EXPECT_NEAR(ff(), expected, 1e-12);
  ASSERT_TRUE(refs[1]->set_value(-.04));
  tick();
  EXPECT_NEAR(ff(), -expected, 1e-12);
  ASSERT_TRUE(refs[1]->set_value(0.));
  tick();
  EXPECT_DOUBLE_EQ(ff(), 0.);
  ASSERT_TRUE(refs[1]->set_value(1e-8));
  tick();
  EXPECT_LT(std::abs(ff()), 1e-6);
  set("friction_compensation.coulomb", 0.);
  set("friction_compensation.viscous", 0.);
  ASSERT_TRUE(refs[1]->set_value(.4));
  tick();
  EXPECT_DOUBLE_EQ(ff(), 0.);
}

TEST_F(PositionFrictionTest, FrictionAndCombinedLimitsBothApply) {
  configure(true, .5, 10000., true, 10.);
  activate();
  set("friction_compensation.coulomb", 5.);
  ASSERT_TRUE(refs[1]->set_value(-1.));
  tick();
  EXPECT_DOUBLE_EQ(ff(), -10.);
  const auto factor = controller.get_node()->set_parameter(
      rclcpp::Parameter("gravity_compensation.factor", 0.));
  ASSERT_TRUE(factor.successful);
  tick();
  EXPECT_DOUBLE_EQ(ff(), -.5);
  ASSERT_TRUE(refs[1]->set_value(1.));
  tick();
  EXPECT_DOUBLE_EQ(ff(), .5);
}

TEST_F(PositionFrictionTest, ZeroFrictionLimitDisablesThatContribution) {
  configure(true, 0.);
  activate();
  ASSERT_TRUE(refs[1]->set_value(.2));
  tick();
  EXPECT_DOUBLE_EQ(ff(), 0.);
}

TEST_F(PositionFrictionTest, RuntimeChangesAndZeroVelocityHonorTotalRateLimit) {
  configure(false, .5, .2);
  activate();
  ASSERT_TRUE(refs[1]->set_value(.04));
  tick(.1);
  EXPECT_DOUBLE_EQ(ff(), 0.);
  enable(true);
  tick(.1);
  EXPECT_NEAR(ff(), .02, 1e-12);
  set("friction_compensation.coulomb", 1.);
  tick(.1);
  EXPECT_NEAR(ff(), .04, 1e-12);
  enable(false);
  tick(.1);
  EXPECT_NEAR(ff(), .02, 1e-12);
  tick(.1);
  EXPECT_NEAR(ff(), 0., 1e-12);
  enable(true);
  tick(.1);
  tick(.1);
  EXPECT_NEAR(ff(), .04, 1e-12);
  ASSERT_TRUE(refs[1]->set_value(0.));
  tick(.1);
  EXPECT_NEAR(ff(), .02, 1e-12);
  tick(.1);
  EXPECT_NEAR(ff(), 0., 1e-12);
}

TEST_F(PositionFrictionTest,
       RejectsInvalidRuntimeChangesWithoutChangingOutput) {
  configure(false);
  activate();
  ASSERT_TRUE(refs[1]->set_value(.04));
  const auto invalid = controller.get_node()->set_parameters_atomically(
      {rclcpp::Parameter("friction_compensation.enabled", true),
       rclcpp::Parameter("friction_compensation.coulomb",
                         std::vector<double>{.1, .2})});
  EXPECT_FALSE(invalid.successful);
  tick();
  EXPECT_DOUBLE_EQ(ff(), 0.);
  for (const auto &parameter : std::vector<rclcpp::Parameter>{
           rclcpp::Parameter("friction_compensation.coulomb",
                             std::vector<double>{}),
           rclcpp::Parameter("friction_compensation.coulomb",
                             std::vector<double>{-1.}),
           rclcpp::Parameter("friction_compensation.viscous",
                             std::vector<double>{11.}),
           rclcpp::Parameter("friction_compensation.velocity_smoothing",
                             std::vector<double>{0.}),
           rclcpp::Parameter(
               "friction_compensation.coulomb",
               std::vector<double>{std::numeric_limits<double>::quiet_NaN()}),
           rclcpp::Parameter(
               "friction_compensation.viscous",
               std::vector<double>{std::numeric_limits<double>::infinity()}),
           rclcpp::Parameter("friction_compensation.coulomb",
                             std::string("invalid")),
           rclcpp::Parameter("friction_compensation.torque_limit",
                             std::vector<double>{2.})}) {
    EXPECT_FALSE(controller.get_node()->set_parameter(parameter).successful)
        << parameter.get_name();
  }
  enable(true);
  tick();
  EXPECT_NEAR(ff(), .4 * std::tanh(1.) + .008, 1e-12);
}

TEST_F(PositionFrictionTest, ActivationAndDeactivationPreserveInheritedTorque) {
  configure(true, .5, 2.);
  ASSERT_TRUE(position->set_value(.1));
  ASSERT_TRUE(velocity->set_value(.2));
  ASSERT_TRUE(commands[0]->set_value(.2));
  ASSERT_TRUE(commands[1]->set_value(.3));
  ASSERT_TRUE(commands[4]->set_value(2.));
  activate();
  EXPECT_DOUBLE_EQ(*refs[0]->get_optional(), .1);
  EXPECT_DOUBLE_EQ(*refs[1]->get_optional(), 0.);
  EXPECT_NEAR(ff(), 15.2, 1e-12);
  tick(.1);
  EXPECT_NEAR(ff(), 15., 1e-12);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_NEAR(ff(), 15., 1e-12);
  activate();
  EXPECT_NEAR(ff(), 15., 1e-12);
}

TEST_F(PositionFrictionTest, NonfiniteFeedbackRetainsPreviousFeedforward) {
  configure(true);
  activate();
  ASSERT_TRUE(refs[1]->set_value(.04));
  tick();
  const double previous = ff();
  ASSERT_TRUE(position->set_value(std::numeric_limits<double>::quiet_NaN()));
  tick();
  EXPECT_DOUBLE_EQ(ff(), previous);
}

TEST_F(PositionFrictionTest,
       PublishesRawLimitedAndTotalTargetsAfterExistingFields) {
  configure(true, .1);
  activate();
  ASSERT_TRUE(refs[1]->set_value(.04));
  auto observer = std::make_shared<rclcpp::Node>("friction_observer");
  control_msgs::msg::DynamicJointState::SharedPtr message;
  auto sub =
      observer->create_subscription<control_msgs::msg::DynamicJointState>(
          "/position_friction/mit_state", rclcpp::SensorDataQoS(),
          [&message](control_msgs::msg::DynamicJointState::SharedPtr m) {
            message = m;
          });
  for (int i = 0; i < 200 && !message; ++i) {
    tick(.02);
    rclcpp::spin_some(observer);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(message);
  ASSERT_EQ(message->interface_values.size(), 1U);
  const auto &v = message->interface_values[0];
  ASSERT_EQ(v.interface_names.size(), 12U);
  ASSERT_EQ(v.values.size(), 12U);
  EXPECT_EQ(v.interface_names[5], "feedforward_torque");
  EXPECT_EQ(v.interface_names[8], "commanded_torque");
  EXPECT_EQ(v.interface_names[9], "raw_friction_torque");
  EXPECT_EQ(v.interface_names[10], "friction_compensation_torque");
  EXPECT_EQ(v.interface_names[11], "target_feedforward_torque");
  EXPECT_NEAR(v.values[9], .4 * std::tanh(1.) + .008, 1e-12);
  EXPECT_DOUBLE_EQ(v.values[10], .1);
  EXPECT_DOUBLE_EQ(v.values[11], .1);
  EXPECT_DOUBLE_EQ(v.values[5], .1);
}
TEST(PositionFrictionConfiguration, RejectsInvalidStartupArrays) {
  for (const auto &invalid : std::vector<rclcpp::Parameter>{
           rclcpp::Parameter("friction_compensation.coulomb",
                             std::vector<double>{.1, .2}),
           rclcpp::Parameter("friction_compensation.viscous",
                             std::vector<double>{.1, .2}),
           rclcpp::Parameter("friction_compensation.velocity_smoothing",
                             std::vector<double>{.04, .04}),
           rclcpp::Parameter("friction_compensation.torque_limit",
                             std::vector<double>{.5, .5}),
           rclcpp::Parameter("friction_compensation.velocity_smoothing",
                             std::vector<double>{0.}),
           rclcpp::Parameter("friction_compensation.coulomb",
                             std::vector<double>{-1.}),
           rclcpp::Parameter("friction_compensation.torque_limit",
                             std::vector<double>{
                                 std::numeric_limits<double>::quiet_NaN()})}) {
    SCOPED_TRACE(invalid.get_name());
    mit_position_adapter_controller::MitPositionAdapterController controller;
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("joints", std::vector<std::string>{"joint1"}),
         rclcpp::Parameter("downstream_controller", "mit"),
         rclcpp::Parameter("position_kp", std::vector<double>{120.}),
         rclcpp::Parameter("position_kd", std::vector<double>{4.}), invalid});
    const auto initialized =
        controller.init("invalid_friction", kUrdf, 200, "", options);
    if (initialized == controller_interface::return_type::OK) {
      EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()),
                controller_interface::CallbackReturn::ERROR);
    } else {
      EXPECT_EQ(initialized, controller_interface::return_type::ERROR);
    }
  }
}
} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
