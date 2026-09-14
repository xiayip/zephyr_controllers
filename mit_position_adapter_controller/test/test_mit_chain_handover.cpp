// Real controller instances, loaned interfaces and lifecycle transitions.
// No CAN transport, hardware launch, or trajectory action server is used.
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "mit_controller/mit_controller.hpp"
#include "mit_position_adapter_controller/mit_position_adapter_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

namespace {
constexpr std::array<const char *, 5> kInterfaces = {"position", "velocity",
                                                     "kp", "kd", "t_ff"};
constexpr std::array<double, 6> kKp = {50, 150, 150, 50, 50, 50};
constexpr std::array<double, 6> kKd = {3, 10, 10, 5, 4, 4};

class MitChainTest : public ::testing::Test {
protected:
  void configure(bool loaded, bool insufficient_limit = false,
                 bool friction = false) {
    std::vector<std::string> joints;
    for (int i = 1; i <= 6; ++i)
      joints.push_back("joint" + std::to_string(i));
    rclcpp::NodeOptions mit_options, adapter_options;
    mit_options.parameter_overrides({rclcpp::Parameter("joints", joints)});
    ASSERT_EQ(mit.init("chain_output", "", 200, "", mit_options),
              controller_interface::return_type::OK);
    ASSERT_EQ(mit.on_configure(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    mit_refs = mit.export_reference_interfaces();
    std::vector<double> limits(6, 100.0);
    if (insufficient_limit)
      limits[5] = .01;
    adapter_options.parameter_overrides(
        {rclcpp::Parameter("joints", joints),
         rclcpp::Parameter("downstream_controller", "chain_output"),
         rclcpp::Parameter("position_kp",
                           std::vector<double>{40, 120, 120, 40, 40, 40}),
         rclcpp::Parameter("position_kd",
                           std::vector<double>{2, 4, 4, 3, 3, 3}),
         rclcpp::Parameter("gravity_compensation.torque_limit", limits),
         rclcpp::Parameter("friction_compensation.enabled", friction),
         rclcpp::Parameter("friction_compensation.coulomb",
                           std::vector<double>{0, 0, .4, 0, 0, 0}),
         rclcpp::Parameter("friction_compensation.viscous",
                           std::vector<double>{0, 0, .2, 0, 0, 0}),
         rclcpp::Parameter("friction_compensation.torque_limit",
                           std::vector<double>{0, 0, .5, 0, 0, 0})});
    ASSERT_EQ(adapter.init("chain_adapter", "", 200, "", adapter_options),
              controller_interface::return_type::OK);
    ASSERT_EQ(adapter.on_configure(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    adapter_refs = adapter.export_reference_interfaces();
    const std::array<double, 6> load = {-.1, 1.5, 5.7, 1.75, -.1, .1};
    const std::array<double, 6> q = {0, .25, .30, -.01, .02, 0};
    std::vector<hardware_interface::LoanedCommandInterface> hardware_loans,
        adapter_loans;
    std::vector<hardware_interface::LoanedStateInterface> mit_states,
        adapter_states;
    for (std::size_t i = 0; i < 6; ++i) {
      const double preload = loaded ? load[i] / kKp[i] : 0.0;
      const std::array<double, 5> tuple = {q[i] + preload, 0, kKp[i], kKd[i],
                                           0};
      for (std::size_t field = 0; field < 5; ++field) {
        auto handle = std::make_shared<hardware_interface::CommandInterface>(
            joints[i], kInterfaces[field], "double");
        ASSERT_TRUE(handle->set_value(tuple[field]));
        hardware.push_back(handle);
        hardware_loans.emplace_back(handle, nullptr);
        adapter_loans.emplace_back(mit_refs[i * 5 + field], nullptr);
      }
      for (std::size_t field = 0; field < 2; ++field) {
        auto handle = std::make_shared<hardware_interface::StateInterface>(
            joints[i], kInterfaces[field], "double");
        ASSERT_TRUE(handle->set_value(field == 0 ? q[i] : 0.0));
        states.push_back(handle);
        mit_states.emplace_back(handle);
        adapter_states.emplace_back(handle);
      }
    }
    mit.assign_interfaces(std::move(hardware_loans), std::move(mit_states));
    adapter.assign_interfaces(std::move(adapter_loans),
                              std::move(adapter_states));
  }

  void TearDown() override {
    adapter.release_interfaces();
    mit.release_interfaces();
  }
  std::array<double, 6> torque() const {
    std::array<double, 6> result;
    for (std::size_t i = 0; i < 6; ++i) {
      const auto get = [this, i](std::size_t field) {
        return *hardware[i * 5 + field]->get_optional();
      };
      result[i] = get(2) * (get(0) - *states[i * 2]->get_optional()) +
                  get(3) * (get(1) - *states[i * 2 + 1]->get_optional()) +
                  get(4);
    }
    return result;
  }
  void expect_torque(const std::array<double, 6> &expected) {
    const auto actual = torque();
    for (std::size_t i = 0; i < 6; ++i)
      EXPECT_NEAR(actual[i], expected[i], 1e-10) << "joint" << i + 1;
  }
  void run_cycle(bool loaded) {
    configure(loaded);
    const auto initial = torque();
    ASSERT_EQ(mit.on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    expect_torque(initial);
    // MIT-only interval between spawners must not clear an inherited preload.
    ASSERT_EQ(mit.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005)),
              controller_interface::return_type::OK);
    expect_torque(initial);
    ASSERT_TRUE(mit.set_chained_mode(true));
    ASSERT_EQ(adapter.on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    for (std::size_t i = 0; i < 6; ++i) {
      // These are the interfaces a JTC reads when it seeds its initial hold.
      EXPECT_DOUBLE_EQ(*adapter_refs[i * 2]->get_optional(),
                       *states[i * 2]->get_optional());
      EXPECT_DOUBLE_EQ(*adapter_refs[i * 2 + 1]->get_optional(), 0.0);
    }
    ASSERT_EQ(mit.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005)),
              controller_interface::return_type::OK);
    expect_torque(initial);

    // Emulate a preceding trajectory controller's moving reference, then exit
    // with nonzero actual velocity and a different actual position.
    ASSERT_TRUE(adapter_refs[4]->set_value(.32));
    ASSERT_TRUE(adapter_refs[5]->set_value(.04));
    ASSERT_EQ(adapter.update_and_write_commands(
                  rclcpp::Time(0), rclcpp::Duration::from_seconds(.02)),
              controller_interface::return_type::OK);
    ASSERT_EQ(mit.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005)),
              controller_interface::return_type::OK);
    ASSERT_TRUE(states[4]->set_value(.303));
    ASSERT_TRUE(states[5]->set_value(.015));
    const auto moving = torque();
    ASSERT_EQ(adapter.on_deactivate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(mit.set_chained_mode(false));
    expect_torque(moving);
    for (std::size_t i = 0; i < 6; ++i) {
      EXPECT_DOUBLE_EQ(*hardware[i * 5]->get_optional(),
                       *states[i * 2]->get_optional());
      EXPECT_DOUBLE_EQ(*hardware[i * 5 + 1]->get_optional(), 0.0);
      EXPECT_DOUBLE_EQ(*hardware[i * 5 + 2]->get_optional(), kKp[i]);
      EXPECT_DOUBLE_EQ(*hardware[i * 5 + 3]->get_optional(), kKd[i]);
    }
    ASSERT_EQ(mit.on_deactivate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    expect_torque(moving);
    // In-process reactivation must not reintroduce an offset or lose bias.
    ASSERT_EQ(mit.on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    ASSERT_TRUE(mit.set_chained_mode(true));
    ASSERT_EQ(adapter.on_activate(rclcpp_lifecycle::State()),
              controller_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(mit.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005)),
              controller_interface::return_type::OK);
    expect_torque(moving);
    for (std::size_t i = 0; i < 6; ++i)
      EXPECT_DOUBLE_EQ(*adapter_refs[i * 2]->get_optional(),
                       *states[i * 2]->get_optional());
  }
  mit_controller::MitController mit;
  mit_position_adapter_controller::MitPositionAdapterController adapter;
  std::vector<hardware_interface::CommandInterface::SharedPtr> hardware,
      mit_refs, adapter_refs;
  std::vector<hardware_interface::StateInterface::SharedPtr> states;
};

TEST_F(MitChainTest, ColdActivationMovingStopAndReactivation) {
  run_cycle(false);
}
TEST_F(MitChainTest, WarmActivationMovingStopAndReactivation) {
  run_cycle(true);
}

TEST_F(MitChainTest, FrictionReachesOnlyConfiguredJointThroughMitChain) {
  configure(false, false, true);
  ASSERT_EQ(mit.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  ASSERT_TRUE(mit.set_chained_mode(true));
  ASSERT_EQ(adapter.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  for (const double velocity : {.04, -.04, 0.}) {
    for (std::size_t i = 0; i < 6; ++i)
      ASSERT_TRUE(adapter_refs[i * 2 + 1]->set_value(velocity));
    ASSERT_EQ(adapter.update_and_write_commands(
                  rclcpp::Time(0), rclcpp::Duration::from_seconds(1.)),
              controller_interface::return_type::OK);
    ASSERT_EQ(mit.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005)),
              controller_interface::return_type::OK);
    for (std::size_t i = 0; i < 6; ++i) {
      const double expected =
          i == 2 ? .4 * std::tanh(velocity / .04) + .2 * velocity : 0.;
      EXPECT_NEAR(*hardware[i * 5 + 4]->get_optional(), expected, 1e-12)
          << "joint" << i + 1;
      EXPECT_DOUBLE_EQ(*hardware[i * 5 + 1]->get_optional(), velocity);
      EXPECT_DOUBLE_EQ(*hardware[i * 5]->get_optional(),
                       *states[i * 2]->get_optional());
    }
  }
}

TEST_F(MitChainTest, SixthJointRejectionDoesNotModifyTheFirstFiveReferences) {
  configure(true, true);
  ASSERT_EQ(mit.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  const auto initial = torque();
  std::vector<double> before;
  for (const auto &handle : mit_refs)
    before.push_back(*handle->get_optional());
  EXPECT_EQ(adapter.on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  for (std::size_t i = 0; i < mit_refs.size(); ++i)
    EXPECT_DOUBLE_EQ(*mit_refs[i]->get_optional(), before[i]);
  expect_torque(initial);
}
} // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
