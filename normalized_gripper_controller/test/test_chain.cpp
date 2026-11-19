#include <gtest/gtest.h>
#include <normalized_gripper_controller/normalized_gripper_controller.hpp>

using normalized_gripper_controller::NormalizedGripperController;
using controller_interface::CallbackReturn;
using controller_interface::return_type;

TEST(GripperChain, BoundsAndNoStaleCommandOnRelease) {
  NormalizedGripperController c;
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({rclcpp::Parameter("joints",std::vector<std::string>{"grip"}),
      rclcpp::Parameter("topic_prefix","test_gripper"),
      rclcpp::Parameter("command_interfaces",std::vector<std::string>{"position"}),
      rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position"})});
  ASSERT_EQ(c.init("test_gripper","",200,"",opts),return_type::OK);
  ASSERT_EQ(c.on_configure(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  auto refs=c.export_reference_interfaces(); ASSERT_EQ(refs.size(),3U);
  auto command=std::make_shared<hardware_interface::CommandInterface>("grip","position","double");
  auto state=std::make_shared<hardware_interface::StateInterface>("grip","position","double");
  ASSERT_TRUE(command->set_value(.2)); ASSERT_TRUE(state->set_value(.2));
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  commands.emplace_back(command,nullptr); states.emplace_back(state);
  c.assign_interfaces(std::move(commands),std::move(states));
  ASSERT_EQ(c.on_activate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
  ASSERT_TRUE(c.set_chained_mode(true));
  ASSERT_TRUE(refs[0]->set_value(.7));
  ASSERT_TRUE(refs[1]->set_value(.5));
  ASSERT_TRUE(refs[2]->set_value(.4));
  const auto dt=rclcpp::Duration::from_seconds(.005);
  EXPECT_EQ(c.update(rclcpp::Time(0),dt),return_type::OK);
  EXPECT_DOUBLE_EQ(command->get_optional().value(),.7);
  for (size_t i=0;i<3;++i) {
    const auto old=refs[i]->get_optional().value();
    ASSERT_TRUE(refs[i]->set_value(-.1));
    EXPECT_EQ(c.update(rclcpp::Time(0),dt),return_type::ERROR);
    EXPECT_DOUBLE_EQ(command->get_optional().value(),.7);
    ASSERT_TRUE(refs[i]->set_value(old));
  }
  ASSERT_TRUE(c.set_chained_mode(false));
  EXPECT_EQ(c.update(rclcpp::Time(0),dt),return_type::OK);
  EXPECT_DOUBLE_EQ(command->get_optional().value(),.7); // Not pre-chain .2.
  EXPECT_EQ(c.on_deactivate(rclcpp_lifecycle::State()),CallbackReturn::SUCCESS);
}
int main(int argc,char** argv) {
  rclcpp::init(argc,argv); testing::InitGoogleTest(&argc,argv);
  const int result=RUN_ALL_TESTS(); rclcpp::shutdown(); return result;
}
