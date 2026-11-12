// SPDX-License-Identifier: Apache-2.0
#include "../plugin/controller_switcher.hpp"
#include <gtest/gtest.h>
#include <thread>

namespace zephyr::mpc_rviz {
namespace {
using State = controller_manager_msgs::msg::ControllerState;
State controller(const std::string& name, const std::string& state, const std::string& type) {
  State value;
  value.name = name;
  value.state = state;
  value.type = type;
  value.required_command_interfaces = {"adapter/joint1/position"};
  if (state == "active") value.claimed_interfaces = value.required_command_interfaces;
  return value;
}

class ControllerSwitcherTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  void SetUp() override {
    node = std::make_shared<rclcpp::Node>("test_controller_switcher");
    states = {controller("mpc_controller", "inactive", "mpc_controller/mpc_controller"),
              controller("zephyr_arm_trajectory_controller", "active", "joint_trajectory_controller/JointTrajectoryController"),
              controller("adapter", "active", "test/Adapter"),
              controller("gripper", "active", "test/Gripper")};
    states[2].required_command_interfaces = states[2].claimed_interfaces = {"mit/joint1/position"};
    states[3].required_command_interfaces = states[3].claimed_interfaces = {"finger/position"};
    list = node->create_service<ControllerSwitcher::List>("/controller_manager/list_controllers",
        [this](ControllerSwitcher::List::Request::SharedPtr,
               ControllerSwitcher::List::Response::SharedPtr response) { response->controller = states; });
    service = node->create_service<ControllerSwitcher::Switch>("/controller_manager/switch_controller",
        [this](ControllerSwitcher::Switch::Request::SharedPtr request,
               ControllerSwitcher::Switch::Response::SharedPtr response) {
          ++requests;
          last = *request;
          response->ok = success;
          if (success && apply) {
            for (auto& c : states) {
              if (std::find(request->deactivate_controllers.begin(), request->deactivate_controllers.end(), c.name) !=
                  request->deactivate_controllers.end()) { c.state = "inactive"; c.claimed_interfaces.clear(); }
              if (std::find(request->activate_controllers.begin(), request->activate_controllers.end(), c.name) !=
                  request->activate_controllers.end()) { c.state = "active"; c.claimed_interfaces = c.required_command_interfaces; }
            }
          }
        });
    executor.add_node(node);
    client = std::make_unique<ControllerSwitcher>(node);
  }
  void TearDown() override { client.reset(); executor.remove_node(node); }
  template<class Predicate> bool until(Predicate done, int timeout_seconds = 3) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      client->poll();
      if (done()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }
  void ready() { ASSERT_TRUE(until([this] { return client->view().enabled; })); }
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Service<ControllerSwitcher::List>::SharedPtr list;
  rclcpp::Service<ControllerSwitcher::Switch>::SharedPtr service;
  std::unique_ptr<ControllerSwitcher> client;
  std::vector<State> states;
  ControllerSwitcher::Switch::Request last;
  int requests{0};
  bool success{true}, apply{true};
};

TEST_F(ControllerSwitcherTest, RequiresDiscoveryAndFreshControllerState) {
  EXPECT_FALSE(client->view().enabled);
  EXPECT_FALSE(client->request("mpc_controller"));
  ready();
  EXPECT_EQ(client->view().target, "mpc_controller");
  std::this_thread::sleep_for(std::chrono::milliseconds(2100));
  EXPECT_FALSE(client->view().enabled);
  EXPECT_FALSE(client->request("mpc_controller"));
  EXPECT_EQ(requests, 0);
}

TEST_F(ControllerSwitcherTest, StrictUpstreamSwitchPreservesDependenciesAndSupportsReturn) {
  ready();
  ASSERT_TRUE(client->request("mpc_controller"));
  EXPECT_TRUE(client->busy());
  EXPECT_FALSE(client->request("mpc_controller"));
  ASSERT_TRUE(until([this] { return !client->busy(); }));
  EXPECT_EQ(last.activate_controllers, std::vector<std::string>{"mpc_controller"});
  EXPECT_EQ(last.deactivate_controllers, std::vector<std::string>{"zephyr_arm_trajectory_controller"});
  EXPECT_EQ(last.strictness, ControllerSwitcher::Switch::Request::STRICT);
  EXPECT_FALSE(last.activate_asap);
  EXPECT_EQ(last.timeout.sec, 5);
  EXPECT_EQ(states[2].state, "active");
  EXPECT_EQ(states[3].state, "active");
  EXPECT_EQ(client->view().target, "zephyr_arm_trajectory_controller");
  ASSERT_TRUE(client->request(client->view().target));
  ASSERT_TRUE(until([this] { return !client->busy(); }));
  EXPECT_EQ(last.deactivate_controllers, std::vector<std::string>{"mpc_controller"});
  EXPECT_EQ(last.activate_controllers, std::vector<std::string>{"zephyr_arm_trajectory_controller"});
  EXPECT_EQ(requests, 2);
}

TEST_F(ControllerSwitcherTest, BothInactiveDoesNotDeactivateAnInactiveController) {
  states[1].state = "inactive";
  ready();
  ASSERT_TRUE(client->request("mpc_controller"));
  ASSERT_TRUE(until([this] { return !client->busy(); }));
  EXPECT_TRUE(last.deactivate_controllers.empty());
}

TEST_F(ControllerSwitcherTest, RejectsStaleDestinationAfterConfirmation) {
  states[0].state = "active";
  states[1].state = "inactive";
  ready();
  EXPECT_FALSE(client->request("mpc_controller"));
  EXPECT_EQ(requests, 0);
}

TEST_F(ControllerSwitcherTest, MissingJtcDoesNotAutoLoadOrDisableMpc) {
  states.erase(states.begin() + 1);
  ASSERT_TRUE(until([this] { return client->view().message.find("先加载") != std::string::npos; }));
  EXPECT_FALSE(client->view().enabled);
  EXPECT_FALSE(client->request("mpc_controller"));
  EXPECT_EQ(requests, 0);
}

TEST_F(ControllerSwitcherTest, UnconfiguredControllerIsNotAutomaticallyConfigured) {
  states[0].state = "unconfigured";
  ASSERT_TRUE(until([this] { return client->view().message.find("不能安全切换") != std::string::npos; }));
  EXPECT_FALSE(client->view().enabled);
}

TEST_F(ControllerSwitcherTest, DoesNotStealInterfacesFromOtherControllers) {
  states[1].state = "inactive";
  states.push_back(controller("teleop", "active", "test/Teleop"));
  ASSERT_TRUE(until([this] { return client->view().message.find("teleop") != std::string::npos; }));
  EXPECT_FALSE(client->request("mpc_controller"));
  EXPECT_EQ(requests, 0);
}

TEST_F(ControllerSwitcherTest, FailureIsReportedWithoutRetryOrRollback) {
  success = false;
  ready();
  ASSERT_TRUE(client->request("mpc_controller"));
  ASSERT_TRUE(until([this] { return !client->busy(); }));
  EXPECT_NE(client->view().message.find("失败"), std::string::npos);
  for (int i = 0; i < 20; ++i) { executor.spin_some(); client->poll(); }
  EXPECT_EQ(requests, 1);
  EXPECT_EQ(states[1].state, "active");
}

TEST_F(ControllerSwitcherTest, SuccessfulResponseStillRequiresStateVerification) {
  apply = false;
  ready();
  ASSERT_TRUE(client->request("mpc_controller"));
  ASSERT_TRUE(until([this] { return !client->busy(); }));
  EXPECT_NE(client->view().message.find("不一致"), std::string::npos);
  EXPECT_EQ(requests, 1);
}

TEST_F(ControllerSwitcherTest, LostResponseIsUnknownAndNeverAutomaticallyRetried) {
  service.reset();
  service = node->create_service<ControllerSwitcher::Switch>("/controller_manager/switch_controller",
      [this](std::shared_ptr<rclcpp::Service<ControllerSwitcher::Switch>>,
             std::shared_ptr<rmw_request_id_t>, ControllerSwitcher::Switch::Request::SharedPtr) {
        ++requests;  // Deliberately omit the response, simulating a lost connection.
      });
  ready();
  ASSERT_TRUE(client->request("mpc_controller"));
  ASSERT_TRUE(until([this] { return !client->busy(); }, 12));
  EXPECT_NE(client->view().message.find("结果未知"), std::string::npos);
  EXPECT_EQ(requests, 1);
  EXPECT_EQ(states[1].state, "active");
}

TEST_F(ControllerSwitcherTest, ConfigurableControllerNames) {
  states[0].name = "arm_mpc";
  states[1].name = "arm_jtc";
  node->declare_parameter("mpc_controller_name", "arm_mpc");
  node->declare_parameter("trajectory_controller_name", "arm_jtc");
  client = std::make_unique<ControllerSwitcher>(node);
  ready();
  ASSERT_TRUE(client->request("arm_mpc"));
  ASSERT_TRUE(until([this] { return !client->busy(); }));
  EXPECT_EQ(last.activate_controllers, std::vector<std::string>{"arm_mpc"});
  EXPECT_EQ(last.deactivate_controllers, std::vector<std::string>{"arm_jtc"});
}
}  // namespace
}  // namespace zephyr::mpc_rviz
