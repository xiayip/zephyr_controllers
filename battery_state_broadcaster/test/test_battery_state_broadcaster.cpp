// Copyright 2023 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "test_battery_state_broadcaster.hpp"

#include <vector>
#include <memory>
#include <utility>

namespace
{
constexpr auto NODE_SUCCESS = controller_interface::CallbackReturn::SUCCESS;
constexpr auto NODE_ERROR = controller_interface::CallbackReturn::ERROR;
}  // namespace


void BatteryStateBroadcasterTest::SetUpTestCase() {}

void BatteryStateBroadcasterTest::TearDownTestCase() {}

void BatteryStateBroadcasterTest::SetUp()
{
  this->btr_broadcaster_ = std::make_unique<FriendBatteryStateBroadcaster>();
  this->values_.resize(6, std::numeric_limits<double>::quiet_NaN());

  for (uint i = 0; i < this->state_interface_names.size(); i++) {
    state_interfaces_.emplace_back(
      hardware_interface::StateInterface(
        this->sensor_name_, this->state_interface_names[i], &this->values_[i]));
  }
}

void BatteryStateBroadcasterTest::TearDown()
{
  this->btr_broadcaster_.reset(nullptr);
}


void BatteryStateBroadcasterTest::SetUpBatteryBroadcaster()
{
  const auto result = this->btr_broadcaster_->init("test_battery_state_broadcaster");
  ASSERT_EQ(result, controller_interface::return_type::OK);
  std::vector<hardware_interface::LoanedStateInterface> state_ifs;
  for (size_t i = 0; i < 6; i++) {
    state_ifs.emplace_back(this->state_interfaces_[i]);
  }

  this->btr_broadcaster_->assign_interfaces({}, std::move(state_ifs));
}


void BatteryStateBroadcasterTest::subscribe_and_get_message(
  sensor_msgs::msg::BatteryState & btr_msg)
{
  rclcpp::Node test_subscription_node("test_subscription_node");
  auto subs_callback = [&](const sensor_msgs::msg::BatteryState::SharedPtr) {};
  auto subscription = test_subscription_node.create_subscription<sensor_msgs::msg::BatteryState>(
    "/test_battery_state_broadcaster/battery_state", rclcpp::SensorDataQoS(), subs_callback);

  int max_sub_check_loop_count = 5;
  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription);
  while (max_sub_check_loop_count--) {
    this->btr_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01));
    if (wait_set.wait(std::chrono::milliseconds(2)).kind() == rclcpp::WaitResultKind::Ready) {
      break;
    }
  }

  ASSERT_GE(max_sub_check_loop_count, 0) << "Test was unable to publish a message through "
    "controller/broadcaster update loop";

  // take message from subscription
  rclcpp::MessageInfo msg_info;
  ASSERT_TRUE(subscription->take(btr_msg, msg_info));
}

TEST_F(BatteryStateBroadcasterTest, SensorName_Configure_Success)
{
  SetUpBatteryBroadcaster();

  this->btr_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->btr_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});

  ASSERT_EQ(this->btr_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(BatteryStateBroadcasterTest, SensorName_Activate_Success)
{
  SetUpBatteryBroadcaster();

  this->btr_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->btr_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});

  ASSERT_EQ(this->btr_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(this->btr_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(BatteryStateBroadcasterTest, SensorName_Update_Success)
{
  SetUpBatteryBroadcaster();

  this->btr_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->btr_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});

  ASSERT_EQ(this->btr_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(this->btr_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    this->btr_broadcaster_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(BatteryStateBroadcasterTest, SensorName_Publish_Success)
{
  SetUpBatteryBroadcaster();

  this->btr_broadcaster_->get_node()->set_parameter({"sensor_name", this->sensor_name_});
  this->btr_broadcaster_->get_node()->set_parameter({"frame_id", this->frame_id_});

  ASSERT_EQ(this->btr_broadcaster_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(this->btr_broadcaster_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  sensor_msgs::msg::BatteryState btr_msg;
  subscribe_and_get_message(btr_msg);

  EXPECT_EQ(btr_msg.header.frame_id, this->frame_id_);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
