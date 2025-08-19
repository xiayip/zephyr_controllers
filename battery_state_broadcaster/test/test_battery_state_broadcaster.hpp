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

#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>

#include <battery_state_broadcaster/battery_state_broadcaster.hpp>

class FriendBatteryStateBroadcaster
  : public battery_state_broadcaster::BatteryStateBroadcaster
{
  FRIEND_TEST(BatteryStateBroadcasterTest, SensorNameParameterNotSet);
  FRIEND_TEST(BatteryStateBroadcasterTest, InterfaceNamesParameterNotSet);
  FRIEND_TEST(BatteryStateBroadcasterTest, FrameIdParameterNotSet);
  FRIEND_TEST(BatteryStateBroadcasterTest, SensorNameParameterIsEmpty);
  FRIEND_TEST(BatteryStateBroadcasterTest, InterfaceNameParameterIsEmpty);

  FRIEND_TEST(BatteryStateBroadcasterTest, ActivateSuccess);
  FRIEND_TEST(BatteryStateBroadcasterTest, UpdateTest);
  FRIEND_TEST(BatteryStateBroadcasterTest, SensorStatePublishTest);
};


class BatteryStateBroadcasterTest : public ::testing::Test
{
protected:
  const std::string sensor_name_ = "btr_sensor";
  const std::string frame_id_ = "btr_frame";
  const float voltage_ = 3.4;
  const float temperature_ = 24.5;
  const float current_ = 0.5;
  const float charge_ = 24.1;
  const float capacity_ = 4.8;
  const float percentage_ = 0.7;

  const std::array<std::string, 6> state_interface_names = {
    "voltage", "temperature", "current", "charge", "capacity", "percentage"
  };

  std::vector<double> values_;
  std::unique_ptr<FriendBatteryStateBroadcaster> btr_broadcaster_;

  std::vector<hardware_interface::StateInterface> state_interfaces_;

public:
  static void SetUpTestCase();
  static void TearDownTestCase();

  void SetUp();
  void TearDown();

  void SetUpBatteryBroadcaster();

  void subscribe_and_get_message(sensor_msgs::msg::BatteryState &);
};
