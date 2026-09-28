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

#include <rclcpp/rclcpp.hpp>
#include <semantic_components/semantic_component_interface.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace semantic_components
{
class BatteryState : public SemanticComponentInterface<sensor_msgs::msg::BatteryState>
{
public:
  explicit BatteryState(const std::string & name, const std::vector<std::string> & interfaces)
  : SemanticComponentInterface(name, interfaces.size())
  {
    for (const auto & interface : interfaces) {
      this->interface_names_.emplace_back(this->name_ + "/" + interface);
    }
  }

  virtual ~BatteryState() = default;

  bool get_value_as_message(sensor_msgs::msg::BatteryState & message)
  {
    for (const auto & state_interface : this->state_interfaces_) {
      const auto & name = state_interface.get().get_interface_name();
      const auto value = state_interface.get().get_optional().value_or(
        std::numeric_limits<double>::quiet_NaN());
      if (name == "voltage") {
        message.voltage = value;
      } else if (name == "temperature") {
        message.temperature = value;
      } else if (name == "current") {
        message.current = value;
      } else if (name == "charge") {
        message.charge = value;
      } else if (name == "capacity") {
        message.capacity = value;
      } else if (name == "percentage") {
        message.percentage = value;
      } else if (name == "power_supply_status") {
        message.power_supply_status = static_cast<uint8_t>(value);
      } else if (name == "power_supply_health") {
        message.power_supply_health = static_cast<uint8_t>(value);
      }
    }
    return true;
  }
};
}  // namespace semantic_components
