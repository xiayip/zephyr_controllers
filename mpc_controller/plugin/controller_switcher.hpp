// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <optional>
#include <string>
#include <vector>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace zephyr::mpc_rviz {

// UI-side service client only. Controller lifecycle remains the source of truth.
// poll(), request() and view() run on the Qt thread; busy() is also read by ROS callbacks.
class ControllerSwitcher {
 public:
  using List = controller_manager_msgs::srv::ListControllers;
  using Switch = controller_manager_msgs::srv::SwitchController;
  using Clock = std::chrono::steady_clock;
  struct View {
    bool enabled{false};
    std::string target;
    std::string button_label{"切换控制器"};
    std::string states;
    std::string message;
  };

  explicit ControllerSwitcher(const rclcpp::Node::SharedPtr& node) {
    const auto parameter = [&node](const char* key, const char* fallback) {
      return node->has_parameter(key) ? node->get_parameter(key).as_string() : std::string(fallback);
    };
    auto manager = parameter("controller_manager_name", "/controller_manager");
    while (manager.size() > 1 && manager.back() == '/') manager.pop_back();
    mpc_ = parameter("mpc_controller_name", "mpc_controller");
    jtc_ = parameter("trajectory_controller_name", "zephyr_arm_trajectory_controller");
    list_ = node->create_client<List>(manager + "/list_controllers");
    switch_ = node->create_client<Switch>(manager + "/switch_controller");
  }

  bool busy() const { return busy_.load(); }
  View view() const {
    View out;
    out.message = message_;
    const auto* mpc = find(mpc_);
    const auto* jtc = find(jtc_);
    out.states = "MPC: " + (mpc ? mpc->state : "未加载") +
                 " / JTC: " + (jtc ? jtc->state : "未加载");
    if (busy()) { out.message = message_; return out; }
    if (received_ == Clock::time_point{} || Clock::now() - received_ > std::chrono::seconds(2)) {
      out.message = "等待 controller_manager 的新鲜状态";
      return out;
    }
    if (!mpc || !jtc) { out.message = "需先加载并配置 MPC 和 JTC；按钮不会自动加载控制器"; return out; }
    if (mpc_ == jtc_ || mpc->type != "mpc_controller/mpc_controller" ||
        jtc->type != "joint_trajectory_controller/JointTrajectoryController") {
      out.message = "控制器名称或类型不匹配，请检查 RViz 参数";
      return out;
    }
    const auto configured = [](const auto& c) { return c.state == "active" || c.state == "inactive"; };
    if (!configured(*mpc) || !configured(*jtc) || (mpc->state == "active" && jtc->state == "active")) {
      out.message = "控制器状态不能安全切换，请检查 controller_manager";
      return out;
    }
    out.target = mpc->state == "active" ? jtc_ : mpc_;
    out.button_label = mpc->state == "active" ? "切回 JTC" : "切换到 MPC";
    const auto* target = find(out.target);
    for (const auto& c : controllers_) {
      if (c.state != "active" || c.name == mpc_ || c.name == jtc_) continue;
      for (const auto& interface : c.claimed_interfaces) {
        if (std::find(target->required_command_interfaces.begin(), target->required_command_interfaces.end(),
                      interface) != target->required_command_interfaces.end()) {
          out.message = "命令接口被 " + c.name + " 占用；不会自动停用其他控制器";
          return out;
        }
      }
    }
    out.enabled = switch_->service_is_ready();
    if (!out.enabled) out.message = "switch_controller 服务不可用";
    return out;
  }

  // Explicit user action, never invoked by discovery/status recovery. Recheck
  // the selected destination after the confirmation dialog, not just at display time.
  bool request(const std::string& target) {
    const auto current = view();
    if (!current.enabled || current.target != target) return false;
    auto request = std::make_shared<Switch::Request>();
    request->activate_controllers = {target};
    const auto& other = target == mpc_ ? jtc_ : mpc_;
    if (find(other)->state == "active") request->deactivate_controllers = {other};
    request->strictness = Switch::Request::STRICT;
    request->activate_asap = false;
    request->timeout.sec = 5;
    if (listing_) { list_->remove_pending_request(*listing_); listing_.reset(); }
    received_ = {};
    busy_ = true;
    expected_ = target;
    message_ = "正在切换到 " + target + "，暂不发送目标…";
    try {
      switching_ = switch_->async_send_request(request);
      switch_deadline_ = Clock::now() + std::chrono::seconds(10);
    } catch (const std::exception& e) {
      message_ = std::string("切换请求失败，请核对实际状态：") + e.what();
      expected_.clear();
      next_list_ = {};
    }
    return true;
  }

  void poll() {
    const auto now = Clock::now();
    if (switching_) {
      if (switching_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          if (!switching_->get()->ok) {
            message_ = "切换被拒绝或失败；请检查机器人日志，不会自动重试";
            expected_.clear();
          } else message_ = "切换服务已返回，等待核对实际控制器状态…";
        } catch (const std::exception& e) {
          message_ = std::string("切换响应异常，请核对实际状态：") + e.what();
          expected_.clear();
        }
        switching_.reset();
        next_list_ = {};
      } else if (now >= switch_deadline_) {
        switch_->remove_pending_request(*switching_);
        switching_.reset();
        expected_.clear();
        message_ = "切换响应超时，结果未知；等待状态核对，不会自动重试";
        next_list_ = {};
      } else return;
    }
    if (listing_) {
      if (listing_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
          controllers_ = listing_->get()->controller;
          received_ = now;
          if (busy()) {
            if (!expected_.empty()) {
              const auto* active = find(expected_);
              const auto* other = find(expected_ == mpc_ ? jtc_ : mpc_);
              message_ = active && other && active->state == "active" && other->state == "inactive" ?
                  "已切换到 " + expected_ + "；MPC 就绪后才允许发送目标" :
                  "实际状态与切换请求不一致，请检查机器人日志";
            }
            expected_.clear();
            busy_ = false;
          }
        } catch (const std::exception& e) {
          received_ = {};
          message_ = std::string("读取状态失败：") + e.what();
        }
        listing_.reset();
      } else if (now >= list_deadline_) {
        list_->remove_pending_request(*listing_);
        listing_.reset();
        received_ = {};
      }
    }
    if (!listing_ && now >= next_list_ && list_->service_is_ready()) {
      try {
        listing_ = list_->async_send_request(std::make_shared<List::Request>());
        list_deadline_ = now + std::chrono::seconds(2);
        next_list_ = now + std::chrono::seconds(1);
      } catch (const std::exception& e) {
        message_ = std::string("状态服务不可用：") + e.what();
        next_list_ = now + std::chrono::seconds(1);
      }
    }
  }

 private:
  const controller_manager_msgs::msg::ControllerState* find(const std::string& name) const {
    const auto it = std::find_if(controllers_.begin(), controllers_.end(),
                               [&name](const auto& c) { return c.name == name; });
    return it == controllers_.end() ? nullptr : &*it;
  }
  rclcpp::Client<List>::SharedPtr list_;
  rclcpp::Client<Switch>::SharedPtr switch_;
  std::optional<rclcpp::Client<List>::FutureAndRequestId> listing_;
  std::optional<rclcpp::Client<Switch>::FutureAndRequestId> switching_;
  std::vector<controller_manager_msgs::msg::ControllerState> controllers_;
  std::string mpc_, jtc_, expected_, message_;
  Clock::time_point received_{}, next_list_{}, list_deadline_{}, switch_deadline_{};
  std::atomic_bool busy_{false};
};
}  // namespace zephyr::mpc_rviz
