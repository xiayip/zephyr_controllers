// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <QComboBox>
#include <mutex>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/float_property.hpp>
namespace arms_rviz_control_plugin { class ErrorDashboard; }

namespace zephyr::mpc_rviz
{

class TcpTargetManager;

class target_coordinate_panel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit target_coordinate_panel(QWidget* parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onSendClicked();

private:
  void onTargetReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TcpTargetManager> marker_manager_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pub_;
  geometry_msgs::msg::PoseStamped latest_target_;
  bool have_target_{false};

  QLabel* frame_label_{nullptr};
  QLabel* current_label_{nullptr};
  QLabel* status_label_{nullptr};
  std::array<QDoubleSpinBox*, 3> xyz_inputs_{};
  QPushButton* send_button_{nullptr};
};

class tcp_pose_panel final : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit tcp_pose_panel(QWidget* parent=nullptr);
    void onInitialize() override;
private:
    void send();
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<TcpTargetManager> marker_manager_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr absolute_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr relative_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_;
    std::array<QDoubleSpinBox*,7> values_{};
    QComboBox* mode_;
    QLabel* status_;
    QPushButton* button_;
    std::string frame_;
    bool initialized_{false};
};

class tcp_error_display final : public rviz_common::Display {
    Q_OBJECT
public:
    tcp_error_display();
    ~tcp_error_display() override;
    void update(float wall_dt,float ros_dt) override;
    void reset() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;
protected:
    void onInitialize() override;
    void onDisable() override;
private Q_SLOTS:
    void updateScale();
private:
    struct Data {
        std::mutex mutex;
        geometry_msgs::msg::PoseStamped actual,target;
        std::chrono::steady_clock::time_point received{};
        bool have_actual{false},have_target{false};
    };
    std::shared_ptr<Data> data_{std::make_shared<Data>()};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr actual_,target_;
    // Reuse upstream rendering only; all subscriptions and error logic are native TCP.
    std::unique_ptr<arms_rviz_control_plugin::ErrorDashboard> dashboard_;
    rviz_common::properties::FloatProperty* scale_;
    std::array<double,4> thresholds_{5,20,1,5};
    float elapsed_{0};
};
}
