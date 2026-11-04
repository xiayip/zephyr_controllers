// SPDX-License-Identifier: Apache-2.0
// Zephyr adaptation of fiveages-sim/arms_ros2_control; see UPSTREAM.md.

#include "tcp_rviz.hpp"
#include <arms_rviz_control_plugin/error_dashboard.hpp>
#include <algorithm>
#include <unordered_map>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <cmath>
#include <rviz_common/display_context.hpp>
#include <QFormLayout>
#include <QGroupBox>
#include <QMetaObject>
#include <QString>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/render_system.hpp>

namespace zephyr::mpc_rviz {
namespace {
constexpr const char* error_threshold_keys[] = {"ErrorPositionGreenMm","ErrorPositionRedMm","ErrorOrientationGreenDeg","ErrorOrientationRedDeg"};
}

/** Marker server owned by the RViz panels, using RViz's existing node/executor. */
class TcpTargetManager final : public std::enable_shared_from_this<TcpTargetManager> {
public:
    static std::shared_ptr<TcpTargetManager> acquire(const rclcpp::Node::SharedPtr& node) {
        static std::mutex mutex;
        static std::unordered_map<rclcpp::Node*,std::weak_ptr<TcpTargetManager>> managers;
        std::lock_guard<std::mutex> lock(mutex);
        if (auto existing=managers[node.get()].lock()) return existing;
        auto manager=std::shared_ptr<TcpTargetManager>(new TcpTargetManager(node));
        manager->initialize(); managers[node.get()]=manager;
        return manager;
    }
private:
    explicit TcpTargetManager(rclcpp::Node::SharedPtr node)
        : node_(std::move(node)), tf_(node_->get_clock()), listener_(tf_,node_,false) {
        frame_=node_->has_parameter("marker_fixed_frame") ? node_->get_parameter("marker_fixed_frame").as_string() : "";
        tcp_=node_->has_parameter("tcp_ee_frame") ? node_->get_parameter("tcp_ee_frame").as_string() : "omni_picker_tcp";
        server_=std::make_unique<interactive_markers::InteractiveMarkerServer>("tcp_target_manager",node_);
        immediate_=node_->create_publisher<geometry_msgs::msg::Pose>("tcp_target",1);
        stamped_=node_->create_publisher<geometry_msgs::msg::PoseStamped>("tcp_target/stamped",1);
    }
    void initialize() {
        const std::weak_ptr<TcpTargetManager> weak=shared_from_this();
        menu_.insert("发送目标",[weak](auto){ if(auto self=weak.lock()) self->send(true); });
        menu_.insert("切换连续跟踪 / 手动发送",[weak](auto){ if(auto self=weak.lock()) { self->continuous_=!self->continuous_; self->dragging_=false; } });
        target_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>("tcp_current_target",rclcpp::QoS(1).transient_local(),
            [weak](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose){
                auto self=weak.lock(); if (!self) return;
                self->reference_frame_=pose->header.frame_id;
                self->latest_target_=*pose; self->have_target_=true;
                if (self->dragging_ || std::chrono::duration<double>(std::chrono::steady_clock::now()-self->last_command_).count()<0.3) return;
                self->update(*pose);
            });
        current_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>("tcp_current_pose",rclcpp::SensorDataQoS().keep_last(1),
            [weak](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose){
                auto self=weak.lock(); if (!self) return;
                if(self->reference_frame_.empty()) self->reference_frame_=pose->header.frame_id;
                if(!self->initialized_||!self->active_) self->update(*pose);
            });
        status_sub_=node_->create_subscription<std_msgs::msg::Int32>("/fsm_state",rclcpp::QoS(1).transient_local(),
            [weak](std_msgs::msg::Int32::ConstSharedPtr status){
                auto self=weak.lock(); if (!self) return;
                const bool enabled=status->data==3;
                if(enabled==self->active_) return;
                self->active_=enabled; self->dragging_=false;
                // Topic delivery order is not guaranteed: an actual pose
                // received before status must not replace the cached goal.
                if(enabled&&self->have_target_) self->update(self->latest_target_);
                if(self->initialized_) self->createMarker();
            });
        timer_=node_->create_wall_timer(std::chrono::milliseconds(500),[weak]{
            auto self=weak.lock(); if(!self||self->initialized_||self->frame_.empty()) return;
            try {
                const auto transform=self->tf_.lookupTransform(self->frame_,self->tcp_,tf2::TimePointZero);
                geometry_msgs::msg::PoseStamped pose; pose.header.frame_id=self->frame_;
                pose.pose.position.x=transform.transform.translation.x;
                pose.pose.position.y=transform.transform.translation.y;
                pose.pose.position.z=transform.transform.translation.z;
                pose.pose.orientation=transform.transform.rotation; self->update(pose);
            } catch(const tf2::TransformException&) {}
        });
    }
    void update(const geometry_msgs::msg::PoseStamped& source) {
        if(source.header.frame_id.empty()) return;
        if(frame_.empty()) frame_=source.header.frame_id;
        try {
            geometry_msgs::msg::PoseStamped pose=source;
            if(source.header.frame_id!=frame_)
                tf2::doTransform(source,pose,tf_.lookupTransform(frame_,source.header.frame_id,tf2::TimePointZero));
            pose_=pose.pose;
            if(!initialized_) { initialized_=true; createMarker(); }
            else { server_->setPose(tcp_,pose_); server_->applyChanges(); }
        } catch(const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,"TCP marker TF unavailable: %s",e.what());
        }
    }
    void createMarker() {
        using Control=visualization_msgs::msg::InteractiveMarkerControl;
        visualization_msgs::msg::InteractiveMarker marker;
        marker.name=tcp_; marker.description="TCP target"; marker.header.frame_id=frame_; marker.pose=pose_; marker.scale=0.20;
        Control center; center.always_visible=true; center.interaction_mode=active_?Control::MOVE_ROTATE_3D:Control::NONE;
        visualization_msgs::msg::Marker sphere; sphere.type=visualization_msgs::msg::Marker::SPHERE;
        sphere.pose.orientation.w=1; sphere.scale.x=sphere.scale.y=sphere.scale.z=0.035;
        sphere.color.r=0.2; sphere.color.g=0.6; sphere.color.b=1.0; sphere.color.a=0.8;
        center.markers.push_back(sphere); marker.controls.push_back(center);
        if(active_) for(int axis=0;axis<3;++axis) {
            Control control; control.orientation.w=std::sqrt(0.5);
            if(axis==0) control.orientation.x=std::sqrt(0.5);
            if(axis==1) control.orientation.z=std::sqrt(0.5);
            if(axis==2) control.orientation.y=std::sqrt(0.5);
            control.name="rotate_"+std::to_string(axis); control.interaction_mode=Control::ROTATE_AXIS; marker.controls.push_back(control);
            control.name="move_"+std::to_string(axis); control.interaction_mode=Control::MOVE_AXIS; marker.controls.push_back(control);
        }
        const std::weak_ptr<TcpTargetManager> weak=shared_from_this();
        server_->insert(marker,[weak](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback){
            auto self=weak.lock(); if(!self||!self->active_) return;
            using Feedback=visualization_msgs::msg::InteractiveMarkerFeedback;
            if(feedback->event_type==Feedback::MOUSE_DOWN) self->dragging_=true;
            if(feedback->event_type==Feedback::POSE_UPDATE||feedback->event_type==Feedback::MOUSE_UP) {
                self->pose_=feedback->pose;
                if(self->continuous_) self->send(false);
            }
            if(feedback->event_type==Feedback::MOUSE_UP) self->dragging_=false;
        });
        menu_.apply(*server_,tcp_); server_->applyChanges();
    }
    void send(bool interpolate) {
        if(!active_||!initialized_) return;
        geometry_msgs::msg::PoseStamped pose; pose.header.frame_id=frame_; pose.header.stamp=node_->now(); pose.pose=pose_;
        if(interpolate) stamped_->publish(pose);
        else {
            // Headerless continuous commands must be expressed in MPC's frame,
            // even if the marker was configured in another visualization frame.
            if(reference_frame_.empty()) return;
            try {
                if(reference_frame_!=frame_) {
                    geometry_msgs::msg::PoseStamped converted;
                    tf2::doTransform(pose,converted,tf_.lookupTransform(reference_frame_,frame_,tf2::TimePointZero)); pose=converted;
                }
                immediate_->publish(pose.pose);
            } catch(const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,"TCP marker command rejected: %s",e.what()); return;
            }
        }
        last_command_=std::chrono::steady_clock::now();
    }
    rclcpp::Node::SharedPtr node_;
    std::string frame_,tcp_,reference_frame_;
    bool initialized_{false},active_{false},dragging_{false},continuous_{true},have_target_{false};
    geometry_msgs::msg::Pose pose_;
    geometry_msgs::msg::PoseStamped latest_target_;
    std::chrono::steady_clock::time_point last_command_{};
    tf2_ros::Buffer tf_; tf2_ros::TransformListener listener_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler menu_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr immediate_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr stamped_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_,current_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

target_coordinate_panel::target_coordinate_panel(QWidget* parent)
  : Panel(parent)
{
  auto* layout = new QVBoxLayout(this);
  auto* current_group = new QGroupBox(QStringLiteral("当前交互目标"), this);
  auto* current_layout = new QVBoxLayout(current_group);
  frame_label_ = new QLabel(QStringLiteral("坐标系：等待目标话题…"), current_group);
  current_label_ = new QLabel(QStringLiteral("X: —    Y: —    Z: —"), current_group);
  current_label_->setWordWrap(true);
  current_layout->addWidget(frame_label_);
  current_layout->addWidget(current_label_);
  layout->addWidget(current_group);

  auto* input_group = new QGroupBox(QStringLiteral("手动输入目标坐标（米）"), this);
  auto* form = new QFormLayout(input_group);
  const std::array<QString, 3> names{{QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")}};
  for (size_t i = 0; i < xyz_inputs_.size(); ++i)
  {
    xyz_inputs_[i] = new QDoubleSpinBox(input_group);
    xyz_inputs_[i]->setDecimals(4);
    xyz_inputs_[i]->setRange(-20.0, 20.0);
    xyz_inputs_[i]->setSingleStep(0.01);
    xyz_inputs_[i]->setSuffix(QStringLiteral(" m"));
    form->addRow(names[i], xyz_inputs_[i]);
  }
  layout->addWidget(input_group);

  send_button_ = new QPushButton(QStringLiteral("发送目标坐标给 MPC"), this);
  send_button_->setEnabled(false);
  layout->addWidget(send_button_);
  status_label_ = new QLabel(QStringLiteral("等待当前目标位姿…"), this);
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  layout->addStretch();

  connect(send_button_, &QPushButton::clicked, this, &target_coordinate_panel::onSendClicked);
}

void target_coordinate_panel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  marker_manager_ = TcpTargetManager::acquire(node_);
  // Public single-TCP interface.
  target_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tcp_current_target", rclcpp::QoS(1).transient_local(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        QMetaObject::invokeMethod(this, [this, msg]() { onTargetReceived(msg); },
                                  Qt::QueuedConnection);
      });
  command_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/tcp_target/stamped", rclcpp::QoS(10));
}

void target_coordinate_panel::onTargetReceived(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  latest_target_ = *msg;
  if (!have_target_)
  {
    xyz_inputs_[0]->setValue(msg->pose.position.x);
    xyz_inputs_[1]->setValue(msg->pose.position.y);
    xyz_inputs_[2]->setValue(msg->pose.position.z);
    have_target_ = true;
  }
  frame_label_->setText(QStringLiteral("坐标系：%1").arg(QString::fromStdString(msg->header.frame_id)));
  current_label_->setText(QStringLiteral("X: %1 m    Y: %2 m    Z: %3 m")
      .arg(msg->pose.position.x, 0, 'f', 4)
      .arg(msg->pose.position.y, 0, 'f', 4)
      .arg(msg->pose.position.z, 0, 'f', 4));
  send_button_->setEnabled(true);
  status_label_->setText(QStringLiteral("输入坐标将沿用当前目标的坐标系和末端姿态。"));
}

void target_coordinate_panel::onSendClicked()
{
  if (!node_ || !command_pub_ || !have_target_)
  {
    status_label_->setText(QStringLiteral("没有当前目标位姿，无法确定坐标系和末端姿态。"));
    return;
  }

  auto command = latest_target_;
  command.header.stamp = node_->get_clock()->now();
  command.pose.position.x = xyz_inputs_[0]->value();
  command.pose.position.y = xyz_inputs_[1]->value();
  command.pose.position.z = xyz_inputs_[2]->value();
  command_pub_->publish(command);

  status_label_->setText(QStringLiteral("已发送到 MPC；目标坐标系：%1。")
      .arg(QString::fromStdString(command.header.frame_id)));
  RCLCPP_INFO(node_->get_logger(), "Published manual target: frame=%s xyz=(%.4f, %.4f, %.4f)",
      command.header.frame_id.c_str(),
      command.pose.position.x, command.pose.position.y, command.pose.position.z);
}

tcp_pose_panel::tcp_pose_panel(QWidget* parent):Panel(parent) {
    auto* layout=new QVBoxLayout(this); auto* form=new QFormLayout;
    const char* names[]={"TCP x (m)","TCP y (m)","TCP z (m)","TCP qx","TCP qy","TCP qz","TCP qw"};
    for (size_t i=0;i<values_.size();++i) {
        values_[i]=new QDoubleSpinBox(this); values_[i]->setDecimals(4); values_[i]->setSingleStep(i<3?0.01:0.005);
        values_[i]->setRange(i<3?-20.0:-1.0,i<3?20.0:1.0); form->addRow(QString::fromLatin1(names[i]),values_[i]);
    }
    values_[6]->setValue(1.0); layout->addLayout(form);
    mode_=new QComboBox(this); mode_->addItems({QStringLiteral("绝对位姿"),QStringLiteral("相对位姿（参考坐标系）")}); layout->addWidget(mode_);
    connect(mode_,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[this](int mode){
        if (mode==1) { for (auto* field:values_) field->setValue(0); values_[6]->setValue(1); }
        else initialized_=false;
    });
    button_=new QPushButton(QStringLiteral("发送末端位姿"),this); button_->setEnabled(false); layout->addWidget(button_);
    status_=new QLabel(QStringLiteral("等待 TCP 目标…"),this); status_->setWordWrap(true); layout->addWidget(status_); layout->addStretch();
    connect(button_,&QPushButton::clicked,this,[this]{send();});
}
void tcp_pose_panel::onInitialize() {
    node_=getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    marker_manager_=TcpTargetManager::acquire(node_);
    absolute_=node_->create_publisher<geometry_msgs::msg::PoseStamped>("/tcp_target/stamped",10);
    relative_=node_->create_publisher<geometry_msgs::msg::TwistStamped>("/tcp_target/relative",10);
    target_=node_->create_subscription<geometry_msgs::msg::PoseStamped>("/tcp_current_target",rclcpp::QoS(1).transient_local(),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr pose){
            QMetaObject::invokeMethod(this,[this,pose]{
                frame_=pose->header.frame_id;
                if (!initialized_&&mode_->currentIndex()==0) {
                    const auto& p=pose->pose;
                    const double v[]={p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};
                    for(size_t i=0;i<values_.size();++i) { values_[i]->setValue(v[i]); }
                    initialized_=true;
                }
                button_->setEnabled(true); status_->setText(QStringLiteral("坐标系：%1").arg(QString::fromStdString(frame_)));
            },Qt::QueuedConnection);
        });
}
void tcp_pose_panel::send() {
    double qx=values_[3]->value(),qy=values_[4]->value(),qz=values_[5]->value(),qw=values_[6]->value();
    const double norm=std::sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    if(norm<1e-9||frame_.empty()) { status_->setText(QStringLiteral("目标四元数或坐标系无效")); return; }
    qx/=norm;qy/=norm;qz/=norm;qw/=norm;
    if(mode_->currentIndex()==0) {
        geometry_msgs::msg::PoseStamped p; p.header.frame_id=frame_; p.header.stamp=node_->now();
        p.pose.position.x=values_[0]->value(); p.pose.position.y=values_[1]->value(); p.pose.position.z=values_[2]->value();
        p.pose.orientation.x=qx;p.pose.orientation.y=qy;p.pose.orientation.z=qz;p.pose.orientation.w=qw; absolute_->publish(p);
    } else {
        geometry_msgs::msg::TwistStamped p; p.header.frame_id=frame_; p.header.stamp=node_->now();
        p.twist.linear.x=values_[0]->value();p.twist.linear.y=values_[1]->value();p.twist.linear.z=values_[2]->value();
        // q and -q are the same rotation; select the shortest rotation vector.
        if(qw<0) { qx=-qx;qy=-qy;qz=-qz;qw=-qw; }
        const double sine=std::sqrt(qx*qx+qy*qy+qz*qz);
        const double scale=sine>1e-9?2*std::atan2(sine,qw)/sine:2.0;
        p.twist.angular.x=scale*qx;p.twist.angular.y=scale*qy;p.twist.angular.z=scale*qz;relative_->publish(p);
    }
    status_->setText(QStringLiteral("TCP 位姿已发送"));
}

tcp_error_display::tcp_error_display() {
    scale_=new rviz_common::properties::FloatProperty("Scale",0.75f,"TCP tracking error overlay size",this,SLOT(updateScale()));
    scale_->setMin(0.5f);scale_->setMax(2.0f);
}
tcp_error_display::~tcp_error_display() { actual_.reset();target_.reset();dashboard_.reset(); }
void tcp_error_display::onInitialize() {
    rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
    auto node=context_->getRosNodeAbstraction().lock()->get_raw_node();
    actual_=node->create_subscription<geometry_msgs::msg::PoseStamped>("tcp_current_pose",rclcpp::SensorDataQoS().keep_last(1),
        [data=data_](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){std::lock_guard<std::mutex> lock(data->mutex);data->actual=*p;data->received=std::chrono::steady_clock::now();data->have_actual=true;});
    target_=node->create_subscription<geometry_msgs::msg::PoseStamped>("tcp_current_target",rclcpp::QoS(1).transient_local(),
        [data=data_](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){std::lock_guard<std::mutex> lock(data->mutex);data->target=*p;data->have_target=true;});
}
void tcp_error_display::updateScale() { if(dashboard_)dashboard_->setScale(scale_->getFloat()); }
void tcp_error_display::onDisable() { if(dashboard_)dashboard_->hide(); }
void tcp_error_display::reset() { Display::reset();if(dashboard_)dashboard_->clearHistory();std::lock_guard<std::mutex> lock(data_->mutex);data_->have_actual=data_->have_target=false; }
void tcp_error_display::update(float wall_dt,float) {
    if(!isEnabled())return;
    if(!dashboard_) { dashboard_=std::make_unique<arms_rviz_control_plugin::ErrorDashboard>();dashboard_->setActive({true,false,false,false});dashboard_->setScale(scale_->getFloat());dashboard_->show();return; }
    dashboard_->show();elapsed_+=wall_dt;if(elapsed_<0.1f)return;elapsed_=0;dashboard_->refreshLayout();
    std::lock_guard<std::mutex> lock(data_->mutex);
    bool valid=false;
    double mm=0,degrees=0;
    QString status=QStringLiteral("等待位姿");
    if (!data_->have_actual || !data_->have_target) {
        status=QStringLiteral("等待位姿");
    } else if (std::chrono::duration<double>(std::chrono::steady_clock::now()-data_->received).count()>0.5) {
        status=QStringLiteral("当前位姿已超时");
    } else if (data_->actual.header.frame_id.empty() || data_->actual.header.frame_id!=data_->target.header.frame_id) {
        status=QStringLiteral("参考坐标系不一致");
    } else {
        const auto& a=data_->actual.pose; const auto& b=data_->target.pose;
        const auto& qa=a.orientation; const auto& qb=b.orientation;
        const double na=std::sqrt(qa.x*qa.x+qa.y*qa.y+qa.z*qa.z+qa.w*qa.w);
        const double nb=std::sqrt(qb.x*qb.x+qb.y*qb.y+qb.z*qb.z+qb.w*qb.w);
        mm=1000*std::hypot(a.position.x-b.position.x,std::hypot(a.position.y-b.position.y,a.position.z-b.position.z));
        if (std::isfinite(mm)&&std::isfinite(na)&&std::isfinite(nb)&&na>1e-9&&nb>1e-9) {
            const double dot=(qa.x*qb.x+qa.y*qb.y+qa.z*qb.z+qa.w*qb.w)/(na*nb);
            degrees=2*std::acos(std::clamp(std::abs(dot),0.0,1.0))*180/std::acos(-1.0);
            valid=true;
        } else status=QStringLiteral("位姿数据无效");
    }
    dashboard_->setRow(0,mm,degrees,valid,status,thresholds_);
}
void tcp_error_display::load(const rviz_common::Config& config) {
    Display::load(config);
    for(size_t i=0;i<4;i+=2) { float a,b;if(config.mapGetFloat(error_threshold_keys[i],&a)&&config.mapGetFloat(error_threshold_keys[i+1],&b)&&a>0&&b>a) {thresholds_[i]=a;thresholds_[i+1]=b;} }
}
void tcp_error_display::save(rviz_common::Config config) const {
    Display::save(config);
    for(size_t i=0;i<4;++i)config.mapSetValue(error_threshold_keys[i],thresholds_[i]);
}
}

PLUGINLIB_EXPORT_CLASS(zephyr::mpc_rviz::target_coordinate_panel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(zephyr::mpc_rviz::tcp_pose_panel,rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(zephyr::mpc_rviz::tcp_error_display,rviz_common::Display)
