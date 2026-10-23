#include "teleop_joint_controller/teleop_joint_controller.hpp"

#include <limits>
#include <iterator>
#include <urdf/model.h>
#include <set>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace teleop_joint_controller {
using CallbackReturn = controller_interface::CallbackReturn;
using Return = controller_interface::return_type;
int64_t TeleopJointController::steady_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
CallbackReturn TeleopJointController::on_init() {
  try {
    rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
    auto node = get_node();
    node->declare_parameter("joints", std::vector<std::string>{}, fixed);
    node->declare_parameter("downstream_controller", std::string{}, fixed);
    node->declare_parameter("servo_status_topic", std::string("/servo_node/status"), fixed);
    node->declare_parameter("reference_response_time", 0.01, fixed);
    node->declare_parameter("max_velocity", std::vector<double>{}, fixed);
    node->declare_parameter("max_acceleration", std::vector<double>{}, fixed);
    node->declare_parameter("command_timeout", command_timeout_, fixed);
    node->declare_parameter("startup_timeout", startup_timeout_, fixed);
    node->declare_parameter("max_update_period", max_period_, fixed);
    node->declare_parameter("max_target_step", max_target_step_, fixed);
    node->declare_parameter("max_tracking_error", max_tracking_error_, fixed);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
CallbackReturn TeleopJointController::on_configure(const rclcpp_lifecycle::State&) {
  try {
    auto node = get_node();
    joints_ = node->get_parameter("joints").as_string_array();
    downstream_ = node->get_parameter("downstream_controller").as_string();
    const auto vmax = node->get_parameter("max_velocity").as_double_array();
    const auto amax = node->get_parameter("max_acceleration").as_double_array();
    if (joints_.empty() || downstream_.empty() || vmax.size() != joints_.size() ||
        std::set<std::string>(joints_.begin(), joints_.end()).size() != joints_.size())
      throw std::invalid_argument("unique joints, downstream controller and per-joint limits required");
    filter_.configure(vmax, amax, node->get_parameter("reference_response_time").as_double());
    command_timeout_ = node->get_parameter("command_timeout").as_double();
    startup_timeout_ = node->get_parameter("startup_timeout").as_double();
    max_period_ = node->get_parameter("max_update_period").as_double();
    max_target_step_ = node->get_parameter("max_target_step").as_double();
    max_tracking_error_ = node->get_parameter("max_tracking_error").as_double();
    for (double v : {command_timeout_, startup_timeout_, max_period_, max_target_step_, max_tracking_error_})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("guard thresholds must be positive");
    lower_.clear(); upper_.clear();
    urdf::Model model;
    if (!model.initString(get_robot_description())) throw std::invalid_argument("invalid robot description");
    const auto& limits = get_hard_joint_limits();
    for (const auto& joint : joints_) {
      const auto it = limits.find(joint);
      const auto model_joint = model.getJoint(joint);
      if (it != limits.end() && it->second.has_position_limits) {
        lower_.push_back(it->second.min_position); upper_.push_back(it->second.max_position);
      } else if (model_joint && model_joint->limits && model_joint->type != urdf::Joint::CONTINUOUS) {
        lower_.push_back(model_joint->limits->lower); upper_.push_back(model_joint->limits->upper);
      } else {
        throw std::invalid_argument("URDF position limits required for " + joint);
      }
    }
    actual_.resize(joints_.size()); target_.resize(joints_.size()); previous_target_.resize(joints_.size());
    sampled_target_.resize(joints_.size());
    subscription_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "~/joint_trajectory", rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { receive(std::move(msg)); });
    servo_status_subscription_ = node->create_subscription<moveit_msgs::msg::ServoStatus>(
        node->get_parameter("servo_status_topic").as_string(),
        rclcpp::QoS(10).best_effort().durability_volatile(),
        [this](moveit_msgs::msg::ServoStatus::SharedPtr msg) { receive_servo_status(std::move(msg)); });
    state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<State>>(
        node->create_publisher<State>("~/controller_state", rclcpp::QoS(10)));
    auto& state = state_pub_->msg_; state.joint_names = joints_;
    for (auto* point : {&state.reference, &state.feedback, &state.error, &state.output}) {
      point->positions.resize(joints_.size());
      if (point == &state.reference || point == &state.output) point->velocities.resize(joints_.size());
    }
    status_pub_ = std::make_shared<realtime_tools::RealtimePublisher<Diagnostics>>(
        node->create_publisher<Diagnostics>("~/status", rclcpp::QoS(1).transient_local()));
    auto& diag = status_pub_->msg_; diag.status.resize(1);
    diag.status[0].name = node->get_name(); diag.status[0].hardware_id = "teleop_joint_controller";
    const std::vector<std::string> keys = {"state", "joint", "observed", "reference", "limit",
        "joint_position", "joint_reference"};
    diag.status[0].values.resize(keys.size());
    for (size_t i = 0; i < keys.size(); ++i) {
      diag.status[0].values[i].key = keys[i];
      diag.status[0].values[i].value.reserve(128);
    }
    diag.status[0].message.reserve(100); diag.status[0].values[0].value.reserve(16);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Teleop controller configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration TeleopJointController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto& j : joints_) for (const auto* field : {"position", "velocity"})
    c.names.push_back(downstream_ + "/" + j + "/" + field);
  return c;
}
controller_interface::InterfaceConfiguration TeleopJointController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration c{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto& j : joints_) c.names.push_back(j + "/position");
  return c;
}
bool TeleopJointController::read_positions() {
  for (size_t i = 0; i < joints_.size(); ++i) {
    auto v = state_interfaces_[pos_states_[i]].get_optional();
    if (!v || !std::isfinite(*v) || *v < lower_[i] || *v > upper_[i]) return false;
    actual_[i] = *v;
  }
  return true;
}
CallbackReturn TeleopJointController::on_activate(const rclcpp_lifecycle::State&) {
  active_session_.store(0);
  pos_commands_.clear(); vel_commands_.clear(); pos_states_.clear();
  for (const auto& j : joints_) {
    const auto find = [](const auto& interfaces, const std::string& name) {
      for (size_t k = 0; k < interfaces.size(); ++k) if (interfaces[k].get_name() == name) return k;
      return interfaces.size();
    };
    const auto p = find(command_interfaces_, downstream_ + "/" + j + "/position");
    const auto v = find(command_interfaces_, downstream_ + "/" + j + "/velocity");
    const auto s = find(state_interfaces_, j + "/position");
    if (p == command_interfaces_.size() || v == command_interfaces_.size() || s == state_interfaces_.size())
      return CallbackReturn::ERROR;
    pos_commands_.push_back(p); vel_commands_.push_back(v); pos_states_.push_back(s);
  }
  if (!read_positions()) return CallbackReturn::ERROR;
  filter_.reset(actual_); target_ = actual_; previous_target_ = actual_;
  seen_command_ = faulted_ = servo_hold_ = timeout_hold_ = false; reason_ = "waiting for fresh Servo positions";
  fault_joint_ = -1; fault_observed_ = fault_reference_ = fault_limit_ = fault_actual_ = fault_output_ = NAN;
  activated_ns_ = steady_now(); last_received_ns_ = last_source_ns_ = 0; status_elapsed_ = 1.0;
  ++current_session_;
  if (!write()) return CallbackReturn::ERROR;
  active_session_.store(current_session_);
  return CallbackReturn::SUCCESS;
}
CallbackReturn TeleopJointController::on_deactivate(const rclcpp_lifecycle::State&) {
  if (!active_session_.exchange(0)) return CallbackReturn::SUCCESS;
  filter_.stop(); reason_ = "inactive";
  return write() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}
void TeleopJointController::receive(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
  const auto session = active_session_.load(); if (!session) return;
  // Keep the rolling position window so update() can sample at its own clock.
  // Velocities are intentionally ignored: compressed Servo intervals must not
  // become spline derivatives or be forwarded to the motor velocity reference.
  bool valid = msg->joint_names == joints_ && !msg->points.empty() &&
    msg->header.stamp.sec >= 0 && msg->header.stamp.nanosec < 1000000000u;
  Command command;
  command.session = session;
  if (valid) {
    const int64_t header_ns = int64_t(msg->header.stamp.sec) * 1000000000 + msg->header.stamp.nanosec;
    command.points.reserve(msg->points.size());
    for (const auto& point : msg->points) {
      if (point.positions.size() != joints_.size() || point.time_from_start.sec < 0 ||
          point.time_from_start.nanosec >= 1000000000u) { valid = false; break; }
      const int64_t stamp_ns = header_ns + int64_t(point.time_from_start.sec) * 1000000000 +
        point.time_from_start.nanosec;
      if (stamp_ns <= 0 || (!command.points.empty() && stamp_ns <= command.points.back().stamp_ns)) {
        valid = false; break;
      }
      for (size_t i = 0; i < joints_.size(); ++i)
        valid = valid && std::isfinite(point.positions[i]) &&
          point.positions[i] >= lower_[i] && point.positions[i] <= upper_[i];
      if (!valid) break;
      command.points.push_back(Waypoint{point.positions, stamp_ns});
    }
    if (valid) {
      command.source_ns = command.points.back().stamp_ns;
      const double age = (get_node()->now().nanoseconds() - command.source_ns) * 1e-9;
      valid = age <= command_timeout_ && age >= -max_period_;
    }
  }
  if (!valid) { invalid_session_.store(session); return; }
  if (callback_session_ != session) { callback_session_ = session; callback_source_ns_ = 0; }
  if (command.source_ns <= callback_source_ns_) return;
  callback_source_ns_ = command.source_ns;
  command.received_ns = steady_now();
  commands_.writeFromNonRT(command);
}
void TeleopJointController::receive_servo_status(const moveit_msgs::msg::ServoStatus::SharedPtr msg) {
  const auto session = active_session_.load(); if (!session) return;
  using Status = moveit_msgs::msg::ServoStatus;
  if (callback_servo_hold_.session != session) callback_servo_hold_ = {session, 0, false, 0};
  callback_servo_hold_.active = msg->code == Status::JOINT_BOUND || msg->code == Status::HALT_FOR_SINGULARITY;
  if (callback_servo_hold_.active) {
    callback_servo_hold_.last_hold_ns = steady_now();
    callback_servo_hold_.code = msg->code;
  }
  servo_hold_state_.writeFromNonRT(callback_servo_hold_);
  switch (msg->code) {
    case Status::HALT_FOR_COLLISION:
    case Status::INVALID:
      // Preserve the first stop even if a later status returns to normal before
      // the next update. A stop may arrive before Servo finishes its rolling
      // trajectory; don't keep following that window until a command timeout.
      if (servo_stop_session_.load() != session) {
        servo_stop_code_.store(msg->code);
        servo_stop_session_.store(session);
      }
      break;
    case Status::HALT_FOR_SINGULARITY:
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Servo singularity hold: teleop remains active; resume when Servo clears the singularity");
      break;
    case Status::JOINT_BOUND:
      // Publish the hold before logging; update() never waits for console I/O.
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Servo joint bound: holding without exiting teleop; move back within joint limits");
      break;
    case Status::DECELERATE_FOR_APPROACHING_SINGULARITY:
    case Status::DECELERATE_FOR_LEAVING_SINGULARITY:
    case Status::DECELERATE_FOR_COLLISION:
      // Upstream can warn at every 200 Hz cycle. Z2 hides that one logger's
      // repeated warnings and reports its actual status here at most every 2 s.
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "Servo slowdown (code %d): %s", int(msg->code), msg->message.c_str());
      break;
    default:
      break;
  }
}
void TeleopJointController::fault(const char* reason, int joint, double observed, double reference, double limit) {
  if (!faulted_) {
    reason_ = reason; status_elapsed_ = 1.0;
    fault_joint_ = joint; fault_observed_ = observed; fault_reference_ = reference; fault_limit_ = limit;
    if (joint >= 0) { fault_actual_ = actual_[joint]; fault_output_ = filter_.position[joint]; }
  }
  faulted_ = true; filter_.stop();
}
bool TeleopJointController::write() {
  for (size_t i = 0; i < joints_.size(); ++i)
    if (!command_interfaces_[pos_commands_[i]].set_value(filter_.position[i]) ||
        !command_interfaces_[vel_commands_[i]].set_value(filter_.velocity[i])) return false;
  return true;
}
Return TeleopJointController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto now = steady_now(); const double dt = period.seconds();
  if (!active_session_.load()) return Return::OK;
  if (servo_stop_session_.load() == current_session_) {
    using Status = moveit_msgs::msg::ServoStatus;
    switch (servo_stop_code_.load()) {
      case Status::HALT_FOR_COLLISION: fault("Servo halted for collision; reactivation required"); break;
      default: fault("Servo rejected command; reactivation required"); break;
    }
  }
  if (!read_positions()) fault("invalid hardware position feedback");
  if (!std::isfinite(dt) || dt <= 0 || dt > max_period_) fault("control update period outside allowed range", -1, dt, 0.0, max_period_);
  if (invalid_session_.load() == current_session_) fault("invalid or stale Servo positions; reactivation required");
  // Check the old receipt before accepting a queued packet, so a packet arriving
  // just ahead of update() still has to pass the recovery proximity check.
  if (!faulted_ && seen_command_ && (now - last_received_ns_) * 1e-9 > command_timeout_) {
    if (!timeout_hold_) status_elapsed_ = 1.0;
    timeout_hold_ = true;
    filter_.stop();
    reason_ = "Servo command timeout; waiting for fresh positions near current joints";
  }
  const auto* cmd = commands_.readFromRT();
  bool accept_command = !faulted_ && cmd && cmd->session == current_session_ &&
      cmd->received_ns >= activated_ns_ && cmd->received_ns > last_received_ns_ &&
      cmd->source_ns > last_source_ns_;
  if (accept_command && timeout_hold_) {
    accept_command = (now - cmd->received_ns) * 1e-9 <= command_timeout_ &&
        (time.nanoseconds() - cmd->source_ns) * 1e-9 <= command_timeout_;
    // Validate the entire rolling window, not just its endpoint. Resume the
    // bounded filter from its held position with zero velocity, without a jump.
    const double recovery_limit = std::min(max_target_step_, max_tracking_error_);
    for (const auto& point : cmd->points) for (size_t i = 0; i < joints_.size(); ++i)
      if (std::abs(point.positions[i] - actual_[i]) > recovery_limit ||
          std::abs(point.positions[i] - filter_.position[i]) > recovery_limit)
        accept_command = false;
    if (accept_command) {
      timeout_hold_ = false;
      previous_target_ = target_ = filter_.position;
      status_elapsed_ = 1.0;
    }
  }
  if (accept_command) {
    for (size_t i = 0; i < joints_.size(); ++i)
      if (std::abs(cmd->points.back().positions[i] - previous_target_[i]) > max_target_step_)
        fault("Servo position discontinuity; reactivation required", i,
            cmd->points.back().positions[i], previous_target_[i], max_target_step_);
    if (!faulted_) {
      previous_target_ = cmd->points.back().positions;
      last_received_ns_ = cmd->received_ns; last_source_ns_ = cmd->source_ns;
      seen_command_ = true; reason_ = "tracking Servo positions";
    }
  }
  if (!faulted_ && !seen_command_ && (now - activated_ns_) * 1e-9 > startup_timeout_)
    fault("Servo command timeout; reactivation required", -1,
        (now - activated_ns_) * 1e-9, 0.0, startup_timeout_);
  if (!faulted_) for (size_t i = 0; i < joints_.size(); ++i)
    if (std::abs(filter_.position[i] - actual_[i]) > max_tracking_error_)
      fault("joint tracking error; reactivation required", i, actual_[i], filter_.position[i], max_tracking_error_);
  const auto* bound = servo_hold_state_.readFromRT();
  const bool hold = bound && bound->session == current_session_ && bound->last_hold_ns > 0 &&
      (bound->active || !seen_command_ || last_received_ns_ <= bound->last_hold_ns);
  if (hold != servo_hold_) status_elapsed_ = 1.0;
  servo_hold_ = hold;
  if (!faulted_ && servo_hold_ && !timeout_hold_) {
    filter_.stop();
    reason_ = bound->code == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY ?
        "singularity hold; awaiting clear status and fresh Servo positions" :
        "joint bound hold; awaiting clear status and fresh Servo positions";
  }
  if (!faulted_ && seen_command_ && !timeout_hold_) {
    if (!servo_hold_) reason_ = "tracking Servo positions";
    // Interpolate positions only, at the actual controller time. Outside the
    // received window hold its endpoint; never extrapolate a missing command.
    // The bounded filter remains responsible for position/velocity consistency.
    const auto& points = cmd->points;
    const int64_t stamp_ns = time.nanoseconds();
    const auto next = std::upper_bound(points.begin(), points.end(), stamp_ns,
        [](int64_t stamp, const Waypoint& point) { return stamp < point.stamp_ns; });
    if (next == points.begin()) sampled_target_ = points.front().positions;
    else if (next == points.end()) sampled_target_ = points.back().positions;
    else {
      const auto& before = *std::prev(next);
      const double fraction = double(stamp_ns - before.stamp_ns) / double(next->stamp_ns - before.stamp_ns);
      for (size_t i = 0; i < joints_.size(); ++i)
        sampled_target_[i] = before.positions[i] + fraction * (next->positions[i] - before.positions[i]);
    }
    for (size_t i = 0; i < joints_.size(); ++i)
      if (std::abs(sampled_target_[i] - target_[i]) > max_target_step_)
        fault("sampled Servo position discontinuity; reactivation required", i,
            sampled_target_[i], target_[i], max_target_step_);
    if (!faulted_) {
      // Keep the input continuity baseline current during a bound hold. Servo
      // may advance unblocked joints while our output stays frozen; comparing
      // recovery against the pre-hold sample would turn gradual motion into a
      // false single-step fault. The output filter resumes from its held state.
      target_ = sampled_target_;
      if (!servo_hold_) filter_.step(target_, dt);
    }
    for (size_t i = 0; i < joints_.size(); ++i) if (filter_.position[i] < lower_[i] || filter_.position[i] > upper_[i]) {
      const double position = filter_.position[i];
      fault("joint position limit; reactivation required", i, position,
          std::clamp(position, lower_[i], upper_[i]), 0.0);
      filter_.position[i] = std::clamp(position, lower_[i], upper_[i]);
    }
  }
  if (!write()) return Return::ERROR;
  publish(time, std::isfinite(dt) && dt > 0 ? dt : 0.0);
  return Return::OK;  // Only timeout/Servo holds recover; other faults remain latched.
}
void TeleopJointController::publish(const rclcpp::Time& time, double dt) {
  if (state_pub_->trylock()) {
    auto& m = state_pub_->msg_; m.header.stamp = time;
    for (size_t i = 0; i < joints_.size(); ++i) {
      m.reference.positions[i] = m.output.positions[i] = filter_.position[i];
      m.reference.velocities[i] = m.output.velocities[i] = filter_.velocity[i];
      m.feedback.positions[i] = actual_[i]; m.error.positions[i] = filter_.position[i] - actual_[i];
    }
    state_pub_->unlockAndPublish();
  }
  status_elapsed_ += dt;
  if (status_elapsed_ >= 0.05 && status_pub_->trylock()) {
    auto& m = status_pub_->msg_; m.header.stamp = time;
    m.status[0].level = faulted_ ? 2 : (servo_hold_ || timeout_hold_) ? 1 : 0; m.status[0].message = reason_;
    m.status[0].values[0].value = faulted_ ? "FAULT" : (servo_hold_ || timeout_hold_) ? "HOLDING" : seen_command_ ? "RUNNING" : "WAITING";
    m.status[0].values[1].value = faulted_ && fault_joint_ >= 0 ? joints_[fault_joint_] : "";
    const double details[] = {fault_observed_, fault_reference_, fault_limit_, fault_actual_, fault_output_};
    for (size_t i = 0; i < 5; ++i)
      m.status[0].values[i + 2].value = faulted_ && std::isfinite(details[i]) ? std::to_string(details[i]) : "";
    status_pub_->unlockAndPublish(); status_elapsed_ = 0.0;
  }
}
} // namespace teleop_joint_controller
PLUGINLIB_EXPORT_CLASS(teleop_joint_controller::TeleopJointController, controller_interface::ControllerInterface)
