// SPDX-License-Identifier: Apache-2.0
#include <mpc_controller/control/MotionProfileServer.h>

namespace zephyr::mpc {
namespace {
constexpr int64_t kFeedbackAgeNs = 200000000;
zephyr_msgs::msg::MotionProfile message(const RuntimeProfileConfig& config) {
  zephyr_msgs::msg::MotionProfile result;
  result.base_mode = config.base_mode;
  result.tcp_cost_scale = config.tcp_cost_scale;
  result.input_cost_scale = config.input_cost_scale;
  return result;
}
bool fresh(const MotionSample& sample, int64_t now) {
  return sample.valid && sample.received_ns > 0 && now >= sample.received_ns &&
         now - sample.received_ns <= kFeedbackAgeNs;
}
}  // namespace

MotionProfileServer::MotionProfileServer(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    std::shared_ptr<TcpReferenceManager> reference,
    std::shared_ptr<RuntimeMotionProfile> profile,
    std::shared_ptr<MotionRuntime> runtime)
    : node_(node), reference_(std::move(reference)), profile_(std::move(profile)),
      runtime_(std::move(runtime)) {
  runtime_->profile_output_allowed = false;
  requested_config_ = profile_->requestedConfig();
  detail_ = "waiting for initial MPC policy";
  publisher_ = rclcpp::create_publisher<ProfileStatus>(node_, "~/motion_profile_status", 1);
  service_ = node_->create_service<SetProfile>("~/set_motion_profile",
      [this](SetProfile::Request::SharedPtr req, SetProfile::Response::SharedPtr res) {
        request(*req, *res);
      });
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this] { tick(); });
}

void MotionProfileServer::request(const SetProfile::Request& req, SetProfile::Response& res) {
  std::lock_guard<std::mutex> lock(mutex_);
  RuntimeProfileConfig next = requested_config_;
  ++next.revision;
  next.base_mode = req.profile.base_mode;
  next.tcp_cost_scale = req.profile.tcp_cost_scale;
  next.input_cost_scale = req.profile.input_cost_scale;
  if (!validateRuntimeProfileConfig(next, res.message)) return;
  const auto sample = runtime_->samples.read();
  if (!runtime_->active || runtime_->stopRequested() || !fresh(sample, motionSteadyNs())) {
    res.message = "controller must be active with fresh feedback and no pending stop";
    return;
  }
  if (sample.reason != MotionStatus::NONE ||
      (sample.state != MotionStatus::IDLE &&
       !(sample.state == MotionStatus::INITIALIZING && transition_pending_))) {
    res.message = "controller must be idle; finish or cancel its motion first";
    return;
  }
  ocs2::vector_t pose(7);
  for (size_t i = 0; i < 7; ++i) pose[i] = sample.pose[i];
  // Reserve all target inputs and queue a measured hold BEFORE submitting the
  // new profile. The solver captures its profile before reading the reference.
  if (!reference_->beginProfileChange(pose)) {
    res.message = "motion owns the reference; finish or cancel it before changing profile";
    return;
  }
  owns_reference_gate_ = true;
  runtime_->profile_output_allowed = false;
  request_activation_epoch_ = runtime_->activation_epoch.load();
  transition_pending_ = true;
  profile_submitted_ = profile_->submit(next, res.message);
  handover_failed_ = !profile_submitted_;
  if (!profile_submitted_) {
    detail_ = "profile handover failed: " + res.message;
    return;
  }
  requested_config_ = next;
  detail_ = "waiting for matching MPC policy";
  res.accepted = true;
  res.revision = next.revision;
  res.message = detail_;
}

void MotionProfileServer::tick() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_activation_epoch_ &&
      (!runtime_->active || runtime_->activation_epoch.load() != request_activation_epoch_)) {
    handover_failed_ = true;
    transition_pending_ = true;
    detail_ = "controller activation changed; reapply profile after activation";
  }
  const auto sample = runtime_->samples.read();
  const bool healthy = runtime_->active && !runtime_->stopRequested() &&
      fresh(sample, motionSteadyNs()) && sample.reason == MotionStatus::NONE &&
      (sample.state == MotionStatus::IDLE || sample.state == MotionStatus::INITIALIZING);
  const bool ready = healthy && profile_submitted_ && !handover_failed_ &&
      sample.profile_revision == requested_config_.revision;
  runtime_->profile_output_allowed = ready;
  if (ready) {
    if (owns_reference_gate_) reference_->endProfileChange();
    owns_reference_gate_ = false;
    transition_pending_ = false;
    detail_ = "MPC profile applied";
  }
  ProfileStatus status;
  status.header.stamp = node_->now();
  status.requested = message(requested_config_);
  status.requested_revision = requested_config_.revision;
  status.active.base_mode = sample.profile_base_mode;
  status.active.tcp_cost_scale = sample.profile_tcp_cost_scale;
  status.active.input_cost_scale = sample.profile_input_cost_scale;
  status.active_revision = runtime_->active ? sample.profile_revision : 0;
  status.applied = ready;
  status.transition_pending = transition_pending_;
  status.message = !runtime_->active ? "controller inactive" : detail_;
  publisher_->publish(status);
}
}  // namespace zephyr::mpc
