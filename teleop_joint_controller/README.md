# teleop_joint_controller

Samples the rolling position window from official MoveIt Servo
`trajectory_msgs/JointTrajectory` on `~/joint_trajectory`. It claims the downstream
position adapter's position and velocity references instead of JTC. Joint names
and order must match `joints` and the Servo group. Z2 uses joint1 through joint6.

Only the latest window is retained (volatile, depth one). The 200 Hz controller
linearly samples its positions at the controller timestamp, holding the first or
last point outside the received window without extrapolation. This avoids chasing
each newly received future endpoint as a step. The controller clock then advances
a bounded position reference; velocity is the actual reference
step divided by that control period. Velocity and acceleration limits apply to
normal tracking, including target reversals. A stopping-distance bound slows
approach to a stationary target. It does not use cubic interpolation or forward
an independent, potentially inconsistent velocity field. The bound also applies
when an unusually short source interval produces a steep linear segment.
`reference_response_time` defaults to 0.01 s: a short first-order response reduces
velocity ripple when asynchronous Servo/controller ticks repeat or skip a waypoint.
This trades roughly 10 ms of low-speed reference response for smoother output;
the existing velocity/acceleration limits still apply. Zero disables this response.

Activation starts at measured joint positions with zero reference velocity and
requires a fresh command from the new activation. Wrong joint names, empty or
wrong-length/nonfinite positions, stale or excessively future-dated waypoints,
URDF position violations, discontinuous targets, excessive tracking error, long
control periods, and startup timeout latch FAULT. Faults freeze the last
reference and set velocity to zero, intentionally overriding acceleration limits.
New packets cannot resume a faulted controller: deactivate and reactivate it.
Every waypoint must have finite in-range positions and strictly increasing time.
Repeated or out-of-order window endpoints neither replace the accepted window
nor refresh the watchdog.
Invalid hardware feedback also holds the last valid reference. This is a position
hold, not an emergency stop or a substitute for motor-side protections.

`~/controller_state` publishes commanded positions/velocities and measured
positions. `~/status` publishes a diagnostic with WAITING, RUNNING, HOLDING or FAULT at
20 Hz while active. The robot state manager handles FAULT by disabling the pose
adapter, pausing Servo and switching back to JTC without a restore motion.
After a stream has started, command timeout instead enters recoverable `HOLDING`:
freeze the last output position and zero velocity. A fresh command can resume only
when every waypoint is within `min(max_target_step, max_tracking_error)` radians
of both measured joint feedback and the held reference, for every joint. Distant
commands leave output paused, rather than faulting or moving toward the target.
The filter resumes from its held state with zero velocity. Timeout is checked
before consuming a newly queued packet, so callback ordering cannot bypass this
proximity check. Invalid packets, feedback errors and other faults remain latched.
The timeout is enforced inside the controller even if the state manager is unavailable.

The controller also consumes `moveit_msgs/ServoStatus` on `servo_status_topic`
(default `/servo_node/status`). Collision stops and invalid commands latch a hold on the next update and preserve the first stop
reason, even if fresh trajectories or a normal status arrive afterward. This
prevents a stop from being reported later merely as a command timeout, or the
controller continuing to chase the remaining rolling reference. Reactivation is
still explicit.

`JOINT_BOUND` and `HALT_FOR_SINGULARITY` instead enter a recoverable HOLDING state: freeze the last reference
and set velocity to zero, without asking the state manager to exit teleop. Resume
only after Servo clears the restriction and a valid trajectory received after the last
holding status is available. A normal status alone cannot replay the old window.
During the hold, continue sampling and validating the incoming trajectory without
advancing the output reference. This keeps the continuity baseline current, so
cumulative motion during a hold is not mistaken for a single-update jump when
tracking resumes. Resume from the held reference through the bounded filter.
Move the target back within the joint limits to recover; no reactivation is needed.
Official Servo joint limits and all controller input, tracking and timeout checks
remain active during the hold. The pose adapter recognizes fresh intentional Servo
restrictions and does not interpret their Cartesian lag as failed tracking.
The joint-reference versus hardware tracking check remains active.
Joint-bound warnings are throttled to once every two seconds, outside update().

Deceleration statuses keep tracking with Servo's scaled output.
They are reported at most every two seconds while active. Z2 relays the official
Servo rosout logger through `servo_log_relay.py`, retaining exact joint names in
joint-limit warnings while throttling identical warnings to once every two seconds.
Errors are not throttled. The official Servo package remains unmodified.

FAULT diagnostics preserve the first triggering joint and observed/reference/limit
values, plus that joint's hardware and commanded positions. RobotStateManager logs
these fields, holding reasons and recovery transitions outside the control update.
Joint position/error values are radians; timeout/update-period values are seconds.
Official Servo must permit a valid command before recovery. A truly singular
configuration may remain blocked by Servo even though teleop stays active.

Z2 loads the controller inactive. EnableTeleopModeBT switches JTC off and this
controller on after the Quest identity handshake, then unpauses Servo. Normal
exit switches back before the saved-pose action. Presets continue to use JTC.
The MIT output controller, gravity compensation and position adapter stay active.

Z2 velocity/acceleration limits come from `z2_bringup/config/teleop.yaml`'s existing
`joint_limits` section, shared with Servo. Position bounds come from the robot
model. Guard thresholds and joint order are in `whole_robot.yaml`. Official Servo
keeps IK, collision/singularity handling, joint velocity limits and its continuous
rolling reference. Z2 sets `servo.use_smoothing: false`: the external controller
owns joint-reference smoothing and acceleration limits. Keeping position-only
trajectory output preserves Servo's rolling reference while avoiding JTC splines.

The Z2 baseline was checked on physical hardware on 2026-10-22 with the existing
Quest APK. One session stopped at a Servo singularity; another completed about
19 seconds with normal exit and no Servo warning or tracking fault. Recorded
joint references respected the configured velocity/acceleration bounds within
sampling tolerance. This validates those motions, not the complete workspace or
motor capabilities. Pose conditioning still contributes latency; wider movements
can still reach singularity or collision guards.
