.. _swerve_drive_controller_userdoc:

swerve_drive_controller
=========================

ROS 2 control plugin for mobile bases with four velocity-controlled drive joints
and four position-controlled steering joints.

Execution logic of the controller
----------------------------------

The controller consumes planar ``Twist`` commands and computes each wheel's drive
velocity and steering position. It supports coordinated turns, crab motion, and
in-place rotation. Commands older than ``cmd_vel_timeout`` stop and center all
wheels.


Description of controller's interfaces
--------------------------------------

Subscribers
,,,,,,,,,,,,

If ``use_stamped_vel=true``, ``cmd_vel_topic`` uses
``geometry_msgs/msg/TwistStamped``.

If ``use_stamped_vel=false``, ``cmd_vel_topic`` uses
``geometry_msgs/msg/Twist``.

Publishers
,,,,,,,,,,,
~/odom [nav_msgs::msg::Odometry]
  Estimated pose and planar velocity.

/tf [tf2_msgs::msg::TFMessage]
  ``odom_frame_id`` to ``base_frame_id`` transform, when ``enable_odom_tf=true``.


Parameters
,,,,,,,,,,,

Required joint parameters are ``front_left_wheel_joint``,
``front_right_wheel_joint``, ``rear_left_wheel_joint``,
``rear_right_wheel_joint``, and their corresponding ``*_steering_joint`` names.

Geometry is configured with ``wheel_base``, ``track_width``, and ``wheel_radius``.
Each optional ``*_steering_direction`` is ``1.0`` or ``-1.0`` and maps controller
angles to the joint axis convention. ``steering_limit`` defines the canonical
angle range; an equivalent reversed wheel velocity is selected when it shortens
the steering motion.

``open_loop`` selects commanded wheel values instead of state feedback for
odometry. ``max_wheel_velocity`` applies a symmetric wheel-speed limit.

Legacy joint, geometry, frame, and per-wheel threshold parameter names remain
accepted for one compatibility cycle and emit a deprecation warning.
