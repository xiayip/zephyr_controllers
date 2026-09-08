# zephyr_controllers

This is a meta-package that groups all custom ROS2 controllers developed for the zephyr robotics platform.

## Included Packages

This meta-package includes the following controller packages:

### Core Controllers
- **battery_state_broadcaster** - Controller to publish readings of Battery state
- **diagnostic_broadcaster** - The diagnostic message broadcaster
- **dexhand_controller** - Dexterous hand controller
- **gravity_compensation_model** - Realtime Pinocchio gravity model and feed-forward shaping library
- **mit_controller** - Chainable output stage for the MIT position, velocity, gain, and feed-forward tuple
- **mit_damping_controller** - Gravity-compensated damping mode with filtered velocity,
  torque shaping, and URDF soft limits
- **mit_impedance_controller** - Gravity-compensated joint impedance mode with rate-limited
  equilibrium references, bounded motor-side MIT spring feedback, configurable smooth
  Coulomb/viscous friction feed-forward, ROS feed-forward shaping, and a blocked-trajectory guard
- **mit_position_adapter_controller** - Completes position and velocity references into full MIT
  commands and publishes their realtime torque decomposition
- **omni_picker_controller** - The Omni picker controller
- **swerve_drive_controller** - Controller for a 4 wheel swerve drive robot base

## Installation

To install all zephyr controllers, simply install this meta-package:

```bash
# Build the workspace
colcon build --packages-select zephyr_controllers

# Or build the MIT controller stack and its dependencies
colcon build --packages-up-to mit_controller mit_position_adapter_controller \
  mit_damping_controller mit_impedance_controller
```

## Usage

Each controller can be loaded and used through the ROS2 control framework:

```bash
# Load controller through controller manager
ros2 control load_controller <controller_name>

# Start controller
ros2 control set_controller_state <controller_name> active
```

## MIT startup and shutdown handover

The motor-side MIT tuple represents
`tau = kp * (q_ref - q) + kd * (v_ref - dq) + t_ff`.
Both position and velocity **state** interfaces are required by `mit_controller`.

- Activation adopts the complete existing hardware tuple, including its velocity
  reference and feed-forward. A warm motor may already support a load using a
  position preload; that preload must not be cleared.
- The Position Adapter exports **measured position** and zero velocity as its
  initial upstream references, so JTC does not interpret a preload as a motion
  goal. It transfers the inherited PD/feed-forward torque into an equivalent
  initial `t_ff`, accounting for both old and new gains and measured velocity.
  Nonfinite startup feedback/commands or an initial feed-forward outside the
  configured torque limit reject activation before changing downstream commands.
- Startup uses one feed-forward blend from inherited torque to calibrated
  Pinocchio gravity. `gravity_compensation.factor` starts at its configured value;
  changes to that factor **after activation** still use the existing factor ramp.
  Torque and torque-rate limits remain enabled.
- On upstream disconnection or output-controller deactivation, MIT latches a
  stationary hold with zero velocity reference and the inherited startup gains.
  Its equivalent current-position feed-forward is
  `old_t_ff + old_kp*(old_q_ref-q) + old_kd*old_v_ref + (hold_kd-old_kd)*dq`.
  It does not reset feed-forward to zero. Invalid feedback uses the last finite
  hold, and consecutive unchain/deactivate callbacks reuse the latched command.
- `mit_controller.max_position_step` defaults to **0.02 rad per write** and must
  not exceed the downstream hardware position-step limit. Run this synchronous
  controller at the hardware write rate. When the motor reference must step,
  position and modeled torque are interpolated together, with feed-forward
  recalculated for the intermediate position. This avoids double-counting a
  transferred preload when the hardware applies its own position-only limit.
  A genuine large position request also advances its modeled torque gradually;
  feed-forward does not immediately apply the full requested torque.

The hold position in the motor tuple may still be an intermediate stepped
reference. Its compensated torque is equivalent at the handover state; it need
not equal the measured position after a single shutdown write. Torque continuity
assumes commands remain within downstream gain/torque/position limits and ignores
motor quantization, feedback delay and modeling error. This is **not** an
emergency stop, guaranteed exact-position lock, or protection against power loss.
Keep a cold, unsupported arm mechanically supported until compensation is stable.

For real-machine validation, controller `active` does not mean startup settling
is complete. Wait for both the startup blend and torque-rate-limited output to
settle, then verify stable joint positions/velocities before sending a trajectory.
Use the Adapter's `~/mit_state` decomposition (`scaled_gravity_torque`,
`feedforward_torque`, PD torques, etc.) along with joint feedback. A steady-state
tracking-error threshold applied partway through the startup blend is not a
startup-readiness test. Adapter telemetry describes its requested tuple; a
step-limited MIT output may temporarily use a different PD/feed-forward split.

Offline handover regressions:

```bash
ROS_DOMAIN_ID=211 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST colcon test \
  --packages-select mit_controller mit_position_adapter_controller shared_can_ros2
```

These include cold/warm six-joint controller-chain lifecycles, gain/velocity
changes, invalid feedback, torque/rate limits and final hardware command flushing.
They do not open CAN or establish real-machine safety.

## Dependencies

This meta-package depends on:
- ROS2 control framework
- controller_interface
- hardware_interface
- Individual controller packages listed above

## License

Apache-2.0

## Maintainer

xiayip (yipeng.xia@hotmail.com)
