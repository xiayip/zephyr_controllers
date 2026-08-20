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
  equilibrium references, motor-side MIT stiffness/damping, and ROS feed-forward shaping
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
