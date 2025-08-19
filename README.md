# zwind_controllers

This is a meta-package that groups all custom ROS2 controllers developed for the zwind robotics platform.

## Included Packages

This meta-package includes the following controller packages:

### Core Controllers
- **battery_state_broadcaster** - Controller to publish readings of Battery state
- **diagnostic_broadcaster** - The diagnostic message broadcaster
- **dexhand_controller** - Dexterous hand controller
- **omni_picker_controller** - The Omni picker controller
- **swerve_drive_controller** - Controller for a 4 wheel swerve drive robot base

## Installation

To install all zwind controllers, simply install this meta-package:

```bash
# Build the workspace
colcon build --packages-select zwind_controllers

# Or build specific controller packages
colcon build --packages-select battery_state_broadcaster dexhand_controller
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
