# upstream provenance

This package adapts the ros2_control integration from
https://github.com/fiveages-sim/arms_ros2_control under Apache-2.0.
The complete upstream license is included in `LICENSE`.

Baseline: `2309556624b41d0f45267c369358e35bcdd8f431` (the local origin/main at migration).

| Local implementation | Source at baseline |
| --- | --- |
| `src/mpc_controller.cpp`, corresponding header | `controller/ocs2_arm_controller/src/Ocs2ArmController.cpp`, corresponding header |
| `src/control`, `include/.../control` | `controller/ocs2_arm_controller/src/control`, corresponding headers |
| MPC worker in `src/mpc_controller.cpp` | Replaces the former FSM adaptation with direct OCS2 execution |
| `src/visualization/mpc_visualizer.cpp`, corresponding header | `libraries/ocs2_controller_common/src/visualization/Ocs2PinocchioVisualizer.cpp`, corresponding header |
| `plugin/tcp_rviz.cpp` and `plugin/tcp_rviz.hpp` (Qt AUTOMOC) | Zephyr addition previously kept inside `command/arms_rviz_control_plugin` |
| `include/.../control/JointInterfaces.h` | `libraries/arms_controller_common/include/arms_controller_common/CtrlInterfaces.h`; reduced to interface handles and position command cache |
| `include/.../runtime/ThreadScheduling.h`, `RtCycleTiming.h`, `MpcExecutionParams.hpp` | `libraries/ocs2_controller_common/include/ocs2_controller_common/rt/{ThreadScheduling.h,RtCycleTiming.h}` and `mpc/MpcExecutionParams.hpp`; local namespace, unused state removed |
| `plugin/error_dashboard.hpp`, `error_sparkline.hpp` | `command/arms_rviz_control_plugin/include/arms_rviz_control_plugin/`; reduced to a single TCP card |

The controller integration is adapted locally because the upstream controller
is `final`, owns its component privately, and has no extension point for the
mobile-base observation/command offsets or replacement visualizer.
All adapted C++ symbols live in `zephyr` namespaces to avoid collisions with
upstream libraries. Only the Zephyr integration and visualization are built
here. MPC/DDP solvers, model construction and costs/constraints remain linked OCS2
implementations. Minimal joint handles, scheduling/timing and single-TCP error graphics
are maintained locally under Apache-2.0; there is no build/runtime dependency on
`arms_controller_common`, `ocs2_controller_common`, `arms_ros2_control_msgs`, or
`arms_rviz_control_plugin`. Native single-TCP references,
IK/FK, marker interaction, pose UI and recording are implemented locally without arm-side
name translation. FK/IK use `moveit_msgs` contracts (not MoveIt runtime);
motion targets use standard geometry_msgs/nav_msgs topics, with shape generation in higher-level demos.
There is no zephyr_msgs dependency. No source-file injection, include-directory shadowing, duplicate
package names, or post-clone patch step is used.

The `ocs2_wbc_controller` entry in arms_ros2_control is an uninitialized external
submodule in this workspace. This migration preserves the working mobile
manipulator MPC model rather than switching to a different controller/model.

Updating OCS2 libraries requires rebuilding their dependants. Upstream API
changes may require adapting this package; upstream changes to copied controller
integration do not propagate automatically. Compare the listed source files
against this recorded baseline when adopting such changes.

The marker server lives in the RViz plugin and shares the RViz ROS node.
No standalone marker executable is needed, and the headless controller does not load Qt.

TcpReferenceManager and TcpTrajectoryRecorder each have a dedicated header/source
pair within this package. CtrlComponent coordinates them and the separate prediction
visualizer. TcpKinematics has its own implementation and tests. Copying these small
helpers does not vendor the OCS2 algorithms or create a fork of its ROS integration packages.
