cmake_minimum_required(VERSION 3.18)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(controller_manager_msgs REQUIRED)
# Resolve OCS2's Boost component targets before Pinocchio's legacy FindBoost path.
find_package(ocs2_core REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Boost CONFIG REQUIRED)
set(controller_dependencies
  pluginlib controller_interface hardware_interface ament_index_cpp
  rclcpp rclcpp_lifecycle rclcpp_action rcl_interfaces zephyr_msgs
  geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs visualization_msgs realtime_tools
  tf2 tf2_ros tf2_geometry_msgs moveit_msgs
  ocs2_core ocs2_oc ocs2_mpc ocs2_ddp ocs2_msgs ocs2_mobile_manipulator
  ocs2_pinocchio_interface ocs2_self_collision ocs2_self_collision_visualization
  ocs2_ros_interfaces)
foreach(dependency IN LISTS controller_dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

set(controller_target ${PROJECT_NAME}_core)
add_library(${controller_target} SHARED
  src/mpc_controller.cpp
  src/control/CtrlComponent.cpp
  src/control/TcpKinematics.cpp
  src/control/TcpReferenceManager.cpp
  src/control/MotionActionServer.cpp
  src/control/RuntimeMotionProfile.cpp
  src/control/MotionProfileServer.cpp
  src/control/TcpTrajectoryRecorder.cpp
  src/visualization/mpc_visualizer.cpp)
set_target_properties(${controller_target} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_compile_features(${controller_target} PUBLIC cxx_std_17)
target_include_directories(${controller_target} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${controller_target} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_definitions(${controller_target} PUBLIC ${OCS2_CXX_DEFINITIONS})
ament_target_dependencies(${controller_target} PUBLIC ${controller_dependencies})
target_link_libraries(${controller_target} PUBLIC Eigen3::Eigen pinocchio::pinocchio
  Boost::headers Threads::Threads)
pluginlib_export_plugin_description_file(controller_interface mpc_controller.xml)

# Keep Qt/RViz dependencies out of the controller's runtime library.
option(BUILD_MPC_RVIZ_PLUGIN "Build the optional TCP RViz panels and error display" ON)
if(BUILD_MPC_RVIZ_PLUGIN)
find_package(rviz_common REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
add_library(mpc_rviz_panel SHARED plugin/tcp_rviz.cpp plugin/tcp_rviz.hpp)
set_target_properties(mpc_rviz_panel PROPERTIES AUTOMOC ON)
target_compile_features(mpc_rviz_panel PUBLIC cxx_std_17)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(interactive_markers REQUIRED)
ament_target_dependencies(mpc_rviz_panel PUBLIC
  geometry_msgs zephyr_msgs controller_manager_msgs visualization_msgs interactive_markers
  tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(mpc_rviz_panel PUBLIC
  Qt${QT_VERSION_MAJOR}::Widgets rviz_common::rviz_common
  rviz_rendering::rviz_rendering rviz_ogre_vendor::OgreOverlay
  ${rclcpp_TARGETS} ${pluginlib_TARGETS})
pluginlib_export_plugin_description_file(rviz_common plugin/rviz_plugins.xml)
install(TARGETS mpc_rviz_panel
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endif()

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${controller_target}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES README.md UPSTREAM.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(${controller_dependencies})
ament_export_dependencies(Eigen3 pinocchio Boost Threads)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Threads REQUIRED)
  ament_add_gtest(test_control_safety test/test_control_safety.cpp)
  target_include_directories(test_control_safety PRIVATE include)
  target_compile_features(test_control_safety PRIVATE cxx_std_17)
  target_link_libraries(test_control_safety Eigen3::Eigen Threads::Threads)
  ament_add_gtest(test_command_status test/test_command_status.cpp)
  target_compile_features(test_command_status PRIVATE cxx_std_17)
  ament_target_dependencies(test_command_status zephyr_msgs)
  ament_add_gtest(test_controller_switcher test/test_controller_switcher.cpp)
  target_compile_features(test_controller_switcher PRIVATE cxx_std_17)
  ament_target_dependencies(test_controller_switcher rclcpp controller_manager_msgs)
  ament_add_gtest(test_reference_handoff test/test_reference_handoff.cpp)
  target_link_libraries(test_reference_handoff ${controller_target} Threads::Threads)
  ament_add_gtest(test_motion_action test/test_motion_action.cpp)
  target_link_libraries(test_motion_action ${controller_target} Threads::Threads)
  ament_add_gtest(test_runtime_motion_profile test/test_runtime_motion_profile.cpp)
  target_link_libraries(test_runtime_motion_profile ${controller_target} Threads::Threads)
  ament_add_gtest(test_motion_profile_server test/test_motion_profile_server.cpp)
  target_link_libraries(test_motion_profile_server ${controller_target} Threads::Threads)
  ament_add_gtest(test_tcp_visualizer test/test_tcp_visualizer.cpp)
  target_link_libraries(test_tcp_visualizer ${controller_target} Threads::Threads)
  ament_add_gtest(test_tcp_trajectory_recorder
    test/test_tcp_trajectory_recorder.cpp src/control/TcpTrajectoryRecorder.cpp)
  target_include_directories(test_tcp_trajectory_recorder PRIVATE include)
  target_compile_features(test_tcp_trajectory_recorder PRIVATE cxx_std_17)
  target_link_libraries(test_tcp_trajectory_recorder Threads::Threads)
  ament_add_gtest(test_base_kinematics test/test_base_kinematics.cpp)
  target_link_libraries(test_base_kinematics ${controller_target})
  ament_add_gtest(test_tcp_kinematics test/test_tcp_kinematics.cpp)
  target_link_libraries(test_tcp_kinematics ${controller_target})
  ament_add_gtest(test_runtime_helpers test/test_runtime_helpers.cpp)
  target_link_libraries(test_runtime_helpers ${controller_target})
endif()
ament_package()
