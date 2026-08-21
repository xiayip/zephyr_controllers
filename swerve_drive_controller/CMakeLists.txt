cmake_minimum_required(VERSION 3.10)
project(swerve_drive_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(swerve_drive_controller SHARED
  src/swerve_drive_controller.cpp
  src/swerve_drive_kinematics.cpp
)

target_compile_features(swerve_drive_controller PUBLIC cxx_std_17)
target_include_directories(swerve_drive_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(swerve_drive_controller PUBLIC
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_msgs
)
pluginlib_export_plugin_description_file(controller_interface swerve_drive_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_swerve_drive_kinematics
    test/test_swerve_drive_kinematics.cpp
  )
  target_link_libraries(test_swerve_drive_kinematics swerve_drive_controller)

  ament_add_gtest(test_plugin_loading
    test/test_plugin_loading.cpp
  )
  ament_target_dependencies(test_plugin_loading
    controller_interface
    pluginlib
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS swerve_drive_controller
  EXPORT export_swerve_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_swerve_drive_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_msgs
)
ament_package()
