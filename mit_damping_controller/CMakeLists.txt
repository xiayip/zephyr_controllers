cmake_minimum_required(VERSION 3.16)
project(mit_damping_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(gravity_compensation_model REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

generate_parameter_library(
  mit_damping_controller_parameters
  src/mit_damping_controller_parameters.yaml
)

add_library(mit_damping_controller SHARED src/mit_damping_controller.cpp)
target_compile_features(mit_damping_controller PUBLIC cxx_std_17)
target_include_directories(mit_damping_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mit_damping_controller>
)
target_link_libraries(
  mit_damping_controller PUBLIC mit_damping_controller_parameters)
ament_target_dependencies(mit_damping_controller PUBLIC
  control_msgs
  control_toolbox
  controller_interface
  gravity_compensation_model
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

pluginlib_export_plugin_description_file(
  controller_interface mit_damping_controller.xml)

install(DIRECTORY include/ DESTINATION include/mit_damping_controller)
install(
  TARGETS mit_damping_controller mit_damping_controller_parameters
  EXPORT export_mit_damping_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    test_mit_damping_controller
    test/test_mit_damping_controller.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_link_libraries(test_mit_damping_controller mit_damping_controller)
  ament_target_dependencies(test_mit_damping_controller
    control_msgs
    control_toolbox
    controller_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    realtime_tools
  )
endif()

ament_export_dependencies(
  control_msgs control_toolbox controller_interface gravity_compensation_model
  hardware_interface realtime_tools)
ament_export_libraries(mit_damping_controller)
ament_export_targets(export_mit_damping_controller HAS_LIBRARY_TARGET)
ament_package()
