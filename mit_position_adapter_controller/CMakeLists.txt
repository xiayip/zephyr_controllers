cmake_minimum_required(VERSION 3.16)
project(mit_position_adapter_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(gravity_compensation_model REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

generate_parameter_library(
  mit_position_adapter_controller_parameters
  src/mit_position_adapter_controller_parameters.yaml
)

add_library(mit_position_adapter_controller SHARED src/mit_position_adapter_controller.cpp)
target_compile_features(mit_position_adapter_controller PUBLIC cxx_std_17)
target_include_directories(mit_position_adapter_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mit_position_adapter_controller>
)
target_link_libraries(
  mit_position_adapter_controller PUBLIC mit_position_adapter_controller_parameters)
ament_target_dependencies(mit_position_adapter_controller PUBLIC
  control_msgs
  controller_interface
  gravity_compensation_model
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

pluginlib_export_plugin_description_file(
  controller_interface mit_position_adapter_controller.xml)

install(DIRECTORY include/ DESTINATION include/mit_position_adapter_controller)
install(
  TARGETS mit_position_adapter_controller mit_position_adapter_controller_parameters
  EXPORT export_mit_position_adapter_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(mit_controller REQUIRED)
  ament_add_gtest(
    test_mit_position_adapter_controller
    test/test_mit_position_adapter_controller.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_link_libraries(
    test_mit_position_adapter_controller mit_position_adapter_controller)
  ament_target_dependencies(test_mit_position_adapter_controller
    control_msgs
    controller_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    realtime_tools
  )
  ament_add_gtest(test_mit_chain_handover
    test/test_mit_chain_handover.cpp
    SKIP_LINKING_MAIN_LIBRARIES
  )
  target_link_libraries(test_mit_chain_handover mit_position_adapter_controller)
  ament_target_dependencies(test_mit_chain_handover
    mit_controller controller_interface hardware_interface rclcpp rclcpp_lifecycle
  )
endif()

ament_export_dependencies(
  control_msgs controller_interface gravity_compensation_model
  hardware_interface realtime_tools)
ament_export_libraries(mit_position_adapter_controller)
ament_export_targets(export_mit_position_adapter_controller HAS_LIBRARY_TARGET)
ament_package()
