cmake_minimum_required(VERSION 3.10)
project(normalized_gripper_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(PARAMETERS_TARGET ${PROJECT_NAME}_parameters)
generate_parameter_library(
  ${PARAMETERS_TARGET}
  src/${PROJECT_NAME}_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE NORMALIZED_GRIPPER_CONTROLLER_BUILDING_DLL)
target_link_libraries(${PROJECT_NAME} ${PARAMETERS_TARGET})
pluginlib_export_plugin_description_file(
  controller_interface normalized_gripper_controller.xml)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
