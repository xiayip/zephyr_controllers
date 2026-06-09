cmake_minimum_required(VERSION 3.10)
project(diagnostic_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(diagnostic_msgs REQUIRED)
ament_auto_find_build_dependencies()

# Parameters Library ================================================
set(TARGET ${PROJECT_NAME}_parameters)
generate_parameter_library(${TARGET} src/${TARGET}.yaml)

# ROS2 Controller Plugin ============================================
set(TARGET ${PROJECT_NAME})
ament_auto_add_library(${TARGET} SHARED src/${TARGET}.cpp)
target_link_libraries(${TARGET} ${TARGET}_parameters ${diagnostic_msgs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface ${TARGET}.xml)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)