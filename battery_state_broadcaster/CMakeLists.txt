cmake_minimum_required(VERSION 3.8)
project(battery_state_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Parameters Library ================================================
set(TARGET ${PROJECT_NAME}_parameters)
generate_parameter_library(${TARGET} src/${TARGET}.yaml)

# ROS2 Controller Plugin ============================================
set(TARGET ${PROJECT_NAME})
ament_auto_add_library(${TARGET} SHARED src/${TARGET}.cpp)
target_link_libraries(${TARGET} ${TARGET}_parameters)
pluginlib_export_plugin_description_file(controller_interface ${TARGET}.xml)

pluginlib_export_plugin_description_file(
  controller_interface battery_state_broadcaster.xml)

# Testing ===========================================================
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()

#   set(TARGET test_load_${PROJECT_NAME})
#   add_rostest_with_parameters_gmock(${TARGET} test/${TARGET}.cpp
#     ${CMAKE_CURRENT_SOURCE_DIR}/test/${TARGET}_params.yaml)
#   target_link_libraries(${TARGET}
#     ${${PROJECT_NAME}_LIBRARIES})
#   ament_target_dependencies(${TARGET}
#     ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
#     ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

#   set(TARGET test_${PROJECT_NAME})
#   ament_add_gmock(${TARGET} test/${TARGET}.cpp)
#   target_link_libraries(${TARGET} ${${PROJECT_NAME}_LIBRARIES})
#   ament_target_dependencies(${TARGET}
#     ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
#     ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
# endif()

ament_auto_package()