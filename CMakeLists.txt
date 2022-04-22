cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME differential_flatness_based_controller)
set(PLUGIN_NAME differential_flatness_based_controller_plugin)
project(${PROJECT_NAME})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


# find dependencies

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  nav_msgs
  Eigen3
  pluginlib
  as2_core
  as2_msgs
  as2_control_command_handlers
  std_srvs
  controller_plugin_base
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PLUGIN_NAME} src/DF_controller.cpp)

target_link_libraries(${PLUGIN_NAME} ${EIGEN3_LIBS})

target_include_directories(${PLUGIN_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PLUGIN_NAME}
  ${PROJECT_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(controller_plugin_base plugins.xml)

install(
  TARGETS ${PLUGIN_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  ${PLUGIN_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()