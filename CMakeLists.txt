cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME controller_plugin_differential_flatness)
set(DFLIB DF_controller)
project(${PROJECT_NAME})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99) 
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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

# set(HEADER_HPP_FILES
#   include/${PROJECT_NAME}/DF_controller.hpp
#   include/${PROJECT_NAME}/DF_plugin.hpp
# )

# set(SOURCE_CPP_FILES 
#   lib/DF_controller.cpp
#   lib/DF_plugin.cpp
# )

# Library ${PROJECT_NAME} depends on lib/DF_controller.cpp. Link it.
# add_library(${PROJECT_NAME} ${SOURCE_CPP_FILES})
# target_link_libraries(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})



include_directories(
  include
  include/${PROJECT_NAME}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED lib/DF_plugin.cpp )
# add_library(${DFLIB} SHARED lib/DF_controller.cpp )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(controller_plugin_base plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
