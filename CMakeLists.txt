cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME differential_flatness_based_controller)
project(${PROJECT_NAME})

# set release mode
set(CMAKE_BUILD_TYPE Release)

# Default to C++17
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
  as2_core
  as2_msgs
  as2_control_command_handlers
  std_srvs
  Eigen3
  controller_plugin_base
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(SOURCE_CPP_FILES 
  src/DF_controller.cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main_nodes.cpp ${SOURCE_CPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})


# if(BUILD_TESTING)
#   find_package(ament_cmake_cppcheck REQUIRED)
#   find_package(ament_cmake_clang_format REQUIRED)

#   ament_cppcheck(src/ include/)
#   ament_clang_format(src/ include/)

#   include(tests/profiling_cmake.txt)

# endif()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  # ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()