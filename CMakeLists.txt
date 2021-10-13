cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME differential_flatness_based_controller)
project(${PROJECT_NAME})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(aerostack2_core REQUIRED)
find_package(aerostack2_msgs REQUIRED)
find_package(as2_control_command_handlers REQUIRED)
find_package(std_srvs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(SOURCE_CPP_FILES 
  src/DF_controller.cpp
)

include_directories(
  include
)

add_executable(${PROJECT_NAME}_node src/main_nodes.cpp ${SOURCE_CPP_FILES} ${HEADER_HPP_FILES})
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp 
  sensor_msgs 
  std_msgs
  std_srvs
  nav_msgs
  aerostack2_core
  aerostack2_msgs
  geometry_msgs
  trajectory_msgs
  as2_control_command_handlers
)


if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  
  ament_cpplint(src/ include/)
  ament_cppcheck(src/ include/)
  ament_clang_format(src/ include/)

endif()

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
