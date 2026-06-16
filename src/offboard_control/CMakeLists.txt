cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Eigen3 3.3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/lib)

set(SOURCE_FILES
  src/offboard_control_node.cpp
  src/commander.cpp
)

set(MATH_SOURCE_FILES
  lib/mathlib/inc/transformations.hpp
  lib/mathlib/src/transformations.cpp
)

add_library(mathlib ${MATH_SOURCE_FILES})

add_executable(offboard_control_node ${SOURCE_FILES})

ament_target_dependencies(offboard_control_node
  rclcpp
  px4_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(offboard_control_node mathlib)

install(TARGETS offboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
