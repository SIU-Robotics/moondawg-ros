cmake_minimum_required(VERSION 3.8)
project(moondawg_camera)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  diagnostic_msgs
  cv_bridge
  OpenCV
)

add_executable(
  moondawg_camera_node
  src/moondawg_camera_node.cpp
)

add_library(
  moondawg_camera_node_component
  src/moondawg_camera_node.cpp
)

ament_target_dependencies(
  moondawg_camera_node
  ${DEPENDENCIES}
)

target_link_libraries(
  moondawg_camera_node
  ${OpenCV_LIBS}
)

ament_package()