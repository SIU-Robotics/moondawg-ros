cmake_minimum_required(VERSION 3.8)
project(moondawg_marker)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_library(
  moondawg_marker_component
  SHARED
  src/marker_component.cpp
)
ament_target_dependencies(
  moondawg_marker_component
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "OpenCV"
  "image_geometry"
  "tf2_ros"
  "tf2_geometry_msgs"
)

rclcpp_components_register_nodes(
  moondawg_marker_component
  "moondawg::MarkerComponent"
)

install(TARGETS
  moondawg_marker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
