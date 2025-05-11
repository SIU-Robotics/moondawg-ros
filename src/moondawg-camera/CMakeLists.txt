cmake_minimum_required(VERSION 3.8)
project(moondawg_camera)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory (../lib lib)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Camera Node Component
add_library(camera_node_component SHARED
  src/camera_node.cpp
)
target_compile_definitions(camera_node_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(camera_node_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  diagnostic_msgs
  cv_bridge
  image_transport
  OpenCV
)
rclcpp_components_register_nodes(camera_node_component "moondawg::CameraNode")
install(TARGETS
  camera_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Camera Node Executable
add_executable(camera_node
  src/camera_node_main.cpp
)
target_link_libraries(camera_node
  camera_node_component
  ${OpenCV_LIBS}
)
install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install directories
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
