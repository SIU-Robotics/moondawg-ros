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
message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
message(STATUS "cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

# Include directories for the current package's headers
include_directories(include)

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
)
rclcpp_components_register_nodes(camera_node_component "moondawg::CameraNode")

# Image Compression Node Component
add_library(image_compression_node_component SHARED
  src/image_compression_node.cpp
)
target_compile_definitions(image_compression_node_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

# This should handle include directories and linking for all listed dependencies.
ament_target_dependencies(image_compression_node_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  diagnostic_msgs
  cv_bridge       # This is crucial for finding <cv_bridge/cv_bridge.h>
  image_transport
  OpenCV
)

# Add this package's own include directory to the target,
# so it can find its own .hpp files (like image_compression_node.hpp from image_compression_node.cpp)
# and so that other packages linking to this component can find its .hpp files.
target_include_directories(image_compression_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Explicit target_link_libraries for cv_bridge and OpenCV are removed
# as ament_target_dependencies should handle this.

rclcpp_components_register_nodes(image_compression_node_component "moondawg::ImageCompressionNode")

install(TARGETS
  camera_node_component
  image_compression_node_component # Add new component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Remove linking for the old standalone camera_node executable if it existed
# target_link_libraries(camera_node
#   camera_node_component
#   ${OpenCV_LIBS}
# )
# install(TARGETS
#   camera_node
#   DESTINATION lib/${PROJECT_NAME}
# )

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
