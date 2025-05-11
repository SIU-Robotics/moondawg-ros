#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "camera_node.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  // Create a node with default parameters
  rclcpp::NodeOptions options;
  auto node = std::make_shared<moondawg::CameraNode>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  // Clean up
  rclcpp::shutdown();
  return 0;
}
