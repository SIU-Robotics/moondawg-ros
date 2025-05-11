#include "camera_node.hpp"

#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace moondawg
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
  initStateVariables();
  declareParameters();
  setupCommunications();
  RCLCPP_INFO(this->get_logger(), "Camera node (relay) initialized and ready");
}

void CameraNode::initStateVariables()
{
  diagnostic_status_.name = this->get_name();
  diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_status_.message = "Initializing camera relay node";
}

void CameraNode::declareParameters()
{
  // Parameter for intra-process communication
  this->declare_parameter("use_intra_process_comms", true);
}

void CameraNode::setupCommunications()
{
  use_intra_process_comms_ = this->get_parameter("use_intra_process_comms").as_bool();
  
  auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();
  
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable();
  
  diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "~/diag", // Relative topic
    reliable_qos
  );

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&CameraNode::heartbeat, this)
  );
  
  setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera relay node ready");
}

void CameraNode::heartbeat()
{
  diagnostic_status_.values.clear(); 
  diag_publisher_->publish(diagnostic_status_);
}

void CameraNode::setDiagnosticStatus(uint8_t level, const std::string& message)
{
  diagnostic_status_.level = level;
  diagnostic_status_.message = message;
  if (level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
    diag_publisher_->publish(diagnostic_status_);
  }
}

} // namespace moondawg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moondawg::CameraNode)
