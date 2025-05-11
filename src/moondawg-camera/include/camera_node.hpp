#ifndef MOONDAWG_CAMERA_NODE_HPP
#define MOONDAWG_CAMERA_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace moondawg
{

/**
 * @class CameraNode
 * @brief Node for handling camera image processing and publishing.
 *
 * This node handles:
 * - Processing camera feeds from various sources
 * - Compressing and formatting images for web transmission
 * - Managing multiple camera streams
 */
class CameraNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the CameraNode composable node.
   * @param options Node options for configuration
   */
  explicit CameraNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Virtual destructor
   */
  virtual ~CameraNode() = default;

private:
  /**
   * @brief Initialize all state variables for the node
   */
  void initStateVariables();

  /**
   * @brief Declare all ROS parameters for this node
   */
  void declareParameters();

  /**
   * @brief Set up all publishers, subscribers, and timers
   */
  void setupCommunications();

  /**
   * @brief Heartbeat function for publishing diagnostic information
   */
  void heartbeat();
  
  /**
   * @brief Set diagnostic status for the node
   * @param level Diagnostic level (OK, WARN, ERROR)
   * @param message Diagnostic message
   */
  void setDiagnosticStatus(uint8_t level, const std::string& message);

  // ROS Communications
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Parameters
  bool use_intra_process_comms_;

  // Diagnostics
  diagnostic_msgs::msg::DiagnosticStatus diagnostic_status_;
};

} // namespace moondawg

#endif // MOONDAWG_CAMERA_NODE_HPP
