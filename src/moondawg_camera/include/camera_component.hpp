#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

namespace moondawg
{

/**
 * @class CameraComponent
 * @brief Node for handling camera image processing, compression, and publishing.
 *
 * This node handles:
 * - Subscribing to raw camera feeds
 * - Processing camera feeds (resizing, color mapping for depth)
 * - Compressing images to JPEG
 * - Publishing compressed images as base64 strings
 * - Managing diagnostic information including processing latency
 */
class CameraComponent : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the CameraComponent composable node.
   * @param options Node options for configuration
   */
  explicit CameraComponent(const rclcpp::NodeOptions & options);

  /**
   * @brief Virtual destructor
   */
  virtual ~CameraComponent() = default;

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
   * @brief Callback for incoming raw image messages.
   * @param message Shared pointer to the raw image message.
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr message);

  /**
   * @brief Processes an OpenCV image, compresses it, and publishes it.
   * @param cv_image The OpenCV image to process.
   * @param input_topic_name The name of the input topic for logging.
   * @return True if the image was processed and published, false otherwise.
   */
  bool processAndPublishImage(const cv::Mat & cv_image, const std::string & input_topic_name);

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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr compressed_image_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Parameters
  bool use_intra_process_comms_;
  int image_compression_quality_;
  int image_frame_rate_;  // Kept for backward compatibility but no longer used for limiting
  std::string camera_key_; 
  
  // New optimization parameters
  bool is_depth_camera_;
  int skip_frames_;
  bool use_optimized_encoding_;
  int frame_count_;
  int depth_max_value_mm_; // Maximum depth value in mm for normalization

  // Variables for latency tracking
  double last_processed_time_;
  
  // Diagnostics
  diagnostic_msgs::msg::DiagnosticStatus diagnostic_status_;
  struct LatencyStat {
    long long count = 0;
    double total_latency = 0.0;
    double avg_latency = 0.0;
    double max_latency = 0.0;
    double last_latency = 0.0;
  };
  LatencyStat latency_stats_;
};

} // namespace moondawg