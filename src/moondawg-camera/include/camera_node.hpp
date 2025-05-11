#ifndef MOONDAWG_CAMERA_NODE_HPP
#define MOONDAWG_CAMERA_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
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
   * @brief Common image processing function to optimize camera feed handling
   *
   * @param cv_image OpenCV image to process
   * @param last_time_attr Reference to last frame time for rate limiting
   * @param publisher ROS publisher to send the processed image to
   * @param is_depth True if this is a depth image (for specialized processing)
   * @param camera_key Key to identify the camera for latency tracking
   * @return True if image was processed and published, False if skipped
   */
  bool processImage(const cv::Mat & cv_image, 
                   double & last_time_ref,
                   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
                   bool is_depth = false,
                   const std::string & camera_key = "");

  /**
   * @brief Convert ROS Image messages to compressed base64 strings for web transmission
   * @param message The Image message from the camera
   */
  void imageTranslator(const sensor_msgs::msg::Image::SharedPtr message);

  /**
   * @brief Process RealSense 1 color camera images
   * @param message The Image message from the camera
   */
  void rs1ColorTranslator(const sensor_msgs::msg::Image::SharedPtr message);

  /**
   * @brief Process RealSense 1 depth camera images
   * @param message The Image message from the camera
   */
  void rs1DepthTranslator(const sensor_msgs::msg::Image::SharedPtr message);

  /**
   * @brief Process RealSense 2 color camera images
   * @param message The Image message from the camera
   */
  void rs2ColorTranslator(const sensor_msgs::msg::Image::SharedPtr message);

  /**
   * @brief Process RealSense 2 depth camera images
   * @param message The Image message from the camera
   */
  void rs2DepthTranslator(const sensor_msgs::msg::Image::SharedPtr message);

  /**
   * @brief Update and publish diagnostic status with the given level and message
   * @param level The diagnostic level (OK, WARN, ERROR)
   * @param message The diagnostic message
   */
  void setDiagnosticStatus(int level, const std::string & message);

  /**
   * @brief Publish regular heartbeat messages to confirm node is alive
   */
  void heartbeat();

  // Publishers
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr image_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rs1_color_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rs1_depth_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rs2_color_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rs2_depth_publisher_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rs1_color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rs1_depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rs2_color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rs2_depth_subscription_;

  // Timers
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Last frame time tracking for frame rate control
  double last_frame_time_;
  double last_rs1_color_time_;
  double last_rs1_depth_time_;
  double last_rs2_color_time_;
  double last_rs2_depth_time_;

  // Latency tracking for each camera feed
  struct LatencyStats {
    int count;
    double total_latency;
    double avg_latency;
    double max_latency;
    double last_latency;
  };
  std::map<std::string, LatencyStats> latency_stats_;

  // Diagnostic status
  diagnostic_msgs::msg::DiagnosticStatus diagnostic_status_;
};

} // namespace moondawg

#endif // MOONDAWG_CAMERA_NODE_HPP
