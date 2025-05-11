#ifndef MOONDAWG_IMAGE_COMPRESSION_NODE_HPP
#define MOONDAWG_IMAGE_COMPRESSION_NODE_HPP

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace moondawg
{

class ImageCompressionNode : public rclcpp::Node
{
public:
  explicit ImageCompressionNode(const rclcpp::NodeOptions & options);
  virtual ~ImageCompressionNode() = default;

private:
  void declareParameters();
  void setupCommunications();
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr message);
  bool processAndPublishImage(const cv::Mat & cv_image, const std::string & input_topic_name);
  void heartbeat();
  void setDiagnosticStatus(uint8_t level, const std::string& message);


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_publisher_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;


  // Parameters
  int image_compression_quality_;
  int image_frame_rate_;
  int max_image_width_;
  std::string camera_key_; // To identify the camera for logging/diagnostics

  // Frame rate control
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

#endif // MOONDAWG_IMAGE_COMPRESSION_NODE_HPP
