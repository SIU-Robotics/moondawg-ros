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
  // Initialize state variables
  initStateVariables();
  
  // Declare ROS parameters
  declareParameters();
  
  // Set up publishers, subscribers, and timers
  setupCommunications();
  
  RCLCPP_INFO(this->get_logger(), "Camera node initialized and ready");
}

void CameraNode::initStateVariables()
{
  // Last frame time tracking for frame rate control
  last_frame_time_ = 0.0;
  last_rs1_color_time_ = 0.0;
  last_rs1_depth_time_ = 0.0;
  last_rs2_color_time_ = 0.0;
  last_rs2_depth_time_ = 0.0;
  
  // Initialize latency tracking for each camera feed
  latency_stats_ = {
    {"main_camera", {0, 0.0, 0.0, 0.0, 0.0}},
    {"rs1_color", {0, 0.0, 0.0, 0.0, 0.0}},
    {"rs1_depth", {0, 0.0, 0.0, 0.0, 0.0}},
    {"rs2_color", {0, 0.0, 0.0, 0.0, 0.0}},
    {"rs2_depth", {0, 0.0, 0.0, 0.0, 0.0}}
  };
  
  // Initialize diagnostic status
  diagnostic_status_.name = this->get_name();
  diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_status_.message = "Initializing camera node";
}

void CameraNode::declareParameters()
{
  // Image processing parameters
  this->declare_parameter("image_compression_quality", 20);
  this->declare_parameter("image_frame_rate", 15);  // Parameter for frame rate control
  this->declare_parameter("max_image_width", 640);  // Maximum width for images, aspect ratio maintained
  this->declare_parameter("use_intra_process_comms", true);  // Parameter for intra-process communication
}

void CameraNode::setupCommunications()
{
  // Get intra-process communication setting
  bool use_ipc = this->get_parameter("use_intra_process_comms").as_bool();
  
  // Define QoS profile optimized for camera streaming with intra-process communication
  // For image data, we want to prioritize latest data and low latency
  auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()        // For camera feeds, we prefer getting the latest frame
    .durability_volatile(); // No need to store past messages
  
  // Standard QoS for diagnostic data
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable();
  
  // Diagnostic publisher
  diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "/camera_node/diag", 
    reliable_qos
  );
  
  // Image publishers for Web UI
  image_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/camera_node/compressed_image", 
    reliable_qos
  );
  rs1_color_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/camera_node/rs1_color_image", 
    reliable_qos
  );
  rs1_depth_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/camera_node/rs1_depth_image", 
    reliable_qos
  );
  rs2_color_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/camera_node/rs2_color_image", 
    reliable_qos
  );
  rs2_depth_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/camera_node/rs2_depth_image", 
    reliable_qos
  );
  
  // Camera image subscriptions
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image", 
    camera_qos, 
    std::bind(&CameraNode::imageTranslator, this, std::placeholders::_1)
  );
  
  // RealSense camera subscriptions
  rs1_color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera1/color/image_raw", 
    camera_qos, 
    std::bind(&CameraNode::rs1ColorTranslator, this, std::placeholders::_1)
  );
  rs1_depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera1/depth/image_rect_raw", 
    camera_qos, 
    std::bind(&CameraNode::rs1DepthTranslator, this, std::placeholders::_1)
  );
  rs2_color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera2/color/image_raw", 
    camera_qos, 
    std::bind(&CameraNode::rs2ColorTranslator, this, std::placeholders::_1)
  );
  rs2_depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera2/depth/image_rect_raw", 
    camera_qos, 
    std::bind(&CameraNode::rs2DepthTranslator, this, std::placeholders::_1)
  );
  
  // Heartbeat timer
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&CameraNode::heartbeat, this)
  );
  
  // Set initial diagnostic status
  setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera node ready");
}

bool CameraNode::processImage(
  const cv::Mat & cv_image,
  double & last_time_ref,
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
  bool is_depth,
  const std::string & camera_key)
{
  try {
    // Start processing time for latency measurement
    double processing_start_time = this->now().seconds();
    
    // Check frame rate control - only process every nth frame
    int frame_rate = this->get_parameter("image_frame_rate").as_int();
    
    // Skip frames based on frame rate setting (basic throttling)
    if (last_time_ref > 0.0 && (processing_start_time - last_time_ref) < (1.0 / frame_rate)) {
      return false;
    }
    
    last_time_ref = processing_start_time;
    
    // Create a copy of the input image to avoid modifying the original
    cv::Mat processed_image = cv_image.clone();
    
    // Resize the image to reduce CPU usage
    int max_width = this->get_parameter("max_image_width").as_int();
    if (max_width > 0 && processed_image.cols > max_width) {
      // Calculate new dimensions maintaining aspect ratio
      double aspect_ratio = static_cast<double>(processed_image.rows) / static_cast<double>(processed_image.cols);
      int new_width = max_width;
      int new_height = static_cast<int>(new_width * aspect_ratio);
      // Use INTER_NEAREST for depth images or INTER_AREA for color images (best for downsampling)
      int interpolation = is_depth ? cv::INTER_NEAREST : cv::INTER_AREA;
      cv::resize(processed_image, processed_image, cv::Size(new_width, new_height), 0, 0, interpolation);
    }
    
    // Use a fixed quality value
    int quality = this->get_parameter("image_compression_quality").as_int();
    
    // Always use JPEG encoding
    std::vector<uchar> buffer;
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, quality};
    cv::imencode(".jpg", processed_image, buffer, encode_params);
    std::string mime_type = "image/jpeg";
    
    // Convert to base64 and publish
    std::string base64_data;
    base64_data.resize(4 * ((buffer.size() + 2) / 3));
    
    // Define the base64 character set
    static const char* base64_chars = 
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    size_t i = 0, j = 0;
    for (; i + 2 < buffer.size(); i += 3) {
        uint32_t triplet = (buffer[i] << 16) | (buffer[i + 1] << 8) | buffer[i + 2];
        base64_data[j++] = base64_chars[(triplet >> 18) & 0x3F];
        base64_data[j++] = base64_chars[(triplet >> 12) & 0x3F];
        base64_data[j++] = base64_chars[(triplet >> 6) & 0x3F];
        base64_data[j++] = base64_chars[triplet & 0x3F];
    }
    
    // Handle the remaining bytes
    if (i < buffer.size()) {
        uint32_t triplet = buffer[i] << 16;
        if (i + 1 < buffer.size()) triplet |= buffer[i + 1] << 8;
        
        base64_data[j++] = base64_chars[(triplet >> 18) & 0x3F];
        base64_data[j++] = base64_chars[(triplet >> 12) & 0x3F];
        
        if (i + 1 < buffer.size())
            base64_data[j++] = base64_chars[(triplet >> 6) & 0x3F];
        else
            base64_data[j++] = '=';
        
        base64_data[j++] = '=';
    }
    
    base64_data.resize(j);
    
    auto data_with_mime = mime_type + "," + base64_data;
    auto msg = std_msgs::msg::String();
    msg.data = data_with_mime;
    publisher->publish(msg);
    
    // Calculate and store latency information
    if (!camera_key.empty() && latency_stats_.find(camera_key) != latency_stats_.end()) {
      double processing_end_time = this->now().seconds();
      double latency = processing_end_time - processing_start_time;
      
      // Update latency statistics
      latency_stats_[camera_key].count++;
      latency_stats_[camera_key].total_latency += latency;
      latency_stats_[camera_key].avg_latency = 
        latency_stats_[camera_key].total_latency / latency_stats_[camera_key].count;
      latency_stats_[camera_key].max_latency = std::max(
        latency_stats_[camera_key].max_latency, 
        latency
      );
      latency_stats_[camera_key].last_latency = latency;
    }
    
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
    return false;
  }
}

void CameraNode::imageTranslator(const sensor_msgs::msg::Image::SharedPtr message)
{
  try {
    // Convert ROS Image to OpenCV image
    auto cv_image = cv_bridge::toCvShare(message, "bgr8")->image;
    processImage(cv_image, last_frame_time_, image_publisher_, false, "main_camera");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing camera image: %s", e.what());
  }
}

void CameraNode::rs1ColorTranslator(const sensor_msgs::msg::Image::SharedPtr message)
{
  try {
    auto cv_image = cv_bridge::toCvShare(message, "bgr8")->image;
    processImage(cv_image, last_rs1_color_time_, rs1_color_publisher_, false, "rs1_color");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing RealSense1 color image: %s", e.what());
  }
}

void CameraNode::rs1DepthTranslator(const sensor_msgs::msg::Image::SharedPtr message)
{
  try {
    auto cv_image = cv_bridge::toCvShare(message)->image;
    // Normalize depth values for better visualization (16-bit to 8-bit conversion)
    cv::Mat normalized_image;
    cv::normalize(cv_image, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8U);
    // Apply a colormap for better visualization
    cv::Mat colormap_image;
    cv::applyColorMap(normalized_image, colormap_image, cv::COLORMAP_JET);
    
    processImage(colormap_image, last_rs1_depth_time_, rs1_depth_publisher_, true, "rs1_depth");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing RealSense1 depth image: %s", e.what());
  }
}

void CameraNode::rs2ColorTranslator(const sensor_msgs::msg::Image::SharedPtr message)
{
  try {
    auto cv_image = cv_bridge::toCvShare(message, "bgr8")->image;
    processImage(cv_image, last_rs2_color_time_, rs2_color_publisher_, false, "rs2_color");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing RealSense2 color image: %s", e.what());
  }
}

void CameraNode::rs2DepthTranslator(const sensor_msgs::msg::Image::SharedPtr message)
{
  try {
    auto cv_image = cv_bridge::toCvShare(message)->image;
    // Normalize depth values for better visualization (16-bit to 8-bit conversion)
    cv::Mat normalized_image;
    cv::normalize(cv_image, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8U);
    // Apply a colormap for better visualization
    cv::Mat colormap_image;
    cv::applyColorMap(normalized_image, colormap_image, cv::COLORMAP_JET);
    
    processImage(colormap_image, last_rs2_depth_time_, rs2_depth_publisher_, true, "rs2_depth");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing RealSense2 depth image: %s", e.what());
  }
}

void CameraNode::setDiagnosticStatus(int level, const std::string & message)
{
  diagnostic_status_.level = level;
  diagnostic_status_.message = message;
  
  // Create latency values for diagnostics
  std::vector<diagnostic_msgs::msg::KeyValue> values;
  
  diagnostic_msgs::msg::KeyValue node_name_kv;
  node_name_kv.key = "node_name";
  node_name_kv.value = this->get_name();
  values.push_back(node_name_kv);
  
  diagnostic_msgs::msg::KeyValue timestamp_kv;
  timestamp_kv.key = "timestamp";
  timestamp_kv.value = std::to_string(this->now().seconds());
  values.push_back(timestamp_kv);
  
  // Add latency information for each camera
  for (const auto & [camera_key, stats] : latency_stats_) {
    if (stats.count > 0) {
      // Format to 2 decimal places for readability (convert to ms for easier reading)
      double last_latency_ms = stats.last_latency * 1000;
      double avg_latency_ms = stats.avg_latency * 1000;
      double max_latency_ms = stats.max_latency * 1000;
      
      diagnostic_msgs::msg::KeyValue frames_kv;
      frames_kv.key = camera_key + "_frames";
      frames_kv.value = std::to_string(stats.count);
      values.push_back(frames_kv);
      
      diagnostic_msgs::msg::KeyValue last_latency_kv;
      last_latency_kv.key = camera_key + "_last_latency_ms";
      std::stringstream ss_last;
      ss_last << std::fixed << std::setprecision(2) << last_latency_ms;
      last_latency_kv.value = ss_last.str();
      values.push_back(last_latency_kv);
      
      diagnostic_msgs::msg::KeyValue avg_latency_kv;
      avg_latency_kv.key = camera_key + "_avg_latency_ms";
      std::stringstream ss_avg;
      ss_avg << std::fixed << std::setprecision(2) << avg_latency_ms;
      avg_latency_kv.value = ss_avg.str();
      values.push_back(avg_latency_kv);
      
      diagnostic_msgs::msg::KeyValue max_latency_kv;
      max_latency_kv.key = camera_key + "_max_latency_ms";
      std::stringstream ss_max;
      ss_max << std::fixed << std::setprecision(2) << max_latency_ms;
      max_latency_kv.value = ss_max.str();
      values.push_back(max_latency_kv);
    }
  }
  
  diagnostic_status_.values = values;
  diag_publisher_->publish(diagnostic_status_);
}

void CameraNode::heartbeat()
{
  // Create a summary of latency for all cameras
  std::string latency_summary = "";
  int active_cameras = 0;
  
  for (const auto & [camera_key, stats] : latency_stats_) {
    if (stats.count > 0) {
      active_cameras++;
      // Convert to ms for more readable values
      double last_latency_ms = stats.last_latency * 1000;
      double avg_latency_ms = stats.avg_latency * 1000;
      
      // Add to the summary
      std::stringstream ss;
      ss << camera_key << ": " << std::fixed << std::setprecision(1) 
         << last_latency_ms << "ms (avg: " << avg_latency_ms << "ms) | ";
      latency_summary += ss.str();
    }
  }
  
  // Trim the trailing separator
  if (!latency_summary.empty()) {
    latency_summary = latency_summary.substr(0, latency_summary.length() - 3);
  }
  
  // Set the message based on active cameras
  std::string message;
  if (active_cameras == 0) {
    message = "Camera node heartbeat - No active cameras";
  } else {
    message = "Camera node heartbeat - Latency: " + latency_summary;
  }
  
  setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, message);
}

} // namespace moondawg

// Register the component with the component manager
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moondawg::CameraNode)
