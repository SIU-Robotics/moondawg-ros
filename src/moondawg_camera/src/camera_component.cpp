#include "camera_component.hpp"

#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

// OpenCV and cv_bridge includes
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace moondawg
{

CameraComponent::CameraComponent(const rclcpp::NodeOptions & options)
: Node("camera_node", options), last_processed_time_(0.0) // Initialize last_processed_time_
{
  initStateVariables();
  declareParameters();
  setupCommunications();
  RCLCPP_INFO(this->get_logger(), "Camera node initialized and ready. Subscribing to 'image_raw', publishing to 'image_compressed'.");
}

void CameraComponent::initStateVariables()
{
  diagnostic_status_.name = this->get_name();
  diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_status_.message = "Initializing camera node";
  latency_stats_ = {0, 0.0, 0.0, 0.0, 0.0}; // Initialize latency stats
}

void CameraComponent::declareParameters()
{
  // Parameter for intra-process communication
  this->declare_parameter("use_intra_process_comms", true);
  // Parameters moved from ImageCompressionNode
  this->declare_parameter("image_compression_quality", 20);
  this->declare_parameter("max_image_width", 640);
  this->declare_parameter("camera_key", "camera"); // Default key
  
  // New parameters for depth image optimization
  this->declare_parameter("is_depth_camera", false);
  this->declare_parameter("skip_frames", 0);  // Skip frames for depth cameras to reduce processing load
  this->declare_parameter("downsample_before_processing", false);
  this->declare_parameter("use_optimized_encoding", true);
}

void CameraComponent::setupCommunications()
{
  use_intra_process_comms_ = this->get_parameter("use_intra_process_comms").as_bool();
  image_compression_quality_ = this->get_parameter("image_compression_quality").as_int();
  max_image_width_ = this->get_parameter("max_image_width").as_int();
  camera_key_ = this->get_parameter("camera_key").as_string();
  
  // Get optimization parameters
  is_depth_camera_ = this->get_parameter("is_depth_camera").as_bool();
  skip_frames_ = this->get_parameter("skip_frames").as_int();
  downsample_before_processing_ = this->get_parameter("downsample_before_processing").as_bool();
  use_optimized_encoding_ = this->get_parameter("use_optimized_encoding").as_bool();
  frame_count_ = 0;
  
  auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();
  
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable();
  
  diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "~/diag", // Relative topic
    reliable_qos
  );

  // Subscription to raw image topic (input)
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", // Input topic, to be remapped if necessary
    camera_qos,
    std::bind(&CameraComponent::imageCallback, this, std::placeholders::_1)
  );

  // Publisher for compressed image string (output)
  compressed_image_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "image_compressed", // Output topic
    reliable_qos
  );

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&CameraComponent::heartbeat, this)
  );
  
  setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Camera node ready");
}

void CameraComponent::imageCallback(const sensor_msgs::msg::Image::SharedPtr message)
{
  // Apply frame skipping if configured (useful for depth cameras)
  if (skip_frames_ > 0) {
    if (frame_count_++ % (skip_frames_ + 1) != 0) {
      return; // Skip this frame
    }
  }

  try
  {
    std::string encoding = "bgr8";
    bool is_depth_image = false;

    // Detect if this is a depth image
    if (message->encoding.find("mono") != std::string::npos || 
        message->encoding.find("16UC1") != std::string::npos || 
        message->encoding.find("depth") != std::string::npos) {
        encoding = message->encoding;
        is_depth_image = true;
    }

    // For depth images, we can downsample early to reduce processing cost
    if (is_depth_image && downsample_before_processing_ && max_image_width_ > 0 && 
        message->width > static_cast<uint32_t>(max_image_width_)) {
        
        // Calculate downsampling ratio
        double scale = static_cast<double>(max_image_width_) / message->width;
        int new_width = max_image_width_;
        int new_height = static_cast<int>(message->height * scale);
        
        // Create downsampled image
        cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(message, encoding);
        cv::Mat resized;
        cv::resize(cv_image_ptr->image, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_NEAREST);
        
        // Update the image, no need to convert back and forth
        cv_image_ptr->image = resized;
        
        // Process the downsampled image directly
        cv::Mat processed_cv_image;
        
        cv::Mat processed_cv_image;
        if (is_depth_image) {
            // Optimize depth visualization by using a faster colormap
            cv::Mat normalized_image;
            cv::Mat single_channel_image = cv_image_ptr->image;
            
            // Ensure single channel for depth processing
            if (single_channel_image.channels() != 1) {
                 if (single_channel_image.channels() > 1) {
                    std::vector<cv::Mat> channels;
                    cv::split(single_channel_image, channels);
                    single_channel_image = channels[0];
                 } else {
                    RCLCPP_ERROR(this->get_logger(), "Cannot convert multi-channel depth image to single channel for normalization.");
                    return;
                 }
            }
            
            // Use faster normalize with fixed range for depth images
            double min_val = 0;
            double max_val = 10000; // Use a fixed range that works for your sensor
            
            // Convert to 8-bit for colormap with a fixed range (faster than NORM_MINMAX which has to scan the image)
            single_channel_image.convertTo(normalized_image, CV_8U, 255.0 / max_val);
            
            // Apply colormap - COLORMAP_JET is commonly used for depth, but HOT or RAINBOW might be faster
            cv::applyColorMap(normalized_image, processed_cv_image, cv::COLORMAP_HOT);
        } else {
            processed_cv_image = cv_image_ptr->image;
        }
        
        processAndPublishImage(processed_cv_image, image_subscription_->get_topic_name());
        return;
    }

    auto cv_image_ptr = cv_bridge::toCvShare(message, encoding);
    if (!cv_image_ptr) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge::toCvShare failed for encoding: %s", encoding.c_str());
        setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "cv_bridge conversion failed");
        return;
    }
    
    cv::Mat processed_cv_image;
    if (is_depth_image) {
        // Optimize depth image processing
        cv::Mat normalized_image;
        cv::Mat single_channel_image = cv_image_ptr->image;
        
        if (single_channel_image.channels() != 1) {
             if (single_channel_image.channels() > 1) {
                std::vector<cv::Mat> channels;
                cv::split(single_channel_image, channels);
                single_channel_image = channels[0];
             } else {
                RCLCPP_ERROR(this->get_logger(), "Cannot convert multi-channel depth image to single channel for normalization.");
                setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Depth image channel error");
                return;
             }
        }            // Convert directly to 8-bit using a fixed range
        double max_val = 10000; // Adjust this based on your depth camera range
        single_channel_image.convertTo(normalized_image, CV_8U, 255.0 / max_val);
        
        // Use a faster colormap for depth visualization
        cv::applyColorMap(normalized_image, processed_cv_image, cv::COLORMAP_HOT);
    } else {
        processed_cv_image = cv_image_ptr->image;
    }

    processAndPublishImage(processed_cv_image, image_subscription_->get_topic_name());
  }
  catch (const cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "cv_bridge exception");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error in imageCallback: %s", e.what());
    setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Runtime error in callback");
  }
}

bool CameraComponent::processAndPublishImage(const cv::Mat & cv_image, const std::string & input_topic_name)
{
  try
  {
    double current_time = this->now().seconds();
    last_processed_time_ = current_time;
    double processing_start_time = current_time;

    cv::Mat image_to_process = cv_image;
    
    // Only clone if we need to resize
    if (max_image_width_ > 0 && image_to_process.cols > max_image_width_ && !downsample_before_processing_)
    {
      image_to_process = cv_image.clone();
      double aspect_ratio = static_cast<double>(image_to_process.rows) / static_cast<double>(image_to_process.cols);
      int new_width = max_image_width_;
      int new_height = static_cast<int>(new_width * aspect_ratio);
      int interpolation = (cv_image.type() == CV_8UC1 || cv_image.type() == CV_16UC1) ? cv::INTER_NEAREST : cv::INTER_AREA;
      if (cv_image.channels() == 3) interpolation = cv::INTER_AREA; 
      cv::resize(image_to_process, image_to_process, cv::Size(new_width, new_height), 0, 0, interpolation);
    }

    std::vector<uchar> buffer;
    
    // Depth images might benefit from different compression parameters
    std::vector<int> encode_params;
    if (is_depth_camera_) {
      // For depth images, use higher quality to preserve detail
      encode_params = {cv::IMWRITE_JPEG_QUALITY, std::min(image_compression_quality_ + 10, 100)};
    } else {
      encode_params = {cv::IMWRITE_JPEG_QUALITY, image_compression_quality_};
    }
    
    cv::imencode(".jpg", image_to_process, buffer, encode_params);
    std::string mime_type = "image/jpeg";

    // Use a more efficient base64 encoding implementation
    std::string base64_data;
    
    if (use_optimized_encoding_) {
      // More optimized base64 encoding
      static const char base64_chars[] = 
          "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
      
      size_t buffer_size = buffer.size();
      base64_data.reserve(((buffer_size + 2) / 3) * 4); // Pre-allocate memory
      
      size_t i = 0;
      for (; i + 2 < buffer_size; i += 3) {
          uint32_t a = buffer[i];
          uint32_t b = buffer[i+1];
          uint32_t c = buffer[i+2];
          uint32_t triple = (a << 16) | (b << 8) | c;
          
          base64_data.push_back(base64_chars[(triple >> 18) & 0x3F]);
          base64_data.push_back(base64_chars[(triple >> 12) & 0x3F]);
          base64_data.push_back(base64_chars[(triple >> 6) & 0x3F]);
          base64_data.push_back(base64_chars[triple & 0x3F]);
      }
      
      if (i + 1 == buffer_size) {
          uint32_t a = buffer[i];
          uint32_t triple = (a << 16);
          
          base64_data.push_back(base64_chars[(triple >> 18) & 0x3F]);
          base64_data.push_back(base64_chars[(triple >> 12) & 0x3F]);
          base64_data.push_back('=');
          base64_data.push_back('=');
      } 
      else if (i + 2 == buffer_size) {
          uint32_t a = buffer[i];
          uint32_t b = buffer[i+1];
          uint32_t triple = (a << 16) | (b << 8);
          
          base64_data.push_back(base64_chars[(triple >> 18) & 0x3F]);
          base64_data.push_back(base64_chars[(triple >> 12) & 0x3F]);
          base64_data.push_back(base64_chars[(triple >> 6) & 0x3F]);
          base64_data.push_back('=');
      }
    } 
    else {
      // Original base64 encoding implementation
      base64_data.resize(4 * ((buffer.size() + 2) / 3)); 
      
      static const char* base64_chars =
          "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
          "abcdefghijklmnopqrstuvwxyz"
          "0123456789+/";

      size_t i = 0, j = 0;
      unsigned char char_array_3[3];

      for (i = 0; (i + 2) < buffer.size(); i += 3) {
          char_array_3[0] = buffer[i];
          char_array_3[1] = buffer[i+1];
          char_array_3[2] = buffer[i+2];

          base64_data[j++] = base64_chars[(char_array_3[0] & 0xfc) >> 2];
          base64_data[j++] = base64_chars[((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4)];
          base64_data[j++] = base64_chars[((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6)];
          base64_data[j++] = base64_chars[char_array_3[2] & 0x3f];
      }

      if (i < buffer.size()) {
          char_array_3[0] = buffer[i];
          char_array_3[1] = (i + 1 < buffer.size()) ? buffer[i+1] : 0;

          base64_data[j++] = base64_chars[(char_array_3[0] & 0xfc) >> 2];
          base64_data[j++] = base64_chars[((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4)];
          if (i + 1 < buffer.size()) {
              base64_data[j++] = base64_chars[((char_array_3[1] & 0x0f) << 2)];
          } else {
              base64_data[j++] = '=';
          }
          base64_data[j++] = '=';
      }
      base64_data.resize(j);
    }

    auto msg = std_msgs::msg::String();
    msg.data = mime_type + "," + base64_data;
    compressed_image_publisher_->publish(msg);

    double processing_end_time = this->now().seconds();
    double latency = processing_end_time - processing_start_time;
    
    latency_stats_.count++;
    latency_stats_.total_latency += latency;
    latency_stats_.avg_latency = latency_stats_.total_latency / latency_stats_.count;
    latency_stats_.max_latency = std::max(latency_stats_.max_latency, latency);
    latency_stats_.last_latency = latency;

    setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Processing images");
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error processing and publishing image from topic %s: %s", input_topic_name.c_str(), e.what());
    setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error in processing/publishing");
    return false;
  }
}

void CameraComponent::heartbeat()
{
  diagnostic_status_.values.clear(); 
  // Add latency stats to diagnostics
  diagnostic_msgs::msg::KeyValue kv_last_latency;
  kv_last_latency.key = camera_key_ + "_last_latency_ms";
  kv_last_latency.value = std::to_string(latency_stats_.last_latency * 1000.0);
  diagnostic_status_.values.push_back(kv_last_latency);

  diagnostic_msgs::msg::KeyValue kv_avg_latency;
  kv_avg_latency.key = camera_key_ + "_avg_latency_ms";
  kv_avg_latency.value = std::to_string(latency_stats_.avg_latency * 1000.0);
  diagnostic_status_.values.push_back(kv_avg_latency);
  
  diagnostic_msgs::msg::KeyValue kv_max_latency;
  kv_max_latency.key = camera_key_ + "_max_latency_ms";
  kv_max_latency.value = std::to_string(latency_stats_.max_latency * 1000.0);
  diagnostic_status_.values.push_back(kv_max_latency);

  diagnostic_msgs::msg::KeyValue kv_processed_count;
  kv_processed_count.key = camera_key_ + "_processed_frames";
  kv_processed_count.value = std::to_string(latency_stats_.count);
  diagnostic_status_.values.push_back(kv_processed_count);

  diag_publisher_->publish(diagnostic_status_);
}

void CameraComponent::setDiagnosticStatus(uint8_t level, const std::string& message)
{
  diagnostic_status_.level = level;
  diagnostic_status_.message = message;
  if (level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
    // Publish immediately if it's an error or warning
    diag_publisher_->publish(diagnostic_status_);
  }
}

} // namespace moondawg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moondawg::CameraComponent)
