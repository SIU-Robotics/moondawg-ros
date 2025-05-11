#include "image_compression_node.hpp"

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

ImageCompressionNode::ImageCompressionNode(const rclcpp::NodeOptions & options)
: Node("image_compression_node", options), last_processed_time_(0.0)
{
  declareParameters();
  setupCommunications();
  
  diagnostic_status_.name = this->get_name();
  diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_status_.message = "Initializing image compression node";
  latency_stats_ = {0, 0.0, 0.0, 0.0, 0.0};


  RCLCPP_INFO(this->get_logger(), "ImageCompressionNode initialized and ready for topic: %s", this->get_effective_namespace().c_str());
}

void ImageCompressionNode::declareParameters()
{
  this->declare_parameter("image_compression_quality", 20);
  this->declare_parameter("image_frame_rate", 15);
  this->declare_parameter("max_image_width", 640);
  this->declare_parameter("camera_key", "compressed_camera"); // Default key
}

void ImageCompressionNode::setupCommunications()
{
  image_compression_quality_ = this->get_parameter("image_compression_quality").as_int();
  image_frame_rate_ = this->get_parameter("image_frame_rate").as_int();
  max_image_width_ = this->get_parameter("max_image_width").as_int();
  camera_key_ = this->get_parameter("camera_key").as_string();

  auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // Subscription to raw image topic (input)
  // The topic name "image_raw" is a common convention, remapped in the launch file
  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", // Input topic, to be remapped
    camera_qos,
    std::bind(&ImageCompressionNode::imageCallback, this, std::placeholders::_1)
  );

  // Publisher for compressed image string (output)
  // The topic name "image_compressed" is a common convention, can be remapped
  publisher_ = this->create_publisher<std_msgs::msg::String>(
    "image_compressed", // Output topic
    reliable_qos
  );

  diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "~/diag", // Diagnostic topic relative to node's namespace
    reliable_qos
  );

  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ImageCompressionNode::heartbeat, this)
  );
  setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Image compression node ready");
}

void ImageCompressionNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr message)
{
  try
  {
    // Determine encoding - bgr8 for color, or keep original for depth/mono
    std::string encoding = "bgr8";
    if (message->encoding.find("mono") != std::string::npos || message->encoding.find("16UC1") != std::string::npos || message->encoding.find("depth") != std::string::npos) {
        encoding = message->encoding; // Use original encoding for mono/depth
    }

    auto cv_image_ptr = cv_bridge::toCvShare(message, encoding);
    if (!cv_image_ptr) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge::toCvShare failed for encoding: %s", encoding.c_str());
        setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "cv_bridge conversion failed");
        return;
    }
    
    // For depth images (e.g., 16UC1), normalize and apply colormap before compression
    cv::Mat processed_cv_image;
    if (message->encoding == "16UC1" || message->encoding.find("depth") != std::string::npos) {
        cv::Mat normalized_image;
        // Ensure it's single channel before normalizing
        cv::Mat single_channel_image = cv_image_ptr->image;
        if (single_channel_image.channels() != 1) {
             RCLCPP_WARN(this->get_logger(), "Depth image has %d channels, expected 1. Trying to convert.", single_channel_image.channels());
             // Attempt to convert to single channel if it's not, e.g. take the first channel
             // This might not be correct for all formats, but a common case for depth.
             if (single_channel_image.channels() > 1) {
                std::vector<cv::Mat> channels;
                cv::split(single_channel_image, channels);
                single_channel_image = channels[0];
             } else {
                RCLCPP_ERROR(this->get_logger(), "Cannot convert multi-channel depth image to single channel for normalization.");
                setDiagnosticStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Depth image channel error");
                return;
             }
        }
        cv::normalize(single_channel_image, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(normalized_image, processed_cv_image, cv::COLORMAP_JET);
    } else {
        processed_cv_image = cv_image_ptr->image;
    }

    processAndPublishImage(processed_cv_image, subscription_->get_topic_name());
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

bool ImageCompressionNode::processAndPublishImage(const cv::Mat & cv_image, const std::string & input_topic_name)
{
  try
  {
    double current_time = this->now().seconds();
    if (image_frame_rate_ > 0 && (current_time - last_processed_time_) < (1.0 / image_frame_rate_))
    {
      return false; // Skip frame
    }
    last_processed_time_ = current_time;
    double processing_start_time = current_time;

    cv::Mat image_to_process = cv_image.clone();

    if (max_image_width_ > 0 && image_to_process.cols > max_image_width_)
    {
      double aspect_ratio = static_cast<double>(image_to_process.rows) / static_cast<double>(image_to_process.cols);
      int new_width = max_image_width_;
      int new_height = static_cast<int>(new_width * aspect_ratio);
      // Use INTER_NEAREST for depth images (after colormap) or INTER_AREA for color images
      int interpolation = (cv_image.type() == CV_8UC1 || cv_image.type() == CV_16UC1) ? cv::INTER_NEAREST : cv::INTER_AREA;
      if (cv_image.channels() == 3) interpolation = cv::INTER_AREA; // If it became 3 channels (e.g. colormapped depth)
      cv::resize(image_to_process, image_to_process, cv::Size(new_width, new_height), 0, 0, interpolation);
    }

    std::vector<uchar> buffer;
    std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, image_compression_quality_};
    cv::imencode(".jpg", image_to_process, buffer, encode_params);
    std::string mime_type = "image/jpeg";

    std::string base64_data;
    base64_data.resize(4 * ((buffer.size() + 2) / 3)); // Pre-allocate memory

    static const char* base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    size_t i = 0, j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

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
        // char_array_3[2] is not used here

        base64_data[j++] = base64_chars[(char_array_3[0] & 0xfc) >> 2];
        base64_data[j++] = base64_chars[((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4)];
        if (i + 1 < buffer.size()) {
            base64_data[j++] = base64_chars[((char_array_3[1] & 0x0f) << 2)];
        } else {
            base64_data[j++] = '=';
        }
        base64_data[j++] = '=';
    }
    base64_data.resize(j); // Resize to actual used size


    auto msg = std_msgs::msg::String();
    msg.data = mime_type + "," + base64_data;
    publisher_->publish(msg);

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

void ImageCompressionNode::heartbeat() {
    diagnostic_status_.values.clear();
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

void ImageCompressionNode::setDiagnosticStatus(uint8_t level, const std::string& message) {
    diagnostic_status_.level = level;
    diagnostic_status_.message = message;
    // Publish immediately if it's an error or warning
    if (level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        diag_publisher_->publish(diagnostic_status_);
    }
}

} // namespace moondawg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moondawg::ImageCompressionNode)
