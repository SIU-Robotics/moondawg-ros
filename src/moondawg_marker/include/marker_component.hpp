#pragma once
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_geometry/pinhole_camera_model.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <opencv2/opencv.hpp>

namespace moondawg
{

struct DetectedMarker
{
  cv::Point2f pixel_position;
  cv::Point3f world_position;
  float area;
  int cluster_id;
};

struct ClusterInfo
{
  std::vector<DetectedMarker> points;
  cv::Point2f centroid;
  float total_area;
};

class MarkerComponent : public rclcpp::Node
{
public:
  explicit MarkerComponent(const rclcpp::NodeOptions & options);
  virtual ~MarkerComponent() = default;

private:
  void declareParameters();
  void setupCommunications();
  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr message);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr message);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr message);
  
  cv::Mat detectOrangeMarkers(const cv::Mat & rgb_image);
  // FIX: removed unused rgb_image parameter
  std::vector<ClusterInfo> clusterDetections(const cv::Mat & binary_image);
  std::vector<DetectedMarker> extract3DPositions(const std::vector<ClusterInfo> & clusters);
  
  geometry_msgs::msg::PoseArray computeDumpZone(const std::vector<DetectedMarker> & markers);
  geometry_msgs::msg::Pose computeMarkerPose(const DetectedMarker & marker);
  cv::Point3f pixelTo3D(float u, float v, float depth);
  
  cv::Point2f computeRectangleCenter(const std::vector<cv::Point2f> & points);
  float computeRectangleOrientation(const std::vector<cv::Point2f> & points, const cv::Point2f & center);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
  
  image_geometry::PinholeCameraModel camera_model_;
  bool camera_info_received_;
  bool depth_synchronized_;
  
  cv::Mat latest_depth_;
  std::mutex depth_mutex_;
  
  int hsv_h_min_, hsv_h_max_, hsv_s_min_, hsv_s_max_, hsv_v_min_, hsv_v_max_;
  int min_marker_area_;
  int cluster_distance_threshold_;
  int expected_marker_count_;
  std::string target_frame_;
  std::string camera_topic_prefix_;
  bool enable_debug_output_;
  int median_filter_size_;

  // FIX: new shape filtering parameters
  double min_circularity_;
  double min_aspect_ratio_;
  double max_aspect_ratio_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace moondawg