#include "marker_component.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
 
namespace moondawg
{
 
MarkerComponent::MarkerComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("marker_detector", options),
  camera_info_received_(false),
  depth_synchronized_(false)
{
  declareParameters();
  setupCommunications();
  
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  RCLCPP_INFO(this->get_logger(), "Marker detector initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to RGB: %s/color/image_raw", camera_topic_prefix_.c_str());
  RCLCPP_INFO(this->get_logger(), "Subscribing to Depth: %s/depth/image_rect_raw", camera_topic_prefix_.c_str());
}
 
void MarkerComponent::declareParameters()
{
  hsv_h_min_ = this->declare_parameter("hsv_h_min", 5);
  hsv_h_max_ = this->declare_parameter("hsv_h_max", 25);
  hsv_s_min_ = this->declare_parameter("hsv_s_min", 100);
  hsv_s_max_ = this->declare_parameter("hsv_s_max", 255);
  hsv_v_min_ = this->declare_parameter("hsv_v_min", 100);
  hsv_v_max_ = this->declare_parameter("hsv_v_max", 255);
  
  min_marker_area_ = this->declare_parameter("min_marker_area", 100);
  cluster_distance_threshold_ = this->declare_parameter("cluster_distance_threshold", 50);
  expected_marker_count_ = this->declare_parameter("expected_marker_count", 4);
  camera_topic_prefix_ = this->declare_parameter("camera_topic_prefix", "/realsense/camera1");
  target_frame_ = this->declare_parameter("target_frame", "map");
  enable_debug_output_ = this->declare_parameter("enable_debug_output", true);
  median_filter_size_ = this->declare_parameter("median_filter_size", 5);
 
  min_circularity_ = this->declare_parameter("min_circularity", 0.4);
  min_aspect_ratio_ = this->declare_parameter("min_aspect_ratio", 0.5);
  max_aspect_ratio_ = this->declare_parameter("max_aspect_ratio", 2.0);
}
 
void MarkerComponent::setupCommunications()
{
  // Image topics: RELIABLE + VOLATILE matches RealSense image publishers
  rclcpp::QoS image_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();
 
  // Camera info: RELIABLE + TRANSIENT_LOCAL matches RealSense camera_info publisher
  rclcpp::QoS info_qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliable()
    .durability_volatile();
 
  auto rgb_topic   = camera_topic_prefix_ + "/color/image_raw";
  auto depth_topic = camera_topic_prefix_ + "/depth/image_rect_raw";
  auto info_topic  = camera_topic_prefix_ + "/color/camera_info";
 
  rgb_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    rgb_topic, image_qos,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { rgbCallback(msg); });
 
  depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic, image_qos,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) { depthCallback(msg); });
 
  camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    info_topic, info_qos,
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });
 
  detections_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "/marker/detections", 10);
 
  if (enable_debug_output_)
  {
    debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/marker/debug_image", 1);
  }
}
 
void MarkerComponent::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr message)
{
  if (!camera_info_received_)
  {
    camera_model_.fromCameraInfo(message);
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received and model initialized");
  }
}
 
void MarkerComponent::depthCallback(const sensor_msgs::msg::Image::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  
  try
  {
    auto encoding = message->encoding;
    cv::Mat depth_frame;
    
    if (encoding == "16UC1")
    {
      depth_frame = cv_bridge::toCvCopy(message, "16UC1")->image;
    }
    else if (encoding == "32FC1")
    {
      depth_frame = cv_bridge::toCvCopy(message, "32FC1")->image;
      depth_frame *= 1000.0f;
      depth_frame.convertTo(depth_frame, CV_16UC1);
    }
    else
    {
      depth_frame = cv_bridge::toCvCopy(message, encoding)->image;
    }
    
    if (median_filter_size_ > 1 && median_filter_size_ % 2 == 1)
    {
      cv::medianBlur(depth_frame, latest_depth_, median_filter_size_);
    }
    else
    {
      latest_depth_ = depth_frame.clone();
    }
    
    depth_synchronized_ = true;
  }
  catch (const cv_bridge::Exception & e)
  {
    RCLCPP_WARN(this->get_logger(), "Depth cv_bridge exception: %s", e.what());
  }
}
 
void MarkerComponent::rgbCallback(const sensor_msgs::msg::Image::SharedPtr message)
{
  if (!camera_info_received_)
  {
    return;
  }
  
  try
  {
    auto cv_image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat binary_mask = detectOrangeMarkers(cv_image->image);
    auto clusters = clusterDetections(binary_mask);
    auto markers_3d = extract3DPositions(clusters);
    auto pose_array = computeDumpZone(markers_3d);
    detections_publisher_->publish(pose_array);
    
    if (enable_debug_output_ && debug_image_publisher_)
    {
      cv::Mat debug_img = cv_image->image.clone();
      
      for (const auto & marker : markers_3d)
      {
        cv::circle(debug_img, marker.pixel_position, 10, cv::Scalar(0, 255, 0), 2);
        cv::putText(debug_img,
          "(" + std::to_string((int)marker.world_position.x) + "," +
          std::to_string((int)marker.world_position.y) + "," +
          std::to_string((int)marker.world_position.z) + ")",
          marker.pixel_position + cv::Point2f(15, -10),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      }
      
      cv::cvtColor(binary_mask * 255, binary_mask, cv::COLOR_GRAY2BGR);
      cv::addWeighted(debug_img, 0.7, binary_mask, 0.3, 0, debug_img);
      
      auto debug_msg = cv_bridge::CvImage(message->header, "bgr8", debug_img).toImageMsg();
      debug_image_publisher_->publish(*debug_msg);
    }
  }
  catch (const cv_bridge::Exception & e)
  {
    RCLCPP_WARN(this->get_logger(), "RGB cv_bridge exception: %s", e.what());
  }
}
 
cv::Mat MarkerComponent::detectOrangeMarkers(const cv::Mat & rgb_image)
{
  cv::Mat hsv_image;
  cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);
  
  cv::Mat mask;
  cv::inRange(hsv_image,
    cv::Scalar(hsv_h_min_, hsv_s_min_, hsv_v_min_),
    cv::Scalar(hsv_h_max_, hsv_s_max_, hsv_v_max_),
    mask);
 
  cv::Mat mask_low, mask_high;
  cv::inRange(hsv_image,
    cv::Scalar(0, hsv_s_min_, hsv_v_min_),
    cv::Scalar(5, hsv_s_max_, hsv_v_max_),
    mask_low);
  cv::inRange(hsv_image,
    cv::Scalar(170, hsv_s_min_, hsv_v_min_),
    cv::Scalar(180, hsv_s_max_, hsv_v_max_),
    mask_high);
  cv::bitwise_or(mask, mask_low, mask);
  cv::bitwise_or(mask, mask_high, mask);
  
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
  
  return mask;
}
 
std::vector<ClusterInfo> MarkerComponent::clusterDetections(const cv::Mat & binary_image)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  std::vector<ClusterInfo> clusters;
  
  for (const auto & contour : contours)
  {
    float area = cv::contourArea(contour);
    if (area < min_marker_area_)
    {
      continue;
    }
 
    cv::Rect bbox = cv::boundingRect(contour);
    float aspect_ratio = (float)bbox.width / (float)bbox.height;
    if (aspect_ratio < min_aspect_ratio_ || aspect_ratio > max_aspect_ratio_)
    {
      RCLCPP_DEBUG(this->get_logger(), "Rejected blob: aspect ratio %.2f", aspect_ratio);
      continue;
    }
 
    float perimeter = cv::arcLength(contour, true);
    if (perimeter > 0.0f)
    {
      float circularity = 4.0f * M_PI * area / (perimeter * perimeter);
      if (circularity < min_circularity_)
      {
        RCLCPP_DEBUG(this->get_logger(), "Rejected blob: circularity %.2f", circularity);
        continue;
      }
    }
    
    ClusterInfo cluster;
    cluster.total_area = area;
    
    cv::Moments moments = cv::moments(contour);
    cluster.centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
    
    DetectedMarker marker;
    marker.pixel_position = cluster.centroid;
    marker.area = area;
    marker.cluster_id = clusters.size();
    cluster.points.push_back(marker);
    
    clusters.push_back(cluster);
  }
  
  std::sort(clusters.begin(), clusters.end(),
    [](const ClusterInfo & a, const ClusterInfo & b) {
      return a.total_area > b.total_area;
    });
  
  while (clusters.size() > static_cast<size_t>(expected_marker_count_))
  {
    clusters.pop_back();
  }
  
  return clusters;
}
 
std::vector<DetectedMarker> MarkerComponent::extract3DPositions(const std::vector<ClusterInfo> & clusters)
{
  std::lock_guard<std::mutex> lock(depth_mutex_);
  
  if (!depth_synchronized_ || latest_depth_.empty())
  {
    return {};
  }
  
  std::vector<DetectedMarker> markers_3d;
  
  for (const auto & cluster : clusters)
  {
    int u = static_cast<int>(cluster.centroid.x);
    int v = static_cast<int>(cluster.centroid.y);
    
    if (u < 0 || u >= latest_depth_.cols || v < 0 || v >= latest_depth_.rows)
    {
      continue;
    }
    
    float depth_mm = latest_depth_.at<uint16_t>(v, u);
    
    if (depth_mm < 100 || depth_mm > 10000)
    {
      continue;
    }
    
    float depth_m = depth_mm / 1000.0f;
    
    DetectedMarker marker_3d;
    marker_3d.pixel_position = cluster.centroid;
    marker_3d.world_position = pixelTo3D(static_cast<float>(u), static_cast<float>(v), depth_m);
    marker_3d.area = cluster.total_area;
    marker_3d.cluster_id = cluster.points[0].cluster_id;
    
    markers_3d.push_back(marker_3d);
  }
  
  return markers_3d;
}
 
cv::Point3f MarkerComponent::pixelTo3D(float u, float v, float depth)
{
  cv::Point3f point;
  
  if (camera_model_.initialized())
  {
    cv::Point2d rectified_uv(u, v);
    cv::Point3d ray = camera_model_.projectPixelTo3dRay(rectified_uv);
    point.x = ray.x * depth;
    point.y = ray.y * depth;
    point.z = depth;
  }
  else
  {
    const double fx = 615.0;
    const double fy = 615.0;
    const double cx = 320.0;
    const double cy = 240.0;
    point.x = (u - cx) * depth / fx;
    point.y = (v - cy) * depth / fy;
    point.z = depth;
  }
  
  return point;
}
 
geometry_msgs::msg::PoseArray MarkerComponent::computeDumpZone(const std::vector<DetectedMarker> & markers)
{
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = this->get_clock()->now();
  pose_array.header.frame_id = target_frame_;
  
  for (const auto & marker : markers)
  {
    pose_array.poses.push_back(computeMarkerPose(marker));
  }
  
  if (markers.size() >= 2)
  {
    std::vector<cv::Point2f> points_2d;
    for (const auto & m : markers)
    {
      points_2d.push_back(cv::Point2f(m.world_position.x, m.world_position.y));
    }
    
    cv::Point2f center_2d = computeRectangleCenter(points_2d);
    float orientation = computeRectangleOrientation(points_2d, center_2d);
    
    geometry_msgs::msg::Pose center_pose;
    center_pose.position.x = center_2d.x;
    center_pose.position.y = center_2d.y;
    center_pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, orientation);
    center_pose.orientation.x = q.x();
    center_pose.orientation.y = q.y();
    center_pose.orientation.z = q.z();
    center_pose.orientation.w = q.w();
    
    pose_array.poses.push_back(center_pose);
  }
  
  return pose_array;
}
 
geometry_msgs::msg::Pose MarkerComponent::computeMarkerPose(const DetectedMarker & marker)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = marker.world_position.x;
  pose.position.y = marker.world_position.y;
  pose.position.z = marker.world_position.z;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}
 
cv::Point2f MarkerComponent::computeRectangleCenter(const std::vector<cv::Point2f> & points)
{
  cv::Point2f center(0, 0);
  for (const auto & p : points)
  {
    center += p;
  }
  center /= static_cast<float>(points.size());
  return center;
}
 
float MarkerComponent::computeRectangleOrientation(const std::vector<cv::Point2f> & points, const cv::Point2f & center)
{
  if (points.size() < 2)
  {
    return 0.0f;
  }
  
  cv::Point2f reference = points[0] - center;
  float base_angle = std::atan2(reference.y, reference.x);
  
  float furthest_dist = 0.0f;
  cv::Point2f furthest_vec = points[1] - center;
  
  for (size_t i = 1; i < points.size(); ++i)
  {
    cv::Point2f diff = points[i] - center;
    float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    if (dist > furthest_dist)
    {
      furthest_dist = dist;
      furthest_vec = diff;
    }
  }
  
  return std::atan2(furthest_vec.y, furthest_vec.x) - base_angle;
}
 
} // namespace moondawg
 
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(moondawg::MarkerComponent)
 