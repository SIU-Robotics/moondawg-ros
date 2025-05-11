import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from cv_bridge import CvBridge
import cv2
import base64
import datetime
from typing import Dict, Any

class CameraNode(Node):
    """
    Node for handling camera image processing and publishing.
    
    This node handles:
    - Processing camera feeds from various sources
    - Compressing and formatting images for web transmission
    - Managing multiple camera streams
    """

    def __init__(self):
        super().__init__(node_name='camera_node')
        
        # Initialize state variables
        self._init_state_variables()
        
        # Declare ROS parameters
        self._declare_parameters()
        
        # Set up publishers, subscribers, and timers
        self._setup_communications()
        
        self.get_logger().info("Camera node initialized and ready")

    def _init_state_variables(self) -> None:
        """Initialize all state variables for the node."""
        # Bridge for camera processing
        self.br = CvBridge()
        
        # Last frame time tracking for frame rate control
        self._last_frame_time = 0.0
        self._last_rs1_color_time = 0.0
        self._last_rs1_depth_time = 0.0
        self._last_rs2_color_time = 0.0
        self._last_rs2_depth_time = 0.0
        
        # Latency tracking for each camera feed
        self._latency_stats = {
            'main_camera': {'count': 0, 'total_latency': 0.0, 'avg_latency': 0.0, 'max_latency': 0.0, 'last_latency': 0.0},
            'rs1_color': {'count': 0, 'total_latency': 0.0, 'avg_latency': 0.0, 'max_latency': 0.0, 'last_latency': 0.0},
            'rs1_depth': {'count': 0, 'total_latency': 0.0, 'avg_latency': 0.0, 'max_latency': 0.0, 'last_latency': 0.0},
            'rs2_color': {'count': 0, 'total_latency': 0.0, 'avg_latency': 0.0, 'max_latency': 0.0, 'last_latency': 0.0},
            'rs2_depth': {'count': 0, 'total_latency': 0.0, 'avg_latency': 0.0, 'max_latency': 0.0, 'last_latency': 0.0}
        }
        
        # Diagnostic status setup
        self.diagnostic_status = DiagnosticStatus(
            name=self.get_name(), 
            level=DiagnosticStatus.OK,
            message="Initializing camera node"
        )

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters for this node."""
        # Image processing parameters
        self.declare_parameter('image_compression_quality', 20)
        self.declare_parameter('image_frame_rate', 15)  # Parameter for frame rate control
        self.declare_parameter('max_image_width', 640)  # Maximum width for images, aspect ratio maintained

    def _setup_communications(self) -> None:
        """Set up all publishers, subscribers, and timers."""
        # Diagnostic publisher
        self.diag_topic = self.create_publisher(
            DiagnosticStatus, 
            '/camera_node/diag', 
            10
        )
        
        # Image publishers for Web UI
        self.image_pub = self.create_publisher(
            String, 
            '/camera_node/compressed_image', 
            10
        )
        self.rs1_color_pub = self.create_publisher(
            String, 
            '/camera_node/rs1_color_image', 
            10
        )
        self.rs1_depth_pub = self.create_publisher(
            String, 
            '/camera_node/rs1_depth_image', 
            10
        )
        self.rs2_color_pub = self.create_publisher(
            String, 
            '/camera_node/rs2_color_image', 
            10
        )
        self.rs2_depth_pub = self.create_publisher(
            String, 
            '/camera_node/rs2_depth_image', 
            10
        )
        
        # Camera image subscriptions
        self.image_subscription = self.create_subscription(
            Image, 
            'image', 
            self.image_translator, 
            10
        )
        
        # RealSense camera subscriptions
        self.rs1_color_subscription = self.create_subscription(
            Image, 
            '/camera/realsense2_camera_1/color/image_raw', 
            self.rs1_color_translator, 
            10
        )
        self.rs1_depth_subscription = self.create_subscription(
            Image, 
            '/camera/realsense2_camera_1/depth/image_rect_raw', 
            self.rs1_depth_translator, 
            10
        )
        self.rs2_color_subscription = self.create_subscription(
            Image, 
            '/camera/realsense2_camera_2/color/image_raw', 
            self.rs2_color_translator, 
            10
        )
        self.rs2_depth_subscription = self.create_subscription(
            Image, 
            '/camera/realsense2_camera_2/depth/image_rect_raw', 
            self.rs2_depth_translator, 
            10
        )
        
        # Heartbeat timer
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat)
        
        # Set initial diagnostic status
        self.set_diagnostic_status(DiagnosticStatus.OK, "Camera node ready")

    def _process_image(self, cv_image, last_time_attr, publisher, is_depth=False, camera_key=None):
        """
        Common image processing function to optimize camera feed handling.
        
        Args:
            cv_image: OpenCV image to process
            last_time_attr: Attribute name for tracking last frame time
            publisher: ROS publisher to send the processed image to
            is_depth: True if this is a depth image (for specialized processing)
            camera_key: Key to identify the camera for latency tracking
            
        Returns:
            True if image was processed and published, False if skipped
        """
        try:
            # Start processing time for latency measurement
            processing_start_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
            
            # Check frame rate control - only process every nth frame
            frame_rate = self.get_parameter('image_frame_rate').get_parameter_value().integer_value
            
            # Skip frames based on frame rate setting (basic throttling)
            if hasattr(self, last_time_attr) and (processing_start_time - getattr(self, last_time_attr)) < (1.0 / frame_rate):
                return False
            
            setattr(self, last_time_attr, processing_start_time)
            
            # Resize the image to reduce CPU usage
            max_width = self.get_parameter('max_image_width').get_parameter_value().integer_value
            if max_width > 0 and cv_image.shape[1] > max_width:
                # Calculate new dimensions maintaining aspect ratio
                aspect_ratio = cv_image.shape[0] / cv_image.shape[1]
                new_width = max_width
                new_height = int(new_width * aspect_ratio)
                # Use INTER_NEAREST for depth images or INTER_AREA for color images (best for downsampling)
                interpolation = cv2.INTER_NEAREST if is_depth else cv2.INTER_AREA
                cv_image = cv2.resize(cv_image, (new_width, new_height), interpolation=interpolation)
            
            # Use a fixed quality value
            quality = self.get_parameter('image_compression_quality').get_parameter_value().integer_value
            
            # Always use JPEG encoding
            _, encoded_img = cv2.imencode('.jpg', cv_image, 
                                        [cv2.IMWRITE_JPEG_QUALITY, quality])
            mime_type = "image/jpeg"
            
            # Convert to base64 and publish
            base64_data = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
            data_with_mime = f"{mime_type},{base64_data}"
            publisher.publish(String(data=data_with_mime))
            
            # Calculate and store latency information
            if camera_key and camera_key in self._latency_stats:
                processing_end_time = self.get_clock().now().nanoseconds / 1e9
                latency = processing_end_time - processing_start_time
                
                # Update latency statistics
                self._latency_stats[camera_key]['count'] += 1
                self._latency_stats[camera_key]['total_latency'] += latency
                self._latency_stats[camera_key]['avg_latency'] = (
                    self._latency_stats[camera_key]['total_latency'] / 
                    self._latency_stats[camera_key]['count']
                )
                self._latency_stats[camera_key]['max_latency'] = max(
                    self._latency_stats[camera_key]['max_latency'], 
                    latency
                )
                self._latency_stats[camera_key]['last_latency'] = latency
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            return False
    
    def image_translator(self, message: Image) -> None:
        """
        Convert ROS Image messages to compressed base64 strings for web transmission.
        
        Args:
            message: The Image message from the camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(message)
            self._process_image(cv_image, '_last_frame_time', self.image_pub, camera_key='main_camera')
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {str(e)}")
    
    def rs1_color_translator(self, message: Image) -> None:
        """Process RealSense 1 color camera images."""
        try:
            cv_image = self.br.imgmsg_to_cv2(message, desired_encoding='bgr8')
            self._process_image(cv_image, '_last_rs1_color_time', self.rs1_color_pub, camera_key='rs1_color')
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense1 color image: {str(e)}")
    
    def rs1_depth_translator(self, message: Image) -> None:
        """Process RealSense 1 depth camera images."""
        try:
            cv_image = self.br.imgmsg_to_cv2(message)
            # Normalize depth values for better visualization (16-bit to 8-bit conversion)
            cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            # Apply a colormap for better visualization
            cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)
            self._process_image(cv_image, '_last_rs1_depth_time', self.rs1_depth_pub, is_depth=True, camera_key='rs1_depth')
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense1 depth image: {str(e)}")
    
    def rs2_color_translator(self, message: Image) -> None:
        """Process RealSense 2 color camera images."""
        try:
            cv_image = self.br.imgmsg_to_cv2(message, desired_encoding='bgr8')
            self._process_image(cv_image, '_last_rs2_color_time', self.rs2_color_pub, camera_key='rs2_color')
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense2 color image: {str(e)}")
    
    def rs2_depth_translator(self, message: Image) -> None:
        """Process RealSense 2 depth camera images."""
        try:
            cv_image = self.br.imgmsg_to_cv2(message)
            # Normalize depth values for better visualization (16-bit to 8-bit conversion)
            cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            # Apply a colormap for better visualization
            cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)
            self._process_image(cv_image, '_last_rs2_depth_time', self.rs2_depth_pub, is_depth=True, camera_key='rs2_depth')
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense2 depth image: {str(e)}")
    
    def set_diagnostic_status(self, level: int, message: str) -> None:
        """Update and publish diagnostic status with the given level and message."""
        self.diagnostic_status.level = level
        self.diagnostic_status.message = message
        
        # Create latency values for diagnostics
        values = [
            KeyValue(key='node_name', value=self.get_name()),
            KeyValue(key='timestamp', value=str(self.get_clock().now().seconds_nanoseconds()))
        ]
        
        # Add latency information for each camera
        for camera_key, stats in self._latency_stats.items():
            if stats['count'] > 0:
                # Format to 2 decimal places for readability (convert to ms for easier reading)
                last_latency_ms = stats['last_latency'] * 1000
                avg_latency_ms = stats['avg_latency'] * 1000
                max_latency_ms = stats['max_latency'] * 1000
                
                values.extend([
                    KeyValue(key=f'{camera_key}_frames', value=str(stats['count'])),
                    KeyValue(key=f'{camera_key}_last_latency_ms', value=f'{last_latency_ms:.2f}'),
                    KeyValue(key=f'{camera_key}_avg_latency_ms', value=f'{avg_latency_ms:.2f}'),
                    KeyValue(key=f'{camera_key}_max_latency_ms', value=f'{max_latency_ms:.2f}')
                ])
        
        self.diagnostic_status.values = values
        self.diag_topic.publish(self.diagnostic_status)
    
    def heartbeat(self) -> None:
        """Publish regular heartbeat messages to confirm node is alive."""
        # Create a summary of latency for all cameras
        latency_summary = ""
        active_cameras = 0
        
        for camera_key, stats in self._latency_stats.items():
            if stats['count'] > 0:
                active_cameras += 1
                # Convert to ms for more readable values
                last_latency_ms = stats['last_latency'] * 1000
                avg_latency_ms = stats['avg_latency'] * 1000
                
                # Add to the summary
                latency_summary += f"{camera_key}: {last_latency_ms:.1f}ms (avg: {avg_latency_ms:.1f}ms) | "
        
        # Trim the trailing separator
        if latency_summary:
            latency_summary = latency_summary[:-3]
        
        # Set the message based on active cameras
        if active_cameras == 0:
            message = "Camera node heartbeat - No active cameras"
        else:
            message = f"Camera node heartbeat - Latency: {latency_summary}"
        
        self.set_diagnostic_status(DiagnosticStatus.OK, message)

def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
