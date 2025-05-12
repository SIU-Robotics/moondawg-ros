import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import message_filters

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Parameters for costmap
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_resolution', 0.05),
                ('map_width', 10.0),  # meters
                ('map_height', 8.0),  # meters
                ('inflation_radius', 0.45),
                ('front_camera_max_range', 3.0),
                ('rear_camera_max_range', 6.0),
            ])

        self.bridge = CvBridge()
        self._setup_communications()
        self.initialize_costmap()
        
        self.get_logger().info('Perception node initialized')

    def _setup_communications(self):
        # Use message filters to synchronize front and rear depth images
        self.front_depth_sub = message_filters.Subscriber(
            self, Image, '/realsense/camera1/depth/image_rect_raw')
        self.rear_depth_sub = message_filters.Subscriber(
            self, Image, '/realsense/camera2/depth/image_rect_raw')
            
        # Synchronize messages with timestamp tolerance
        sync = message_filters.ApproximateTimeSynchronizer(
            [self.front_depth_sub, self.rear_depth_sub], 
            queue_size=5, 
            slop=0.1)
        sync.registerCallback(self.depth_callback)
        
        # Publisher for costmap
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/local_costmap', 10)
            
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def initialize_costmap(self):
        """Initialize empty costmap."""
        resolution = self.get_parameter('map_resolution').value
        width = int(self.get_parameter('map_width').value / resolution)
        height = int(self.get_parameter('map_height').value / resolution)
        
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = "map"
        self.costmap.info.resolution = resolution
        self.costmap.info.width = width
        self.costmap.info.height = height
        self.costmap.info.origin.position.x = -self.get_parameter('map_width').value / 2
        self.costmap.info.origin.position.y = -self.get_parameter('map_height').value / 2
        
        # Initialize as unknown (-1)
        self.costmap.data = [-1] * (width * height)

    def depth_callback(self, front_msg, rear_msg):
        """Process synchronized depth images and update costmap."""
        try:
            # Convert depth images to point clouds in camera frame
            front_points = self.depth_to_points(
                front_msg, 
                self.get_parameter('front_camera_max_range').value)
                
            rear_points = self.depth_to_points(
                rear_msg,
                self.get_parameter('rear_camera_max_range').value)
            
            # Transform points to map frame
            front_points_map = self.transform_points('map', 'front_cam_link', front_points)
            rear_points_map = self.transform_points('map', 'rear_cam_link', rear_points)
            
            # Update costmap with both point clouds
            self.update_costmap(front_points_map)
            self.update_costmap(rear_points_map)
            
            # Apply inflation
            self.inflate_obstacles()
            
            # Publish updated costmap
            self.costmap.header.stamp = self.get_clock().now().to_msg()
            self.costmap_pub.publish(self.costmap)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth images: {str(e)}')

    def depth_to_points(self, depth_msg, max_range):
        """Convert depth image to 3D points in camera frame."""
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
        
        # Get camera parameters from message or use defaults
        fx = 462.1  # focal length x
        fy = 462.1  # focal length y
        cx = 320.5  # optical center x
        cy = 240.5  # optical center y
        
        # Create meshgrid of pixel coordinates
        rows, cols = depth_image.shape
        x_grid, y_grid = np.meshgrid(np.arange(cols), np.arange(rows))
        
        # Convert to normalized camera coordinates
        x = (x_grid - cx) * depth_image / fx
        y = (y_grid - cy) * depth_image / fy
        z = depth_image
        
        # Filter points beyond max range
        mask = z < max_range
        points = np.stack((x[mask], y[mask], z[mask])).T
        
        return points

    def transform_points(self, target_frame, source_frame, points):
        """Transform points from source frame to target frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time())
                
            # Apply transform to points
            # ... transform math here ...
            return transformed_points
            
        except Exception as e:
            self.get_logger().warning(f'Transform failed: {str(e)}')
            return None

    def update_costmap(self, points):
        """Update costmap with new point cloud data."""
        if points is None:
            return
            
        resolution = self.costmap.info.resolution
        
        # Convert points to grid cells
        cells_x = ((points[:,0] - self.costmap.info.origin.position.x) / resolution).astype(int)
        cells_y = ((points[:,1] - self.costmap.info.origin.position.y) / resolution).astype(int)
        
        # Filter points within map bounds
        mask = (cells_x >= 0) & (cells_x < self.costmap.info.width) & \
               (cells_y >= 0) & (cells_y < self.costmap.info.height)
               
        cells_x = cells_x[mask]
        cells_y = cells_y[mask]
        
        # Mark cells as occupied (100)
        for x, y in zip(cells_x, cells_y):
            idx = y * self.costmap.info.width + x
            self.costmap.data[idx] = 100

    def inflate_obstacles(self):
        """Apply inflation to obstacles in costmap."""
        inflation_cells = int(self.get_parameter('inflation_radius').value / 
                            self.costmap.info.resolution)
                            
        # Create circular kernel for inflation
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (2 * inflation_cells + 1, 2 * inflation_cells + 1))
            
        # Convert costmap to image for dilation
        grid = np.array(self.costmap.data).reshape(
            self.costmap.info.height, 
            self.costmap.info.width)
            
        obstacles = (grid == 100).astype(np.uint8)
        inflated = cv2.dilate(obstacles, kernel)
        
        # Update costmap with inflated obstacles
        inflated_flat = inflated.flatten()
        for i in range(len(self.costmap.data)):
            if inflated_flat[i] > 0:
                self.costmap.data[i] = 90  # Inflated cost

def main():
    rclpy.init()
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
