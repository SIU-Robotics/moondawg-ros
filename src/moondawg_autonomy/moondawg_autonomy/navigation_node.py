import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import tf2_ros
from typing import List, Tuple

class NavigationNode(Node):
    """
    Handles local navigation and obstacle avoidance.
    Implements the specified control policy:
    1. Update costmap from depth scans
    2. Plan waypoint ≤1m ahead
    3. Steer wheels in-place
    4. Drive straight
    5. Stop and repeat
    """
    
    def __init__(self):
        super().__init__('navigation_node')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('inflation_radius', 0.45),
                ('safety_margin', 0.40),
                ('waypoint_distance', 1.0),
                ('stop_tol_position', 0.10),
                ('stop_tol_heading', 0.05),
            ])
            
        self._setup_communications()
        
        # Initialize state
        self.current_goal = None
        self.current_path = None
        self.costmap = None
        
        # Start planning timer
        self.create_timer(0.1, self.planning_callback)
        
        self.get_logger().info('Navigation node initialized')

    def _setup_communications(self):
        """Set up ROS communications."""
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
            
        self.path_pub = self.create_publisher(
            Path, '/planned_path', 10)
            
        self.nav_complete_pub = self.create_publisher(
            Bool, '/navigation_complete', 10)
            
        # Subscribers
        self.create_subscription(
            PoseStamped, '/navigate_to_pose', self.goal_callback, 10)
            
        self.create_subscription(
            OccupancyGrid, '/local_costmap', self.costmap_callback, 10)
            
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def planning_callback(self):
        """Main planning loop implementing the control policy."""
        if not all([self.current_goal, self.costmap]):
            return
            
        try:
            # 1. Get current pose
            robot_pose = self.get_robot_pose()
            if not robot_pose:
                return
                
            # 2. Plan next waypoint
            next_waypoint = self.plan_next_waypoint(robot_pose)
            if not next_waypoint:
                self.get_logger().warn('No valid waypoint found')
                self.stop_robot()
                return
                
            # 3. Steer wheels to waypoint heading
            heading = self.compute_heading(robot_pose, next_waypoint)
            if not self.steer_to_heading(heading):
                return  # Still turning
                
            # 4. Drive straight to waypoint
            self.drive_to_waypoint(next_waypoint)
            
            # 5. Check if final goal reached
            if self.check_goal_reached(robot_pose):
                self.nav_complete_pub.publish(Bool(data=True))
                self.stop_robot()
                
        except Exception as e:
            self.get_logger().error(f'Planning error: {str(e)}')
            self.stop_robot()

    def plan_next_waypoint(self, robot_pose: PoseStamped) -> PoseStamped:
        """
        Plan next waypoint that's collision-free and within waypoint_distance.
        Uses costmap for collision checking.
        """
        waypoint_dist = self.get_parameter('waypoint_distance').value
        safety_margin = self.get_parameter('safety_margin').value
        
        # Get vector to goal
        goal_vec = np.array([
            self.current_goal.pose.position.x - robot_pose.pose.position.x,
            self.current_goal.pose.position.y - robot_pose.pose.position.y
        ])
        
        dist_to_goal = np.linalg.norm(goal_vec)
        if dist_to_goal < waypoint_dist:
            waypoint_dist = dist_to_goal
            
        # Normalize and scale
        if dist_to_goal > 0:
            goal_vec = goal_vec / dist_to_goal * waypoint_dist
            
        # Create candidate waypoint
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.pose.position.x = robot_pose.pose.position.x + goal_vec[0]
        waypoint.pose.position.y = robot_pose.pose.position.y + goal_vec[1]
        
        # Check collision (simplified)
        if self.check_collision(waypoint, safety_margin):
            return None
            
        return waypoint
        
    def check_collision(self, pose: PoseStamped, safety_margin: float) -> bool:
        """Check if pose collides with obstacles in costmap."""
        if not self.costmap:
            return False
            
        # Convert pose to costmap cell
        resolution = self.costmap.info.resolution
        x_cell = int((pose.pose.position.x - self.costmap.info.origin.position.x) / resolution)
        y_cell = int((pose.pose.position.y - self.costmap.info.origin.position.y) / resolution)
        
        # Check cells within safety margin
        margin_cells = int(safety_margin / resolution)
        for dx in range(-margin_cells, margin_cells + 1):
            for dy in range(-margin_cells, margin_cells + 1):
                cell_x = x_cell + dx
                cell_y = y_cell + dy
                
                if 0 <= cell_x < self.costmap.info.width and 0 <= cell_y < self.costmap.info.height:
                    idx = cell_y * self.costmap.info.width + cell_x
                    if self.costmap.data[idx] >= 50:  # Occupied threshold
                        return True
                        
        return False
        
    def steer_to_heading(self, target_heading: float) -> bool:
        """
        Steer wheels in-place to target heading.
        Returns True when heading achieved.
        """
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return False
            
        # Get current heading
        current_heading = self.get_yaw(robot_pose.pose.orientation)
        
        # Check if within tolerance
        heading_error = self.normalize_angle(target_heading - current_heading)
        if abs(heading_error) < self.get_parameter('stop_tol_heading').value:
            return True
            
        # Send steering command
        cmd = Twist()
        cmd.angular.z = np.sign(heading_error) * min(0.5, abs(heading_error))
        self.cmd_vel_pub.publish(cmd)
        return False
        
    def drive_to_waypoint(self, waypoint: PoseStamped):
        """Drive straight to waypoint."""
        cmd = Twist()
        cmd.linear.x = 0.3  # Fixed speed for now
        self.cmd_vel_pub.publish(cmd)
        
    def stop_robot(self):
        """Stop all robot motion."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
    def get_robot_pose(self) -> PoseStamped:
        """Get current robot pose in map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
            
        except Exception as e:
            self.get_logger().warning(f'Could not get robot pose: {str(e)}')
            return None
            
    def goal_callback(self, msg):
        """Handle new navigation goal."""
        self.current_goal = msg
        self.current_path = None  # Reset path
        
    def costmap_callback(self, msg):
        """Store latest costmap."""
        self.costmap = msg

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
        
    @staticmethod
    def get_yaw(quaternion):
        """Extract yaw from quaternion."""
        return np.arctan2(
            2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        )

def main():
    rclpy.init()
    navigator = NavigationNode()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
