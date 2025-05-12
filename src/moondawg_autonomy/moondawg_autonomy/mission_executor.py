import rclpy
from rclpy.node import Node
from enum import Enum, auto
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String
from geometry_msgs.msg import PoseStamped, Point
import tf2_ros
import math
import json

class MissionState(Enum):
    INIT = auto()
    SEARCH_ZONES = auto()  # New state for finding QR codes
    TRAVERSE_TO_DIG = auto()
    EXCAVATE = auto()
    TRAVERSE_TO_DUMP = auto()
    DEPOSIT = auto()

class MissionExecutor(Node):
    """
    Coordinates the overall autonomous mission sequence.
    """
    def __init__(self):
        super().__init__('mission_executor')
        
        # Modified parameters - zones now start as None until discovered
        self.declare_parameters(
            namespace='',
            parameters=[
                ('search_pattern_points', [  # Search pattern for finding QR codes
                    [2.0, 1.0],  # Center-front of arena
                    [2.0, -1.0], # Center-back of arena
                    [4.0, 0.0],  # Mid-arena
                    [6.0, 1.0],  # Far end front
                    [6.0, -1.0]  # Far end back
                ]),
                ('safety_margin', 0.4),
                ('waypoint_distance', 1.0),
                ('stop_tol_position', 0.1),
                ('stop_tol_heading', 0.05),
                ('max_obstacle_size', 0.42),
                ('min_hopper_fill', 0.75),
            ])

        # Zone locations start as unknown
        self.excavation_zone = None
        self.deposit_zone = None
        self.current_search_point = 0
        
        # Rest of initialization
        self.current_state = MissionState.INIT
        self.hopper_fill_level = 0.0
        self.robot_pose = None
        
        self._setup_communications()
        self.create_timer(0.1, self.state_machine_tick)
        
        self.get_logger().info('Mission executor initialized - waiting to discover zones')

    def _setup_communications(self):
        """Set up ROS communications."""
        # Publishers
        self.nav_goal_pub = self.create_publisher(
            PoseStamped, '/navigate_to_pose', 10)
            
        self.arm_command_pub = self.create_publisher(
            Int32, '/arm_command', 10)
            
        # Subscribers
        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        self.create_subscription(
            Bool, '/navigation_complete', self.nav_complete_callback, 10)
            
        self.create_subscription(
            String, '/qr_detection', self.qr_callback, 10)
            
        self.create_subscription(
            Float32, '/hopper_fill_level', self.hopper_fill_callback, 10)
            
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def state_machine_tick(self):
        """Main state machine update function."""
        if not self.robot_pose:
            return  # Wait for odometry
            
        try:
            if self.current_state == MissionState.INIT:
                # Start searching for zones if we haven't found both
                self._start_zone_search()
                
            elif self.current_state == MissionState.SEARCH_ZONES:
                # Continue searching until both zones are found
                if self.excavation_zone and self.deposit_zone:
                    self._start_traverse_to_dig()
                else:
                    self._continue_zone_search()
                    
            elif self.current_state == MissionState.TRAVERSE_TO_DIG:
                # Check if we've reached the dig site
                if self._check_position_reached():
                    self._start_excavation()
                    
            elif self.current_state == MissionState.EXCAVATE:
                # Monitor hopper fill level
                if self.hopper_fill_level >= self.get_parameter('min_hopper_fill').value:
                    self._start_traverse_to_dump()
                    
            elif self.current_state == MissionState.TRAVERSE_TO_DUMP:
                # Check if we've reached the dump site 
                if self._check_position_reached():
                    self._start_deposit()
                    
            elif self.current_state == MissionState.DEPOSIT:
                # Monitor hopper level
                if self.hopper_fill_level <= 0.1:  # Nearly empty
                    self._start_traverse_to_dig()
                    
        except Exception as e:
            self.get_logger().error(f'State machine error: {str(e)}')
            
    def _start_zone_search(self):
        """Begin searching for QR codes marking zones."""
        self.current_state = MissionState.SEARCH_ZONES
        self.current_search_point = 0
        self._go_to_next_search_point()
        
    def _continue_zone_search(self):
        """Continue searching pattern for zones."""
        if self._check_position_reached():
            self.current_search_point += 1
            search_points = self.get_parameter('search_pattern_points').value
            
            if self.current_search_point >= len(search_points):
                # Reset search if we haven't found both zones
                self.current_search_point = 0
                
            self._go_to_next_search_point()

    def _go_to_next_search_point(self):
        """Navigate to next search pattern point."""
        search_points = self.get_parameter('search_pattern_points').value
        if self.current_search_point < len(search_points):
            point = search_points[self.current_search_point]
            
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = point[0]
            goal.pose.position.y = point[1]
            
            self.nav_goal_pub.publish(goal)
            self.get_logger().info(f'Searching for zones - moving to point {self.current_search_point}')

    def qr_callback(self, msg):
        """Handle QR code detection."""
        try:
            qr_data = json.loads(msg.data)
            
            if 'type' not in qr_data or 'position' not in qr_data:
                return
                
            if qr_data['type'] == 'excavation':
                self.excavation_zone = Point(
                    x=float(qr_data['position']['x']),
                    y=float(qr_data['position']['y']),
                    z=0.0
                )
                self.get_logger().info('Found excavation zone')
                
            elif qr_data['type'] == 'deposit':
                self.deposit_zone = Point(
                    x=float(qr_data['position']['x']),
                    y=float(qr_data['position']['y']),
                    z=0.0
                )
                self.get_logger().info('Found deposit zone')
                
        except Exception as e:
            self.get_logger().error(f'Error parsing QR data: {str(e)}')

    def _start_traverse_to_dig(self):
        """Start traversing to the excavation zone."""
        if not self.excavation_zone:
            self.get_logger().warn('Attempted to traverse to unknown excavation zone')
            return
            
        self.current_state = MissionState.TRAVERSE_TO_DIG
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position = self.excavation_zone
        
        self.nav_goal_pub.publish(goal)
        self.get_logger().info('Starting traverse to excavation zone')
        
    def _start_excavation(self):
        """Begin excavation sequence."""
        self.current_state = MissionState.EXCAVATE
        
        # Lower arm and start collection
        self.arm_command_pub.publish(Int32(data=1))  # 1 = start digging
        self.get_logger().info('Starting excavation')
        
    def _start_traverse_to_dump(self):
        """Start traversing to deposit zone."""
        if not self.deposit_zone:
            self.get_logger().warn('Attempted to traverse to unknown deposit zone')
            return
            
        self.current_state = MissionState.TRAVERSE_TO_DUMP
        
        # Raise arm first
        self.arm_command_pub.publish(Int32(data=0))  # 0 = stow arm
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position = self.deposit_zone
        
        self.nav_goal_pub.publish(goal)
        self.get_logger().info('Starting traverse to deposit zone')
        
    def _start_deposit(self):
        """Begin material deposition."""
        self.current_state = MissionState.DEPOSIT
        
        # Start dumping sequence
        self.arm_command_pub.publish(Int32(data=2))  # 2 = start dumping
        self.get_logger().info('Starting deposition')
        
    def _check_position_reached(self):
        """Check if target position has been reached within tolerances."""
        return math.hypot(
            self.target_pose.position.x - self.robot_pose.position.x,
            self.target_pose.position.y - self.robot_pose.position.y
        ) < self.get_parameter('stop_tol_position').value
        
    def odom_callback(self, msg):
        """Store latest odometry."""
        self.robot_pose = msg.pose.pose
        
    def nav_complete_callback(self, msg):
        """Handle navigation completion."""
        if msg.data:
            self.get_logger().info('Navigation waypoint reached')
            
    def hopper_fill_callback(self, msg):
        """Update hopper fill level."""
        self.hopper_fill_level = msg.data

def main():
    rclpy.init()
    executor = MissionExecutor()
    
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
