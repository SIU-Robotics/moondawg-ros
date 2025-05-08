#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import re
import math
from typing import Dict, Any, List

class I2CAddress:
    # Mirror of controller_parser address constants
    FRONT_LEFT = 0x10
    FRONT_RIGHT = 0x11
    REAR_LEFT = 0x12
    REAR_RIGHT = 0x13
    STEERING_SERVO = 0x14
    EXCAVATION = 0x20
    DEPOSITION = 0x21

class GazeboControllerBridge(Node):
    """
    Bridge node that translates I2C commands from controller_parser 
    to Gazebo robot control commands.
    """
    
    def __init__(self):
        super().__init__('gazebo_controller_bridge')
        
        # Constants from controller_parser
        self.MOTOR_STOPPED = 90
        self.MOTOR_FULL_FORWARD = 110
        self.MOTOR_FULL_REVERSE = 70
        
        # Track current steering angles for each wheel
        self.steering_angles = {1: 90, 2: 90, 3: 90, 4: 90}
        
        # Track current wheel speeds
        self.wheel_speeds = {
            I2CAddress.FRONT_LEFT: self.MOTOR_STOPPED,
            I2CAddress.FRONT_RIGHT: self.MOTOR_STOPPED,
            I2CAddress.REAR_LEFT: self.MOTOR_STOPPED,
            I2CAddress.REAR_RIGHT: self.MOTOR_STOPPED
        }
        
        # Subscribe to I2C commands from controller_parser
        self.i2c_sub = self.create_subscription(
            String,
            '/i2c_node/command',
            self.i2c_callback,
            10
        )
        
        # Publisher for Gazebo robot vel commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/omni_drive_controller/cmd_vel',
            10
        )
        
        # Timer for publishing commands to Gazebo at a fixed rate
        self.control_timer = self.create_timer(0.05, self.control_callback)
        
        self.get_logger().info("Gazebo Controller Bridge initialized")

    def i2c_callback(self, msg: String) -> None:
        """Process I2C commands from controller_parser"""
        try:
            # Parse the I2C command (format: "0x10:90" or "0x14:1,90")
            if ':' not in msg.data:
                return
                
            addr_str, data_str = msg.data.split(':', 1)
            
            # Convert address from hex string to int
            addr = int(addr_str, 16)
            
            # Process based on the address type
            if addr == I2CAddress.STEERING_SERVO:
                # Steering servo commands have format "servo_index,angle"
                if ',' in data_str:
                    parts = data_str.split(',')
                    servo_idx = int(parts[0])
                    angle = int(parts[1])
                    self.steering_angles[servo_idx] = angle
                    self.get_logger().debug(f"Set steering angle {servo_idx} to {angle}")
                    
            elif addr in [I2CAddress.FRONT_LEFT, I2CAddress.FRONT_RIGHT, 
                         I2CAddress.REAR_LEFT, I2CAddress.REAR_RIGHT]:
                # Motor speed commands
                speed = int(data_str)
                self.wheel_speeds[addr] = speed
                self.get_logger().debug(f"Set wheel at {hex(addr)} to speed {speed}")
                
            # Note: We're not handling excavation/deposition commands in simulation
                
        except Exception as e:
            self.get_logger().error(f"Error processing I2C command: {str(e)}")

    def control_callback(self) -> None:
        """
        Convert current steering and wheel speeds to Twist messages for Gazebo
        """
        try:
            # Process the current state and generate a Twist message
            twist = self.calculate_robot_velocity()
            
            # Publish the Twist message
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {str(e)}")

    def calculate_robot_velocity(self) -> Twist:
        """
        Calculate overall robot velocity based on wheel speeds and steering angles
        """
        twist = Twist()
        
        # Check if all wheels are pointing forward (90 degrees)
        all_straight = all(angle == 90 for angle in self.steering_angles.values())
        
        # Check if the wheels are set for rotation (45/135 configuration)
        rotation_mode = (self.steering_angles[1] == 135 and 
                         self.steering_angles[2] == 45 and
                         self.steering_angles[3] == 135 and
                         self.steering_angles[4] == 45) or \
                        (self.steering_angles[1] == 45 and 
                         self.steering_angles[2] == 135 and
                         self.steering_angles[3] == 45 and
                         self.steering_angles[4] == 135)
                         
        # Get normalized wheel speeds (-1.0 to 1.0)
        fl_speed = self.normalize_speed(self.wheel_speeds[I2CAddress.FRONT_LEFT])
        fr_speed = self.normalize_speed(self.wheel_speeds[I2CAddress.FRONT_RIGHT])
        rl_speed = self.normalize_speed(self.wheel_speeds[I2CAddress.REAR_LEFT])
        rr_speed = self.normalize_speed(self.wheel_speeds[I2CAddress.REAR_RIGHT])
        
        if all_straight:
            # Simple forward/reverse motion
            # Average the wheel speeds
            avg_speed = (fl_speed + fr_speed + rl_speed + rr_speed) / 4.0
            twist.linear.x = avg_speed * 1.0  # Scale to m/s
            
        elif rotation_mode:
            # Rotation mode - convert to angular velocity
            # For clockwise rotation: left wheels forward, right wheels reverse
            # For counter-clockwise: right wheels forward, left wheels reverse
            left_speed = (fl_speed + rl_speed) / 2.0
            right_speed = (fr_speed + rr_speed) / 2.0
            
            # If left and right are opposite, we're rotating
            if (left_speed > 0 and right_speed < 0) or (left_speed < 0 and right_speed > 0):
                # Angular velocity depends on speed and robot width
                # We use the difference between sides to calculate rotation rate
                twist.angular.z = (left_speed - right_speed) * 2.0  # Scale to rad/s
                
        else:
            # Crab steering mode
            # Get the average angle of all wheels
            avg_angle = sum(self.steering_angles.values()) / 4.0
            
            # Convert to radians, adjusting so that 90° is 0 radians (straight)
            angle_rad = math.radians(avg_angle - 90.0)
            
            # Average speed of all wheels
            avg_speed = (fl_speed + fr_speed + rl_speed + rr_speed) / 4.0
            
            # Calculate x and y components based on angle and speed
            twist.linear.x = avg_speed * math.cos(angle_rad) * 1.0  # Scale to m/s
            twist.linear.y = avg_speed * math.sin(angle_rad) * 1.0  # Scale to m/s
            
        return twist
            
    def normalize_speed(self, speed: int) -> float:
        """
        Convert motor speed (70-110) to normalized value (-1.0 to 1.0)
        """
        if speed == self.MOTOR_STOPPED:
            return 0.0
            
        if speed > self.MOTOR_STOPPED:
            # Forward motion (90-110) -> (0.0-1.0)
            return (speed - self.MOTOR_STOPPED) / (self.MOTOR_FULL_FORWARD - self.MOTOR_STOPPED)
        else:
            # Reverse motion (70-90) -> (-1.0-0.0)
            return (speed - self.MOTOR_STOPPED) / (self.MOTOR_STOPPED - self.MOTOR_FULL_REVERSE)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboControllerBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()