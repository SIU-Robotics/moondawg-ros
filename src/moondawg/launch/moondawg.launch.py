import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for moondawg package."""
    
    # Launch arguments
    enable_rosbridge = LaunchConfiguration('enable_rosbridge', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    debug_mode = LaunchConfiguration('debug', default='false')
    
    # Declare launch arguments so they can be passed on the command line
    args = [
        DeclareLaunchArgument(
            'enable_rosbridge',
            default_value='true',
            description='Enable or disable the ROS bridge websocket server'
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Enable or disable the camera node'
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino communication'
        ),
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus ID'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug mode with additional logging'
        ),
    ]
    
    # Define nodes
    nodes = [
        # ROSBridge for WebSocket connection - conditionally launched
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='websocket',
            output='screen',
            condition=IfCondition(enable_rosbridge),
            parameters=[
                {'port': 9090},
                {'address': '0.0.0.0'},
                {'retry_startup_delay': 5.0}
            ]
        ),
        
        # Camera node - conditionally launched
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='screen',
            condition=IfCondition(enable_camera),
            parameters=[
                {'device': camera_device},
                {'width': 640},
                {'height': 480},
                {'fps': 15.0}
            ]
        ),
        
        # Controller parser node
        Node(
            package='moondawg',
            executable='controller_parser',
            name='controller_parser',
            output='screen',
            parameters=[
                {'joystick_deadzone': 0.1},
                {'turn_sensitivity': 0.5},
                {'image_compression_quality': 20},
                {'debug': debug_mode}
            ]
        ),
        
        # I2C communication node
        Node(
            package='moondawg',
            executable='i2c_node',
            name='i2c_node',
            output='screen',
            parameters=[
                {'bus_id': i2c_bus},
                {'heartbeat_interval': 1.0},
                {'command_timeout': 5.0},
                {'debug': debug_mode}
            ]
        ),
        
        # Optional: Serial communication node (if needed alongside I2C)
        Node(
            package='moondawg',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[
                {'port': serial_port},
                {'baudrate': 9600},
                {'timeout': 1.0},
                {'write_timeout': 1.0},
                {'retry_interval': 5.0},
                {'debug': debug_mode}
            ]
        ),
    ]
    
    # Create and return launch description
    return LaunchDescription(args + nodes)

if __name__ == '__main__':
    generate_launch_description()
