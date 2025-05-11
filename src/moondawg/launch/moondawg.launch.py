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
    
    # Camera enable flags - individual control for each camera
    enable_usb_camera = LaunchConfiguration('enable_usb', default='true')
    enable_depth1 = LaunchConfiguration('enable_depth1', default='true')
    enable_depth2 = LaunchConfiguration('enable_depth2', default='true')
    
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    realsense1_serial = LaunchConfiguration('realsense1_serial', default='')
    realsense2_serial = LaunchConfiguration('realsense2_serial', default='')
    # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    debug_mode = LaunchConfiguration('debug', default='false')
    
    # Controller parser parameters
    joystick_deadzone = LaunchConfiguration('joystick_deadzone', default='0.1')
    turn_sensitivity = LaunchConfiguration('turn_sensitivity', default='0.5')
    image_compression_quality = LaunchConfiguration('image_compression_quality', default='20')
    image_frame_rate = LaunchConfiguration('image_frame_rate', default='15')
    adaptive_quality = LaunchConfiguration('adaptive_quality', default='true')
    max_image_width = LaunchConfiguration('max_image_width', default='640')
    auto_dig_duration_seconds = LaunchConfiguration('auto_dig_duration_seconds', default='30')
    belt_speed_index = LaunchConfiguration('belt_speed_index', default='0')
    
    # Declare launch arguments so they can be passed on the command line
    args = [
        DeclareLaunchArgument(
            'enable_rosbridge',
            default_value='true',
            description='Enable or disable the ROS bridge websocket server'
        ),
        DeclareLaunchArgument(
            'enable_usb',
            default_value='true',
            description='Enable or disable the USB camera'
        ),
        DeclareLaunchArgument(
            'enable_depth1',
            default_value='true',
            description='Enable or disable the first RealSense depth camera'
        ),
        DeclareLaunchArgument(
            'enable_depth2',
            default_value='true',
            description='Enable or disable the second RealSense depth camera'
        ),
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'realsense1_serial',
            default_value='',
            description='Serial number for first RealSense camera (leave empty to use any available)'
        ),
        DeclareLaunchArgument(
            'realsense2_serial',
            default_value='',
            description='Serial number for second RealSense camera (leave empty to use any available)'
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
        DeclareLaunchArgument(
            'joystick_deadzone',
            default_value='0.1',
            description='Deadzone for joystick inputs (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'turn_sensitivity',
            default_value='0.5',
            description='Sensitivity for turn controls (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'image_compression_quality',
            default_value='20',
            description='Image compression quality (1-100)'
        ),
        DeclareLaunchArgument(
            'image_frame_rate',
            default_value='15',
            description='Frame rate for image processing'
        ),
        DeclareLaunchArgument(
            'adaptive_quality',
            default_value='true',
            description='Enable adaptive quality for image compression'
        ),
        DeclareLaunchArgument(
            'max_image_width',
            default_value='640',
            description='Maximum width for images (aspect ratio maintained)'
        ),
        DeclareLaunchArgument(
            'auto_dig_duration_seconds',
            default_value='30',
            description='Duration of autonomous digging sequence in seconds'
        ),
        DeclareLaunchArgument(
            'belt_speed_index',
            default_value='0',
            description='Initial belt speed index (0-2)'
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
        
        # Web camera node - conditionally launched with USB camera flag
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='screen',
            condition=IfCondition(enable_usb_camera),
            parameters=[
                {'device': camera_device},
                # Using native resolution
                {'fps': 15.0}
            ]
        ),
        
        # RealSense camera 1 - conditionally launched with depth1 flag
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_1',
            output='screen',
            condition=IfCondition(enable_depth1),
            parameters=[
                {'serial_no': realsense1_serial},
                {'device_type': 'd435'},
                {'enable_color': True},
                {'enable_depth': True},
                # Using native resolution - removing fixed size
                {'depth_fps': 15.0},
                {'color_fps': 15.0},
                {'filters': 'pointcloud'},
                {'enable_sync': True},
                {'align_depth': True},
                {'camera_name': 'camera1'}
            ]
        ),
        
        # RealSense camera 2 - conditionally launched with depth2 flag
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_2',
            output='screen',
            condition=IfCondition(enable_depth2),
            parameters=[
                {'serial_no': realsense2_serial},
                {'device_type': 'd435'},
                {'enable_color': True},
                {'enable_depth': True},
                # Using native resolution - removing fixed size
                {'depth_fps': 15.0},
                {'color_fps': 15.0},
                {'filters': 'pointcloud'},
                {'enable_sync': True},
                {'align_depth': True},
                {'camera_name': 'camera2'}
            ]
        ),
        
        # Controller parser node
        Node(
            package='moondawg',
            executable='controller_parser',
            name='controller_parser',
            output='screen',
            parameters=[
                {'joystick_deadzone': joystick_deadzone},
                {'turn_sensitivity': turn_sensitivity},
                {'image_compression_quality': image_compression_quality},
                {'image_frame_rate': image_frame_rate},
                {'adaptive_quality': adaptive_quality},
                {'max_image_width': max_image_width},
                {'auto_dig_duration_seconds': auto_dig_duration_seconds},
                {'belt_speed_index': belt_speed_index},
                {'belt_speeds': [180, 125, 120]},  # You can also make this configurable if needed
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
        # Node(
        #     package='moondawg',
        #     executable='serial_node',
        #     name='serial_node',
        #     output='screen',
        #     parameters=[
        #         {'port': serial_port},
        #         {'baudrate': 9600},
        #         {'timeout': 1.0},
        #         {'write_timeout': 1.0},
        #         {'retry_interval': 5.0},
        #         {'debug': debug_mode}
        #     ]
        # ),
    ]
    
    # Create and return launch description
    return LaunchDescription(args + nodes)

if __name__ == '__main__':
    generate_launch_description()
