import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for moondawg package."""    
    # Camera enable flags - individual control for each camera
    enable_usb_camera = LaunchConfiguration('enable_usb', default='true')
    enable_depth1 = LaunchConfiguration('enable_depth1', default='true')
    enable_depth2 = LaunchConfiguration('enable_depth2', default='true')
    
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    realsense1_serial = LaunchConfiguration('realsense1_serial', default='')
    realsense2_serial = LaunchConfiguration('realsense2_serial', default='')

    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    debug_mode = LaunchConfiguration('debug', default='false')
    
    # Controller parser parameters
    joystick_deadzone = LaunchConfiguration('joystick_deadzone', default='0.1')
    turn_sensitivity = LaunchConfiguration('turn_sensitivity', default='0.5')
    image_compression_quality = LaunchConfiguration('image_compression_quality', default='20')
    image_frame_rate = LaunchConfiguration('image_frame_rate', default='15')
    max_image_width = LaunchConfiguration('max_image_width', default='530')
    
    # Declare launch arguments so they can be passed on the command line
    args = [
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
            'useSerial',
            default_value='false',
            description='Use serial communication instead of I2C'
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
            'max_image_width',
            default_value='530',
            description='Maximum width for images (aspect ratio maintained)'
        ),
    ]
    
    # Define regular nodes (not part of the camera composition)
    regular_nodes = [
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='websocket',
            output='screen',
            parameters=[
                {'port': 9090},
                {'address': '0.0.0.0'},
                {'retry_startup_delay': 5.0}
            ]
        ),
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='screen',
            condition=IfCondition(enable_usb_camera),
            parameters=[
                {'device': camera_device},
                {'fps': 15.0}
            ]
        ),
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
                {'max_image_width': max_image_width},
                {'debug': debug_mode}
            ]
        ),
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
        )
    ]
    
    # Define the camera composition container with intra-process communication
    camera_container = ComposableNodeContainer(
        name='camera_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            # RealSense camera 1 as a composable node
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='realsense2_camera_1',
                condition=IfCondition(enable_depth1),
                parameters=[
                    {'serial_no': realsense1_serial},
                    {'device_type': 'd435'},
                    {'enable_color': True},
                    {'enable_depth': True},
                    {'depth_fps': 15.0},
                    {'color_fps': 15.0},
                    {'filters': 'pointcloud'},
                    {'enable_sync': True},
                    {'align_depth': True},
                    {'camera_name': 'camera1'},
                    {'use_intra_process_comms': True}
                ]
            ),
            # RealSense camera 2 as a composable node
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='realsense2_camera_2',
                condition=IfCondition(enable_depth2),
                parameters=[
                    {'serial_no': realsense2_serial},
                    {'device_type': 'd456'},
                    {'enable_color': True},
                    {'enable_depth': True},
                    {'depth_fps': 15.0},
                    {'color_fps': 15.0},
                    {'filters': 'pointcloud'},
                    {'enable_sync': True},
                    {'align_depth': True},
                    {'camera_name': 'camera2'},
                    {'use_intra_process_comms': True}
                ]
            ),
            # Camera node for processing the RealSense data
            ComposableNode(
                package='moondawg',
                plugin='moondawg_camera_node',
                name='camera_node',
                parameters=[
                    {'image_compression_quality': image_compression_quality},
                    {'image_frame_rate': image_frame_rate},
                    {'max_image_width': max_image_width},
                    {'debug': debug_mode},
                    {'use_intra_process_comms': True}
                ]
            )
        ],
        output='screen',
    )
    
    # Create and return launch description
    return LaunchDescription(args + regular_nodes + [camera_container])

if __name__ == '__main__':
    generate_launch_description()
