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
    # Image processing parameters are now mostly per-compression-node
    # General quality/framerate can still be launch args if desired for all compression nodes
    image_compression_quality = LaunchConfiguration('image_compression_quality', default='20')
    image_frame_rate = LaunchConfiguration('image_frame_rate', default='15')
    max_image_width = LaunchConfiguration('max_image_width', default='640') # Corrected default from 530
    
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
            default_value='640', # Corrected default
            description='Maximum width for processed images, aspect ratio maintained'
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
            package='moondawg_control',
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
            package='moondawg_control',
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

    realsense_node_1 = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera1',
        name='realsense_camera1',
        parameters=[{
            'serial_no': realsense1_serial,
            'enable_color': True,
            'enable_depth': True,
            # Add other realsense specific parameters here
            # e.g., image resolutions, frame rates for the camera hardware itself
            'color_width': 640, 'color_height': 480, 'color_fps': 30,
            'depth_width': 640, 'depth_height': 480, 'depth_fps': 30,
            'clip_distance': 3.0, # Example: Clip depth at 3 meters
            'allow_no_texture_points': True,
            'pointcloud.enable': False, # Disable pointcloud if not used by compression
        }],
        output='screen',
        condition=IfCondition(enable_depth1) # Or a more specific enable flag
    )

    realsense_node_2 = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera2',
        name='realsense_camera2',
        parameters=[{
            'serial_no': realsense2_serial,
            'enable_color': True,
            'enable_depth': True,
            'color_width': 640, 'color_height': 480, 'color_fps': 30,
            'depth_width': 640, 'depth_height': 480, 'depth_fps': 30,
            'clip_distance': 3.0,
            'allow_no_texture_points': True,
            'pointcloud.enable': False,
        }],
        output='screen',
        condition=IfCondition(enable_depth2) # Or a more specific enable flag
    )
    
    # --- USB CAMERA NODE ---
    # Standard USB camera node
    usb_camera_node = Node(
        package='usb_cam', # Assuming usb_cam package
        executable='usb_cam_node_exe', # often usb_cam_node or usb_cam_node_exe
        name='usb_camera',
        parameters=[{
            'video_device': camera_device,
            'image_width': 640, # Configure USB cam params
            'image_height': 480,
            'framerate': 30,
            'pixel_format': 'yuyv2rgb', # or mjpeg
        }],
        output='screen',
        condition=IfCondition(enable_usb_camera)
    )

    # --- COMPOSABLE NODE CONTAINER FOR IMAGE COMPRESSION ---
    # All compression nodes will run in this container for efficiency
    compression_container = ComposableNodeContainer(
        name='image_compression_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Compression for USB Camera
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::ImageCompressionNode',
                name='usb_camera_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'image_frame_rate': image_frame_rate,
                    'max_image_width': max_image_width,
                    'camera_key': 'usb_main_camera',
                }],
                remappings=[
                    ('image_raw', '/usb_camera/image_raw'), # Subscription
                    ('image_compressed', '/camera_node/compressed_image') # Publication for web UI
                ],
                condition=IfCondition(enable_usb_camera)
            ),
            # Compression for RealSense 1 Color
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='rs1_color_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'image_frame_rate': image_frame_rate,
                    'max_image_width': max_image_width,
                    'camera_key': 'rs1_color',
                }],
                remappings=[
                    ('image_raw', '/camera1/color/image_raw'),
                    ('image_compressed', '/camera_node/rs1_color_image')
                ],
                condition=IfCondition(enable_depth1)
            ),
            # Compression for RealSense 1 Depth
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='rs1_depth_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality, # Depth might need different quality
                    'image_frame_rate': image_frame_rate,
                    'max_image_width': max_image_width,
                    'camera_key': 'rs1_depth',
                }],
                remappings=[
                    # Realsense depth is often 16UC1, ensure ImageCompressionNode handles it (e.g. normalize and colormap)
                    ('image_raw', '/camera1/depth/image_rect_raw'), 
                    ('image_compressed', '/camera_node/rs1_depth_image')
                ],
                condition=IfCondition(enable_depth1)
            ),
            # Compression for RealSense 2 Color
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='rs2_color_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'image_frame_rate': image_frame_rate,
                    'max_image_width': max_image_width,
                    'camera_key': 'rs2_color',
                }],
                remappings=[
                    ('image_raw', '/camera2/color/image_raw'),
                    ('image_compressed', '/camera_node/rs2_color_image')
                ],
                condition=IfCondition(enable_depth2)
            ),
            # Compression for RealSense 2 Depth
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='rs2_depth_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'image_frame_rate': image_frame_rate,
                    'max_image_width': max_image_width,
                    'camera_key': 'rs2_depth',
                }],
                remappings=[
                    ('image_raw', '/camera2/depth/image_rect_raw'),
                    ('image_compressed', '/camera_node/rs2_depth_image')
                ],
                condition=IfCondition(enable_depth2)
            ),
        ],
        output='screen',
    )
    
    # Create and return launch description
    ld = LaunchDescription(args + regular_nodes)
    ld.add_action(realsense_node_1)
    ld.add_action(realsense_node_2)
    ld.add_action(usb_camera_node)
    ld.add_action(compression_container) # Add the container with compression nodes
    # ld.add_action(camera_node) # Add the (now simpler) camera_node if still needed
    return ld

if __name__ == '__main__':
    generate_launch_description()
