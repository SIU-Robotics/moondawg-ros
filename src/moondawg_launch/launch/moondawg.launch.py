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
    # General quality can still be launch args
    image_compression_quality = LaunchConfiguration('image_compression_quality', default='20')
    
    # Additional launch arguments for navigation
    map_yaml_file = LaunchConfiguration('map')
    nav2_config = LaunchConfiguration('nav2_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    shared_container = ComposableNodeContainer(
        name='camera_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        arguments=['--use-intra-process-comms'],
        composable_node_descriptions=[
            # RealSense Camera 1
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera1',
                namespace='realsense',
                parameters=[{
                    'serial_no': realsense1_serial,
                    'enable_color': True,
                    'enable_depth': True,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'rgb_camera.color_profile': '640x360x15',
                    'depth_module.depth_profile': '640x360x15',
                    'clip_distance': 3.0, # Example: Clip depth at 3 meters
                    'allow_no_texture_points': True,
                    'pointcloud.enable': False,
                    'enable_sync': False,
                    'align_depth.enable': True,
                    'filters': '',
                    'device_type': 'D435',
                    'depth_module.global_time_enabled': False,
                    'enable_auto_exposure': True,
                }],
                condition=IfCondition(enable_depth1)
            ),
            # RealSense Camera 2
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera2',
                namespace='realsense',
                parameters=[{
                    'serial_no': realsense2_serial,
                    'enable_color': True,
                    'enable_depth': True,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'rgb_camera.color_profile': '640x360x15',
                    'depth_module.depth_profile': '640x360x15',
                    'clip_distance': 6.0,
                    'allow_no_texture_points': True,
                    'pointcloud.enable': False,
                    'enable_sync': False,
                    'align_depth.enable': True,
                    'filters': '',
                    'device_type': 'D456',
                    'depth_module.global_time_enabled': False,
                    'enable_auto_exposure': True,
                }],
                condition=IfCondition(enable_depth2)
            ),
            # Compression for USB Camera
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='usb_camera_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'camera_key': 'usb_main_camera',
                }],
                remappings=[
                    ('image_raw', '/usb_camera/image_raw'), # Subscription
                    ('image_compressed', '/camera_node/usb_camera_image') # Publication for web UI with unique topic name
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
                    'camera_key': 'rs1_color',
                }],
                remappings=[
                    ('image_raw', '/realsense/camera1/color/image_raw'),
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
                    'image_compression_quality': image_compression_quality, 
                    'camera_key': 'rs1_depth',
                    'is_depth_camera': True,
                    'skip_frames': 1,  # Process every other frame
                    'downsample_before_processing': True,
                    'use_optimized_encoding': True,
                    'depth_max_value_mm': 3000
                }],
                remappings=[
                    # Realsense depth is often 16UC1, ensure ImageCompressionNode handles it (e.g. normalize and colormap)
                    ('image_raw', '/realsense/camera1/depth/image_rect_raw'), 
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
                    'camera_key': 'rs2_color',
                }],
                remappings=[
                    ('image_raw', '/realsense/camera2/color/image_raw'),
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
                    'camera_key': 'rs2_depth',
                    'is_depth_camera': True,
                    'skip_frames': 1,  # Process every other frame
                    'downsample_before_processing': True,
                    'use_optimized_encoding': True,
                    'depth_max_value_mm': 6000
                }],
                remappings=[
                    ('image_raw', '/realsense/camera2/depth/image_rect_raw'),
                    ('image_compressed', '/camera_node/rs2_depth_image')
                ],
                condition=IfCondition(enable_depth2)
            ),
        ],
        output='screen',
    )
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
    
    # Nav2 stack nodes
    nav_nodes = [
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_file_name': map_yaml_file,
                'map_update_rate': 5.0,
                'resolution': 0.05,
            }]
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_config]
        ),

        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/cmd_vel', '/moondawg/cmd_vel'),
                ('/odom', '/moondawg/odom')
            ]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config]
        ),

        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_config],
            output='screen'
        ),

        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config]
        ),

        # Lifecycle Manager for Navigation
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator'
                ]}
            ]
        )
    ]

    # Add navigation nodes to existing nodes list
    nodes.extend(nav_nodes)
    
    # Create and return launch description
    ld = LaunchDescription(args + regular_nodes)
    ld.add_action(shared_container) # Add the container with compression nodes and RealSense cameras
    return ld

if __name__ == '__main__':
    generate_launch_description()
