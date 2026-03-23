import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description for moondawg package."""    
    # Camera enable flags - individual control for each camera
    enable_usb_camera = LaunchConfiguration('enable_usb', default='false')
    enable_depth1 = LaunchConfiguration('enable_depth1', default='true')
    enable_depth2 = LaunchConfiguration('enable_depth2', default='true')
    
    camera_device = LaunchConfiguration('camera_device', default='6')
    realsense1_serial = LaunchConfiguration('realsense1_serial', default="'335222074167'")
    realsense2_serial = LaunchConfiguration('realsense2_serial', default="'349622073747'")

    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    debug_mode = LaunchConfiguration('debug', default='false')
    
    # Controller parser parameters
    joystick_deadzone = LaunchConfiguration('joystick_deadzone', default='0.1')
    turn_sensitivity = LaunchConfiguration('turn_sensitivity', default='0.5')
    # General quality can still be launch args
    image_compression_quality = LaunchConfiguration('image_compression_quality', default='20')

    # Marker detection parameters
    enable_marker_detection = LaunchConfiguration('enable_marker_detection', default='true')
    marker_camera_id = LaunchConfiguration('marker_camera_id', default='1')
    hsv_h_min = LaunchConfiguration('hsv_h_min', default='5')
    hsv_h_max = LaunchConfiguration('hsv_h_max', default='25')
    hsv_s_min = LaunchConfiguration('hsv_s_min', default='100')
    hsv_s_max = LaunchConfiguration('hsv_s_max', default='255')
    hsv_v_min = LaunchConfiguration('hsv_v_min', default='100')
    hsv_v_max = LaunchConfiguration('hsv_v_max', default='255')
    min_marker_area = LaunchConfiguration('min_marker_area', default='100')
    
    # Declare launch arguments so they can be passed on the command line
    args = [
        DeclareLaunchArgument(
            'enable_usb',
            default_value='false',
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
            default_value='6',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'realsense1_serial',
            default_value="'335222074167'",
            description='Serial number for first RealSense camera (leave empty to use any available)'
        ),
        DeclareLaunchArgument(
            'realsense2_serial',
            default_value="'349622073747'",
            description='Serial number for second RealSense camera (leave empty to use any available)'
        ),
        DeclareLaunchArgument(
            'useSerial',
            default_value='false',
            description='Use serial communication instead of I2C'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
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
            'enable_marker_detection',
            default_value='true',
            description='Enable marker detection for dump zone localization'
        ),
        DeclareLaunchArgument(
            'marker_camera_id',
            default_value='1',
            description='Which camera to use for marker detection (1 or 2)'
        ),
        DeclareLaunchArgument(
            'hsv_h_min',
            default_value='5',
            description='HSV H minimum threshold for orange marker detection'
        ),
        DeclareLaunchArgument(
            'hsv_h_max',
            default_value='25',
            description='HSV H maximum threshold for orange marker detection'
        ),
        DeclareLaunchArgument(
            'hsv_s_min',
            default_value='100',
            description='HSV S minimum threshold for orange marker detection'
        ),
        DeclareLaunchArgument(
            'hsv_s_max',
            default_value='255',
            description='HSV S maximum threshold for orange marker detection'
        ),
        DeclareLaunchArgument(
            'hsv_v_min',
            default_value='100',
            description='HSV V minimum threshold for orange marker detection'
        ),
        DeclareLaunchArgument(
            'hsv_v_max',
            default_value='255',
            description='HSV V maximum threshold for orange marker detection'
        ),
        DeclareLaunchArgument(
            'min_marker_area',
            default_value='100',
            description='Minimum contour area to be considered a marker'
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
                {'device_id': camera_device},
                {'fps': 15.0}
            ]
        ),
        # Node(
        #     package='moondawg_control',
        #     executable='controller_parser',
        #     name='controller_parser',
        #     output='screen',
        #     parameters=[
        #         {'joystick_deadzone': joystick_deadzone},
        #         {'turn_sensitivity': turn_sensitivity},
        #         {'image_compression_quality': image_compression_quality},
        #         {'debug': debug_mode}
        #     ]
        # ),
        # Node(
        #     package='moondawg_control',
        #     executable='serial_node',
        #     name='serial_node',
        #     output='screen'
        # ),
        # Node(
        #     package='unitree_lidar_ros2',
        #     executable='unitree_lidar_ros2_node',
        #     name='unitree_lidar_ros2_node',
        #     output='screen',
        #     parameters= [
        #             {'port': '/dev/ttyUSB0'},
        #             {'rotate_yaw_bias': 0.0},
        #             {'range_scale': 0.001},
        #             {'range_bias': 0.0},
        #             {'range_max': 50.0},
        #             {'range_min': 0.0},
        #             {'cloud_frame': "unilidar_lidar"},
        #             {'cloud_topic': "unilidar/cloud"},
        #             {'cloud_scan_num': 18},
        #             {'imu_frame': "unilidar_imu"},
        #             {'imu_topic': "unilidar/imu"}]
        # ),
        # Node(
        #     package='unitree_lidar_ros2',
        #     executable='lidar_filter_node',
        #     name='lidar_filter_node',
        #     output='screen',
        #     parameters=[
        #             {'wall_rejection_distance': 0.3},
        #             {'ground_height_min': -0.05},
        #             {'ground_height_max': 0.05},
        #             # {'target_frame': 'map'},
        #             {'target_frame': 'unilidar_lidar'}, # For testing
        #             {'source_frame': 'unilidar_lidar'},
        #             {'laser_scan_min_angle': -3.14159},
        #             {'laser_scan_max_angle': 3.14159},
        #             {'laser_scan_angle_increment': 0.00872664626},
        #             {'laser_scan_range_min': 0.1},
        #             {'laser_scan_range_max': 50.0}]
        # )
        # Node(
        #     package='moondawg_control',
        #     executable='i2c_node',
        #     name='i2c_node',
        #     output='screen',
        #     parameters=[
        #         {'bus_id': i2c_bus},
        #         {'heartbeat_interval': 1.0},
        #         {'command_timeout': 5.0},
        #         {'debug': debug_mode}
        #     ]
        # ),
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
                    'clip_distance': 3.0,
                    'allow_no_texture_points': True,
                    'pointcloud.enable': False,
                    'enable_sync': False,
                    'align_depth.enable': True,
                    'filters': '',
                    'device_type': 'D435',
                    'depth_module.global_time_enabled': False,
                    'enable_auto_exposure': True,
                }],
                condition=IfCondition(enable_depth2)
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
                    ('image_raw', '/image'), # Subscription
                    ('image_compressed', '/camera_node/usb_camera_image') # Publication for web UI with unique topic name
                ],
                condition=IfCondition(enable_usb_camera)
            ),
            # Marker detection for camera 1
            ComposableNode(
                package='moondawg_marker',
                plugin='moondawg::MarkerComponent',
                name='marker_detector_camera1',
                parameters=[{
                    'camera_topic_prefix': '/realsense/camera1',
                    'hsv_h_min': hsv_h_min,
                    'hsv_h_max': hsv_h_max,
                    'hsv_s_min': hsv_s_min,
                    'hsv_s_max': hsv_s_max,
                    'hsv_v_min': hsv_v_min,
                    'hsv_v_max': hsv_v_max,
                    'min_marker_area': min_marker_area,
                    'expected_marker_count': 4,
                    'enable_debug_output': True,
                }],
                condition=IfCondition(enable_depth1)
            ),
            # Marker detection for camera 2
            ComposableNode(
                package='moondawg_marker',
                plugin='moondawg::MarkerComponent',
                name='marker_detector_camera2',
                parameters=[{
                    'camera_topic_prefix': '/realsense/camera2',
                    'hsv_h_min': hsv_h_min,
                    'hsv_h_max': hsv_h_max,
                    'hsv_s_min': hsv_s_min,
                    'hsv_s_max': hsv_s_max,
                    'hsv_v_min': hsv_v_min,
                    'hsv_v_max': hsv_v_max,
                    'min_marker_area': min_marker_area,
                    'expected_marker_count': 4,
                    'enable_debug_output': False,
                }],
                condition=IfCondition(enable_depth2)
            ),
        ],
        output='screen',
    )
    
    # Create and return launch description
    ld = LaunchDescription(args + regular_nodes)
    ld.add_action(shared_container) # Add the container with compression nodes and RealSense cameras
    return ld

if __name__ == '__main__':
    generate_launch_description()
