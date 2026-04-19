import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """Generate launch description for moondawg package."""    
    # Camera enable flags - individual control for each camera
    enable_usb_camera = LaunchConfiguration('enable_usb', default='false')
    enable_depth1 = LaunchConfiguration('enable_depth1', default='true')
    enable_depth2 = LaunchConfiguration('enable_depth2', default='true')
    
    camera_device = LaunchConfiguration('camera_device', default='6')

    realsense1_serial = LaunchConfiguration('realsense1_serial', default='335222074167')
    realsense2_serial = LaunchConfiguration('realsense2_serial', default='349622073747')

    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    debug_mode = LaunchConfiguration('debug', default='false')
    
    # Controller parser parameters
    joystick_deadzone = LaunchConfiguration('joystick_deadzone', default='0.1')
    turn_sensitivity = LaunchConfiguration('turn_sensitivity', default='0.5')
    image_compression_quality = LaunchConfiguration('image_compression_quality', default='20')
    
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
            default_value='0',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'realsense1_serial',
            default_value='335222074167',
            description='Serial number for first RealSense camera (USB 3.2, port 3-1)'
        ),
        DeclareLaunchArgument(
            'realsense2_serial',
            default_value='349622073747',
            description='Serial number for second RealSense camera (USB 2.1, port 4-2)'
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
    ]
    
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
            executable='serial_node',
            name='serial_node',
            output='screen'
        ),
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

            # --- Camera 1 ---
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera1',
                namespace='realsense',
                parameters=[{
                    'serial_no': ParameterValue(realsense1_serial, value_type=str),
                    'usb_port_id': '3-1',
                    'enable_color': True,
                    'enable_depth': False,
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'rgb_camera.color_profile': '424x240x15',
                    'depth_module.depth_profile': '424x240x15',
                    'clip_distance': 3.0,
                    'allow_no_texture_points': True,
                    'pointcloud.enable': False,
                    'enable_sync': True,
                    'align_depth.enable': True,
                    'filters': '',
                    'device_type': 'D435',
                    'depth_module.global_time_enabled': False,
                    'enable_auto_exposure': True,
                    'auto_exposure_priority': False,
                    'enable_gyro': False,
                    'enable_accel': False,
                }],
                condition=IfCondition(enable_depth1)
            ),

            # --- Compression: RealSense 1 Color ---
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

            # --- Compression: RealSense 1 Depth ---
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='rs1_depth_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'camera_key': 'rs1_depth',
                    'is_depth_camera': True,
                    'skip_frames': 1,
                    'downsample_before_processing': True,
                    'use_optimized_encoding': True,
                    'depth_max_value_mm': 3000
                }],
                remappings=[
                    ('image_raw', '/realsense/camera1/depth/image_rect_raw'),
                    ('image_compressed', '/camera_node/rs1_depth_image')
                ],
                condition=IfCondition(enable_depth1)
            ),

            # --- Compression: RealSense 2 Color ---
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

            # --- Compression: RealSense 2 Depth ---
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='rs2_depth_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'camera_key': 'rs2_depth',
                    'is_depth_camera': True,
                    'skip_frames': 1,
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

            # --- Compression: USB Camera ---
            ComposableNode(
                package='moondawg_camera',
                plugin='moondawg::CameraComponent',
                name='usb_camera_compression',
                parameters=[{
                    'image_compression_quality': image_compression_quality,
                    'camera_key': 'usb_main_camera',
                }],
                remappings=[
                    ('image_raw', '/image'),
                    ('image_compressed', '/camera_node/usb_camera_image')
                ],
                condition=IfCondition(enable_usb_camera)
            ),
        ],
        output='screen',
    )

    delayed_camera2 = TimerAction(
        period=5.0,
        actions=[
            LoadComposableNodes(
                target_container='camera_processing_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='realsense2_camera',
                        plugin='realsense2_camera::RealSenseNodeFactory',
                        name='camera2',
                        namespace='realsense',
                        parameters=[{
                            'serial_no': ParameterValue(realsense2_serial, value_type=str),
                            'enable_color': True,
                            'enable_depth': False,
                            'enable_infra1': False,
                            'enable_infra2': False,
                            'rgb_camera.color_profile': '424x240x15',
                            'depth_module.depth_profile': '424x240x15',
                            'clip_distance': 3.0,
                            'allow_no_texture_points': True,
                            'pointcloud.enable': False,
                            'enable_sync': False,
                            'align_depth.enable': True,
                            'filters': '',
                            'device_type': 'D435',
                            'depth_module.global_time_enabled': False,
                            'enable_auto_exposure': True,
                            'auto_exposure_priority': False,
                            'enable_gyro': False,
                            'enable_accel': False,
                        }],
                        condition=IfCondition(enable_depth2)
                    ),
                ]
            )
        ]
    )

    ld = LaunchDescription(args + regular_nodes)
    ld.add_action(shared_container)
    ld.add_action(delayed_camera2)
    return ld

if __name__ == '__main__':
    generate_launch_description()