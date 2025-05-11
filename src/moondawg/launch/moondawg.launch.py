import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for moondawg package."""
    
    # Launch arguments
    enable_rosbridge = LaunchConfiguration('enable_rosbridge', default='true')
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    debug_mode = LaunchConfiguration('debug', default='false')
    
    # Additional launch arguments for navigation
    map_yaml_file = LaunchConfiguration('map')
    nav2_config = LaunchConfiguration('nav2_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                get_package_share_directory('moondawg'),
                'maps',
                'map.yaml'
            ),
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'nav2_config',
            default_value=os.path.join(
                get_package_share_directory('moondawg'),
                'config',
                'nav2_params.yaml'
            ),
            description='Full path to nav2 params file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
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
    return LaunchDescription(args + nodes)

if __name__ == '__main__':
    generate_launch_description()
