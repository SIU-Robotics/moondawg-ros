from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Launch file for moondawg autonomy stack."""
    
    pkg_dir = get_package_share_directory('moondawg_autonomy')
    config_dir = os.path.join(pkg_dir, 'config')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    debug = LaunchConfiguration('debug', default='false')
    
    return LaunchDescription([
        # Perception node
        Node(
            package='moondawg_autonomy',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_resolution': 0.05,
                'map_width': 10.0,
                'map_height': 8.0,
                'inflation_radius': 0.45,
                'front_camera_max_range': 3.0,
                'rear_camera_max_range': 6.0,
            }]
        ),
        
        # Navigation node
        Node(
            package='moondawg_autonomy',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'inflation_radius': 0.45,
                'safety_margin': 0.40,
                'waypoint_distance': 1.0,
                'stop_tol_position': 0.10,
                'stop_tol_heading': 0.05,
            }]
        ),
        
        # Mission executor
        Node(
            package='moondawg_autonomy',
            executable='mission_executor',
            name='mission_executor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'excavation_zone_x': 5.0,
                'deposit_zone_x': 2.0,
                'safety_margin': 0.4,
                'waypoint_distance': 1.0,
                'stop_tol_position': 0.1,
                'stop_tol_heading': 0.05,
                'max_obstacle_size': 0.42,
                'min_hopper_fill': 0.75,
            }]
        ),
    ])
