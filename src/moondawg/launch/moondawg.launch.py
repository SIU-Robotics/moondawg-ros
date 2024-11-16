import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='websocket',
            output='screen'
        ),
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='controller_parser',
            name='controller_parser',
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='serial_node',
            name='serial_node',
            output='screen'
        ),
        Node(
            package='unitree_lidar_ros2',
            executable='unitree_lidar_ros2_node',
            name='unitree_lidar_ros2_node',
            output='screen',
            parameters= [
                    {'port': '/dev/ttyUSB0'},
                    {'rotate_yaw_bias': 0.0},
                    {'range_scale': 0.001},
                    {'range_bias': 0.0},
                    {'range_max': 50.0},
                    {'range_min': 0.0},
                    {'cloud_frame': "unilidar_lidar"},
                    {'cloud_topic': "unilidar/cloud"},
                    {'cloud_scan_num': 18},
                    {'imu_frame': "unilidar_imu"},
                    {'imu_topic': "unilidar/imu"}]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
