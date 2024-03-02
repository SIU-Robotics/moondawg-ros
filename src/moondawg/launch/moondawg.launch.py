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
            package='moondawg',
            executable='movement_controller',
            name='movement_controller',
            output='screen'
        ),
        Node(
            package='moondawg',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()