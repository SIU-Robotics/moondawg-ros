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
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
