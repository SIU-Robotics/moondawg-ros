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
            executable='i2c_node',
            name='i2c_node',
            parameters = [{
                'i2c_bus': 1,
                'i2c_drive1': 0x04,
                'i2c_drive2': 0x05,
                'i2c_drive3': 0x06,
                'i2c_drive4': 0x07,
                'i2c_turn1': 0x08,
                'i2c_turn2': 0x09,
                'i2c_turn3': 0x10,
                'i2c_turn4': 0x11,
                'i2c_belt': 0x12,
                'i2c_deploy': 0x13,
                'i2c_deposit': 0x14,
                'i2c,vibrator': 0x15
            }]
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
