from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import keyboard


class ControlsPublisher(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(String, 'control_commands', 10)
        self.key_listener = keyboard.on_press(self.on_press)
        self.get_logger().info('Starting')

    def on_press(self, key):
        self.get_logger().info('Key pressed: %s' % key)
        try:
            if key.char == 'w':
                self.send_command('forward')
            elif key.char == 's':
                self.send_command('backward')
            elif key.char == 'a':
                self.send_command('left')
            elif key.char == 'd':
                self.send_command('right')
        except:
            self.get_logger().error('Error parsing key press')

    def on_release(self, key):
        self.get_logger().info('Key released: %s' % key)
        try:
            if key.char == 'w' or key.char == 's' or key.char == 'a' or key.char == 'd':
                self.send_command('stop')
        except:
            self.get_logger().error('Error parsing key release')

    def send_command(self, command):
        self.get_logger().info('Sending command: %s' % command)
        msg = String()
        # msg.data = command
        self.publisher_.publish(command)
        self.get_logger().info('Sending' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    controller = ControlsPublisher()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
