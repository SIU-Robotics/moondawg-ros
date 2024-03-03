from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from pynput import keyboard
from pynput.keyboard import Listener


class ControlsPublisher(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.publisher_ = self.create_publisher(String, 'control_commands', 10)
        self.key_listener = Listener(on_press=self.on_press)
        self.key_listener.start()

    def on_press(self, key):
        try:
            self.send_command(str(key))
            # if key.char == keyboarKeyCode.from_char("w"):
            #     self.send_command('forward')
            # elif key.char == 's':
            #     self.send_command('backward')
            # elif key.char == 'a':
            #     self.send_command('left')
            # elif key.char == 'd':
            #     self.send_command('right')
        except Exception as e:
            self.get_logger().error('Error parsing key press: ' + e)

    def on_release(self, key):
        try:
            print(key)
            # if key.char == 'w' or key.char == 's' or key.char == 'a' or key.char == 'd':
            #     self.send_command('stop')
        except:
            self.get_logger().error('Error parsing key release')

    def send_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info('Sending ' + msg.data)



def main(args=None):
    rclpy.init(args=args)

    controller = ControlsPublisher()

    rclpy.spin(controller)

    controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()