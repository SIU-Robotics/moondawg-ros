import rclpy
from std_msgs.msg import Int8MultiArray
from rclpy.lifecycle import Node
from sys import exit
from os import _exit


class MovementController(Node):

    def __init__(self):
        super().__init__(node_name='movement_controller')
        self.subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.move_callback, 10)
        self.publisher = self.create_publisher(Int8MultiArray, 'serial_topic', 10)

    def send_movement_command(self, command, speed):
        # Convert command and speed to bytes
        command_byte = bytes([command])
        speed_byte = bytes([speed])

        # Send the command and speed bytes to Arduino
        self.ser.write(command_byte)
        self.ser.write(speed_byte)

    def move_callback(self, request):
        try:
            self.publisher.publish(request)
            # self.get_logger().info(request)
            # self.send_movement_command(1, 100)  # Move forward with speed 100

        except:
            self.get_logger().error("Failed to send movement command")



def main(args=None):
    rclpy.init(args=args)

    movement_controller = MovementController()

    try:
        rclpy.spin(movement_controller)
    except KeyboardInterrupt:
        movement_controller.get_logger().warning('CTRL+C pressed: movement_service node stopped.')
        try:
            exit(130)
        except:
            _exit(130)

    rclpy.shutdown()

if __name__ == '__main__':
    main()