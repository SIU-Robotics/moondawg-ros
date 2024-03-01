from std_msgs.msg import String

import rclpy
from rclpy.lifecycle import Node

from rclpy.qos import ReliabilityPolicy
import serial


class MovementService(Node):

    def __init__(self):
        super().__init__(node_name='movement_node')
        self.subscription = self.create_subscription(String, 'control_commands', self.move_callback, 10)

        try:
            self.ser = serial.Serial('/dev/serial1', 9600)  # Replace '/dev/ttyUSB0' with the correct port and baud rate
        
        except Exception as e:
            self.get_logger().error("Failed to connect to Arduino")

    def send_movement_command(self, command, speed):
        # Convert command and speed to bytes
        command_byte = bytes([command])
        speed_byte = bytes([speed])

        # Send the command and speed bytes to Arduino
        self.ser.write(command_byte)
        self.ser.write(speed_byte)

    def move_callback(self, request):
        try:
            self.get_logger().info(request.data)
            # self.send_movement_command(1, 100)  # Move forward with speed 100

        except:
            self.get_logger().error("Failed to send movement command")



def main(args=None):
    rclpy.init(args=args)

    movement_service = MovementService()

    rclpy.spin(movement_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
