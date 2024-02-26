from std_srvs.srv import Empty as Move

import rclpy

from rclpy.lifecycle import Node

from rpi_pkg.srv import Move
import serial


class MovementService(Node):

    def __init__(self):
        super().__init__(node_name='movement_service')
        self.srv = self.create_service(Move, 'move', self.move_callback)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace '/dev/ttyUSB0' with the correct port and baud rate
        

    def send_movement_command(self, command, speed):
        # Convert command and speed to bytes
        command_byte = bytes([command])
        speed_byte = bytes([speed])

        # Send the command and speed bytes to Arduino
        self.ser.write(command_byte)
        self.ser.write(speed_byte)

    def move_callback(self, request, response):
        try:

            self.send_movement_command(1, 100)  # Move forward with speed 100

            response.success = True
            response.message = 'Moving ' + request.direction
        except:
            response.success = False
            response.message = 'Unable to make move request.'



def main(args=None):
    rclpy.init(args=args)

    movement_service = MovementService()

    rclpy.spin(movement_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
