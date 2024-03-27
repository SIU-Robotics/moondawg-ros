from serial import Serial, SerialException
from sys import exit
from os import _exit
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String


class SerialBridge(Node): 

    def __init__(self):
        super().__init__(node_name='serial_bridge')
        self.rate = 9600
        self.port = "/dev/ttyACM0"
        try:
            self.serial = Serial(port=self.port, baudrate=self.rate)
        except SerialException:
            self.get_logger().error("Could not open connection to Arduino.")
            self.serial = None
        self.subscription = self.create_subscription(String, 'serial_topic', self.serial_callback, 10)

    def serial_callback(self, message):
        try:
            string_to_send = message.data

            if self.serial is not None:
                self.serial.write(string_to_send.encode())
            else:
                self.get_logger().info("Theoretically would have wrote: " + string_to_send)

        except Exception as e:
            self.get_logger().error("Error sending data to arduino: " + str(e))

def main(args=None): 
    rclpy.init(args=args)

    serial_bridge = SerialBridge()

    try:
        rclpy.spin(serial_bridge)
    except KeyboardInterrupt:
        serial_bridge.get_logger().warning('Ctrl-C pressed, shutting down...')
        try:
            exit(130)
        except:
            _exit(130)

    rclpy.shutdown()

if __name__ == '__main__': 
    main()
