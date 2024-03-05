from serial import Serial
from sys import exit
from os import _exit
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import Int8MultiArray

class SerialBridge(Node): 

    def __init__(self):
        super().__init__(node_name='serial_bridge')
        self.rate = 9600
        self.port = "/dev/ttyACM0"
        self.serial = Serial(port=self.port, baudrate=self.rate)
        self.subscription = self.create_subscription(Int8MultiArray, 'serial_topic', self.serial_callback, 10)

    def serial_callback(self, data):
        try:
            self.serial.write(bytearray(data.data, ))
        except Exception as e:
            self.get_logger().error("Error sending data to arduino: " + str(e))

def main(args=None): 
    rclpy.init(args=args)

    serial_bridge = SerialBridge()

    try:
        rclpy.spin(serial_bridge)
    except KeyboardInterrupt:
        serial_bridge.get_logger().warning('CTRL+C pressed: movement_service node stopped.')
        try:
            exit(130)
        except:
            _exit(130)

    rclpy.shutdown()

if __name__ == '__main__': 
    main()