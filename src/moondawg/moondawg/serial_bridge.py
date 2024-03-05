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

            direction = data.data[0]
            speed = (data.data[5]+100)/2

            if direction < 10 and direction > -10:
                direction = 0

            left_speed = speed - (direction*(speed/100))
            right_speed = speed + (direction*(speed/100))


            left_speed = max(0, min(left_speed, 90))+90
            right_speed = max(0, min(right_speed, 90))+90

            message = f"m,{left_speed},{right_speed}"
            
            # self.get_logger().info(message)
            self.serial.write(message.encode())
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