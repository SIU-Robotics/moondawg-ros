from serial import Serial, SerialException
from sys import exit
from os import _exit
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus
from time import sleep

hardware_id = 1

class SerialBridge(Node): 

    def __init__(self):
        super().__init__(node_name='serial_bridge')

        # create heartbeat
        heartbeat_interval = 1
        self.diag = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK, hardware_id=str(hardware_id))
        self.diag_topic = self.create_publisher(DiagnosticStatus, 'serial_bridge_diag', 10)
        self.heartbeat = self.create_timer(heartbeat_interval, self.heartbeat)

        # establish serial connection
        self.rate = 9600
        self.port = "/dev/ttyACM0"
        self.serial = None
        try:
            self.serial = Serial(port=self.port, baudrate=self.rate)
        except SerialException:
            self.serial_retry_timer = self.create_timer(2, self.retry_serial)
            self.get_logger().error("Could not open connection to Arduino. Retrying in 2.0s")
            self.serial = None
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = "No serial connection."
            sleep(2)
        self.subscription = self.create_subscription(String, 'serial_topic', self.serial_callback, 10)

    def retry_serial(self):
        try:
            self.serial = Serial(port=self.port, baudrate=self.rate)
        except SerialException:
            self.serial_retry_timer = self.create_timer(2, self.retry_serial)
            self.get_logger().error("Could not open connection to Arduino. Retrying in 2.0s")
            self.serial = None
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = "No serial connection."
            sleep(2)
        if self.serial is not None:
            self.diag.level = DiagnosticStatus.OK
            self.diag.message = "Serial connected."
            self.serial_retry_timer.cancel()


    def serial_callback(self, message):
        try:
            string_to_send = message.data

            if self.serial is not None:
                self.serial.write(string_to_send.encode())
            else:
                self.get_logger().info("Theoretically would have wrote: " + string_to_send)

        except Exception as e:
            self.get_logger().error("Error sending data to arduino: " + str(e))

            
    def heartbeat(self):
        self.diag_topic.publish(self.diag)

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
