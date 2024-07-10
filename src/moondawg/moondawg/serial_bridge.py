from serial import Serial, SerialException
from sys import exit
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus

hardware_id = 1

class SerialBridge(Node): 

    def __init__(self):
        super().__init__(node_name='serial_bridge')

        # create heartbeat
        heartbeat_interval = 1
        self.diag = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK, hardware_id=str(hardware_id), message="Waiting to connect...")
        self.diag_topic = self.create_publisher(DiagnosticStatus, 'serial_bridge_diag', 10)
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)

        # establish serial connection
        self.declare_parameter('rate', 9600)
        self.declare_parameter('port', "/dev/ttyACM0")
        self.rate = self.get_parameter('rate').get_parameter_value().integer_value
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.add_on_set_parameters_callback(self.set_parameters)

        self.serial = None
        self.serial_retry_timer = self.create_timer(5, self.connect_serial)
        
        self.subscription = self.create_subscription(String, 'serial_topic', self.serial_callback, 10)

    def set_parameters(self, params):
        for param in params:
            if param.name == 'rate' and param.value != self.rate:
                self.rate = param.value
                if self.serial is not None:
                    try:
                        self.serial.baudrate = self.rate
                        self.serial.close()
                        self.serial.open()
                        self.get_logger().info(f"Serial baudrate set to {self.rate}")
                    except SerialException:
                        self.get_logger().error(f"Failed to set baudrate to {self.rate}")
                        self.diag.level = DiagnosticStatus.ERROR
                        self.diag.message = "Failure while setting baudrate."
                        self.serial = None
            elif param.name == 'port' and param.value != self.port:
                self.port = param.value
                if self.serial is not None:
                    try:
                        self.serial.port = self.port
                        self.serial.close()
                        self.serial.open()
                        self.get_logger().info(f"Serial port set to {self.port}")
                    except SerialException:
                        self.get_logger().error(f"Failed to bind to new port {self.port}")
                        self.diag.level = DiagnosticStatus.ERROR
                        self.diag.message = "Failure while setting port."
                        self.serial = None
        return rclpy.parameter.ParameterEvent()
    
    def connect_serial(self):
        if self.serial is not None:
            return

        try:
            self.serial = Serial(port=self.port, baudrate=self.rate)
            self.get_logger().info("Serial connected.")
            self.diag.level = DiagnosticStatus.OK
            self.diag.message = "Serial connected."
        except SerialException as e:
            self.get_logger().error(f"Could not open connection to Arduino: {e}. Retrying in 5s")
            self.serial = None
            if self.diag.level != DiagnosticStatus.ERROR:
                self.diag.level = DiagnosticStatus.ERROR
                self.diag.message = "Failure while trying to open serial connection."

    def serial_callback(self, message):
        if self.serial is None:
            self.get_logger().warn("Failed to send packet: Serial connection is not established.")
            return

        string_to_send = message.data + "\n"
        try:
            self.serial.write(string_to_send.encode())
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                self.get_logger().debug(f"Wrote: {message.data}")
        except SerialException as e:
            self.get_logger().error(f"Error sending data to Arduino: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

            
    def heartbeat(self):
        self.diag.header.stamp = self.get_clock().now().to_msg()
        self.diag_topic.publish(self.diag)
        
        if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.get_logger().debug("Heartbeat published with diagnostic status: "
                               f"Level: {self.diag.level}, Message: {self.diag.message}")

def main(args=None): 
    rclpy.init(args=args)

    serial_bridge = SerialBridge()

    try:
        rclpy.spin(serial_bridge)
    except KeyboardInterrupt:
        serial_bridge.get_logger().warning('Ctrl-C pressed, shutting down...')
    finally:
        rclpy.shutdown()
        exit(130)

if __name__ == '__main__': 
    main()
