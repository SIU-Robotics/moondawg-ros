from serial import Serial, SerialException
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String, Header
from diagnostic_msgs.msg import DiagnosticStatus

class SerialNode(Node): 

    def __init__(self):
        super().__init__(node_name='serial_node')

        # create heartbeat
        heartbeat_interval = 1
        self.diagnostic_status = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK)
        self.diag_topic = self.create_publisher(DiagnosticStatus, '/serial_node/diag', 10)
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)

        # establish serial connection
        self.serial = None
        self.rate = 9600
        self.declare_parameter('port', "/dev/ttyACM0")
        self.serial_retry_timer = self.create_timer(5, self.connect_serial)
        
        self.subscription = self.create_subscription(String, '/serial_node/serial', self.serial_callback, 10)
        self.add_on_set_parameters_callback(self.set_parameters)

        self.diag(DiagnosticStatus.OK, "Serial node ready.")

    def set_parameters(self, params):
        for param in params:
            # Try to open serial on new port
            if param.name == 'port':
                self.get_logger().info(f"Serial port set to {param.value}")
                self.connect_serial()
                if self.serial is None:
                    if self.serial_retry_timer.is_canceled():
                        self.serial_retry_timer.reset()
                    return rclpy.parameter.SetParametersResult(successful=False)
        return rclpy.parameter.SetParametersResult(successful=True)
    
    # connect to arduino
    def connect_serial(self):
        try:
            if self.serial is not None and self.serial.is_open:
                self.serial.close()
            port = self.get_parameter('port').get_parameter_value().string_value
            self.serial = Serial(
                port=port, 
                baudrate=self.rate
            )
            self.diag(DiagnosticStatus.OK, "Serial connected.")
            if not self.serial_retry_timer.is_canceled():
                self.serial_retry_timer.cancel()
        except SerialException as e:
            self.serial = None
            self.diag(DiagnosticStatus.ERROR, f"Error starting serial on {port} @ {self.rate}")

    # send message over serial
    def serial_callback(self, message):
        if self.serial is None:
            self.get_logger().warn("Failed to send packet: Serial connection is not established.")
            return

        # C needs the newline character appended, using "read_until('\n')"
        string_to_send = message.data + "\n"
        try:
            self.serial.write(string_to_send.encode())
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                self.get_logger().debug(f"Wrote: {message.data}")
        except SerialException as e:
            self.diag(DiagnosticStatus.WARN, f"Could not send message: {message}")
        except Exception as e:
            self.diag(DiagnosticStatus.ERROR, f"Unexpected error: {e}")

    def stop(self):
        self.get_logger().info("Stopping serial_node")
        if self.serial is not None:
            self.serial.close()

    def diag(self, status, message):
        self.get_logger().error(message)
        self.diagnostic_status.level = status
        self.diagnostic_status.message = message
    
    # called on an interval to publish info about the node
    def heartbeat(self):
        self.diag_topic.publish(self.diagnostic_status)
        
        if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.get_logger().debug("Heartbeat published with diagnostic status: "
                               f"Level: {self.diag.level}, Message: {self.diag.message}")

# start node
def main(args=None): 
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)
    serial_node.stop()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
