import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus
from smbus2 import SMBus

class I2CNode(Node):
    def __init__(self):
        super().__init__('i2c_node')

        self.bus_id = 1
        self.bus = None
        self.diag_status = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK)
        self.diag_pub = self.create_publisher(DiagnosticStatus, '/i2c_node/diag', 10)

        self.create_subscription(String, '/i2c_node/command', self.command_callback, 10)
        self.create_timer(1.0, self.heartbeat)

        self.bus = SMBus(self.bus_id)
        self.diag(DiagnosticStatus.OK, f"I2C bus {self.bus_id} open.")

    def command_callback(self, msg):
        parts = msg.data.split(':', 1)
        if len(parts) < 2:
            self.diag(DiagnosticStatus.WARN, f"Invalid command format: {msg} should be <address>:<data>.")
            return
        addr, data_str = parts[0], parts[1]
        try:
            address = int(addr, 0)
            payload = list(data_str.encode())
            self.bus.write_i2c_block_data(address, 0, payload)
        except Exception as e:
            self.diag(DiagnosticStatus.ERROR, f"I2C write error: {e}")

    def diag(self, level, message):
        self.diag_status.level = level
        self.diag_status.message = message
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(message)
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def heartbeat(self):
        self.diag_pub.publish(self.diag_status)

def main(args=None):
    rclpy.init(args=args)
    node = I2CNode()
    rclpy.spin(node)
    node.bus.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()