import rclpy
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.lifecycle import Node
from sys import exit
from os import _exit
import time


class Diagnostics(Node):


    def __init__(self):
        super().__init__(node_name='diagnostics')

        # create heartbeat
        self.diagnostic_info = DiagnosticArray()
        self.diagnostic_info.status = [DiagnosticStatus(), DiagnosticStatus()]
        heartbeat_interval = 1
        self.heartbeat = self.create_timer(heartbeat_interval, self.heartbeat)
        self.diag_topic = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        # Register subscription to all nodes heartbeat
        self.controller_node_diag = self.create_subscription(DiagnosticStatus, 'xbox_translator_diag', self.update_diag, 10)
        self.serial_bridge_diag = self.create_subscription(DiagnosticStatus, 'serial_bridge_diag', self.update_diag, 10)

    def update_diag(self, info):
        # self.get_logger().info(str(int(str(info.hardware_id))))
        self.diagnostic_info.status[int(info.hardware_id)] = info

    def heartbeat(self):
        current_time = Time()
        current_time.sec = int(time.time())
        self.diagnostic_info.header = Header(stamp=current_time)
        self.diag_topic.publish(self.diagnostic_info)



def main(args=None):
    rclpy.init(args=args)

    diagnostic_node = Diagnostics()

    try:
        rclpy.spin(diagnostic_node)
    except KeyboardInterrupt:
        diagnostic_node.get_logger().warning('Ctrl-C pressed, shutting down...')
        try:
            exit(130)
        except:
            _exit(130)

    rclpy.shutdown()

if __name__ == '__main__':
    main()