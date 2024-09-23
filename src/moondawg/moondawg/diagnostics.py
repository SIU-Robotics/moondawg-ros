import rclpy
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.lifecycle import Node
from sys import exit


class Diagnostics(Node):

    def __init__(self):
        super().__init__(node_name='diagnostics')

        # create heartbeat
        self.diagnostic_info = DiagnosticArray()
        self.diagnostic_info.status = []
        heartbeat_interval = 1
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)
        self.diag_topic = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        # Register subscription to all nodes heartbeat
        self.controller_node_diag = self.create_subscription(DiagnosticStatus, 'xbox_translator_diag', self.update_diag, 10)
        self.serial_bridge_diag = self.create_subscription(DiagnosticStatus, 'serial_bridge_diag', self.update_diag, 10)

        self.get_logger().info('Diagnostics node initialized')


    def update_diag(self, info: DiagnosticStatus):
        try:
            for status in self.diagnostic_info.status:
                if status.hardware_id == info.hardware_id:
                    status = info
                    self.get_logger().debug(f'Updated diagnostic info for hardware_id: {info.hardware_id}')
                else:
                    self.diagnostic_info.status.append(info)
        except Exception as e:
            self.get_logger().error(f'Failed to update diagnostic info: {e}')

    # called on an interval to publish node info to be read in on the website
    def heartbeat(self):
        current_time = self.get_clock().now().to_msg()
        self.diagnostic_info.header = Header(stamp=current_time)
        self.diag_topic.publish(self.diagnostic_info)
        self.get_logger().debug('Heartbeat published')

# start node
def main(args=None):
    rclpy.init(args=args)

    diagnostic_node = Diagnostics()

    try:
        rclpy.spin(diagnostic_node)
    except KeyboardInterrupt:
        diagnostic_node.get_logger().warning('Ctrl-C pressed, shutting down...')
    finally:
        rclpy.shutdown()
        exit(130)

if __name__ == '__main__':
    main()