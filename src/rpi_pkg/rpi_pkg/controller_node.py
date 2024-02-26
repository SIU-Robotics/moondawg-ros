from std_srvs.srv import Empty as Move

import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__(node_name='controller_node')
        self.move_serv = self.create_client(Move, srv_name='movement_service')       # CHANGE
        while not self.move_serv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Move.Request()                                   # CHANGE

    def send_request(self):

        self.future = self.move_serv.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()

    while rclpy.ok():
        cmd = input("Enter a command (w: forward, s: backward, a: left, d: right, q: quit): ")
        if cmd == 'q':
            break
        elif cmd in ['w', 's', 'a', 'd']:
            # minimal_client.req.command = cmd
            minimal_client.send_request()
        else:
            print("Invalid command. Please try again.")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
