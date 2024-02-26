from std_srvs.srv import Empty as Move

import rclpy
from rclpy.node import Node


class MovementService(Node):

    def __init__(self):
        super().__init__(node_name='movement_service')
        self.srv = self.create_service(Move, 'move', self.move_callback)

    def move_callback(self, request, response):
        try:

            print("movement command recieved")

            # 
            # Add code to move the robot forward/backward
            #

            response.success = True
            response.message = 'Moving ' + request.direction
        except:
            response.success = False
            response.message = 'Unable to make move request.'

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)

    movement_service = MovementService()

    rclpy.spin(movement_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
