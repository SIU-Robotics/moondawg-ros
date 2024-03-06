import rclpy
from std_msgs.msg import Int8MultiArray, String
from rclpy.lifecycle import Node
from sys import exit
from os import _exit

full_reverse = 20
full_forward = 160
stopped = (full_reverse+full_forward)/2

class MovementController(Node):

    def __init__(self):
        super().__init__(node_name='movement_controller')
        self.subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.move_callback, 10)
        self.publisher = self.create_publisher(String, 'serial_topic', 10)

    def move_callback(self, request):
        try:
            direction = request.data[0] # the direction the bot should go (-100 for left, 0 for striaght, 100 for right)

            # data.data[5] comes in as -100 (trigger not pressed) to 100 (fully pressed)
            speed = request.data[5]

            #we want to normalize the data between stopped and full_forward
            speed = ((speed + 100) / 2) * full_forward/100
            speed = speed + stopped

            if direction < 15 and direction > -15:
                direction = 0

            # For these motors, we want a value between 0 and 180, with 90 being stopped.
            # speed + 100 gives us a value between 0 and 200
            # (speed + 100) * 0.9 gives a value between 0 and 180
            if direction < 0:
                left_speed = speed
                right_speed = speed + (direction*(speed-(stopped))/100)
            elif direction > 0:
                left_speed = speed - (direction*(speed-(stopped))/100)
                right_speed = speed
            else:
                left_speed = speed
                right_speed = speed

            # Constrain the speeds between 0 and 180
            left_speed = round(max(full_reverse, min(left_speed, full_forward)))
            right_speed = round(max(full_reverse, min(right_speed, full_forward)))

            message = String()
            message.data = f"m,{left_speed},{right_speed}X"
            self.publisher.publish(message)
            # self.get_logger().info(request)
            # self.send_movement_command(1, 100)  # Move forward with speed 100

        except Exception as e:
            self.get_logger().error("Failed to send movement command:" + str(e))



def main(args=None):
    rclpy.init(args=args)

    movement_controller = MovementController()

    try:
        rclpy.spin(movement_controller)
    except KeyboardInterrupt:
        movement_controller.get_logger().warning('Ctrl-C pressed, shutting down...')
        try:
            exit(130)
        except:
            _exit(130)

    rclpy.shutdown()

if __name__ == '__main__':
    main()