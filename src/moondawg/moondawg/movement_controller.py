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

        # Register subscriptions to the gamepad topics
        self.axis_subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.move_callback, 10)
        self.button_subscription = self.create_subscription(Int8MultiArray, 'gamepad_button', self.button_callback, 10)

        # Register publisher for the serial topic
        self.serial_publisher = self.create_publisher(String, 'serial_topic', 10)

        # Initialize the reverse flag
        self.reverse = False

        # Initialize previous values to prevent serial flooding
        self.dpad_up = 0
        self.dpad_down = 0
        self.dpad_left = 0
        self.dpad_right = 0

    def move_callback(self, request):
        try:
            data = request.data

            # the direction the bot should go (-100 for left, 0 for striaght, 100 for right)
            lstick_x = data[2] 

            # data.data[1] comes in as -100 (trigger not pressed) to 100 (fully pressed)
            lstick_y = data[3]

            if direction < 15 and direction > -15:
                direction = 0

            if speed < 15 and speed > -15:
                speed = 0

            #we want to normalize the data between stopped and full_forward
            speed = (-lstick_y/100) * stopped + stopped
            direction = (lstick_y/100) * (stopped - full_reverse) * 0.75

            # For these motors, we want a value between 0 and 180, with 90 being stopped.
            # speed + 100 gives us a value between 0 and 200
            # (speed + 100) * 0.9 gives a value between 0 and 180
            left_speed = speed - direction
            right_speed = speed + direction

            # Constrain the speeds
            left_speed = round(max(full_reverse, min(left_speed, full_forward)))
            right_speed = round(max(full_reverse, min(right_speed, full_forward)))

            message = String()
            message.data = f"m,{left_speed},{right_speed}"
            self.serial_publisher.publish(message)

        except Exception as e:
            self.get_logger().error("Exception in gamepad axis callback:" + str(e))

    def button_callback(self, request):
        try:
            data = request.data

            dpad_up = data[12]
            dpad_down = data[13]
            dpad_left = data[14]
            dpad_right = data[15]

            if (dpad_up != self.dpad_up):
                message = String()
                message.data = f"g,{dpad_up},r"
                self.serial_publisher.publish(message)
                self.dpad_up = dpad_up

            if (dpad_down != self.dpad_down):
                message = String()
                message.data = f"g,{dpad_down},l"
                self.serial_publisher.publish(message)
                self.dpad_down = dpad_down

            if (dpad_right != self.dpad_right):
                message = String()
                message.data = f"d,{dpad_right},0"
                self.serial_publisher.publish(message)
                self.dpad_right = dpad_right

            if (dpad_left != self.dpad_left):
                message = String()
                message.data = f"b,{dpad_left},0"
                self.serial_publisher.publish(message)
                self.dpad_left = dpad_left

            
        except Exception as e:
            self.get_logger().error("Exception in gamepad button callback:" + str(e))


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