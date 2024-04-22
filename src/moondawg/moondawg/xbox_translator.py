import rclpy
from std_msgs.msg import Int8MultiArray, String
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import Node
from sys import exit
from os import _exit
from datetime import datetime

hardware_id = 0
forward = 'f'
backward = 'b'
left = 'l'
right = 'r'
up = 'u'
down = 'd'


class XboxTranslator(Node):


    def __init__(self):
        super().__init__(node_name='xbox_translator')

        # create heartbeat
        heartbeat_interval = 1
        self.diag = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK, hardware_id=str(hardware_id))
        self.diag_topic = self.create_publisher(DiagnosticStatus, 'xbox_translator_diag', 10)
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)

        # set movement values
        self.full_reverse = 50
        self.full_forward = 130
        self.stopped = (self.full_reverse+self.full_forward)/2


        self.belt_speeds = [100, 120, 130]
        self.belt_speed = 0

        # Register subscriptions to the gamepad topics
        self.axis_subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.axis_callback, 10)
        self.button_subscription = self.create_subscription(Int8MultiArray, 'gamepad_button', self.button_callback, 10)

        # Register publisher for the serial topic
        self.serial_publisher = self.create_publisher(String, 'serial_topic', 10)

        # Initialize previous values to prevent serial flooding
        self.dpad_up = 0
        self.dpad_down = 0
        self.dpad_left = 0
        self.dpad_right = 0
        self.left_speed = 0
        self.right_speed = 0
        self.button_x = 0

    def axis_callback(self, request):
        try:
            data = request.data

            # the direction the bot should go (-100 for left, 0 for striaght, 100 for right)
            lstick_x = data[0] 

            # data.data[1] comes in as -100 (trigger not pressed) to 100 (fully pressed)
            lstick_y = data[1]

            if lstick_x < 15 and lstick_x > -15:
                lstick_x = 0

            if lstick_y < 15 and lstick_y > -15:
                lstick_y = 0

            #we want to normalize the data between stopped and full_forward
            speed = (-lstick_y) * ((self.full_forward-self.full_reverse)/200) + self.stopped
            direction = (lstick_x) * ((self.full_forward-self.full_reverse)/200) * 0.75

            # For these motors, we want a value between 0 and 180, with 90 being stopped.
            # speed + 100 gives us a value between 0 and 200
            # (speed + 100) * 0.9 gives a value between 0 and 180
            left_speed = speed - direction
            right_speed = speed + direction

            # Constrain the speeds
            left_speed = round(max(self.full_reverse, min(left_speed, self.full_forward)))
            right_speed = round(max(self.full_reverse, min(right_speed, self.full_forward)))

            if (left_speed == self.left_speed and right_speed == self.right_speed):
                return

            message = String()
            message.data = f"m,{left_speed},{right_speed}"
            self.serial_publisher.publish(message)

            self.left_speed = left_speed
            self.right_speed = right_speed

        except Exception as e:
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = "Exception in gamepad axis callback."
            self.get_logger().error("Exception in gamepad axis callback:" + str(e))

    def parse_buttons(self, data):
        return {
            "dpad_up": data[12], 
            "dpad_down": data[13], 
            "dpad_left": data[14], 
            "dpad_right": data[15], 
            "button_x": data[3]
        }

    def button_callback(self, request):
        try:
            data = request.data

            buttons = self.parse_buttons(data)

            if (buttons["button_x"] and buttons["button_x"] != self.button_x):
                self.button_x = buttons["button_x"]
                self.belt_speed = (self.belt_speed + 1) % len(self.belt_speeds)
            elif (buttons["button_x"] == 0):
                self.button_x = 0

            if (buttons["dpad_up"] != self.dpad_up):
                message = self.belt_position_string(buttons["dpad_up"], up)
                self.serial_publisher.publish(message)
                self.dpad_up = buttons["dpad_up"]

            elif (buttons["dpad_down"] != self.dpad_down):
                message = self.belt_position_string(buttons["dpad_down"], down)
                self.serial_publisher.publish(message)
                self.dpad_down = buttons["dpad_down"]

            if (buttons["dpad_right"] != self.dpad_right):
                message = self.deposit_string(buttons["dpad_right"], forward)
                self.serial_publisher.publish(message)
                self.dpad_right = buttons["dpad_right"]

            elif (buttons["dpad_left"] != self.dpad_left):
                message = self.belt_string(buttons["dpad_left"])
                self.serial_publisher.publish(message)
                self.dpad_left = buttons["dpad_left"]
            
        except Exception as e:
            self.diag.level = DiagnosticStatus.WARN
            self.diag.message = "Exception in gamepad button callback."
            self.get_logger().error("Exception in gamepad button callback:" + str(e))

    def heartbeat(self):
        self.diag_topic.publish(self.diag)

    def deposit_string(self, enabled, direction):
        string = String()
        string.data = f"d,{enabled},{direction}"
        return string
    
    def belt_string(self, enabled):
        string = String()
        string.data = f"b,{enabled},{self.belt_speeds[self.belt_speed]}"
        return string
    
    def belt_position_string(self, enabled, direction):
        string = String()
        string.data = f"g,{enabled},{direction}"
        return string


def main(args=None):
    rclpy.init(args=args)

    xbox_translator = XboxTranslator()

    try:
        rclpy.spin(xbox_translator)
    except KeyboardInterrupt:
        xbox_translator.get_logger().warning('Ctrl-C pressed, shutting down...')
        try:
            exit(130)
        except:
            _exit(130)

    rclpy.shutdown()

if __name__ == '__main__':
    main()