from shutil import move
from rclpy import init, shutdown, spin
from std_msgs.msg import Int8MultiArray, String
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import Node
from rclpy.parameter import Parameter
from sys import exit
from os import _exit

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

        # Set interval of heartbeat
        heartbeat_interval = 1

        # Initialize belt speeds and current speed
        self.belt_speeds = [100, 120, 130]
        self.belt_speed_index = 0

        # Initialize previous values to prevent serial flooding
        self.dpad_up = 0
        self.dpad_down = 0
        self.dpad_left = 0
        self.dpad_right = 0
        self.left_speed = 0
        self.right_speed = 0
        self.button_x = 0
        self.button_a = 0

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('xbox_translator/belt_speed', Parameter.Type.INTEGER, 0),
                ('xbox_translator/wheel_full_speed', Parameter.Type.INTEGER, 130),
                ('xbox_translator/wheel_full_stopped', Parameter.Type.INTEGER, 90),
            ])

        # Initialize diagnostics
        self.diag = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK, hardware_id=str(hardware_id))
        self.diag_topic = self.create_publisher(DiagnosticStatus, 'xbox_translator_diag', 10)

        # Register subscriptions to the gamepad topics
        self.axis_subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.axis_callback, 10)
        self.button_subscription = self.create_subscription(Int8MultiArray, 'gamepad_button', self.button_callback, 10)

        # Register publisher for the serial topic
        self.serial_publisher = self.create_publisher(String, 'serial_topic', 10)

        # Create heartbeat callback timer
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)

    def parse_axis(self, data):
        return {
            "lstick_x": data[0],
            "lstick_y": data[1],
            "rstick_x": data[2],
            "rstick_y": data[3]
        }
    
    def calculate_speed(self, axis):

        # Get the full forward and full reverse speeds
        full_forward = self.get_parameter('xbox_translator/wheel_full_speed').value
        stopped = self.get_parameter('xbox_translator/wheel_full_stopped').value
        full_reverse = stopped - (full_forward - stopped)

        # If the sticks are close to the center, set the speeds to 0
        if axis["lstick_x"] < 15 and axis["lstick_x"] > -15 and axis["lstick_y"] < 15 and axis["lstick_y"] > -15:
            axis["lstick_x"] = 0
            axis["lstick_y"] = 0

        # Calculate the speeds
        speed = (-axis["lstick_y"]) * ((full_forward-full_reverse)/200) + stopped
        direction = (axis["lstick_x"]) * ((full_forward-full_reverse)/200) * 0.75
        left_speed = speed - direction
        right_speed = speed + direction

        # Constrain the speeds
        left_speed = round(max(full_reverse, min(left_speed, full_forward)))
        right_speed = round(max(full_reverse, min(right_speed, full_forward)))

        return left_speed, right_speed

    # This function is called when the gamepad axis data is received
    def axis_callback(self, request):
        try:
            # Parse the data from the request
            axis = self.parse_axis(request.data)
            
            left_speed, right_speed = self.calculate_speed(axis)

            # If the speeds are the same as the last time, don't re-send the message            
            if (left_speed == self.left_speed and right_speed == self.right_speed):
                return
            else:
                self.left_speed = left_speed
                self.right_speed = right_speed

            # Publish the message to the serial topic
            message = self.movement_string(left_speed, right_speed)
            self.serial_publisher.publish(message)

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
            "button_x": data[3],
            "button_a": data[0]
        }

    # This function is called when the gamepad button data is received
    def button_callback(self, request):
        try:
            # Parse the data from the request
            data = request.data
            buttons = self.parse_buttons(data)

            # If the belt speed button is pressed (X), cycle the belt speed
            if (buttons["button_x"] and buttons["button_x"] != self.button_x):
                self.button_x = buttons["button_x"]
                self.belt_speed = (self.belt_speed + 1) % len(self.belt_speeds)
            elif (buttons["button_x"] == 0):
                self.button_x = 0

            # If dpad up or down is pressed, move the belt up or down
            if (buttons["button_a"] != self.button_a):
                self.button_a = buttons["button_a"]
                message = self.vibrator_string(buttons["button_a"])
                self.serial_publisher.publish(message)

            if (buttons["dpad_up"] != self.dpad_up):
                message = self.belt_position_string(buttons["dpad_up"], up)
                self.serial_publisher.publish(message)
                self.dpad_up = buttons["dpad_up"]
            elif (buttons["dpad_down"] != self.dpad_down):
                message = self.belt_position_string(buttons["dpad_down"], down)
                self.serial_publisher.publish(message)
                self.dpad_down = buttons["dpad_down"]

            # If dpad right is pressed, start depositing
            if (buttons["dpad_right"] != self.dpad_right):
                message = self.deposit_string(buttons["dpad_right"], forward)
                self.serial_publisher.publish(message)
                self.dpad_right = buttons["dpad_right"]

            # If dpad left is pressed, start digging
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
        self.parameter_topic.publish(self.get_parameters())

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

        return string
        
    def movement_string(self, lspeed, rspeed):
        string = String()
        string.data = f"m,{lspeed},{rspeed}"
        return string
    def vibrator_string(self, enabled):
        string = String()
        string.data = f"v,{enabled},v"
        return string


def main(args=None):
    init(args=args)

    xbox_translator = XboxTranslator()

    try:
        spin(xbox_translator)
    except KeyboardInterrupt:
        xbox_translator.get_logger().warning('Ctrl-C pressed, shutting down...')
        try:
            exit(130)
        except:
            _exit(130)

    shutdown()

if __name__ == '__main__':
    main()