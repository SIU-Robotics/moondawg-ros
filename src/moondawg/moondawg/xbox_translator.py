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
        self.belt_speeds = [120, 125, 180]
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
        self.button_b = 0
        
        self.camera_pitch = 0
        self.camera_angle = 0
        self.camera_arm = 0

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('xbox_translator/belt_speed_index', 0),
                ('xbox_translator/wheel_full_speed', 130),
                ('xbox_translator/wheel_full_stopped', 90),
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
    
    def calculate_speed(self, x, y):

        # Get the full forward and full reverse speeds
        full_forward = self.get_parameter('xbox_translator/wheel_full_speed').value
        stopped = self.get_parameter('xbox_translator/wheel_full_stopped').value
        full_reverse = stopped - (full_forward - stopped)

        # If the sticks are close to the center, set the speeds to 0
        if x < 15 and x > -15 and y < 15 and y > -15:
            x = 0
            y = 0

        # Calculate the speeds
        speed = (-y) * ((full_forward-full_reverse)/200) + stopped
        direction = (x) * ((full_forward-full_reverse)/200) * 0.75
        left_speed = speed - direction
        right_speed = speed + direction

        # Constrain the speeds
        left_speed = round(max(full_reverse, min(left_speed, full_forward)))
        right_speed = round(max(full_reverse, min(right_speed, full_forward)))

        return left_speed, right_speed

    def movement_handler(self, axis):

        left_speed, right_speed = self.calculate_speed(axis['lstick_x'], axis['lstick_y'])

        # If the speeds are the same as the last time, don't re-send the message            
        if (left_speed == self.left_speed and right_speed == self.right_speed):
            return
        else:
            self.left_speed = left_speed
            self.right_speed = right_speed

        # Publish the message to the serial topic
        message = self.movement_string(left_speed, right_speed)
        self.serial_publisher.publish(message)

    def camera_position_handler(self, axis):
        x = axis['rstick_x']
        y = axis['rstick_y']
        # If the sticks are close to the center, set the speeds to 0
        if x < 15 and x > -15 and y < 15 and y > -15:
            x = 0
            y = 0
            return
        
        if (x != 0):
            self.camera_angle = self.camera_angle + (x * 0.05)
        if (y != 0):
            self.camera_pitch = self.camera_pitch + (y * -0.05)
        

        self.camera_angle = max(0, min(self.camera_angle, 180))
        self.camera_pitch = max(0, min(self.camera_pitch, 180))

        self.serial_publisher.publish(self.camera_angle_string(round(self.camera_angle)))
        self.serial_publisher.publish(self.camera_pitch_string(round(self.camera_pitch)))

        

    # This function is called when the gamepad axis data is received
    def axis_callback(self, request):
        try:
            # Parse the data from the request
            axis = self.parse_axis(request.data)

            self.movement_handler(axis)
            self.camera_position_handler(axis)

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
            "button_x": data[2],
            "button_a": data[0],
            "button_b": data[1],
            "ltrigger": data[6],
            "rtrigger": data[7],
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
                self.belt_speed_index = (self.belt_speed_index + 1) % len(self.belt_speeds)
            elif (buttons["button_x"] == 0):
                self.button_x = 0

            if (buttons["button_b"] and buttons["button_b"] != self.button_b):
                self.button_b = buttons["button_b"]
                message = self.belt_speed_string(buttons["button_b"], 30)
                self.serial_publisher.publish(message)
            elif (buttons["button_b"] == 0):
                self.button_b = 0

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

            if (buttons['rtrigger']):
                self.camera_arm = self.camera_arm + 5
                self.camera_arm = max(0, min(self.camera_arm, 180))
                self.serial_publisher.publish(self.camera_arm_string(round(self.camera_arm)))
            elif (buttons['ltrigger']):
                self.camera_arm = self.camera_arm - 5
                self.camera_arm = max(0, min(self.camera_arm, 180))
                self.serial_publisher.publish(self.camera_arm_string(round(self.camera_arm)))
            
        except Exception as e:
            self.diag.level = DiagnosticStatus.WARN
            self.diag.message = "Exception in gamepad button callback."
            self.get_logger().error("Exception in gamepad button callback:" + str(e))

    def heartbeat(self):
        self.diag_topic.publish(self.diag)
        # self.parameter_topic.publish(self.get_parameters())

    def deposit_string(self, enabled, direction):
        string = String()
        string.data = f"d,{enabled},{direction}"
        return string
    
    def belt_string(self, enabled):
        string = String()
        string.data = f"b,{enabled},{self.belt_speeds[self.belt_speed_index]}"
        return string
    
    def belt_speed_string(self, enabled, speed):
        string = String()
        string.data = f"b,{enabled},{speed}"
        return string
    
    def belt_position_string(self, enabled, direction):
        string = String()
        string.data = f"g,{enabled},{direction}"
        return string
        
    def movement_string(self, lspeed, rspeed):
        string = String()
        string.data = f"m,{lspeed},{rspeed}"
        return string
    def vibrator_string(self, enabled):
        string = String()
        string.data = f"v,{enabled},v"
        return string
    
    def camera_pitch_string(self, pitch):
        string = String()
        string.data = f"e,{pitch},0"
        return string

    def camera_angle_string(self, angle):
        string = String()
        string.data = f"h,{angle},0"
        return string

    def camera_arm_string(self, angle):
        string = String()
        string.data = f"a,{angle},0"
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