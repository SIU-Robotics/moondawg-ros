from math import ceil
import rclpy
from std_msgs.msg import Int8MultiArray, String, Byte
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import Node
from sys import exit
from os import _exit
from cv_bridge import CvBridge
import cv2
import base64
import datetime
from .string_gen import StringGen

hardware_id = 0
belt_reverse_speed = 30
forward = 'f'
backward = 'b'
left = 'l'
right = 'r'
up = 'u'
down = 'd'

# handle the controller data and send a parsable string to the serial node
class XboxTranslator(Node):

    def init_vars(self):
        self.belt_speeds = [180, 125, 120]
        self.camera_pitch = 90
        self.camera_angle = 0
        self.camera_arm = 0
        self.belt_speed_index = 0
        self.connection_time = 0
        self.connected = 0

        # autonomy states
        self.time_start = 0
        self.depositing = False
        self.digging = False

        # button states
        self.dpad_up = 0
        self.dpad_down = 0
        self.dpad_left = 0
        self.dpad_right = 0
        self.left_speed = 0
        self.right_speed = 0
        self.button_x = 0
        self.button_a = 0
        self.button_b = 0
        self.button_y = 0
        self.lbutton = 0
        self.rbutton = 0
        self.ltrigger = 0
        self.rtrigger = 0
        self.select = 0
        self.menu = 0
        self.lstickbutton = 0
        self.rstickbutton = 0


    def __init__(self):
        super().__init__(node_name='xbox_translator')

        self.init_vars()
        self.declare_parameters(
            namespace='',
            parameters=[
                ('belt_speed_index', 0),
                ('wheel_full_speed', 130),
                ('wheel_full_stopped', 90),
            ])
        
        self.br = CvBridge()
        self.diag = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK, hardware_id=str(hardware_id))

        # create subscriptions, publishers, and timers
        self.diag_topic = self.create_publisher(DiagnosticStatus, 'xbox_translator_diag', 10)
        self.connection_topic = self.create_subscription(Byte, 'connection_status', self.connection_callback, 10)
        self.axis_subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.axis_callback, 10)
        self.button_subscription = self.create_subscription(Int8MultiArray, 'gamepad_button', self.button_callback, 10)
        self.serial_publisher = self.create_publisher(String, 'serial_topic', 10)
        self.heartbeat_timer = self.create_timer(1, self.heartbeat)
        self.auto_timer = self.create_timer(0.5, self.auto_callback)
        self.image_subscription = self.create_subscription(Image, 'image', self.image_translator, 10)
        self.image_pub = self.create_publisher(String, 'compressed_image', 10)

    # subroutine to dig without input
    def auto_callback(self):
        if (self.digging):
            if (self.time_start == 0):
                self.time_start = datetime.datetime.now()
            diff = ceil((datetime.datetime.now() - self.time_start).total_seconds())
            if (diff < 3):
                self.serial_publisher.publish(StringGen.belt_position_string(1, left))
            elif (diff >= 3 and diff < 5):
                self.serial_publisher.publish(StringGen.belt_string(1))
            elif (diff >= 17 and diff < 19):
                self.serial_publisher.publish(StringGen.belt_position_string(0, right))
            elif (diff >= 30):
                self.digging = False
        elif (not self.digging and self.time_start != 0):
            self.get_logger().info("stopping now!!")
            self.time_start = 0
            self.serial_publisher.publish(StringGen.belt_string(0))
            self.serial_publisher.publish(StringGen.belt_position_string(1, right))

    # called by website, tracks last time connected
    def connection_callback(self, message):
        self.connection_time = datetime.datetime.now()
        self.connected = 1

    # convert camera image to compressed base64
    def image_translator(self, message):
        _, encoded_img = cv2.imencode('.jpg', self.br.imgmsg_to_cv2(message), [cv2.IMWRITE_JPEG_QUALITY, 20])
        self.image_pub.publish(String(data=base64.b64encode(encoded_img.tobytes()).decode('utf-8')))

    # parse controller data from website to be easily referenced
    def parse_axis(self, data):
        return {
            "lstick_x": data[0],
            "lstick_y": data[1],
            "rstick_x": data[2],
            "rstick_y": data[3]
        }    
        
    def parse_buttons(self, data):
        return {
            "button_a": data[0]/100,
            "button_b": data[1]/100,
            "button_x": data[2]/100,
            "button_y": data[3]/100,
            "lbutton": data[4]/100,
            "rbutton": data[5]/100,
            "ltrigger": data[6],
            "rtrigger": data[7],
            "menu": data[8]/100,
            "select": data[9]/100,
            "lstick": data[11]/100,
            "rstick": data[10]/100,
            "dpad_up": data[12]/100, 
            "dpad_down": data[13]/100, 
            "dpad_left": data[14]/100, 
            "dpad_right": data[15]/100,
        }

    # This function is called when the gamepad axis data is received
    def axis_callback(self, request):
        try:
            # Parse the data from the request
            axis = self.parse_axis(request.data)

            self.movement_stick_handler(axis)
            self.camera_stick_handler(axis)

        except Exception as e:
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = "Exception in gamepad axis callback."
            self.get_logger().error("Exception in gamepad axis callback:" + str(e))

    def movement_stick_handler(self, axis):
        left_speed, right_speed = self.calculate_speed(axis['lstick_x'], axis['lstick_y'])
      
        if left_speed == self.left_speed and right_speed == self.right_speed:
            return
        
        self.left_speed = left_speed
        self.right_speed = right_speed

        self.serial_publisher.publish(StringGen.movement_string(left_speed, right_speed))

    def camera_stick_handler(self, axis):
        x = axis['rstick_x']
        y = axis['rstick_y']
        
        if abs(y) < 15:
            y = 0
        if abs(x) < 15:
            x = 0      

        camera_angle = max(0, min(self.camera_angle + (x * -0.05), 180))
        camera_pitch = max(90, min(self.camera_pitch - (y * -0.05), 180))

        if (camera_angle != self.camera_angle):
            self.camera_angle = camera_angle
            self.serial_publisher.publish(StringGen.camera_angle_string(round(self.camera_angle)))

        if (camera_pitch != self.camera_pitch):
            self.camera_pitch = camera_pitch
            self.serial_publisher.publish(StringGen.camera_pitch_string(round(self.camera_pitch)))
    
    def calculate_speed(self, x_axis, y_axis):
        full_forward = self.get_parameter('xbox_translator/wheel_full_speed').value
        stopped = self.get_parameter('xbox_translator/wheel_full_stopped').value
        full_reverse = stopped - (full_forward - stopped)

        if abs(x_axis) < 15 and abs(y_axis) < 15:
            x_axis = 0
            y_axis = 0

        speed = (-y_axis) * ((full_forward-full_reverse)/200) + stopped
        direction = (x_axis) * ((full_forward-full_reverse)/200) * 0.75

        left_speed = round(max(full_reverse, min(speed - direction, full_forward)))
        right_speed = round(max(full_reverse, min(speed + direction, full_forward)))

        return left_speed, right_speed

    # checks if autonomy is running
    def check_auto(self, buttons):
        if (buttons["select"] != self.select):
            self.select = buttons["select"]
            if (self.select):
                self.digging = not self.digging

    def camera_preset_handler(self, buttons):

        if (buttons["rstick"] != self.rstickbutton):
            self.rstickbutton = buttons["rstick"]
            if (self.rstickbutton):
                self.camera_arm = 105
                self.camera_angle = 5
                self.camera_pitch = 110
                self.serial_publisher.publish(StringGen.camera_arm_string(self.camera_arm))
                self.serial_publisher.publish(StringGen.camera_angle_string(self.camera_angle))
                self.serial_publisher.publish(StringGen.camera_pitch_string(self.camera_pitch))
            
        if (buttons["lstick"] != self.lstickbutton):
            self.lstickbutton = buttons["lstick"]
            if (self.lstickbutton):
                self.camera_arm = 105
                self.camera_angle = 180
                self.camera_pitch = 105
                self.serial_publisher.publish(StringGen.camera_arm_string(self.camera_arm))
                self.serial_publisher.publish(StringGen.camera_angle_string(self.camera_angle))
                self.serial_publisher.publish(StringGen.camera_pitch_string(self.camera_pitch))

        if (buttons["menu"] != self.menu):
            self.menu = buttons["menu"]
            if (self.menu):
                self.camera_arm = 180
                self.camera_angle = 44
                self.camera_pitch = 100
                self.serial_publisher.publish(StringGen.camera_arm_string(self.camera_arm))
                self.serial_publisher.publish(StringGen.camera_angle_string(self.camera_angle))
                self.serial_publisher.publish(StringGen.camera_pitch_string(self.camera_pitch))

    def belt_handler(self, buttons):

        if (buttons["button_y"] and buttons["button_y"] != self.button_y):
            self.button_y = buttons["button_y"]
            self.belt_speed_index = (self.belt_speed_index + 1) % len(self.belt_speeds)
        elif (buttons["button_y"] == 0 and buttons["button_y"] != self.button_y):
            self.button_y = 0

        if (buttons["button_b"] != self.button_b):
            self.button_b = buttons["button_b"]
            self.serial_publisher.publish(StringGen.belt_string(buttons["button_b"], belt_reverse_speed))

        if (buttons["dpad_up"] != self.dpad_up):
            self.serial_publisher.publish(StringGen.belt_position_string(buttons["dpad_up"], right))
            self.dpad_up = buttons["dpad_up"]
        
        if (buttons["dpad_down"] != self.dpad_down):
            self.serial_publisher.publish(StringGen.belt_position_string(buttons["dpad_down"], left))
            self.dpad_down = buttons["dpad_down"]

        if (buttons["lbutton"] != self.lbutton):
            self.serial_publisher.publish(StringGen.belt_string(buttons["lbutton"], self.belt_speeds[self.belt_speed_index]))
            self.lbutton = buttons["lbutton"]

    def button_movement_handler(self, buttons):

        if (buttons["rbutton"] != self.rbutton):
            self.rbutton = buttons["rbutton"]
            if (self.rbutton):
                self.serial_publisher.publish(StringGen.movement_string(100, 100))
            else:
                self.serial_publisher.publish(StringGen.movement_string(90, 90))

        if (buttons['rtrigger'] != self.rtrigger):
            self.rtrigger = buttons['rtrigger']
            if (self.rtrigger):
                lspeed, rspeed = self.calculate_speed(0, -(self.rtrigger*0.8)-15)
                self.serial_publisher.publish(StringGen.movement_string(lspeed, rspeed))
            else:
                self.serial_publisher.publish(StringGen.movement_string(90, 90))

        if (buttons['ltrigger'] != self.ltrigger):
            self.ltrigger = buttons['ltrigger']
            if (self.ltrigger):
                lspeed, rspeed = self.calculate_speed(0, (self.ltrigger*0.8)+15)
                self.serial_publisher.publish(StringGen.movement_string(lspeed, rspeed))
            else:
                self.serial_publisher.publish(StringGen.movement_string(90, 90))

    def misc_button_handler(self, buttons):

        if (buttons["button_a"] != self.button_a):
            self.button_a = buttons["button_a"]
            self.serial_publisher.publish(StringGen.vibrator_string(buttons["button_a"]))
        if (buttons["button_x"] != self.button_x):
            self.serial_publisher.publish(StringGen.deposit_string(buttons["button_x"], forward))
            self.button_x = buttons["button_x"]
        if (buttons['dpad_left']):
            self.camera_arm = self.camera_arm + 5
            self.camera_arm = max(0, min(self.camera_arm, 180))
            self.serial_publisher.publish(StringGen.camera_arm_string(round(self.camera_arm)))
        elif (buttons['dpad_right']):
            self.camera_arm = self.camera_arm - 5
            self.camera_arm = max(0, min(self.camera_arm, 180))
            self.serial_publisher.publish(StringGen.camera_arm_string(round(self.camera_arm)))

    # called when website sends controller data
    def button_callback(self, request):
        try:
            data = request.data
            buttons = self.parse_buttons(data)

            self.check_auto(buttons)

            if (self.digging):
                return
            
            self.camera_preset_handler(buttons)
            self.belt_handler(buttons)
            self.button_movement_handler(buttons)
            self.misc_button_handler(buttons)

        except Exception as e:
            self.diag.level = DiagnosticStatus.WARN
            self.diag.message = "Exception in gamepad button callback."
            self.get_logger().error("Exception in gamepad button callback:" + str(e))

    # called when everything must stop!
    def stop_all(self):
        self.serial_publisher.publish(StringGen.movement_string(90, 90))
        self.serial_publisher.publish(StringGen.belt_string(0, 90))
        self.serial_publisher.publish(StringGen.belt_position_string(0, right))
        self.serial_publisher.publish(StringGen.deposit_string(0, forward))
        self.serial_publisher.publish(StringGen.vibrator_string(0))

    # called on an interval to publish info about the node
    def heartbeat(self):
        self.diag_topic.publish(self.diag)
        if (self.connected == 1 and (datetime.datetime.now() - self.connection_time).total_seconds() > 2):
            self.connected = 0
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = "Connection to gamepad lost."
            self.get_logger().info("**\n**\n**\nConnection lost!!!!\n**\n**\n**")
            self.stop_all()

# start node
def main(args=None):
    rclpy.init(args=args)

    xbox_translator = XboxTranslator()

    try:
        rclpy.spin(xbox_translator)
    except KeyboardInterrupt:
        xbox_translator.get_logger().warning('Ctrl-C pressed, shutting down...')
    finally:
        xbox_translator.stop_all()
        rclpy.shutdown()
        exit(130)

if __name__ == '__main__':
    main()