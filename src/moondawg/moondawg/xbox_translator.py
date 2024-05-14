from math import ceil, floor
from shutil import move
from time import sleep
from rclpy import init, shutdown, spin
from std_msgs.msg import Int8MultiArray, String, Byte
from sensor_msgs.msg import Image, CompressedImage
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import Node
from rclpy.parameter import Parameter
from sys import exit
from os import _exit
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import datetime

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
        self.belt_speeds = [180, 125, 120]
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
        self.button_y = 0
        self.lbutton = 0
        self.rbutton = 0
        self.ltrigger = 0
        self.rtrigger = 0
        self.select = 0
        self.menu = 0
        self.lstickbutton = 0
        self.rstickbutton = 0
        
        self.camera_pitch = 90
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
        self.connection_topic = self.create_subscription(Byte, 'connection_status', self.connection_callback, 10)
        self.connection_status = 0
        self.connected = 0

        # Register subscriptions to the gamepad topics
        self.axis_subscription = self.create_subscription(Int8MultiArray, 'gamepad_axis', self.axis_callback, 10)
        self.button_subscription = self.create_subscription(Int8MultiArray, 'gamepad_button', self.button_callback, 10)

        # Register publisher for the serial topic
        self.serial_publisher = self.create_publisher(String, 'serial_topic', 10)

        # Create heartbeat callback timer
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)


        self.time_start = 0
        self.depositing = False
        self.digging = False
        self.auto_timer = self.create_timer(0.5, self.auto_callback)

        
        # Assuming 'img' is your image loaded with OpenCV
        self.br = CvBridge()

        camera_angle = 90
        self.image_subscription = self.create_subscription(Image, 'image', self.image_translator, 10)
        self.image_pub = self.create_publisher(String, 'compressed_image', 10)
        self.temp = 0

    def auto_callback(self):
        # if (self.depositing):
        #     if (self.time_start == 0):
        #         self.time_start = datetime.datetime.now()
        #     diff = (datetime.datetime.now() - self.time_start).total_seconds()
        #     if (diff > 3 and diff < 5):
        #         self.depositing = False
        #         self.time_start = 0
        #         message = self.deposit_string(0, forward)
        #         self.serial_publisher.publish(message)
        #     elif (diff > 5 and diff < 30):
        #         pass

        if (self.digging):
            if (self.time_start == 0):
                self.time_start = datetime.datetime.now()
            diff = ceil((datetime.datetime.now() - self.time_start).total_seconds())
            if (diff < 3):
                message = self.belt_position_string(1, left)
                self.serial_publisher.publish(message)
            elif (diff >= 3 and diff < 5):
                message = self.belt_string(1)
                self.serial_publisher.publish(message)
            elif (diff >= 15 and diff < 17):
                message = self.belt_position_string(0, right)
                self.serial_publisher.publish(message)
            # elif (diff >= 16 and diff <  and diff % 2 == 0):
            #     message = self.movement_string(98, 98)
            #     self.serial_publisher.publish(message)
            # elif (diff >= 10 and diff < 30 and diff % 2 == 1):
            #     message = self.movement_string(90, 90)
            #     self.serial_publisher.publish(message)
            elif (diff >= 30):
                self.digging = False
        elif (not self.digging and self.time_start != 0):
            self.get_logger().info("stopping now!!")
            self.time_start = 0
            message = self.belt_string(0)
            self.serial_publisher.publish(message)
            message = self.belt_position_string(1, right)
            self.serial_publisher.publish(message)

    def connection_callback(self, message):
        self.connection_status = datetime.datetime.now()
        self.connected = 1

    def image_translator(self, message):
        frame = self.br.imgmsg_to_cv2(message)
        # Encode image to JPEG format
        _, encoded_img = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 20])

        # Convert the encoded image to bytes
        encoded_bytes = encoded_img.tobytes()

        # Encode bytes to base64
        encoded_base64 = base64.b64encode(encoded_bytes).decode('utf-8')

        img_to_pub = String()
        img_to_pub.data = encoded_base64

        self.image_pub.publish(img_to_pub)

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
        if y < 15 and y > -15:
            y = 0

        if x < 15 and x > -15:
            x = 0
        
        camera_angle = self.camera_angle + (x * -0.05)        
        camera_pitch = self.camera_pitch - (y * -0.05)        

        camera_angle = max(0, min(camera_angle, 180))
        camera_pitch = max(90, min(camera_pitch, 180))

        if (camera_angle != self.camera_angle):
            self.camera_angle = camera_angle
            self.serial_publisher.publish(self.camera_angle_string(round(self.camera_angle)))

        if (camera_pitch != self.camera_pitch):
            self.serial_publisher.publish(self.camera_pitch_string(round(self.camera_pitch)))
            self.camera_pitch = camera_pitch

        

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
            "lstick": data[10]/100,
            "rstick": data[11]/100,
            "dpad_up": data[12]/100, 
            "dpad_down": data[13]/100, 
            "dpad_left": data[14]/100, 
            "dpad_right": data[15]/100,
        }

    # This function is called when the gamepad button data is received
    def button_callback(self, request):
        try:
            # Parse the data from the request
            data = request.data
            buttons = self.parse_buttons(data)

            if (buttons["select"] != self.select):
                self.select = buttons["select"]
                if (self.select):
                    self.digging = not self.digging

            if (self.digging):
                return

            if (buttons["rstick"] != self.rstickbutton):
                self.rstickbutton = buttons["rstick"]
                if (self.rstickbutton):
                    message = self.camera_arm_string(105)
                    self.serial_publisher.publish(message)
                    message = self.camera_angle_string(5)
                    self.serial_publisher.publish(message)
                    message = self.camera_pitch_string(99)
                    self.serial_publisher.publish(message)
            
            if (buttons["lstick"] != self.lstickbutton):
                self.lstickbutton = buttons["lstick"]
                if (self.lstickbutton):
                    message = self.camera_arm_string(105)
                    self.serial_publisher.publish(message)
                    message = self.camera_angle_string(180)
                    self.serial_publisher.publish(message)
                    message = self.camera_pitch_string(110)
                    self.serial_publisher.publish(message)

            if (buttons["menu"] != self.menu):
                self.menu = buttons["menu"]
                if (self.menu):
                    message = self.camera_arm_string(180)
                    self.serial_publisher.publish(message)
                    message = self.camera_angle_string(44)
                    self.serial_publisher.publish(message)
                    message = self.camera_pitch_string(100)
                    self.serial_publisher.publish(message)

            # If the belt speed button is pressed (X), cycle the belt speed
            if (buttons["button_y"] and buttons["button_y"] != self.button_x):
                self.button_y = buttons["button_y"]
                self.belt_speed_index = (self.belt_speed_index + 1) % len(self.belt_speeds)
            elif (buttons["button_y"] == 0 and buttons["button_y"] != self.button_y):
                self.button_y = 0


            if (buttons["button_b"] != self.button_b):
                self.button_b = buttons["button_b"]
                message = self.belt_speed_string(buttons["button_b"], 30)
                self.serial_publisher.publish(message)

            if (buttons["rbutton"] != self.rbutton):
                self.rbutton = buttons["rbutton"]
                if (self.rbutton):
                    message = self.movement_string(100, 100)
                else:
                    message = self.movement_string(90, 90)
                self.serial_publisher.publish(message)

            if (buttons['rtrigger'] != self.rtrigger):
                self.rtrigger = buttons['rtrigger']
                if (self.rtrigger):
                    lspeed, rspeed = self.calculate_speed(0, -(self.rtrigger*0.8)-15)
                    message = self.movement_string(lspeed, rspeed)
                else:
                    message = self.movement_string(90, 90)
                self.serial_publisher.publish(message)

            if (buttons['ltrigger'] != self.ltrigger):
                self.ltrigger = buttons['ltrigger']
                if (self.ltrigger):
                    lspeed, rspeed = self.calculate_speed(0, (self.ltrigger*0.8)+15)
                    message = self.movement_string(lspeed, rspeed)
                else:
                    message = self.movement_string(90, 90)
                self.serial_publisher.publish(message)

            # If dpad up or down is pressed, move the belt up or down
            if (buttons["button_a"] != self.button_a):
                self.button_a = buttons["button_a"]
                message = self.vibrator_string(buttons["button_a"])
                self.serial_publisher.publish(message)

            if (buttons["dpad_up"] != self.dpad_up):
                message = self.belt_position_string(buttons["dpad_up"], right)
                self.serial_publisher.publish(message)
                self.dpad_up = buttons["dpad_up"]
            elif (buttons["dpad_down"] != self.dpad_down):
                message = self.belt_position_string(buttons["dpad_down"], left)
                self.serial_publisher.publish(message)
                self.dpad_down = buttons["dpad_down"]

            if (buttons["button_x"] != self.button_x):
                message = self.deposit_string(buttons["button_x"], forward)
                self.serial_publisher.publish(message)
                self.button_x = buttons["button_x"]

            if (buttons["lbutton"] != self.lbutton):
                message = self.belt_string(buttons["lbutton"])
                self.serial_publisher.publish(message)
                self.lbutton = buttons["lbutton"]

            if (buttons['dpad_left']):
                self.camera_arm = self.camera_arm + 5
                self.camera_arm = max(0, min(self.camera_arm, 180))
                self.serial_publisher.publish(self.camera_arm_string(round(self.camera_arm)))
            elif (buttons['dpad_right']):
                self.camera_arm = self.camera_arm - 5
                self.camera_arm = max(0, min(self.camera_arm, 180))
                self.serial_publisher.publish(self.camera_arm_string(round(self.camera_arm)))
            
        except Exception as e:
            self.diag.level = DiagnosticStatus.WARN
            self.diag.message = "Exception in gamepad button callback."
            self.get_logger().error("Exception in gamepad button callback:" + str(e))

    def heartbeat(self):
        self.diag_topic.publish(self.diag)
        if (self.connected == 1 and (datetime.datetime.now() - self.connection_status).total_seconds() > 2):
            self.connected = 0
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = "Connection to gamepad lost."
            self.get_logger().info("**\n**\n**\nConnection lost!!!!\n**\n**\n**")
            self.stop_all()
        # self.parameter_topic.publish(self.get_parameters())


    def stop_all(self):
        message = self.movement_string(90, 90)
        self.serial_publisher.publish(message)
        message = self.belt_speed_string(0, 90)
        self.serial_publisher.publish(message)
        message = self.belt_position_string(0, right)
        self.serial_publisher.publish(message)
        message = self.belt_string(0)
        self.serial_publisher.publish(message)
        message = self.deposit_string(0, forward)
        self.serial_publisher.publish(message)
        message = self.vibrator_string(0)
        self.serial_publisher.publish(message)


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
        string.data = f"e,1,{pitch}"
        return string

    def camera_angle_string(self, angle):
        string = String()
        string.data = f"h,1,{angle}"
        return string

    def camera_arm_string(self, angle):
        string = String()
        string.data = f"a,1,{angle}"
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