from math import ceil, atan2, degrees, hypot
import rclpy
from std_msgs.msg import Int8MultiArray, String, Byte
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.lifecycle import Node
from cv_bridge import CvBridge
import cv2
import base64
import datetime
from .string_gen import StringGen

belt_reverse_speed = 30
forward = 'f'
backward = 'b'
left = 'l'
right = 'r'
up = 'u'
down = 'd'

FRONT_LEFT_ADDR  = 0x10
FRONT_RIGHT_ADDR = 0x11
REAR_LEFT_ADDR   = 0x12
REAR_RIGHT_ADDR  = 0x13
STEERING_SERVO_ADDR = 0x14
BELT_ADDR  = 0x20
AUGER_ADDR = 0x21

def clamp(value, low, high):
    return max(low, min(value, high))


class ControllerParser(Node):

    def init_vars(self):
        self.belt_speeds = [180, 125, 120]
        # self.camera_pitch = 90
        # self.camera_angle = 0
        # self.camera_arm = 0
        self.belt_speed_index = 0
        self.connection_time = 0
        self.connected = 0

        # autonomy states
        self.time_start = 0
        self.depositing = False
        self.digging = False

        # driving mode (0 = “Rotate” mode, 1 = “Crab” mode)
        self.driving_mode = 0

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
        super().__init__(node_name='controller_parser')
        self.init_vars()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('belt_speed_index', 0),
            ]
        )
        
        self.br = CvBridge()
        self.diagnostic_status = DiagnosticStatus(name=self.get_name(), level=DiagnosticStatus.OK)

        # Publishers / Subscribers / Timers
        self.diag_topic = self.create_publisher(DiagnosticStatus, '/controller_parser/diag', 10)
        self.connection_topic = self.create_subscription(Byte, '/connection_status', self.connection_callback, 10)
        self.axis_subscription = self.create_subscription(Int8MultiArray, '/controller_parser/gamepad_axis', self.axis_callback, 10)
        self.button_subscription = self.create_subscription(Int8MultiArray, '/controller_parser/gamepad_button', self.button_callback, 10)

        # We won’t use serial_publisher anymore:
        # self.serial_publisher = self.create_publisher(String, '/serial_node/serial', 10)

        self.i2c_publisher = self.create_publisher(String, '/i2c_node/command', 10)

        self.heartbeat_timer = self.create_timer(1, self.heartbeat)
        self.auto_timer = self.create_timer(0.5, self.auto_callback)

        self.image_subscription = self.create_subscription(Image, 'image', self.image_translator, 10)
        self.image_pub = self.create_publisher(String, '/controller_parser/compressed_image', 10)

        self.diag(DiagnosticStatus.OK, "Controller parser ready.")

    # -------------------------------------------------------------------------
    # Subroutine to dig without input (autonomy)
    # -------------------------------------------------------------------------
    def auto_callback(self):
        if self.digging:
            if self.time_start == 0:
                self.time_start = datetime.datetime.now()
                self.diag(DiagnosticStatus.OK, "Running dig autonomy subroutine.")
            diff = ceil((datetime.datetime.now() - self.time_start).total_seconds())
            if diff < 3:
                # Move belt position left
                cmd = StringGen.belt_position_string(1, left)
                self.send_i2c(BELT_ADDR, cmd)

            elif 3 <= diff < 5:
                # Turn belt on
                cmd = StringGen.belt_string(1)
                self.send_i2c(BELT_ADDR, cmd)

            elif 17 <= diff < 19:
                # Move belt position right
                cmd = StringGen.belt_position_string(0, right)
                self.send_i2c(BELT_ADDR, cmd)

            elif diff >= 30:
                self.digging = False

        elif not self.digging and self.time_start != 0:
            self.diag(DiagnosticStatus.OK, "Dig autonomy subroutine finished.")
            self.time_start = 0
            # Stop belt
            cmd_stop = StringGen.belt_string(0)
            self.send_i2c(BELT_ADDR, cmd_stop)
            # Move belt position right (reset)
            cmd_pos = StringGen.belt_position_string(1, right)
            self.send_i2c(BELT_ADDR, cmd_pos)

    # -------------------------------------------------------------------------
    # Called by website, tracks last time connected
    # -------------------------------------------------------------------------
    def connection_callback(self, message):
        self.connection_time = datetime.datetime.now()
        self.connected = 1

    # -------------------------------------------------------------------------
    # Convert camera image to compressed base64 (unchanged)
    # -------------------------------------------------------------------------
    def image_translator(self, message):
        cv_image = self.br.imgmsg_to_cv2(message)
        _, encoded_img = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 20])
        self.image_pub.publish(String(data=base64.b64encode(encoded_img.tobytes()).decode('utf-8')))

    def parse_axis(self, data):
        return {
            "lstick_x": data[0],  # -100 .. 100
            "lstick_y": data[1],  # -100 .. 100
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
            "rstick": data[10]/100,
            "lstick": data[11]/100,
            "dpad_up": data[12]/100, 
            "dpad_down": data[13]/100, 
            "dpad_left": data[14]/100, 
            "dpad_right": data[15]/100,
        }

    # -------------------------------------------------------------------------
    # Four-wheel steering logic
    # -------------------------------------------------------------------------
    def four_wheel_steering_handler(self, x, y):
        """
        Given x, y from left stick in [-100..100],
        compute servo angle + motor speed for each of the 4 wheels.
        
        Modes:
        - Crab mode (1): All wheels point in the same direction for translation with no rotation
        - Zero-point turn mode (0): Wheels positioned to enable rotation around the center
        
        Each servo gets a separate position command (0-180) and each motor gets a speed (0-180)
        90 = forward/stopped, 0 = left/reverse, 180 = right/forward
        """
        # Normalize inputs to [-1.0, 1.0]
        x_f = x / 100.0
        y_f = y / 100.0
        
        # Calculate magnitude of joystick movement (0.0 to 1.0)
        magnitude = hypot(x_f, y_f)
        
        # Ignore very small movements
        if magnitude < 0.1:
            # Stop all motors and center all wheels
            for addr in [FRONT_LEFT_ADDR, FRONT_RIGHT_ADDR, REAR_LEFT_ADDR, REAR_RIGHT_ADDR]:
                self.send_i2c(addr, 90)  # Stop motor
            
            for channel in range(1, 5):
                self.send_i2c(STEERING_SERVO_ADDR, (90, channel))  # Center servo
            return
            
        if self.driving_mode == 1:
            # -----------------
            # Crab mode - all wheels point in same direction, no rotation
            # -----------------
            
            # Calculate angle from joystick position (in degrees)
            # atan2(y, x) gives angle from positive x-axis, but we need from positive y-axis
            # so we use atan2(x, y) instead and adjust
            angle_radians = atan2(x_f, y_f)
            servo_angle = 90 + degrees(angle_radians)  # Convert to 0-180 range
            servo_angle = clamp(int(servo_angle), 0, 180)
            
            # Set all servos to the same angle
            for channel in range(1, 5):
                self.send_i2c(STEERING_SERVO_ADDR, (servo_angle, channel))
            
            # Calculate speed based on magnitude (90 = stopped, 180 = full forward, 0 = full reverse)
            # Sign based on direction relative to forward
            speed = 90 - int(90 * magnitude * (1 if y_f >= 0 else -1))
            speed = clamp(speed, 0, 180)
            
            # Apply the same speed to all motors
            self.send_i2c(FRONT_LEFT_ADDR, speed)
            self.send_i2c(FRONT_RIGHT_ADDR, speed)
            self.send_i2c(REAR_LEFT_ADDR, speed)
            self.send_i2c(REAR_RIGHT_ADDR, speed)
            
        else:
            # -----------------
            # Zero-point turn mode - wheels positioned to maximize rotation
            # -----------------
            
            # For small Y movement but larger X movement, do a zero-point turn
            if abs(x_f) > abs(y_f) and abs(x_f) > 0.3:
                # For rotation, front wheels point one way, rear wheels the opposite
                if x_f > 0:  # Clockwise rotation
                    # Front wheels right, rear wheels left
                    self.send_i2c(STEERING_SERVO_ADDR, (135, 1))  # Front left -> right
                    self.send_i2c(STEERING_SERVO_ADDR, (135, 2))  # Front right -> right  
                    self.send_i2c(STEERING_SERVO_ADDR, (45, 3))   # Rear left -> left
                    self.send_i2c(STEERING_SERVO_ADDR, (45, 4))   # Rear right -> left
                    
                    # Set speeds for rotation - front wheels forward, rear wheels reverse
                    speed = 90 + int(90 * abs(x_f))
                    rev_speed = 90 - int(90 * abs(x_f))
                    speed = clamp(speed, 0, 180)
                    rev_speed = clamp(rev_speed, 0, 180)
                    
                    self.send_i2c(FRONT_LEFT_ADDR, speed)
                    self.send_i2c(FRONT_RIGHT_ADDR, speed)
                    self.send_i2c(REAR_LEFT_ADDR, rev_speed)
                    self.send_i2c(REAR_RIGHT_ADDR, rev_speed)
                    
                else:  # Counter-clockwise rotation
                    # Front wheels left, rear wheels right
                    self.send_i2c(STEERING_SERVO_ADDR, (45, 1))   # Front left -> left
                    self.send_i2c(STEERING_SERVO_ADDR, (45, 2))   # Front right -> left
                    self.send_i2c(STEERING_SERVO_ADDR, (135, 3))  # Rear left -> right
                    self.send_i2c(STEERING_SERVO_ADDR, (135, 4))  # Rear right -> right
                    
                    # Set speeds for rotation - front wheels reverse, rear wheels forward
                    speed = 90 + int(90 * abs(x_f))
                    rev_speed = 90 - int(90 * abs(x_f))
                    speed = clamp(speed, 0, 180)
                    rev_speed = clamp(rev_speed, 0, 180)
                    
                    self.send_i2c(FRONT_LEFT_ADDR, rev_speed)
                    self.send_i2c(FRONT_RIGHT_ADDR, rev_speed)
                    self.send_i2c(REAR_LEFT_ADDR, speed)
                    self.send_i2c(REAR_RIGHT_ADDR, speed)
            else:
                # For forward/backward movement with some turning
                # First, set all wheels pointing forward
                for channel in range(1, 5):
                    self.send_i2c(STEERING_SERVO_ADDR, (90, channel))
                
                # Calculate speed based on y_axis (positive = forward, negative = backward)
                base_speed = 90 + int(90 * y_f)
                base_speed = clamp(base_speed, 0, 180)
                
                # Apply differential steering for turning while moving
                turn_factor = x_f * 0.5  # Reduce turning sensitivity
                
                left_speed = int(base_speed - (turn_factor * 90))
                right_speed = int(base_speed + (turn_factor * 90))
                
                left_speed = clamp(left_speed, 0, 180)
                right_speed = clamp(right_speed, 0, 180)
                
                # Apply speeds to the left and right sides
                self.send_i2c(FRONT_LEFT_ADDR, left_speed)
                self.send_i2c(REAR_LEFT_ADDR, left_speed)
                self.send_i2c(FRONT_RIGHT_ADDR, right_speed)
                self.send_i2c(REAR_RIGHT_ADDR, right_speed)

    # -------------------------------------------------------------------------
    # Axis callback => 4WS for left stick
    # (Camera control commented out)
    # -------------------------------------------------------------------------
    def axis_callback(self, request):
        try:
            axis = self.parse_axis(request.data)
            self.four_wheel_steering_handler(axis['lstick_x'], axis['lstick_y'])
            
            # Commented out camera control:
            # self.camera_stick_handler(axis)
        except Exception as e:
            self.diag(DiagnosticStatus.ERROR, f"Exception in gamepad axis callback: {str(e)}")


    # -------------------------------------------------------------------------
    # If autonomy is running, ignore manual commands
    # -------------------------------------------------------------------------
    def check_auto(self, buttons):
        if buttons["select"] != self.select:
            self.select = buttons["select"]
            if self.select:
                self.digging = not self.digging

    # -------------------------------------------------------------------------
    # Belt / deposit commands => I2C
    # -------------------------------------------------------------------------
    def belt_handler(self, buttons):
        # Y button cycles belt speeds
        if buttons["button_y"] and buttons["button_y"] != self.button_y:
            self.button_y = buttons["button_y"]
            self.belt_speed_index = (self.belt_speed_index + 1) % len(self.belt_speeds)

        elif buttons["button_y"] == 0 and buttons["button_y"] != self.button_y:
            self.button_y = 0

        # B button => belt reverse, or off
        if buttons["button_b"] != self.button_b:
            self.button_b = buttons["button_b"]
            cmd = StringGen.belt_string(buttons["button_b"], belt_reverse_speed)
            self.send_i2c(BELT_ADDR, cmd)

        # D-pad up => belt position right
        if buttons["dpad_up"] != self.dpad_up:
            cmd = StringGen.belt_position_string(buttons["dpad_up"], right)
            self.send_i2c(BELT_ADDR, cmd)
            self.dpad_up = buttons["dpad_up"]
        
        # D-pad down => belt position left
        if buttons["dpad_down"] != self.dpad_down:
            cmd = StringGen.belt_position_string(buttons["dpad_down"], left)
            self.send_i2c(BELT_ADDR, cmd)
            self.dpad_down = buttons["dpad_down"]

        # Left bumper => belt forward (with current speed index)
        if buttons["lbutton"] != self.lbutton:
            speed = self.belt_speeds[self.belt_speed_index]
            cmd = StringGen.belt_string(buttons["lbutton"], speed)
            self.send_i2c(BELT_ADDR, cmd)
            self.lbutton = buttons["lbutton"]

    # -------------------------------------------------------------------------
    # Movement triggers => old tank logic or toggling 4WS mode
    # (You can remove if not needed anymore)
    # -------------------------------------------------------------------------
    def button_movement_handler(self, buttons):
        # Right bumper toggles driving mode (rotate <-> crab)
        if buttons["rbutton"] != self.rbutton:
            if buttons["rbutton"] == 1 and self.rbutton == 0:
                self.driving_mode = 1 - self.driving_mode
                mode_name = "Crab" if self.driving_mode == 1 else "Rotate"
                self.diag(DiagnosticStatus.OK, f"4WS driving mode changed to: {mode_name}")
            self.rbutton = buttons["rbutton"]

        # If still using triggers for “forward/back” in old style:
        # You can either comment this out or adapt it. 
        # Right trigger => forward, left trigger => reverse (or vice versa).
        if buttons['rtrigger'] != self.rtrigger:
            self.rtrigger = buttons['rtrigger']
            if self.rtrigger:
                lspeed, rspeed = self.calculate_speed(0, -(self.rtrigger*0.8)+15)
                # Instead of old serial:
                #   self.serial_publisher.publish(StringGen.movement_string(lspeed, rspeed))
                # we can do direct I2C to all 4 wheels:
                self.four_wheel_steering_handler(0, -100) 
            else:
                # Stop
                self.stop_all()

        if buttons['ltrigger'] != self.ltrigger:
            self.ltrigger = buttons['ltrigger']
            if self.ltrigger:
                lspeed, rspeed = self.calculate_speed(0, (self.ltrigger*0.8)+15)
                self.four_wheel_steering_handler(0, 100)
            else:
                # Stop
                self.stop_all()

    # -------------------------------------------------------------------------
    # Misc: deposit, etc. (Removed camera + vibrator)
    # -------------------------------------------------------------------------
    def misc_button_handler(self, buttons):
        # Comment out vibration functionality
        # if buttons["button_a"] != self.button_a:
        #     self.button_a = buttons["button_a"]
        #     cmd = StringGen.vibrator_string(buttons["button_a"])
        #     self.send_i2c(SOME_VIB_ADDR, list(cmd.encode('ascii')))

        # X button => deposit forward/back
        if buttons["button_x"] != self.button_x:
            cmd = StringGen.deposit_string(buttons["button_x"], forward)
            # Send to the auger address
            self.send_i2c(AUGER_ADDR, cmd)
            self.button_x = buttons["button_x"]

        # If you previously used dpad_left/dpad_right to move camera arm,
        # comment it out to disable camera movement:
        #
        # if buttons['dpad_left']:
        #     ...
        # if buttons['dpad_right']:
        #     ...

    # -------------------------------------------------------------------------
    # The main button callback
    # -------------------------------------------------------------------------
    def button_callback(self, request):
        try:
            data = request.data
            buttons = self.parse_buttons(data)

            self.check_auto(buttons)
            if self.digging:
                # Autonomy is running => ignore manual
                return
            
            # If not digging, handle everything
            # Commented out camera preset:
            # self.camera_preset_handler(buttons)

            self.belt_handler(buttons)
            self.button_movement_handler(buttons)
            self.misc_button_handler(buttons)

        except Exception as e:
            self.diag(DiagnosticStatus.ERROR, f"Exception in gamepad button callback: {str(e)}")

    # -------------------------------------------------------------------------
    # Stop all movement
    # -------------------------------------------------------------------------
    def stop_all(self):
        # Stop wheels
        self.send_i2c(FRONT_LEFT_ADDR,  [90, 90])
        self.send_i2c(FRONT_RIGHT_ADDR, [90, 90])
        self.send_i2c(REAR_LEFT_ADDR,   [90, 90])
        self.send_i2c(REAR_RIGHT_ADDR,  [90, 90])

        # Stop belt
        belt_stop = StringGen.belt_string(0, 90)
        self.send_i2c(BELT_ADDR, belt_stop)

        # Reset belt position
        belt_pos = StringGen.belt_position_string(0, right)
        self.send_i2c(BELT_ADDR, belt_pos)

        # Stop deposit/auger
        deposit_stop = StringGen.deposit_string(0, forward)
        self.send_i2c(AUGER_ADDR, deposit_stop)

        # Comment out vibrator:
        # vib_stop = StringGen.vibrator_string(0)
        # self.send_i2c(SOME_VIB_ADDR, vib_stop)

    def diag(self, status, message):
        # For quick debugging in console, using error-level logs
        self.get_logger().error(message)
        self.diagnostic_status.level = status
        self.diagnostic_status.message = message

    # -------------------------------------------------------------------------
    # Heartbeat => ensure we stop if no connection
    # -------------------------------------------------------------------------
    def heartbeat(self):
        self.diag_topic.publish(self.diagnostic_status)
        if self.connected == 1 and (datetime.datetime.now() - self.connection_time).total_seconds() > 2:
            self.connected = 0
            self.diag(DiagnosticStatus.ERROR, "Connection to gamepad lost.")
            self.stop_all()

    # -------------------------------------------------------------------------
    # Original tank-like speed utility (if still used by triggers)
    # -------------------------------------------------------------------------
    def calculate_speed(self, x_axis, y_axis):
        full_forward = 130
        stopped = 90
        full_reverse = 50  # (symmetric about 90)

        if abs(x_axis) < 15 and abs(y_axis) < 15:
            x_axis = 0
            y_axis = 0

        speed = (-y_axis) * ((full_forward - full_reverse)/200) + stopped
        direction = (x_axis) * ((full_forward - full_reverse)/200) * 0.75

        left_speed = round(clamp(speed - direction, full_reverse, full_forward))
        right_speed = round(clamp(speed + direction, full_reverse, full_forward))
        return left_speed, right_speed

    # -------------------------------------------------------------------------
    # The new I2C-sending function
    # -------------------------------------------------------------------------
    def send_i2c(self, address, data):
        # data may be an int or a tuple/list
        if isinstance(data, (tuple, list)) and len(data) == 2:
            val, channel = data
            msg_str = f"{hex(address)}:{val},{channel}"
        elif isinstance(data, (tuple, list)):
            # fallback: join all items
            msg_str = f"{hex(address)}:" + ",".join(str(d) for d in data)
        else:
            msg_str = f"{hex(address)}:{data}"
        self.i2c_publisher.publish(String(data=msg_str))


def main(args=None):
    rclpy.init(args=args)
    controller_parser = ControllerParser()
    rclpy.spin(controller_parser)
    controller_parser.stop_all()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
