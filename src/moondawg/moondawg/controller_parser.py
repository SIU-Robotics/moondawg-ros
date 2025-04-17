from math import ceil, atan2, degrees, hypot
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String, Byte
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus
from cv_bridge import CvBridge
import cv2
import base64
import datetime
from typing import Tuple, Union, Dict, List, Any
from .string_gen import StringGen

# Constants for better readability and maintenance
# Movement constants
FORWARD = 'f'
BACKWARD = 'b'
LEFT = 'l'
RIGHT = 'r'
UP = 'u'
DOWN = 'd'

# Motor speed constants
BELT_REVERSE_SPEED = 30
MOTOR_STOPPED = 90
MOTOR_FULL_FORWARD = 180
MOTOR_FULL_REVERSE = 0
CAMERA_CENTER = 90

# I2C addresses - consolidated into a single place for easy maintenance
class I2CAddress:
    FRONT_LEFT = 0x10
    FRONT_RIGHT = 0x11
    REAR_LEFT = 0x12
    REAR_RIGHT = 0x13
    STEERING_SERVO = 0x14
    BELT = 0x20
    AUGER = 0x21

# Driving modes
class DrivingMode:
    ROTATE = 0
    CRAB = 1

def clamp(value: float, low: float, high: float) -> float:
    """Constrain a value between a minimum and maximum value."""
    return max(low, min(value, high))


class ControllerParser(Node):
    """
    Node that parses controller inputs and translates them to robot commands.
    
    This node handles:
    - Xbox controller input processing
    - Robot movement control through I2C commands
    - Belt and auger operation
    - Autonomous digging sequence control
    - Camera feed compression and streaming
    """

    def __init__(self):
        super().__init__(node_name='controller_parser')
        
        # Initialize state variables
        self._init_state_variables()
        
        # Declare ROS parameters
        self._declare_parameters()
        
        # Set up publishers, subscribers, and timers
        self._setup_communications()
        
        self.get_logger().info("Controller parser initialized and ready")

    def _init_state_variables(self) -> None:
        """Initialize all state variables for the node."""
        # Component state
        self.belt_speeds = [180, 125, 120]
        self.belt_speed_index = 0
        self.driving_mode = DrivingMode.ROTATE
        
        # Connection state
        self.connection_time = 0
        self.connected = False
        
        # Autonomy state
        self.time_start = 0
        self.depositing = False
        self.digging = False
        
        # Controller button states - initialized to prevent None comparisons
        self.dpad_up = 0
        self.dpad_down = 0
        self.dpad_left = 0
        self.dpad_right = 0
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
        
        # Bridge for camera processing
        self.br = CvBridge()
        
        # Diagnostic status setup
        self.diagnostic_status = DiagnosticStatus(
            name=self.get_name(), 
            level=DiagnosticStatus.OK,
            message="Initializing controller parser"
        )

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters for this node."""
        self.declare_parameter('belt_speed_index', 0)
        self.declare_parameter('joystick_deadzone', 0.1)
        self.declare_parameter('turn_sensitivity', 0.5)
        self.declare_parameter('image_compression_quality', 20)

    def _setup_communications(self) -> None:
        """Set up all publishers, subscribers, and timers."""
        # Publishers
        self.diag_topic = self.create_publisher(
            DiagnosticStatus, 
            '/controller_parser/diag', 
            10
        )
        self.i2c_publisher = self.create_publisher(
            String, 
            '/i2c_node/command', 
            10
        )
        self.image_pub = self.create_publisher(
            String, 
            '/controller_parser/compressed_image', 
            10
        )
        
        # Subscribers
        self.connection_topic = self.create_subscription(
            Byte, 
            '/connection_status', 
            self.connection_callback, 
            10
        )
        self.axis_subscription = self.create_subscription(
            Int8MultiArray, 
            '/controller_parser/gamepad_axis', 
            self.axis_callback, 
            10
        )
        self.button_subscription = self.create_subscription(
            Int8MultiArray, 
            '/controller_parser/gamepad_button', 
            self.button_callback, 
            10
        )
        self.image_subscription = self.create_subscription(
            Image, 
            'image', 
            self.image_translator, 
            10
        )
        
        # Timers
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat)
        self.auto_timer = self.create_timer(0.5, self.auto_callback)
        
        # Set initial diagnostic status
        self.set_diagnostic_status(DiagnosticStatus.OK, "Controller parser ready")

    # ------------------- Callback handlers -------------------

    def connection_callback(self, message: Byte) -> None:
        """Handle connection status messages from the websocket."""
        self.connection_time = datetime.datetime.now()
        self.connected = True
        
    def axis_callback(self, request: Int8MultiArray) -> None:
        """Process joystick axis data from the controller."""
        try:
            axis = self._parse_axis(request.data)
            self.four_wheel_steering_handler(axis['lstick_x'], axis['lstick_y'])
        except Exception as e:
            self.set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Exception in gamepad axis callback: {str(e)}"
            )

    def button_callback(self, request: Int8MultiArray) -> None:
        """Process button data from the controller."""
        try:
            buttons = self._parse_buttons(request.data)
            
            # Check if we should toggle autonomous mode
            self._check_auto_toggle(buttons)
            
            # If in autonomous mode, don't process manual controls
            if self.digging:
                return
            
            # Process manual control inputs
            self._process_belt_controls(buttons)
            self._process_movement_controls(buttons)
            self._process_misc_controls(buttons)
            
        except Exception as e:
            self.set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Exception in gamepad button callback: {str(e)}"
            )

    def auto_callback(self) -> None:
        """
        Manage the autonomous digging sequence.
        
        This function is called on a timer and manages the state machine
        for the autonomous digging sequence.
        """
        if not self.digging:
            if self.time_start != 0:
                # Cleanup when ending autonomous mode
                self.set_diagnostic_status(
                    DiagnosticStatus.OK, 
                    "Dig autonomy subroutine finished"
                )
                self.time_start = 0
                # Stop belt
                self._stop_belt()
                # Reset belt position
                self._move_belt_position(True, RIGHT)
            return
            
        # Initialize the timer if this is the start of the sequence
        if self.time_start == 0:
            self.time_start = datetime.datetime.now()
            self.set_diagnostic_status(
                DiagnosticStatus.OK, 
                "Running dig autonomy subroutine"
            )
            
        # Calculate elapsed time and run the appropriate sequence step
        elapsed_seconds = ceil((datetime.datetime.now() - self.time_start).total_seconds())
        
        if elapsed_seconds < 3:
            # Phase 1: Lower belt
            self._move_belt_position(True, LEFT)
        elif 3 <= elapsed_seconds < 5:
            # Phase 2: Start belt
            self._start_belt()
        elif 17 <= elapsed_seconds < 19:
            # Phase 3: Raise belt
            self._move_belt_position(True, RIGHT)
        elif elapsed_seconds >= 30:
            # Phase 4: Complete sequence
            self.digging = False

    def image_translator(self, message: Image) -> None:
        """
        Convert ROS Image messages to compressed base64 strings for web transmission.
        
        Args:
            message: The Image message from the camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(message)
            
            # Compress image using JPEG
            quality = self.get_parameter('image_compression_quality').get_parameter_value().integer_value
            _, encoded_img = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            
            # Convert to base64 and publish
            base64_data = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
            self.image_pub.publish(String(data=base64_data))
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {str(e)}")

    def heartbeat(self) -> None:
        """
        Publish diagnostic status and check for controller connection timeout.
        
        This function is called periodically to:
        1. Publish the current diagnostic status
        2. Check if the controller connection has timed out
        3. Stop all motors if connection is lost
        """
        # Publish diagnostics
        self.diag_topic.publish(self.diagnostic_status)
        
        # Check for connection timeout (2 seconds)
        if self.connected and (datetime.datetime.now() - self.connection_time).total_seconds() > 2:
            self.connected = False
            self.set_diagnostic_status(
                DiagnosticStatus.ERROR,
                "Connection to gamepad lost"
            )
            self.stop_all()

    # ------------------- Control handlers -------------------

    def four_wheel_steering_handler(self, x: float, y: float) -> None:
        """
        Handle four wheel steering based on joystick input.
        
        Args:
            x: X-axis joystick value (-100 to 100)
            y: Y-axis joystick value (-100 to 100)
        """
        # Normalize inputs to [-1.0, 1.0]
        x_f = x / 100.0
        y_f = y / 100.0
        
        # Calculate magnitude of joystick movement (0.0 to 1.0)
        magnitude = hypot(x_f, y_f)
        deadzone = self.get_parameter('joystick_deadzone').get_parameter_value().double_value
        
        # Ignore very small movements (deadzone)
        if magnitude < deadzone:
            self._center_wheels()
            self._stop_motors()
            return
            
        if self.driving_mode == DrivingMode.CRAB:
            self._handle_crab_mode(x_f, y_f, magnitude)
        else:
            self._handle_rotate_mode(x_f, y_f, magnitude)

    def _handle_crab_mode(self, x_f: float, y_f: float, magnitude: float) -> None:
        """
        Handle crab steering mode (all wheels point in same direction).
        
        Args:
            x_f: Normalized X-axis (-1.0 to 1.0)
            y_f: Normalized Y-axis (-1.0 to 1.0)
            magnitude: Magnitude of joystick deflection (0.0 to 1.0)
        """
        # Calculate angle from joystick position (in degrees)
        angle_radians = atan2(x_f, y_f)
        servo_angle = 90 + degrees(angle_radians)  # Convert to 0-180 range
        servo_angle = clamp(int(servo_angle), 0, 180)
        
        # Set all servos to the same angle
        for channel in range(1, 5):
            self._set_steering_angle(servo_angle, channel)
        
        # Calculate speed based on magnitude 
        # (90 = stopped, 180 = full forward, 0 = full reverse)
        speed = 90 - int(90 * magnitude * (-1 if y_f >= 0 else 1))
        speed = clamp(speed, 0, 180)
        
        # Apply the same speed to all motors
        self._set_wheel_speeds(speed, speed, speed, speed)

    def _handle_rotate_mode(self, x_f: float, y_f: float, magnitude: float) -> None:
        """
        Handle rotation steering mode.
        
        Args:
            x_f: Normalized X-axis (-1.0 to 1.0)
            y_f: Normalized Y-axis (-1.0 to 1.0)
            magnitude: Magnitude of joystick deflection (0.0 to 1.0)
        """
        # For small Y movement but larger X movement, do a zero-point turn
        if abs(x_f) > abs(y_f) and abs(x_f) > 0.3:
            self._handle_point_turn(x_f)
        else:
            self._handle_differential_steering(x_f, y_f, magnitude)

    def _handle_point_turn(self, x_f: float) -> None:
        """
        Configure wheels for a zero-point turn.
        
        Args:
            x_f: Normalized X-axis (-1.0 to 1.0)
        """
        if x_f > 0:  # Clockwise rotation
            # Set wheel angles for clockwise rotation
            self._set_steering_angle(135, 1)  # Front left -> right
            self._set_steering_angle(135, 2)  # Front right -> right  
            self._set_steering_angle(45, 3)   # Rear left -> left
            self._set_steering_angle(45, 4)   # Rear right -> left
            
            # Set speeds for rotation - front wheels forward, rear wheels reverse
            speed = 90 + int(90 * abs(x_f))
            rev_speed = 90 - int(90 * abs(x_f))
            speed = clamp(speed, 0, 180)
            rev_speed = clamp(rev_speed, 0, 180)
            
            self._set_wheel_speeds(speed, speed, rev_speed, rev_speed)
            
        else:  # Counter-clockwise rotation
            # Set wheel angles for counter-clockwise rotation
            self._set_steering_angle(45, 1)   # Front left -> left
            self._set_steering_angle(45, 2)   # Front right -> left
            self._set_steering_angle(135, 3)  # Rear left -> right
            self._set_steering_angle(135, 4)  # Rear right -> right
            
            # Set speeds for rotation - front wheels reverse, rear wheels forward
            speed = 90 + int(90 * abs(x_f))
            rev_speed = 90 - int(90 * abs(x_f))
            speed = clamp(speed, 0, 180)
            rev_speed = clamp(rev_speed, 0, 180)
            
            self._set_wheel_speeds(rev_speed, rev_speed, speed, speed)

    def _handle_differential_steering(self, x_f: float, y_f: float, magnitude: float) -> None:
        """
        Handle differential steering for forward/backward movement with turning.
        
        Args:
            x_f: Normalized X-axis (-1.0 to 1.0)
            y_f: Normalized Y-axis (-1.0 to 1.0)
            magnitude: Magnitude of joystick deflection (0.0 to 1.0)
        """
        # Set all wheels pointing forward
        self._center_wheels()
        
        # Calculate speed based on y_axis (positive = forward, negative = backward)
        base_speed = 90 + int(90 * y_f)
        base_speed = clamp(base_speed, 0, 180)
        
        # Apply differential steering for turning while moving
        turn_sensitivity = self.get_parameter('turn_sensitivity').get_parameter_value().double_value
        turn_factor = x_f * turn_sensitivity
        
        left_speed = int(base_speed - (turn_factor * 90))
        right_speed = int(base_speed + (turn_factor * 90))
        
        left_speed = clamp(left_speed, 0, 180)
        right_speed = clamp(right_speed, 0, 180)
        
        # Apply speeds to the left and right sides
        self._set_wheel_speeds(left_speed, right_speed, left_speed, right_speed)

    # ------------------- Button processing functions -------------------

    def _check_auto_toggle(self, buttons: Dict[str, Any]) -> None:
        """
        Check if autonomous mode should be toggled.
        
        Args:
            buttons: Dictionary of button states
        """
        if buttons["select"] != self.select:
            self.select = buttons["select"]
            if self.select:
                self.digging = not self.digging
                mode = "started" if self.digging else "stopped"
                self.get_logger().info(f"Autonomous digging {mode}")

    def _process_belt_controls(self, buttons: Dict[str, Any]) -> None:
        """
        Process belt-related button inputs.
        
        Args:
            buttons: Dictionary of button states
        """
        # Y button cycles belt speeds
        if buttons["button_y"] != self.button_y:
            if buttons["button_y"]:
                # Only cycle speeds on button press, not release
                self.belt_speed_index = (self.belt_speed_index + 1) % len(self.belt_speeds)
                self.get_logger().info(f"Belt speed set to level {self.belt_speed_index+1}/{len(self.belt_speeds)}")
            self.button_y = buttons["button_y"]

        # B button => belt reverse, or off
        if buttons["button_b"] != self.button_b:
            self.button_b = buttons["button_b"]
            cmd = StringGen.belt_string(buttons["button_b"], BELT_REVERSE_SPEED)
            self.send_i2c(I2CAddress.BELT, cmd)

        # D-pad up => belt position right
        if buttons["dpad_up"] != self.dpad_up:
            self._move_belt_position(buttons["dpad_up"], RIGHT)
            self.dpad_up = buttons["dpad_up"]
        
        # D-pad down => belt position left
        if buttons["dpad_down"] != self.dpad_down:
            self._move_belt_position(buttons["dpad_down"], LEFT)
            self.dpad_down = buttons["dpad_down"]

        # Left bumper => belt forward (with current speed index)
        if buttons["lbutton"] != self.lbutton:
            speed = self.belt_speeds[self.belt_speed_index]
            cmd = StringGen.belt_string(buttons["lbutton"], speed)
            self.send_i2c(I2CAddress.BELT, cmd)
            self.lbutton = buttons["lbutton"]

    def _process_movement_controls(self, buttons: Dict[str, Any]) -> None:
        """
        Process movement-related button inputs.
        
        Args:
            buttons: Dictionary of button states
        """
        # Right bumper toggles driving mode (rotate <-> crab)
        if buttons["rbutton"] != self.rbutton:
            if buttons["rbutton"] == 1 and self.rbutton == 0:
                self.driving_mode = 1 - self.driving_mode
                mode_name = "Crab" if self.driving_mode == DrivingMode.CRAB else "Rotate"
                self.set_diagnostic_status(
                    DiagnosticStatus.OK, 
                    f"4WS driving mode changed to: {mode_name}"
                )
            self.rbutton = buttons["rbutton"]

        # Right trigger => forward
        if buttons['rtrigger'] != self.rtrigger:
            self.rtrigger = buttons['rtrigger']
            if self.rtrigger:
                self.four_wheel_steering_handler(0, -100) 
            else:
                # Only stop if this trigger was controlling movement
                if self.ltrigger == 0:
                    self.stop_all()

        # Left trigger => reverse
        if buttons['ltrigger'] != self.ltrigger:
            self.ltrigger = buttons['ltrigger']
            if self.ltrigger:
                self.four_wheel_steering_handler(0, 100)
            else:
                # Only stop if this trigger was controlling movement
                if self.rtrigger == 0:
                    self.stop_all()

    def _process_misc_controls(self, buttons: Dict[str, Any]) -> None:
        """
        Process miscellaneous button inputs.
        
        Args:
            buttons: Dictionary of button states
        """
        # X button => deposit forward/back
        if buttons["button_x"] != self.button_x:
            cmd = StringGen.deposit_string(buttons["button_x"], FORWARD)
            self.send_i2c(I2CAddress.AUGER, cmd)
            self.button_x = buttons["button_x"]

    # ------------------- Helper methods -------------------

    def _parse_axis(self, data: List[int]) -> Dict[str, int]:
        """
        Parse axis data from controller.
        
        Args:
            data: List of axis values from controller
            
        Returns:
            Dictionary of named axis values
        """
        return {
            "lstick_x": data[0],  # -100 .. 100
            "lstick_y": data[1],  # -100 .. 100
            "rstick_x": data[2],
            "rstick_y": data[3]
        }

    def _parse_buttons(self, data: List[int]) -> Dict[str, float]:
        """
        Parse button data from controller.
        
        Args:
            data: List of button values from controller
            
        Returns:
            Dictionary of named button values
        """
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

    def _set_steering_angle(self, angle: int, channel: int) -> None:
        """
        Set the steering angle for a specific wheel.
        
        Args:
            angle: Angle to set (0-180)
            channel: Channel number (1-4)
        """
        self.send_i2c(I2CAddress.STEERING_SERVO, (angle, channel))

    def _set_wheel_speeds(self, 
                         front_left: int, 
                         front_right: int, 
                         rear_left: int, 
                         rear_right: int) -> None:
        """
        Set the speeds for all four wheels.
        
        Args:
            front_left: Speed for front left wheel (0-180)
            front_right: Speed for front right wheel (0-180)
            rear_left: Speed for rear left wheel (0-180)
            rear_right: Speed for rear right wheel (0-180)
        """
        self.send_i2c(I2CAddress.FRONT_LEFT, front_left)
        self.send_i2c(I2CAddress.FRONT_RIGHT, front_right)
        self.send_i2c(I2CAddress.REAR_LEFT, rear_left)
        self.send_i2c(I2CAddress.REAR_RIGHT, rear_right)

    def _center_wheels(self) -> None:
        """Set all wheels to center position (straight)."""
        for channel in range(1, 5):
            self._set_steering_angle(90, channel)

    def _stop_motors(self) -> None:
        """Stop all wheel motors."""
        self._set_wheel_speeds(90, 90, 90, 90)

    def _move_belt_position(self, enabled: bool, direction: str) -> None:
        """
        Move the belt position.
        
        Args:
            enabled: Whether movement is enabled
            direction: Direction of movement ('l' or 'r')
        """
        cmd = StringGen.belt_position_string(enabled, direction)
        self.send_i2c(I2CAddress.BELT, cmd)

    def _start_belt(self) -> None:
        """Start the belt with the current speed setting."""
        speed = self.belt_speeds[self.belt_speed_index]
        cmd = StringGen.belt_string(1, speed)
        self.send_i2c(I2CAddress.BELT, cmd)

    def _stop_belt(self) -> None:
        """Stop the belt."""
        cmd = StringGen.belt_string(0, 90)
        self.send_i2c(I2CAddress.BELT, cmd)

    def stop_all(self) -> None:
        """Stop all movement and reset positions."""
        # Stop wheels
        self._stop_motors()
        self._center_wheels()

        # Stop belt
        self._stop_belt()

        # Reset belt position
        self._move_belt_position(0, RIGHT)

        # Stop deposit/auger
        deposit_stop = StringGen.deposit_string(0, FORWARD)
        self.send_i2c(I2CAddress.AUGER, deposit_stop)
        
        self.get_logger().info("All movement stopped")

    def set_diagnostic_status(self, level: int, message: str) -> None:
        """
        Set the diagnostic status and log appropriately.
        
        Args:
            level: Diagnostic level (OK, WARN, ERROR)
            message: Status message
        """
        self.diagnostic_status.level = level
        self.diagnostic_status.message = message
        
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(message)
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def send_i2c(self, address: int, data: Union[int, Tuple, List]) -> None:
        """
        Send an I2C command.
        
        Args:
            address: I2C address
            data: Data to send (can be an int, tuple, or list)
        """
        # Format the message based on data type
        if isinstance(data, (tuple, list)) and len(data) == 2:
            val, channel = data
            msg_str = f"{hex(address)}:{val},{channel}"
        elif isinstance(data, (tuple, list)):
            # Format a list of values
            msg_str = f"{hex(address)}:" + ",".join(str(d) for d in data)
        elif isinstance(data, String):
            # Handle String messages (from StringGen)
            msg_str = f"{hex(address)}:{data.data}"
        else:
            # Handle integer or other simple values
            msg_str = f"{hex(address)}:{data}"
            
        # Publish the formatted message
        self.i2c_publisher.publish(String(data=msg_str))


def main(args=None):
    """Main entry point for the node."""
    try:
        rclpy.init(args=args)
        controller_parser = ControllerParser()
        
        try:
            rclpy.spin(controller_parser)
        except KeyboardInterrupt:
            pass
        finally:
            # Ensure motors are stopped when shutting down
            controller_parser.stop_all()
            controller_parser.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"Error in controller_parser: {str(e)}")


if __name__ == '__main__':
    main()
