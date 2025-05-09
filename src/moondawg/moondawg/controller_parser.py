from math import ceil, atan2, degrees, hypot
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String, Byte
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from cv_bridge import CvBridge
import cv2
import base64
import datetime
import json
from typing import Tuple, Union, Dict, List, Any

# Motor speed constants
BELT_REVERSE_SPEED = 30
MOTOR_STOPPED = 90
MOTOR_FULL_FORWARD = 110
MOTOR_FULL_REVERSE = 70
CAMERA_CENTER = 90

# Deposition command constants
AUGER = 1
VIBRATOR = 2

# Excavation command constants
BELT_SPEED = 1
BELT_POSITION = 2

# Direction constants
UP = 180
DOWN = 0

# Servo movement constants
SERVO_STEP_SIZE = 5  # Degrees to move per update
SERVO_UPDATE_RATE = 0.05  # Seconds between servo position updates
WHEEL_SPEED_STEP_SIZE = 2  # Speed increment per update

SERVO_INDEXES = {
    1: "FL",  # Front Left
    2: "FR",  # Front Right
    3: "RL",  # Rear Left
    4: "RR"   # Rear Right
}
# I2C addresses - consolidated into a single place for easy maintenance
class I2CAddress:
    FRONT_LEFT = 0x10
    FRONT_RIGHT = 0x11
    REAR_LEFT = 0x12
    REAR_RIGHT = 0x13
    STEERING_SERVO = 0x14
    EXCAVATION = 0x20
    DEPOSITION = 0x21

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
        self.belt_speeds = [180, 150, 120]  # Default values that will be overwritten by parameter
        self.belt_speed_index = 0
        
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
        
        # Current trigger states (0-100)
        self.current_ltrigger = 0
        self.current_rtrigger = 0
        
        # Bridge for camera processing
        self.br = CvBridge()
        
        # Track last I2C commands sent to each address
        self.i2c_command_history = {}
        
        # Track steering positions for condensed display
        self.current_steering_positions = {1: 90, 2: 90, 3: 90, 4: 90}
        self.target_steering_positions =  dict(self.current_steering_positions) # Default center positions
        
        # Track current and target wheel speeds
        self.current_wheel_speeds = {
            I2CAddress.FRONT_LEFT: MOTOR_STOPPED,
            I2CAddress.FRONT_RIGHT: MOTOR_STOPPED,
            I2CAddress.REAR_LEFT: MOTOR_STOPPED,
            I2CAddress.REAR_RIGHT: MOTOR_STOPPED
        }
        self.target_wheel_speeds = dict(self.current_wheel_speeds)
        
        # Diagnostic status setup
        self.diagnostic_status = DiagnosticStatus(
            name=self.get_name(), 
            level=DiagnosticStatus.OK,
            message="Initializing controller parser"
        )

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters for this node."""
        self.declare_parameter('belt_speed_index', 0)
        self.declare_parameter('joystick_deadzone', 0.07)
        self.declare_parameter('image_compression_quality', 20)
        self.declare_parameter('auto_dig_duration_seconds', 30)
        self.declare_parameter('belt_speeds', [180, 125, 120])

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
        
        # Image publishers for Web UI
        self.image_pub = self.create_publisher(
            String, 
            '/controller_parser/compressed_image', 
            10
        )
        self.rs1_color_pub = self.create_publisher(
            String, 
            '/controller_parser/rs1_color_image', 
            10
        )
        self.rs1_depth_pub = self.create_publisher(
            String, 
            '/controller_parser/rs1_depth_image', 
            10
        )
        self.rs2_color_pub = self.create_publisher(
            String, 
            '/controller_parser/rs2_color_image', 
            10
        )
        self.rs2_depth_pub = self.create_publisher(
            String, 
            '/controller_parser/rs2_depth_image', 
            10
        )
        
        # New publisher for I2C command history
        self.i2c_history_pub = self.create_publisher(
            String,
            '/controller_parser/i2c_history',
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
        
        # Camera image subscriptions
        self.image_subscription = self.create_subscription(
            Image, 
            'image', 
            self.image_translator, 
            10
        )
        
        # RealSense camera subscriptions
        self.rs1_color_subscription = self.create_subscription(
            Image, 
            '/camera1/color/image_raw', 
            self.rs1_color_translator, 
            10
        )
        self.rs1_depth_subscription = self.create_subscription(
            Image, 
            '/camera1/aligned_depth_to_color/image_raw', 
            self.rs1_depth_translator, 
            10
        )
        self.rs2_color_subscription = self.create_subscription(
            Image, 
            '/camera2/color/image_raw', 
            self.rs2_color_translator, 
            10
        )
        self.rs2_depth_subscription = self.create_subscription(
            Image, 
            '/camera2/aligned_depth_to_color/image_raw', 
            self.rs2_depth_translator, 
            10
        )
        
        # Timers
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat)
        self.auto_timer = self.create_timer(0.5, self.auto_callback)
        
        # Add a timer to publish I2C command history periodically
        self.i2c_history_timer = self.create_timer(0.5, self.publish_i2c_history)
        
        # Add a timer for servo interpolation updates
        self.servo_timer = self.create_timer(SERVO_UPDATE_RATE, self._update_servo_positions)
        
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
            lstick_x, lstick_y = axis['lstick_x'], axis['lstick_y']
            rstick_x = axis['rstick_x']

            # Convert deadzone from 0-1 to 0-100 for comparison with axis data
            deadzone_val = self.get_parameter('joystick_deadzone').get_parameter_value().double_value * 100.0

            lstick_magnitude = hypot(lstick_x, lstick_y)
            rstick_x_abs = abs(rstick_x)

            lstick_active = lstick_magnitude > deadzone_val
            rstick_active_rotate = rstick_x_abs > deadzone_val

            # Trigger values (0-100 from self.current_ltrigger/rtrigger) and deadzone
            # Using a fixed absolute deadzone for triggers (0-100 scale)
            trigger_deadzone_abs = 10 

            trigger_active_l = self.current_ltrigger > trigger_deadzone_abs
            trigger_active_r = self.current_rtrigger > trigger_deadzone_abs

            # Normalized stick values for handlers that expect -1.0 to 1.0
            lstick_x_norm = lstick_x / 100.0
            lstick_y_norm = lstick_y / 100.0
            rstick_x_norm = rstick_x / 100.0

            if rstick_active_rotate:
                self._handle_rotation_from_right_stick(rstick_x_norm)
            elif trigger_active_l or trigger_active_r:
                # Raw trigger diff: rtrigger (0-100) - ltrigger (0-100) => -100 to 100
                trigger_y_raw = self.current_rtrigger - self.current_ltrigger
                # Normalized trigger diff: -1.0 to 1.0
                trigger_y_norm = trigger_y_raw / 100.0
                self._handle_trigger_drive(trigger_y_norm, lstick_x_norm, lstick_y_norm, lstick_active)
            elif lstick_active:
                self._handle_crab_mode(lstick_x_norm, lstick_y_norm)
            else:
                # Both sticks are in deadzone or not causing a prioritized action
                self._center_wheels()
                self._stop_motors()

        except Exception as e:
            self.set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Exception in gamepad axis callback: {str(e)}"
            )

    def button_callback(self, request: Int8MultiArray) -> None:
        """Process button data from the controller."""
        try:
            buttons = self._parse_buttons(request.data)
            
            # Update current trigger states
            self.current_ltrigger = buttons.get('ltrigger', 0)
            self.current_rtrigger = buttons.get('rtrigger', 0)
            
            # Check if we should toggle autonomous mode
            self._check_auto_toggle(buttons)
            
            # If in autonomous mode, don't process manual controls
            if self.digging:
                return
            
            # Process manual control inputs
            self._process_belt_controls(buttons)
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
                self._set_belt_position(UP)
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
            self._set_belt_position(DOWN)
        elif 3 <= elapsed_seconds < 5:
            # Phase 2: Start belt
            self._start_belt()
        elif 17 <= elapsed_seconds < 19:
            # Phase 3: Raise belt
            self._set_belt_position(UP)
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

    def rs1_color_translator(self, message: Image) -> None:
        """
        Convert RealSense 1 color image to compressed base64 strings for web transmission.
        
        Args:
            message: The Image message from the RealSense 1 color camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(message, desired_encoding='bgr8')
            
            # Compress image using JPEG
            quality = self.get_parameter('image_compression_quality').get_parameter_value().integer_value
            _, encoded_img = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            
            # Convert to base64 and publish
            base64_data = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
            self.rs1_color_pub.publish(String(data=base64_data))
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense 1 color image: {str(e)}")
    
    def rs1_depth_translator(self, message: Image) -> None:
        """
        Convert RealSense 1 depth image to compressed base64 strings for web transmission.
        
        Args:
            message: The Image message from the RealSense 1 depth camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(message, desired_encoding='passthrough')
            
            # Normalize depth for visualization (16-bit to 8-bit)
            cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply colormap for better visualization
            cv_image_colormap = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)
            
            # Compress image using JPEG
            quality = self.get_parameter('image_compression_quality').get_parameter_value().integer_value
            _, encoded_img = cv2.imencode('.jpg', cv_image_colormap, [cv2.IMWRITE_JPEG_QUALITY, quality])
            
            # Convert to base64 and publish
            base64_data = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
            self.rs1_depth_pub.publish(String(data=base64_data))
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense 1 depth image: {str(e)}")
    
    def rs2_color_translator(self, message: Image) -> None:
        """
        Convert RealSense 2 color image to compressed base64 strings for web transmission.
        
        Args:
            message: The Image message from the RealSense 2 color camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(message, desired_encoding='bgr8')
            
            # Compress image using JPEG
            quality = self.get_parameter('image_compression_quality').get_parameter_value().integer_value
            _, encoded_img = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, quality])
            
            # Convert to base64 and publish
            base64_data = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
            self.rs2_color_pub.publish(String(data=base64_data))
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense 2 color image: {str(e)}")
    
    def rs2_depth_translator(self, message: Image) -> None:
        """
        Convert RealSense 2 depth image to compressed base64 strings for web transmission.
        
        Args:
            message: The Image message from the RealSense 2 depth camera
        """
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.br.imgmsg_to_cv2(message, desired_encoding='passthrough')
            
            # Normalize depth for visualization (16-bit to 8-bit)
            cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply colormap for better visualization
            cv_image_colormap = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)
            
            # Compress image using JPEG
            quality = self.get_parameter('image_compression_quality').get_parameter_value().integer_value
            _, encoded_img = cv2.imencode('.jpg', cv_image_colormap, [cv2.IMWRITE_JPEG_QUALITY, quality])
            
            # Convert to base64 and publish
            base64_data = base64.b64encode(encoded_img.tobytes()).decode('utf-8')
            self.rs2_depth_pub.publish(String(data=base64_data))
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense 2 depth image: {str(e)}")

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

    def _handle_crab_mode(self, x_f: float, y_f: float) -> None:
        """
        Handle crab steering mode (all wheels point in same direction).
        
        Args:
            x_f: Normalized X-axis (-1.0 to 1.0)
            y_f: Normalized Y-axis (-1.0 to 1.0)
        """
        # Calculate angle from joystick position (in degrees)
        angle_radians = atan2(x_f, y_f)
        angle_degrees = degrees(angle_radians)
        
        # If the angle would require servos to go beyond their range,
        # flip the angle and reverse the motors instead
        if angle_degrees > 90 or angle_degrees < -90:
            # Flip the angle by 180 degrees
            angle_degrees = (angle_degrees + 180) % 360
            if angle_degrees > 180:
                angle_degrees -= 360
        
        # Convert to servo angle (0-180 range)
        servo_angle = 90 + angle_degrees
        servo_angle = clamp(int(servo_angle), 0, 180)
        
        # Set all servos to the same angle
        for servo_index in range(1, 5):
            self._set_steering_angle(servo_index, servo_angle)

        self._set_wheel_speeds(MOTOR_STOPPED, MOTOR_STOPPED, MOTOR_STOPPED, MOTOR_STOPPED)
        
        
    def _handle_rotation_from_right_stick(self, r_x_f: float) -> None:
        """
        Handle robot rotation based on right stick X-axis input.
        Wheels are set to 45 degrees inward for point turn.
        Speed is proportional to stick deflection.

        Args:
            r_x_f: Normalized right stick X-axis value (-1.0 to 1.0)
        """
        # Deadzone check is handled by axis_callback before calling this.
        
        speed_magnitude_factor = abs(r_x_f)
        speed_offset = int((MOTOR_FULL_FORWARD - MOTOR_STOPPED) * speed_magnitude_factor)

        if r_x_f > 0:  # Clockwise rotation
            # Angles: FL points inward-right, FR inward-left, RL inward-right, RR inward-left
            self._set_steering_angle(1, 135)  # Front Left
            self._set_steering_angle(2, 45)   # Front Right
            self._set_steering_angle(3, 135)  # Rear Left (angling right for clockwise)
            self._set_steering_angle(4, 45)   # Rear Right (angling left for clockwise)

            # Speeds for clockwise: Left wheels forward, Right wheels reverse
            fl_speed = clamp(MOTOR_STOPPED + speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            fr_speed = clamp(MOTOR_STOPPED - speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            rl_speed = clamp(MOTOR_STOPPED + speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            rr_speed = clamp(MOTOR_STOPPED - speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            self._set_wheel_speeds(fl_speed, fr_speed, rl_speed, rr_speed)

        elif r_x_f < 0:  # Counter-clockwise rotation
            # Angles: FL points inward-left, FR inward-right, RL inward-left, RR inward-right
            self._set_steering_angle(1, 45)   # Front Left
            self._set_steering_angle(2, 135)  # Front Right
            self._set_steering_angle(3, 45)   # Rear Left (angling left for CCW)
            self._set_steering_angle(4, 135)  # Rear Right (angling right for CCW)

            # Speeds for counter-clockwise: Left wheels reverse, Right wheels forward
            fl_speed = clamp(MOTOR_STOPPED - speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            fr_speed = clamp(MOTOR_STOPPED + speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            rl_speed = clamp(MOTOR_STOPPED - speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            rr_speed = clamp(MOTOR_STOPPED + speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            self._set_wheel_speeds(fl_speed, fr_speed, rl_speed, rr_speed)
        # If r_x_f is 0 (or within deadzone), axis_callback handles stopping motors.

    def _handle_trigger_drive(self, trigger_y_norm: float, stick_x_norm: float, stick_y_norm: float, stick_active_for_steering: bool) -> None:
        """
        Handle robot movement based on triggers for speed and optional left stick for steering.
        Left stick controls wheel direction for crabbing. Triggers control speed along that direction.

        Args:
            trigger_y_norm: Normalized trigger difference (-1.0 for full reverse, 1.0 for full forward).
            stick_x_norm: Normalized left stick X-axis (-1.0 to 1.0).
            stick_y_norm: Normalized left stick Y-axis (-1.0 to 1.0).
            stick_active_for_steering: True if left stick is outside deadzone and should be used for steering.
        """
        if stick_active_for_steering:
            # Calculate wheel angle for crab steering based on left stick direction
            angle_radians = atan2(stick_x_norm, stick_y_norm)
            angle_degrees_raw = degrees(angle_radians) # Raw angle from stick: 0 for Fwd, 90 for Right, etc.
            
            # Adjust angle to be within [-90, 90] relative to chassis forward,
            # so servo angle points the "front" of the crab maneuver along the stick direction.
            effective_angle_degrees = angle_degrees_raw
            if angle_degrees_raw > 90:
                effective_angle_degrees = angle_degrees_raw - 180
            elif angle_degrees_raw < -90:
                effective_angle_degrees = angle_degrees_raw + 180
            
            # Convert to servo angle (0-180 range, 90 is straight)
            servo_angle_calculated = 90 + effective_angle_degrees 
            servo_angle_clamped = clamp(int(round(servo_angle_calculated)), 0, 180)
            
            for servo_idx in range(1, 5): # Servos 1-4
                self._set_steering_angle(servo_idx, servo_angle_clamped)
        else:
            # No stick steering, so wheels point straight forward
            self._center_wheels()

        # Triggers determine the magnitude and direction (forward/reverse) of movement
        # along the line defined by the current wheel angle.
        actual_drive_norm = trigger_y_norm

        # Calculate motor speed based on actual_drive_norm (-1.0 to 1.0)
        # Positive actual_drive_norm means "forward" along the (possibly crabbed) direction.
        # Negative actual_drive_norm means "reverse" along the (possibly crabbed) direction.
        motor_speed = MOTOR_STOPPED
        if actual_drive_norm > 0.001: # Apply a small deadzone to normalized drive
            motor_speed = MOTOR_STOPPED + int((MOTOR_FULL_FORWARD - MOTOR_STOPPED) * actual_drive_norm)
        elif actual_drive_norm < -0.001: # Apply a small deadzone to normalized drive
            # abs(actual_drive_norm) because (MOTOR_STOPPED - MOTOR_FULL_REVERSE) is positive
            motor_speed = MOTOR_STOPPED - int((MOTOR_STOPPED - MOTOR_FULL_REVERSE) * abs(actual_drive_norm))
            
        final_speed = clamp(motor_speed, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
        self._set_wheel_speeds(final_speed, final_speed, final_speed, final_speed)

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
            if self.button_b:
                self._set_belt_speed(BELT_REVERSE_SPEED)
            else:
                self._set_belt_speed(MOTOR_STOPPED)

        # D-pad up => belt position up
        if buttons["dpad_up"] != self.dpad_up:
            self.dpad_up = buttons["dpad_up"]
            if self.dpad_up:
                self._set_belt_position(UP)
            else:
                self._set_belt_position(MOTOR_STOPPED)
        
        # D-pad down => belt position down
        if buttons["dpad_down"] != self.dpad_down:
            self.dpad_down = buttons["dpad_down"]
            if self.dpad_down:
                self._set_belt_position(DOWN)
            else:
                self._set_belt_position(MOTOR_STOPPED)

        # Left bumper => belt forward (with current speed index)
        if buttons["lbutton"] != self.lbutton:
            self.lbutton = buttons["lbutton"]
            if self.lbutton:
                speed = self.belt_speeds[self.belt_speed_index]
                self._set_belt_speed(speed)
            else:
                self._set_belt_speed(MOTOR_STOPPED)

    def _process_misc_controls(self, buttons: Dict[str, Any]) -> None:
        """
        Process miscellaneous button inputs.
        
        Args:
            buttons: Dictionary of button states
        """
        # X button => deposit with auger
        if buttons["button_x"] != self.button_x:
            self.button_x = buttons["button_x"]
            if self.button_x:
                self._set_auger_deposition(MOTOR_FULL_FORWARD)
            else:
                self._set_auger_deposition(MOTOR_STOPPED)

        # A button => vibrator on/off
        if buttons["button_a"] != self.button_a:
            self.button_a = buttons["button_a"]
            if self.button_a:
                self._set_vibrator(MOTOR_FULL_FORWARD)
            else:
                self._set_vibrator(MOTOR_STOPPED)

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

    def _set_belt_position(self, position: int) -> None:
        """
        Set the belt position (up or down).
        
        Args:
            position: The position to set (UP or DOWN)
        """
        self.send_i2c(I2CAddress.EXCAVATION, [BELT_POSITION, position])

    def _set_belt_speed(self, speed: int) -> None:
        """
        Set the belt speed.
        
        Args:
            speed: The speed to set (0-180, 90 is stopped)
        """
        self.send_i2c(I2CAddress.EXCAVATION, [BELT_SPEED, speed])

    def _set_auger_deposition(self, direction: int) -> None:
        """
        Set the auger deposition direction.
        
        Args:
            direction: The direction to set (FORWARD, MOTOR_STOPPED, etc.)
        """
        self.send_i2c(I2CAddress.DEPOSITION, [AUGER, direction])

    def _set_vibrator(self, state: int) -> None:
        """
        Set the vibrator state.
        
        Args:
            state: The state to set (ON, OFF)
        """
        self.send_i2c(I2CAddress.DEPOSITION, [VIBRATOR, state])

    def _parse_buttons(self, data: List[int]) -> Dict[str, float]:
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

    def _set_steering_angle(self, servo_index: int, angle: int) -> None:
        """
        Set the steering angle for a specific wheel.
        
        Args:
            angle: Angle to set (0-180)
            servo_index: servo_index number (1-4)
        """
        # Just store the target angle - the servo timer will gradually move to this position
        self.target_steering_positions[servo_index] = angle

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
        # Store the target speeds - the timer will gradually adjust to these values
        self.target_wheel_speeds[I2CAddress.FRONT_LEFT] = front_left
        self.target_wheel_speeds[I2CAddress.FRONT_RIGHT] = front_right  
        self.target_wheel_speeds[I2CAddress.REAR_LEFT] = rear_left
        self.target_wheel_speeds[I2CAddress.REAR_RIGHT] = rear_right

    def _center_wheels(self) -> None:
        """Set all wheels to center position (straight)."""
        for servo_index in range(1, 5):
            self._set_steering_angle(servo_index, 90)

    def _stop_motors(self) -> None:
        """Stop all wheel motors."""
        self._set_wheel_speeds(90, 90, 90, 90)

    def stop_all(self) -> None:
        """Stop all movement and reset positions."""
        # Stop wheels
        self._stop_motors()
        self._center_wheels()

        # Stop belt
        self._set_belt_speed(MOTOR_STOPPED)

        # Reset belt position
        self._set_belt_position(MOTOR_STOPPED)

        # Stop deposit/auger
        self._set_auger_deposition(MOTOR_STOPPED)
        
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
            device, value = data
            msg_str = f"{hex(address)}:{device},{value}"
            # Store the command in history with a user-friendly description
            self._update_i2c_history(address, f"{device},{value}")
        elif isinstance(data, (tuple, list)):
            # Format a list of values
            msg_str = f"{hex(address)}:" + ",".join(str(d) for d in data)
            # Store the command in history
            self._update_i2c_history(address, ",".join(str(d) for d in data))
        else:
            # Handle integer or other simple values
            msg_str = f"{hex(address)}:{data}"
            # Store the command in history
            self._update_i2c_history(address, str(data))
            
        # Publish the formatted message
        self.i2c_publisher.publish(String(data=msg_str))

    def _update_i2c_history(self, address: int, data_str: str) -> None:
        """
        Update the I2C command history for an address.
        
        Args:
            address: I2C address
            data_str: String representation of the data sent
        """
        # Get human-readable name for the address
        device_name = self._get_device_name(address)
        
        # Store the command in the history with timestamp
        self.i2c_command_history[address] = {
            'device_name': device_name,
            'address': hex(address),
            'last_command': data_str,
            'timestamp': datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        }
        
        # Immediately publish to show instant updates
        self.publish_i2c_history()

    def _get_device_name(self, address: int) -> str:
        """
        Get a human-readable name for an I2C address.
        
        Args:
            address: I2C address
            
        Returns:
            String name of the device
        """
        device_names = {
            I2CAddress.FRONT_LEFT: "Front Left Motor",
            I2CAddress.FRONT_RIGHT: "Front Right Motor",
            I2CAddress.REAR_LEFT: "Rear Left Motor",
            I2CAddress.REAR_RIGHT: "Rear Right Motor",
            I2CAddress.STEERING_SERVO: "Steering Servo",
            I2CAddress.EXCAVATION: "Excavation System",
            I2CAddress.DEPOSITION: "Deposition System",
        }
        
        return device_names.get(address, f"Unknown Device ({hex(address)})")
    
    def publish_i2c_history(self) -> None:
        """
        Publish the I2C command history as a JSON string.
        """
        if not self.i2c_command_history and all(pos == 90 for pos in self.current_steering_positions.values()):
            return
            
        # Create a copy of the history data to work with
        history_data = dict(self.i2c_command_history)
        
        steering_data = ", ".join([f"{SERVO_INDEXES[ch]}: {pos}°" for ch, pos in self.current_steering_positions.items()])
        history_data[I2CAddress.STEERING_SERVO] = {
            'device_name': "Steering Servos",
            'address': hex(I2CAddress.STEERING_SERVO),
            'last_command': steering_data,
            'timestamp': datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
        }
            
        # Convert the history to a JSON string
        history_json = json.dumps(history_data)
        
        # Publish the history
        self.i2c_history_pub.publish(String(data=history_json))

    def _update_servo_positions(self) -> None:
        """
        Update servo positions and wheel speeds gradually to their target positions.
        """
        # Update steering servo positions
        for servo_index, target_position in self.target_steering_positions.items():
            current_position = self.current_steering_positions[servo_index]
            if current_position != target_position:
                step = SERVO_STEP_SIZE if current_position < target_position else -SERVO_STEP_SIZE
                new_position = current_position + step
                if (step > 0 and new_position > target_position) or (step < 0 and new_position < target_position):
                    new_position = target_position
                self.current_steering_positions[servo_index] = new_position
                self.send_i2c(I2CAddress.STEERING_SERVO, (servo_index, new_position))
        
        # Update wheel speeds
        for address, target_speed in self.target_wheel_speeds.items():
            current_speed = self.current_wheel_speeds[address]
            if current_speed != target_speed:
                step = WHEEL_SPEED_STEP_SIZE if current_speed < target_speed else -WHEEL_SPEED_STEP_SIZE
                new_speed = current_speed + step
                if (step > 0 and new_speed > target_speed) or (step < 0 and new_speed < target_speed):
                    new_speed = target_speed
                self.current_wheel_speeds[address] = new_speed
                self.send_i2c(address, new_speed)

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
