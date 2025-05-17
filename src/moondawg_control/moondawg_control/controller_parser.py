from math import ceil, atan2, degrees, hypot
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String, Byte
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
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
    DRIVE_SYSTEM = 0x10
    EXCAVATION_SYSTEM = 0x11

# Motor indexes for DRIVE_SYSTEM
class DriveMotor:
    DRIVE_FL = 1  # Front Left
    DRIVE_FR = 2  # Front Right
    DRIVE_RL = 3  # Rear Left
    DRIVE_RR = 4  # Rear Right
    TURN_FL = 5   # Front Left Servo
    TURN_FR = 6   # Front Right Servo
    TURN_RL = 7   # Rear Left Servo
    TURN_RR = 8   # Rear Right Servo

# Motor indexes for EXCAVATION_SYSTEM
class ExcavationMotor:
    BELT = 1      # Belt
    AUGER = 2     # Auger
    LEFT_ACTUATOR = 3  # Left Actuator
    RIGHT_ACTUATOR = 4 # Right Actuator
    VIBE_MOTOR = 5     # Vibe motor

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
        # Connection state
        self.connection_time = 0
        self.connected = False
        
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
        
        # Track last commands sent to each address
        self.i2c_command_history = {}
        
        # Track steering positions for condensed display
        self.current_steering_positions = {1: 90, 2: 90, 3: 90, 4: 90}
        self.target_steering_positions =  dict(self.current_steering_positions) # Default center positions
        
        # Track current and target wheel speeds
        self.current_wheel_speeds = {
            DriveMotor.DRIVE_FL: MOTOR_STOPPED,
            DriveMotor.DRIVE_FR: MOTOR_STOPPED,
            DriveMotor.DRIVE_RL: MOTOR_STOPPED,
            DriveMotor.DRIVE_RR: MOTOR_STOPPED,
            DriveMotor.TURN_FL: 90,
            DriveMotor.TURN_FR: 90,
            DriveMotor.TURN_RL: 90,
            DriveMotor.TURN_RR: 90
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
        self.declare_parameter('joystick_deadzone', 0.07)

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
        
        # Serial publisher for sending commands to the serial node
        self.serial_publisher = self.create_publisher(
            String,
            '/serial_node/command',
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
        
        # Timers
        self.heartbeat_timer = self.create_timer(1.0, self.heartbeat)
        
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
            
            # Process manual control inputs
            self._process_belt_controls(buttons)
            self._process_misc_controls(buttons)
            
        except Exception as e:
            self.set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Exception in gamepad button callback: {str(e)}"
            )

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

        self._set_steering_angle(1, 45)  # Front Left
        self._set_steering_angle(2, 135)   # Front Right
        self._set_steering_angle(3, 135)  # Rear Left (angling right for clockwise)
        self._set_steering_angle(4, 45)   # Rear Right (angling left for clockwise)
        if r_x_f > 0:  # Clockwise rotation
            # Speeds for clockwise: Left wheels forward, Right wheels reverse
            fl_speed = clamp(MOTOR_STOPPED + speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            fr_speed = clamp(MOTOR_STOPPED - speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            rl_speed = clamp(MOTOR_STOPPED + speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            rr_speed = clamp(MOTOR_STOPPED - speed_offset, MOTOR_FULL_REVERSE, MOTOR_FULL_FORWARD)
            self._set_wheel_speeds(fl_speed, fr_speed, rl_speed, rr_speed)

        elif r_x_f < 0:  # Counter-clockwise rotation
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

    def _process_belt_controls(self, buttons: Dict[str, Any]) -> None:
        """
        Process belt-related button inputs.
        
        Args:
            buttons: Dictionary of button states
        """
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

        # Left bumper => belt forward
        if buttons["lbutton"] != self.lbutton:
            self.lbutton = buttons["lbutton"]
            if self.lbutton:
                self._set_belt_speed(MOTOR_FULL_FORWARD)
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
        self.send_i2c(I2CAddress.EXCAVATION_SYSTEM, [ExcavationMotor.LEFT_ACTUATOR, position])
        self.send_i2c(I2CAddress.EXCAVATION_SYSTEM, [ExcavationMotor.RIGHT_ACTUATOR, position])

    def _set_belt_speed(self, speed: int) -> None:
        """
        Set the belt speed.
        
        Args:
            speed: The speed to set (0-180, 90 is stopped)
        """
        self.send_i2c(I2CAddress.EXCAVATION_SYSTEM, [ExcavationMotor.BELT, speed])

    def _set_auger_deposition(self, direction: int) -> None:
        """
        Set the auger deposition direction.
        
        Args:
            direction: The direction to set (FORWARD, MOTOR_STOPPED, etc.)
        """
        self.send_i2c(I2CAddress.EXCAVATION_SYSTEM, [ExcavationMotor.AUGER, direction])

    def _set_vibrator(self, state: int) -> None:
        """
        Set the vibrator state.
        
        Args:
            state: The state to set (ON, OFF)
        """
        self.send_i2c(I2CAddress.EXCAVATION_SYSTEM, [ExcavationMotor.VIBE_MOTOR, state])

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
        # Map the servo_index (1-4) to the appropriate DriveMotor turn motor
        motor_mapping = {
            1: DriveMotor.TURN_FL,  # Front Left
            2: DriveMotor.TURN_FR,  # Front Right
            3: DriveMotor.TURN_RL,  # Rear Left
            4: DriveMotor.TURN_RR   # Rear Right
        }
        
        # Just store the target angle - the servo timer will gradually move to this position
        self.target_steering_positions[servo_index] = angle
        
        # Also store it in the wheel speeds dictionary for the new motor addressing scheme
        self.target_wheel_speeds[motor_mapping[servo_index]] = angle

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
        self.target_wheel_speeds[DriveMotor.DRIVE_FL] = front_left
        self.target_wheel_speeds[DriveMotor.DRIVE_FR] = front_right
        self.target_wheel_speeds[DriveMotor.DRIVE_RL] = rear_left
        self.target_wheel_speeds[DriveMotor.DRIVE_RR] = rear_right

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
        Send command to the serial node to control the ESP32.
        
        Args:
            address: I2C address or command identifier (now used only for history tracking)
            data: Data to send (can be an int, tuple, or list)
        """
        # For ESP32 serial communication, we only need motor_id and value
        # The ESP32 expects format <motor>,<value> with a newline
        if isinstance(data, (tuple, list)) and len(data) == 2:
            # Translate from our internal motor IDs to ESP32 motor IDs
            motor_id, value = data
            esp32_motor_id = self._translate_motor_id(address, motor_id)
            
            # Format message for ESP32: motor,value
            serial_msg = f"{esp32_motor_id},{value}"
            
            # Keep the I2C history tracking for UI display
            self._update_i2c_history(address, f"{motor_id},{value}")
        else:
            # Handle unexpected format - just convert to string
            self.get_logger().warn(f"Unexpected command format: {data}")
            serial_msg = str(data)
            self._update_i2c_history(address, str(data))
            
        # Send all commands to serial
        self.serial_publisher.publish(String(data=serial_msg))
        self.get_logger().debug(f"Sent serial command: {serial_msg}")

    def _get_motor_name(self, address: int, motor_id: int) -> str:
        """
        Get a human-readable name for a motor ID based on the system address.
        
        Args:
            address: I2C address (system type)
            motor_id: Motor ID within that system
            
        Returns:
            String name of the motor
        """
        if address == I2CAddress.DRIVE_SYSTEM:
            motor_names = {
                DriveMotor.DRIVE_FL: "Front Left Drive",
                DriveMotor.DRIVE_FR: "Front Right Drive",
                DriveMotor.DRIVE_RL: "Rear Left Drive",
                DriveMotor.DRIVE_RR: "Rear Right Drive",
                DriveMotor.TURN_FL: "Front Left Turn",
                DriveMotor.TURN_FR: "Front Right Turn",
                DriveMotor.TURN_RL: "Rear Left Turn",
                DriveMotor.TURN_RR: "Rear Right Turn",
            }
            return motor_names.get(motor_id, f"Unknown Drive Motor ({motor_id})")
        elif address == I2CAddress.EXCAVATION_SYSTEM:
            motor_names = {
                ExcavationMotor.BELT: "Belt",
                ExcavationMotor.AUGER: "Auger",
                ExcavationMotor.LEFT_ACTUATOR: "Left Actuator",
                ExcavationMotor.RIGHT_ACTUATOR: "Right Actuator",
                ExcavationMotor.VIBE_MOTOR: "Vibration Motor",
            }
            return motor_names.get(motor_id, f"Unknown Excavation Motor ({motor_id})")
        else:
            return f"Unknown Motor ({motor_id})"

    def _update_i2c_history(self, address: int, data_str: str) -> None:
        """
        Update the I2C command history for an address.
        
        Args:
            address: I2C address
            data_str: String representation of the data sent
        """
        # Get human-readable name for the address
        device_name = self._get_device_name(address)
        
        # Parse data to get human-readable motor description if it's a list format
        readable_command = data_str
        parts = data_str.split(',')
        if len(parts) >= 2 and parts[0].isdigit():
            motor_id = int(parts[0])
            value = parts[1]
            motor_name = self._get_motor_name(address, motor_id)
            readable_command = f"{motor_name}: {value}"
        
        # Store the command in the history with timestamp
        self.i2c_command_history[address] = {
            'device_name': device_name,
            'address': hex(address),
            'last_command': readable_command,
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
            I2CAddress.DRIVE_SYSTEM: "Drive System",
            I2CAddress.EXCAVATION_SYSTEM: "Excavation System",
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
        
        # No longer need to add specific steering data as it's included in regular I2C commands
            
        # Convert the history to a JSON string
        history_json = json.dumps(history_data)
        
        # Publish the history
        self.i2c_history_pub.publish(String(data=history_json))

    def _update_servo_positions(self) -> None:
        """
        Update servo positions and wheel speeds gradually to their target positions.
        """
        # Create mappings for servo indexes to motor numbers
        servo_to_motor = {
            1: DriveMotor.TURN_FL,  # Front Left
            2: DriveMotor.TURN_FR,  # Front Right
            3: DriveMotor.TURN_RL,  # Rear Left
            4: DriveMotor.TURN_RR   # Rear Right
        }
        
        # Update steering servo positions
        for servo_index, target_position in self.target_steering_positions.items():
            current_position = self.current_steering_positions[servo_index]
            if current_position != target_position:
                step = SERVO_STEP_SIZE if current_position < target_position else -SERVO_STEP_SIZE
                new_position = current_position + step
                if (step > 0 and new_position > target_position) or (step < 0 and new_position < target_position):
                    new_position = target_position
                self.current_steering_positions[servo_index] = new_position
                # Convert servo_index to corresponding motor number and send to DRIVE_SYSTEM address
                self.send_i2c(I2CAddress.DRIVE_SYSTEM, [servo_to_motor[servo_index], new_position])
        
        # Update wheel speeds - only check drive motors, not turn motors (which are handled above)
        drive_motors = [DriveMotor.DRIVE_FL, DriveMotor.DRIVE_FR, DriveMotor.DRIVE_RL, DriveMotor.DRIVE_RR]
        for motor in drive_motors:
            if motor in self.target_wheel_speeds:
                target_speed = self.target_wheel_speeds[motor]
                current_speed = self.current_wheel_speeds[motor]
                if current_speed != target_speed:
                    step = WHEEL_SPEED_STEP_SIZE if current_speed < target_speed else -WHEEL_SPEED_STEP_SIZE
                    new_speed = current_speed + step
                    if (step > 0 and new_speed > target_speed) or (step < 0 and new_speed < target_speed):
                        new_speed = target_speed
                    self.current_wheel_speeds[motor] = new_speed
                    # Send to DRIVE_SYSTEM address with motor as first byte and speed as second byte
                    self.send_i2c(I2CAddress.DRIVE_SYSTEM, [motor, new_speed])

    def _should_use_serial_transport(self, address: int) -> bool:
        """
        Determine if a command should be sent via serial instead of I2C.
        
        Args:
            address: The I2C address or command identifier
            
        Returns:
            True if serial transport should be used, False for I2C
        """
        # Always use serial transport for all commands as per Communication.cpp implementation
        return True

    def _translate_motor_id(self, address: int, motor_id: int) -> int:
        """
        Translate internal motor IDs to the ESP32's expected motor numbering.
        
        The ESP32 uses these motor IDs:
        1-4: Drive motors (FL, FR, RL, RR)
        5-8: Turn/servo motors (FL, FR, RL, RR)
        9: Belt
        10: Auger
        11: Left Actuator
        12: Right Actuator
        13: Vibe motor
        
        Args:
            address: I2C address (used to determine system)
            motor_id: Internal motor ID
            
        Returns:
            Corresponding ESP32 motor ID
        """
        # Drive system motors (already match ESP32 numbering)
        if address == I2CAddress.DRIVE_SYSTEM:
            return motor_id  # DriveMotor IDs already match ESP32's expected numbering
            
        # Excavation system motors need to be remapped
        elif address == I2CAddress.EXCAVATION_SYSTEM:
            # Map excavation motors to ESP32 motor IDs
            excavation_to_esp32 = {
                ExcavationMotor.BELT: 9,           # Belt -> 9
                ExcavationMotor.AUGER: 10,         # Auger -> 10
                ExcavationMotor.LEFT_ACTUATOR: 11, # Left Actuator -> 11
                ExcavationMotor.RIGHT_ACTUATOR: 12, # Right Actuator -> 12
                ExcavationMotor.VIBE_MOTOR: 13     # Vibe Motor -> 13
            }
            return excavation_to_esp32.get(motor_id, motor_id)
            
        # Default: pass through the motor_id unchanged
        return motor_id

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
