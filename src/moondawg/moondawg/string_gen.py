from std_msgs.msg import String
from typing import Literal, Union, overload

class StringGen:
    """
    Utility class for generating formatted command strings to send to the robot's components.
    
    This class provides static methods that create properly formatted command strings for
    different components of the robot, including:
    - Deposit auger
    - Belt operation (speed and position)
    - Movement control
    - Vibration motor
    - Camera control
    
    All methods return ROS String messages with properly formatted data.
    """
    
    @staticmethod
    def deposit_string(enabled: bool, direction: Literal['f', 'b']) -> String:
        """
        Generate a command string to control the deposit auger.

        Args:
            enabled: Whether or not the auger should move at all
            direction: 'f' for forward (depositing) or 'b' for backward

        Returns:
            String: ROS String message with formatted command
        """
        string = String()
        string.data = f"d,{int(enabled)},{direction}"
        return string
    
    @staticmethod
    def belt_string(enabled: bool, speed: int = 90) -> String:
        """
        Generate a command string to control the belt spin.

        Args:
            enabled: Whether or not the belt should spin
            speed: Integer value between 0 and 180, with 90 being stopped,
                  180 being max spin forward, and 0 being max spin backward

        Returns:
            String: ROS String message with formatted command
        """
        # Ensure speed is within valid range
        speed = max(0, min(180, speed))
        
        string = String()
        string.data = f"b,{int(enabled)},{speed}"
        return string
    
    @staticmethod
    def belt_position_string(enabled: bool, direction: Literal['l', 'r']) -> String:
        """
        Generate a command string to control the belt position.

        Args:
            enabled: Whether or not the belt position should move
            direction: 'l' for down or 'r' for up
        
        Returns:
            String: ROS String message with formatted command
        """
        string = String()
        string.data = f"g,{int(enabled)},{direction}"
        return string
        
    @staticmethod
    def movement_string(lspeed: int, rspeed: int) -> String:
        """
        Generate a command string to control wheel movement.

        Args:
            lspeed: The speed of the wheels for the left side (0-180, 90 stopped)
            rspeed: The speed of the wheels for the right side (0-180, 90 stopped)
        
        Returns:
            String: ROS String message with formatted command
        """
        # Ensure speeds are within valid range
        lspeed = max(0, min(180, lspeed))
        rspeed = max(0, min(180, rspeed))
        
        string = String()
        string.data = f"m,{lspeed},{rspeed}"
        return string
        
    @staticmethod
    def vibrator_string(enabled: bool) -> String:
        """
        Generate a command string to control the vibration motor.

        Args:
            enabled: Whether or not the vibration motor should be on
        
        Returns:
            String: ROS String message with formatted command
        """
        string = String()
        string.data = f"v,{int(enabled)},v"
        return string
    
    @staticmethod
    def camera_pitch_string(pitch: int) -> String:
        """
        Generate a command string to control the camera's pitch (up/down).

        Args:
            pitch: Servo position value between 0 and 180
        
        Returns:
            String: ROS String message with formatted command
        """
        # Ensure pitch is within valid range
        pitch = max(0, min(180, pitch))
        
        string = String()
        string.data = f"e,1,{pitch}"
        return string

    @staticmethod
    def camera_angle_string(angle: int) -> String:
        """
        Generate a command string to control the camera's angle (left/right).

        Args:
            angle: Servo position value between 0 and 180
        
        Returns:
            String: ROS String message with formatted command
        """
        # Ensure angle is within valid range
        angle = max(0, min(180, angle))
        
        string = String()
        string.data = f"h,1,{angle}"
        return string

    @staticmethod
    def camera_arm_string(angle: int) -> String:
        """
        Generate a command string to control the camera arm position.

        Args:
            angle: Servo position value between 0 and 180
        
        Returns:
            String: ROS String message with formatted command
        """
        # Ensure angle is within valid range
        angle = max(0, min(180, angle))
        
        string = String()
        string.data = f"a,1,{angle}"
        return string