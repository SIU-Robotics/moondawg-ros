from std_msgs.msg import String

class StringGen():
    
    @staticmethod
    def deposit_string(enabled, direction) -> String:
        """
        Returns a string to send to the arduino to control the deposit auger.

        Parameters:
        enabled: Whether or not the auger should move at all
        direction: 'f' for forward (depositing) or 'b' for backward

        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"d,{enabled},{direction}"
        return string
    
    @staticmethod
    def belt_string(enabled, speed) -> String:
        """
        Returns a string to send to the arduino to control the spin of the belt.

        Parameters:
        enabled: Whether or not the belt should spin at all
        speed: Integer value between 0 and 180, with 90 being not spinning and 180 being max spin forward 

        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"b,{enabled},{speed}"
        return string
    
    @staticmethod
    def belt_position_string(enabled, direction) -> String:
        """
        Returns a string to send to the arduino to control the position of the belt.

        Parameters:
        enabled: Whether or not the belt position should move at all
        direction: 'l' for down or 'r' for up
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"g,{enabled},{direction}"
        return string
        
    @staticmethod
    def movement_string(lspeed, rspeed) -> String:
        """
        Returns a string to send to the arduino to control the movement of the wheels.

        Parameters:
        lspeed: The speed of the wheels for the left side of the bot (0-180, 90 stopped)
        direction: The speed of the wheels for the right side of the bot (0-180, 90 stopped)
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"m,{lspeed},{rspeed}"
        return string
        
    @staticmethod
    def vibrator_string(enabled) -> String:
        """
        Returns a string to send to the arduino to control the vibration motor.

        Parameters:
        enabled: Whether or not the vibration motor should be turned on
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"v,{enabled},v"
        return string
    
    @staticmethod
    def camera_pitch_string(pitch) -> String:
        """
        Returns a string to send to the arduino to control the camera's pitch (up and down)

        Parameters:
        pitch: Servo position value between 0 and 180
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"e,1,{pitch}"
        return string

    @staticmethod
    def camera_angle_string(angle) -> String:
        """
        Returns a string to send to the arduino to control the camera's angle (left and right)

        Parameters:
        pitch: Servo position value between 0 and 180
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"h,1,{angle}"
        return string

    @staticmethod
    def camera_arm_string(angle) -> String:
        """
        Returns a string to send to the arduino to control the camera arm position.

        Parameters:
        pitch: Servo position value between 0 and 180
        
        Returns:
        String: ROS String message format
        """
        string = String()
        string.data = f"a,1,{angle}"
        return string