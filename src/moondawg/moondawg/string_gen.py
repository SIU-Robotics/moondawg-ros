from std_msgs.msg import String

class StringGen():
    
    @staticmethod
    def deposit_string(enabled, direction):
        string = String()
        string.data = f"d,{enabled},{direction}"
        return string
    
    @staticmethod
    def belt_string(enabled, speed):
        string = String()
        string.data = f"b,{enabled},{speed}"
        return string
    
    @staticmethod
    def belt_speed_string(enabled, speed):
        string = String()
        string.data = f"b,{enabled},{speed}"
        return string
    
    @staticmethod
    def belt_position_string(enabled, direction):
        string = String()
        string.data = f"g,{enabled},{direction}"
        return string
        
    @staticmethod
    def movement_string(lspeed, rspeed):
        string = String()
        string.data = f"m,{lspeed},{rspeed}"
        return string
        
    @staticmethod
    def vibrator_string(enabled):
        string = String()
        string.data = f"v,{enabled},v"
        return string
    
    @staticmethod
    def camera_pitch_string(pitch):
        string = String()
        string.data = f"e,1,{pitch}"
        return string

    @staticmethod
    def camera_angle_string(angle):
        string = String()
        string.data = f"h,1,{angle}"
        return string

    @staticmethod
    def camera_arm_string(angle):
        string = String()
        string.data = f"a,1,{angle}"
        return string