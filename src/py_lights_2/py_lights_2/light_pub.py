import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class LightsPublisher(Node): 

    def __init__(self):
        super().__init__('lights_publisher')
        self.publisher_ = self.create_publisher(String, 'lights_2', 10) 
        timer_period = 1 #acting as a clk
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0b0000
        

    def timer_callback(self): 
        msg = String()
        Half_Byte = bin(self.i)[2:]
        while(len(Half_Byte)<4): Half_Byte = '0' + Half_Byte
        msg.data = Half_Byte
        self.publisher_.publish(msg)
        self.get_logger().info('Light code ["%s"] published.' % msg.data) #message is output to console.
        self.i = self.i + 0b0001 if (self.i < 15) else 0b0000 

def main(args=None): 
    rclpy.init(args=args) 
    lights_publisher = LightsPublisher() 
    rclpy.spin(lights_publisher) 
    lights_publisher.destroy_node()
    rclpy.shutdown() 

if __name__ == '__main__': 
    main()