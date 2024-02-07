import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

from std_msgs.msg import String



class LightsSubscriber(Node):
    
    def __init__(self):
        super().__init__('lights_subscriber')
        self.subscription = self.create_subscription(
            String, 
            'lights_2', 
            self.listener_callback, 
            10) 
        self.subscription 

    def listener_callback(self, msg): 
        self.get_logger().info('Received Light code ["%s"].' % msg.data)
        pin_nums = [2,3,4,17]
        gpio_bin_parse(pin_nums, msg.data)

def main(args=None): 
    pin_nums = [2,3,4,17]
    gpio = gpio_init(pin_nums)
    if(not gpio): return 0
    
    rclpy.init(args=args)
    lights_subscriber = LightsSubscriber()
    rclpy.spin(lights_subscriber)
    lights_subscriber.destroy_node()

    gpio_kill()
    rclpy.shutdown()

def gpio_init(pin_numbers = []):
    if(len(pin_numbers)>0):
        #GPIO.setmode(GPIO.BCM)
        for pin in range(pin_numbers):
            print("\npin#", pin)
            GPIO.setup(pin_numbers[pin], GPIO.OUT)
        return 1
    else: return 0

def gpio_bin_parse(pin_numbers = [], bin_msg = ''):
    #LEFT->RIGHT for binary msg.
    if (len(bin_msg)>0 and len(pin_numbers)>0 and len(pin_numbers)==len(bin_msg)):
        for i in range(len(pin_numbers)):
            print("\nposition->state:", i, bin_msg[i])
            GPIO.output(pin_numbers[i], int(bin_msg[i]))
        return 1
    else: return 0

def gpio_kill():
    GPIO.cleanup()
    print("\nkilled gpio")


if __name__ == '__main__':
    main()