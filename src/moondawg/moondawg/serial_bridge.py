#doesnt work right now

import serial as ser
import random
import string
import time
import rclpy
from rclpy.lifecycle import Node
import queue as q
from std_msgs.msg import Int32MultiArray

class SerialBridge(Node): 

    def __init__(self, baud_rate = 9600, port = "/dev/ttyUSB0", size = 0):
        self.rate = baud_rate
        self.port = port
        self.size = size
        self.open = True
        self.queue_out = q.Queue(size) #Arduino Transmit Buffer
        self.queue_in  = q.Queue(size) #Arduino Serial Buffer
        self.serial = ser.Serial(port=self.port, baudrate=self.rate, bytesize=self.size, parity="N")
        self.subscription = self.create_subscription(Int32MultiArray, 'serial_topic', self.serial_callback, 10)

    def enqueue(self, cmd = "", data = ""):
        input = cmd + " " + data + "\0"
        self.queue_in.put(input)

    def enqueue_RAW(self, input):
        self.queue_in.put(input)

    def dequeue(self):
        print("\nArduinio Said: " + self.queue_out.get() + ".\n What a nerd thing to say.\n")

    #def __repr__(self):
    #    return f""

    #def __str__(self):
    #    if(self.open):
    #        return f""
    #    else:
    #        return f""
    
    def get_port(self):
        return self.port
    
    def set_port(self, port):
        self.port = port
    
    def open(self):
        self.serial.open() #the store is now open
        self.open = True

    def close(self):
        self.serial.close() #kill it with fire
        self.open = False

    def serial_status(self):
        if(self.open):
            print(f"{self.port} can take input.")
        else:
            print(f"{self.port} can't take input.")

def generate_random_string(length):
    return (''.join(random.choices(string.ascii_letters + string.digits, k=length-1))+ '\n')

def main(args=None): 
    rclpy.init(args=args)

    serialBridge = SerialBridge(size=8)

    rclpy.spin(serialBridge)

    rclpy.shutdown()

    cereal = RosSerial(size=8)
    cereal.open()
    
    loopstuff = input("\nTest code? Hit y key to do so.\n")
    loopstuff.upper()

    cereal.serial.open()

    while (loopstuff!='Y'):
        rng_jesus = generate_random_string(8)
        print("\n Sent",rng_jesus,"to arduino.\n")
        for c in rng_jesus:
            cereal.serial.write(c.encode())
        from_arduino = cereal.serial.readline()
        print("\nArduino sent us:",from_arduino, ".\n")
        loopstuff = input("\nTest code more? Type y to do so.\n")
        loopstuff.upper()
    
    cereal.close()

if __name__ == '__main__': 
    main()