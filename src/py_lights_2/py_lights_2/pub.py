import rclpy
from rclpy.node import Node
#^All this stuff above^: We get the node class which we can do some inheritance shenanigans with it.
#Basically take the class, take its functions, and then just fit it too your code.

from std_msgs.msg import String
#This lets us use strings messages for the topics whe publish and subscribe too.

class MinimalPublisher(Node): #inheritance at work. Makes this a 'Node'.

    def __init__(self):
        super().__init__('minimal_publisher') #Name of node, by convention it should be named after the class name.
        self.publisher_ = self.create_publisher(String, 'topic', 10) #Publisher node of type String, topic it publishes to is named 'topic', 10 is the queue sized. In this case only 10 things MAX can be waiting to be published to the subscribers of this node if they cannot get messages.
        timer_period = 0.5  #seconds between callbacks to execute.
        self.timer = self.create_timer(timer_period, self.timer_callback) #seconds between executeion of callbacks.
        self.i = 0 #So you can change the initial value of the timer. Arbitrary values are probably bad, but passed in values, such as parameters, make sense for this context.
        #Like change ^this^ only if you have it taking over for another node or its being initialized mid production. 

    def timer_callback(self): 
        msg = String()
        msg.data = 'Hello World: %d' % self.i #Makes message with timer appened to it
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data) #message is output to console.
        self.i += 1 #tic timer per callback


def main(args=None): #This is the example main function
    rclpy.init(args=args) #no idea what this is. Probably initializes the rclpy library functions

    minimal_publisher = MinimalPublisher() #makes the nodes

    rclpy.spin(minimal_publisher) #the heck is this??????

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown() #probably stops the rclpy libraries???


if __name__ == '__main__': #basically a workaround to have a 'main' function in python.
    main()