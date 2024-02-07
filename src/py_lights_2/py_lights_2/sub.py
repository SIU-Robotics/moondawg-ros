import rclpy #Same as publisher
from rclpy.node import Node #Same as publisher


from std_msgs.msg import String #Same as publisher



class MinimalSubscriber(Node):
    #Needs to be evaluated
    def __init__(self):
        super().__init__('minimal_subscriber') #Same as publisher
        self.subscription = self.create_subscription(
            String, #Type must match the desired publisher node message type
            'topic', #Topic name must match the topic you wish to subscribe to
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning ##THE HECK IS THIS???

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None): #SAME AS PUBLISHER FILE. THIS IS PROBABLY BASED ON CONVENTION.
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()