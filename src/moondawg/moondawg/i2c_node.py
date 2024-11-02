import rclpy
import rclpy.logging
from rclpy.lifecycle import Node
from smbus2 import SMBus
from std_msgs.msg import String, Header
from diagnostic_msgs.msg import DiagnosticStatus

class I2CNode(Node): 
    def __init__(self):
        super().__init__(node_name='i2c_node')

        heartbeat_interval = 1
        self.diag = DiagnosticStatus(name=self.ger_name(), level=DiagnosticStatus.OK,
                                     hardware_id=str(hardware_id)), message="Initializing")
        self.diag_topic = self.create_publisher(DiagnosticStatus, 'i2c_node_diag', 10)
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self.heartbeat)

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_drive1', 0x04)
        self.declare_parameter('i2c_drive2', 0x05)
        self.declare_parameter('i2c_drive3', 0x06)
        self.declare_parameter('i2c_drive4', 0x07)

        self.declare_parameter('i2c_turn1', 0x08)
        self.declare_parameter('i2c_turn2', 0x09)
        self.declare_parameter('i2c_turn3', 0x10)
        self.declare_parameter('i2c_turn4', 0x11)

        self.declare_parameter('i2c_belt', 0x12)
        self.declare_parameter('i2c_deploy', 0x13)

        self.declare_parameter('i2c_deposit', 0x14)
        self.declare_parameter('i2c_vibrator', 0x15)

        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.i2c_drive1 = self.get_parameter('i2c_drive1').get_parameter_value().integer_value
        self.i2c_drive2 = self.get_parameter('i2c_drive2').get_parameter_value().integer_value
        self.i2c_drive3 = self.get_parameter('i2c_drive3').get_parameter_value().integer_value
        self.i2c_drive4 = self.get_parameter('i2c_drive4').get_parameter_value().integer_value

        self.i2c_turn1 = self.get_parameter('i2c_turn1').get_parameter_value().integer_value
        self.i2c_turn2 = self.get_parameter('i2c_turn2').get_parameter_value().integer_value
        self.i2c_turn3 = self.get_parameter('i2c_turn3').get_parameter_value().integer_value
        self.i2c_turn4 = self.get_parameter('i2c_turn4').get_parameter_value().integer_value

        self.i2c_belt = self.get_parameter('i2c_belt').get_parameter_value().integer_value
        self.i2c_deploy = self.get_parameter('i2c_deploy').get_parameter_value().integer_value

        self.i2c_deposit = self.get_parameter('i2c_deposit').get_parameter_value().integer_value
        self.i2c_vibrator = self.get_parameter('i2c_vibrator').get_parameter_value().integer_value

        try:
            self.i2c_bus = SMBus(self.i2c_bus)
            self.diag.level = DiagnosticStatus.OK
            self.diag.message = "I2C Connected"
            self.get_logger().info("I2C Connected")
        except Exception as e:
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = "I2C Connection Error: {str(e)}"
            self.get_logger().error(self.diag.message)
            self.i2c_bus = None

        self.subscription_drive1 = self.create_subscription(String, 'drive1', lambda msg: self.drive1_callback(msg, self.i2c_drive1), 10)
        self.subscription_drive2 = self.create_subscription(String, 'drive2', lambda msg: self.drive1_callback(msg, self.i2c_drive2), 10)
        self.subscription_drive3 = self.create_subscription(String, 'drive3', lambda msg: self.drive1_callback(msg, self.i2c_drive3), 10)
        self.subscription_drive4 = self.create_subscription(String, 'drive4', lambda msg: self.drive1_callback(msg, self.i2c_drive4), 10)

        self.subscription_turn1 = self.create_subscription(String, 'turn1', lambda msg: self.turn1_callback(msg, self.i2c_turn1), 10)
        self.subscription_turn2 = self.create_subscription(String, 'turn2', lambda msg: self.turn1_callback(msg, self.i2c_turn2), 10)
        self.subscription_turn3 = self.create_subscription(String, 'turn3', lambda msg: self.turn1_callback(msg, self.i2c_turn3), 10)
        self.subscription_turn4 = self.create_subscription(String, 'turn4', lambda msg: self.turn1_callback(msg, self.i2c_turn4), 10)

        self.subscription_belt = self.create_subscription(String, 'belt', lambda msg: self.belt_callback(msg, self.i2c_belt), 10)
        self.subscription_deploy = self.create_subscription(String, 'deploy', lambda msg: self.deploy_callback(msg, self.i2c_deploy), 10)

        self.subscription_deposit = self.create_subscription(String, 'deposit', lambda msg: self.deposit_callback(msg, self.i2c_deposit), 10)
        self.subscription_vibrator = self.create_subscription(String, 'vibrator', lambda msg: self.vibrator_callback(msg, self.i2c_vibrator), 10)

        self.publisher_drive1 = self.create_publisher(String, 'drive1_recieved_1', 10)
        self.publisher_drive2 = self.create_publisher(String, 'drive2_recieved_1', 10)
        self.publisher_drive3 = self.create_publisher(String, 'drive3_recieved_1', 10)
        self.publisher_drive4 = self.create_publisher(String, 'drive4_recieved_1', 10)

        self.publisher_turn1 = self.create_publisher(String, 'turn1_recieved_1', 10)
        self.publisher_turn2 = self.create_publisher(String, 'turn2_recieved_1', 10)
        self.publisher_turn3 = self.create_publisher(String, 'turn3_recieved_1', 10)
        self.publisher_turn4 = self.create_publisher(String, 'turn4_recieved_1', 10)

        self.publisher_belt = self.create_publisher(String, 'belt_recieved_1', 10)
        self.publisher_deploy = self.create_publisher(String, 'deploy_recieved_1', 10)

        self.publisher_deposit = self.create_publisher(String, 'deposit_recieved_1', 10)
        self.publisher_vibrator = self.create_publisher(String, 'vibrator_recieved_1', 10)

    def i2c_callback(self,message, address):
        if self.i2c_bus is None:
            self.get_logger().error("I2C Bus not connected")
            return
        try:
            data = message.data + '\n'
            self.i2c_bus.write_i2c_block_data(address, 0, list(data.encode()))
            if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
                self.get_logger().debug(f"Wrote to address {hex(address)}: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Error sending data to address {hex(address)}: {str(e)}")
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = f"I2C write error: {str(e)}"

    def read_i2c(self):
        if self.i2c_bus is None:
            self.get_logger().error("I2C Bus not connected")
            return
        
        try:
            drive1_data = self.read_from.device(self.i2c_drive1)
            if drive1_data:
                msg = String()
                msg.data = drive1_data
                self.publisher_drive1.publish(msg)

            drive2_data = self.read_from.device(self.i2c_drive2)
            if drive2_data:
                msg = String()
                msg.data = drive2_data
                self.publisher_drive2.publish(msg)

            drive3_data = self.read_from.device(self.i2c_drive3)
            if drive3_data:
                msg = String()
                msg.data = drive3_data
                self.publisher_drive3.publish(msg)

            drive4_data = self.read_from.device(self.i2c_drive4)
            if drive4_data:
                msg = String()
                msg.data = drive4_data
                self.publisher_drive4.publish(msg)

            turn1_data = self.read_from.device(self.i2c_turn1)
            if turn1_data:
                msg = String()
                msg.data = turn1_data
                self.publisher_turn1.publish(msg)

            turn2_data = self.read_from.device(self.i2c_turn2)
            if turn2_data:
                msg = String()
                msg.data = turn2_data
                self.publisher_turn2.publish(msg)

            turn3_data = self.read_from.device(self.i2c_turn3)
            if turn3_data:
                msg = String()
                msg.data = turn3_data
                self.publisher_turn3.publish(msg)

            turn4_data = self.read_from.device(self.i2c_turn4)
            if turn4_data:
                msg = String()
                msg.data = turn4_data
                self.publisher_turn4.publish(msg)

            belt_data = self.read_from.device(self.i2c_belt)
            if belt_data:
                msg = String()
                msg.data = belt_data
                self.publisher_belt.publish(msg)

            deploy_data = self.read_from.device(self.i2c_deploy)
            if deploy_data:
                msg = String()
                msg.data = deploy_data
                self.publisher_deploy.publish(msg)

            deposit_data = self.read_from.device(self.i2c_deposit)
            if deposit_data:
                msg = String()
                msg.data = deposit_data
                self.publisher_deposit.publish(msg)

            vibrator_data = self.read_from.device(self.i2c_vibrator)
            if vibrator_data:
                msg = String()
                msg.data = vibrator_data
                self.publisher_vibrator.publish(msg)

        except Exception as e:
            self.get_logger().error(f"I2C read error: {str(e)}")
            self.diag.level = DiagnosticStatus.ERROR
            self.diag.message = f"I2C read error: {str(e)}"

    def read_from_device(self, address):
        try:
            data = self.i2c_bus.read_i2c_block_data(address, 0, 32)
            return bytes(data).decode().rstrip('\x00')
        except Exception as e:
            return None
        
    def stop(self):
        if self.i2c_bus is not None:
            self.i2c_bus.close()

    def heartbeat(self):
        self.diag_topic.publish(self.diag)
        if self.get_logger().get_effective_level() <= rclpy.logging.LoggingSeverity.DEBUG:
            self.get_logger().debug(
                f"Heartbeat published with diagnostic status: Level: {self.diag.level},"
                f"Message: {self.diag.message}"
            )

def main(args=None):
    rclpy.init(args=args)
    i2c_node = I2CNode()

    try:
        rclpy.spin(i2c_node)
    except KeyboardInterrupt:
        i2c_node.get_logger().warning('Ctrl-C pressed, shutting down...')
    finally:
        i2c_node.stop()
        rclpy.shutdown()
        exit(130)

if __name__ == '__main__':
    main()