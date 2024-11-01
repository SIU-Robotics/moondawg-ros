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