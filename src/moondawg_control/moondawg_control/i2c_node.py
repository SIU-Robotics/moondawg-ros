import time
from typing import Optional, Dict, Any, List, Tuple
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from smbus2 import SMBus

class I2CNode(Node):
    """
    Node for handling I2C communication with hardware devices.
    
    This node provides a ROS interface to I2C devices on the robot,
    handling command formatting, error detection, and diagnostics.
    """

    def __init__(self):
        super().__init__('i2c_node')
        
        # Set up parameters
        self._declare_parameters()
        
        # I2C bus setup
        self._bus: Optional[SMBus] = None
        self._device_stats: Dict[int, Dict[str, Any]] = {}  # Track stats per device address
        self._last_command_time = time.time()
        
        # Set up publishers, subscribers, and timers
        self._setup_communications()
        
        # Initialize I2C bus connection
        self._initialize_i2c_bus()
        
        self.get_logger().info(f"I2C node initialized on bus {self.get_parameter('bus_id').value}")

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters for this node."""
        self.declare_parameter('bus_id', 1)
        self.declare_parameter('heartbeat_interval', 1.0)
        self.declare_parameter('command_timeout', 5.0)  # Timeout for commands

    def _setup_communications(self) -> None:
        """Set up publishers, subscribers, and timers."""
        # Diagnostic publisher
        self.diag_status = DiagnosticStatus(
            name=self.get_name(), 
            level=DiagnosticStatus.OK,
            message="Initializing I2C node"
        )
        self.diag_pub = self.create_publisher(
            DiagnosticStatus,
            'diagnostics',
            10
        )
        
        # Command subscriber
        self.create_subscription(
            String,
            'i2c_command',
            self._handle_i2c_command,
            10
        )
        
        # Heartbeat timer - regularly publish diagnostic status
        self.heartbeat_timer = self.create_timer(
            self.get_parameter('heartbeat_interval').value,
            self._heartbeat_callback
        )

    def _initialize_i2c_bus(self) -> None:
        """Initialize the I2C bus connection."""
        try:
            bus_id = self.get_parameter('bus_id').value
            self._bus = SMBus(bus_id)
            self.diag_status.message = f"I2C bus {bus_id} connected"
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus: {str(e)}")
            self.diag_status.level = DiagnosticStatus.ERROR
            self.diag_status.message = f"I2C bus connection failed: {str(e)}"
        
        # Publish initial status
        self.diag_pub.publish(self.diag_status)

    def _handle_i2c_command(self, msg: String) -> None:
        """
        Handle incoming I2C command requests.
        
        Message format: "address:data" where data can be comma-separated values
        Example: "0x10:90" to send 90 to address 0x10
                 "0x20:1,2,3" to send [1,2,3] to address 0x20
        """
        try:
            # Update last command time
            self._last_command_time = time.time()
            
            # Parse message
            parts = msg.data.split(':')
            if len(parts) != 2:
                raise ValueError(f"Invalid command format: {msg.data}")
                
            # Parse address (hex or decimal)
            address_str = parts[0].strip()
            if address_str.startswith('0x'):
                address = int(address_str, 16)
            else:
                address = int(address_str)
                
            # Parse data
            data_part = parts[1].strip()
            if ',' in data_part:
                # Multiple values
                data = [int(x.strip()) for x in data_part.split(',')]
            else:
                # Single value
                data = int(data_part)
            
            # Write to I2C bus
            self._write_to_device(address, data)
            
        except Exception as e:
            self.get_logger().error(f"Error processing I2C command '{msg.data}': {str(e)}")
            self.diag_status.level = DiagnosticStatus.ERROR
            self.diag_status.message = f"Command error: {str(e)}"
            self.diag_pub.publish(self.diag_status)

    def _write_to_device(self, address: int, data: Union[int, List[int]]) -> None:
        """
        Write data to an I2C device.
        
        Args:
            address: I2C device address
            data: Single byte or list of bytes to write
        """
        if self._bus is None:
            raise RuntimeError("I2C bus not initialized")
            
        try:
            # Update device stats
            if address not in self._device_stats:
                self._device_stats[address] = {
                    'commands_sent': 0,
                    'last_command': None,
                    'last_error': None,
                    'errors': 0
                }
            
            # Write data
            if isinstance(data, list):
                if len(data) == 1:
                    self._bus.write_byte(address, data[0])
                else:
                    self._bus.write_i2c_block_data(address, data[0], data[1:])
            else:
                self._bus.write_byte(address, data)
                
            # Update stats
            self._device_stats[address]['commands_sent'] += 1
            self._device_stats[address]['last_command'] = data
            
            self.get_logger().debug(f"Sent to address {hex(address)}: {data}")
            
        except Exception as e:
            error_msg = f"I2C write error to {hex(address)}: {str(e)}"
            self.get_logger().error(error_msg)
            
            # Update error stats
            self._device_stats[address]['errors'] += 1
            self._device_stats[address]['last_error'] = error_msg
            
            # Update diagnostic status
            self.diag_status.level = DiagnosticStatus.ERROR
            self.diag_status.message = error_msg
            self.diag_pub.publish(self.diag_status)
            
            # Re-raise for higher level handling
            raise

    def _heartbeat_callback(self) -> None:
        """
        Regular heartbeat callback - publish diagnostic status.
        
        Monitors command timeouts and I2C bus health.
        """
        # Check command timeout
        time_since_last_cmd = time.time() - self._last_command_time
        timeout = self.get_parameter('command_timeout').value
        
        # Build key-value pairs for all device stats
        values = []
        total_errors = 0
        
        for addr, stats in self._device_stats.items():
            addr_hex = hex(addr)
            values.append(KeyValue(key=f"{addr_hex}_commands", value=str(stats['commands_sent'])))
            values.append(KeyValue(key=f"{addr_hex}_errors", value=str(stats['errors'])))
            
            if stats['last_command'] is not None:
                if isinstance(stats['last_command'], list):
                    cmd_str = ','.join(str(x) for x in stats['last_command'])
                else:
                    cmd_str = str(stats['last_command'])
                values.append(KeyValue(key=f"{addr_hex}_last_cmd", value=cmd_str))
                
            if stats['last_error'] is not None:
                values.append(KeyValue(key=f"{addr_hex}_last_error", value=stats['last_error']))
            
            total_errors += stats['errors']
        
        # Add timing info
        values.append(KeyValue(key="time_since_command", value=f"{time_since_last_cmd:.1f}s"))
        
        # Update diagnostic status
        if self._bus is None:
            self.diag_status.level = DiagnosticStatus.ERROR
            self.diag_status.message = "I2C bus not connected"
        elif time_since_last_cmd > timeout and timeout > 0:
            self.diag_status.level = DiagnosticStatus.WARN
            self.diag_status.message = f"No commands for {time_since_last_cmd:.1f}s (timeout: {timeout}s)"
        elif total_errors > 0:
            self.diag_status.level = DiagnosticStatus.WARN
            self.diag_status.message = f"I2C errors: {total_errors}"
        else:
            self.diag_status.level = DiagnosticStatus.OK
            self.diag_status.message = f"I2C bus {self.get_parameter('bus_id').value} operational"
        
        self.diag_status.values = values
        self.diag_pub.publish(self.diag_status)

    def destroy_node(self) -> None:
        """Clean up resources when node is shutting down."""
        if self._bus is not None:
            try:
                self._bus.close()
                self.get_logger().info("I2C bus closed")
            except Exception as e:
                self.get_logger().error(f"Error closing I2C bus: {str(e)}")
                
        super().destroy_node()


def main(args=None):
    """Main entry point for the node."""
    try:
        rclpy.init(args=args)
        node = I2CNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"Error in I2C node: {str(e)}")


if __name__ == '__main__':
    main()
