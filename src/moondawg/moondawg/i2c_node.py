import time
from typing import Optional, Dict, Any, List, Tuple
import rclpy
from rclpy.lifecycle import Node
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
            '/i2c_node/diag', 
            10
        )
        
        # Command subscriber
        self.create_subscription(
            String, 
            '/i2c_node/command', 
            self._command_callback, 
            10
        )
        
        # Heartbeat timer
        heartbeat_interval = self.get_parameter('heartbeat_interval').get_parameter_value().double_value
        self.create_timer(heartbeat_interval, self._heartbeat)

    def _initialize_i2c_bus(self) -> None:
        """Initialize the I2C bus connection."""
        try:
            bus_id = self.get_parameter('bus_id').get_parameter_value().integer_value
            self._bus = SMBus(bus_id)
            self._set_diagnostic_status(
                DiagnosticStatus.OK, 
                f"I2C bus {bus_id} open and ready"
            )
        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Failed to open I2C bus: {str(e)}"
            )

    def _command_callback(self, msg: String) -> None:
        """
        Process I2C command messages.
        
        Format: <address>:<data> where data can be comma-separated values
        Example: "0x10:90,120" - sends values 90 and 120 to device at address 0x10
        
        Args:
            msg: Command message containing address and data
        """
        # Update the last command time
        self._last_command_time = time.time()
        
        try:
            # Parse the command
            parts = msg.data.split(':', 1)
            if len(parts) < 2:
                self._set_diagnostic_status(
                    DiagnosticStatus.WARN, 
                    f"Invalid command format: {msg.data} - should be <address>:<data>"
                )
                return
                
            addr_str, data_str = parts[0], parts[1]
            
            # Parse address (supports hex like 0x10 or decimal)
            try:
                address = int(addr_str, 0)  # Base 0 allows for hex and decimal
            except ValueError:
                self._set_diagnostic_status(
                    DiagnosticStatus.WARN, 
                    f"Invalid I2C address format: {addr_str}"
                )
                return
                
            # Parse data
            if ',' in data_str:
                # Multiple values to send
                try:
                    values = [self._parse_value(v) for v in data_str.split(',')]
                    self._write_i2c_block(address, values)
                except ValueError as e:
                    self._set_diagnostic_status(
                        DiagnosticStatus.WARN, 
                        f"Invalid data format: {str(e)}"
                    )
            else:
                # Single value to send
                try:
                    value = self._parse_value(data_str)
                    self._write_i2c_byte(address, value)
                except ValueError as e:
                    self._set_diagnostic_status(
                        DiagnosticStatus.WARN, 
                        f"Invalid data format: {str(e)}"
                    )
                    
            # Update stats for this device
            if address not in self._device_stats:
                self._device_stats[address] = {
                    'write_count': 0,
                    'error_count': 0,
                    'last_command': '',
                    'last_time': 0
                }
                
            self._device_stats[address]['write_count'] += 1
            self._device_stats[address]['last_command'] = data_str
            self._device_stats[address]['last_time'] = time.time()
            
        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Error processing I2C command: {str(e)}"
            )

    def _parse_value(self, value_str: str) -> int:
        """
        Parse a string value to an integer.
        
        Args:
            value_str: String to parse
            
        Returns:
            Parsed integer value
            
        Raises:
            ValueError: If parsing fails
        """
        # Check if it's a byte string
        if value_str.startswith("b'") or value_str.startswith('b"'):
            return ord(eval(value_str))
            
        # Try to parse as int (hex or decimal)
        try:
            return int(value_str, 0)
        except ValueError:
            raise ValueError(f"Cannot parse value: {value_str}")

    def _write_i2c_byte(self, address: int, value: int) -> None:
        """
        Write a single byte to an I2C device.
        
        Args:
            address: I2C device address
            value: Byte value to write
        """
        if self._bus is None:
            self.get_logger().error(f"Could not write to I2C bus. Would have wrote byte {value} to device {hex(address)}")
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                "I2C bus not initialized"
            )
            return
            
        try:
            self._bus.write_byte(address, value)
            self.get_logger().debug(f"Wrote byte {value} to device {hex(address)}")
        except Exception as e:
            self._record_error(address, f"Error writing byte {value} to {hex(address)}: {str(e)}")

    def _write_i2c_block(self, address: int, values: List[int]) -> None:
        """
        Write a block of data to an I2C device.
        
        Args:
            address: I2C device address
            values: List of values to write
        """
        if self._bus is None:
            self.get_logger().error(f"Could not write to I2C bus. Would have wrote block {values} to device {hex(address)}")
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                "I2C bus not initialized"
            )
            return
            
        try:
            # SMBus protocol requires a command byte followed by data bytes
            # Use the first byte as the command and the rest as data
            if len(values) > 1:
                cmd = values[0]
                data = values[1:]
                self._bus.write_i2c_block_data(address, cmd, data)
                self.get_logger().debug(
                    f"Wrote block data {data} with command {cmd} to device {hex(address)}"
                )
            else:
                # If only one value, use it as a single byte write
                self._bus.write_byte(address, values[0])
                self.get_logger().debug(f"Wrote byte {values[0]} to device {hex(address)}")
                
        except Exception as e:
            self._record_error(address, f"Error writing block {values} to {hex(address)}: {str(e)}")

    def _record_error(self, address: int, message: str) -> None:
        """
        Record an error for a specific device address.
        
        Args:
            address: Device address where error occurred
            message: Error message
        """
        # Initialize stats for this device if needed
        if address not in self._device_stats:
            self._device_stats[address] = {
                'write_count': 0,
                'error_count': 0,
                'last_command': '',
                'last_time': 0
            }
            
        # Update error count
        self._device_stats[address]['error_count'] += 1
        
        # Set diagnostic status
        self._set_diagnostic_status(
            DiagnosticStatus.ERROR,
            message
        )

    def _set_diagnostic_status(self, level: int, message: str) -> None:
        """
        Update diagnostic status and log appropriately.
        
        Args:
            level: Diagnostic level (OK, WARN, ERROR)
            message: Status message
        """
        self.diag_status.level = level
        self.diag_status.message = message
        
        # Update diagnostic values with device statistics
        self.diag_status.values = []
        
        # Add bus information
        if self._bus is not None:
            bus_id = self.get_parameter('bus_id').get_parameter_value().integer_value
            self.diag_status.values.append(KeyValue(key='bus_id', value=str(bus_id)))
            
            # Add command statistics
            cmd_count = sum(stats['write_count'] for stats in self._device_stats.values())
            error_count = sum(stats['error_count'] for stats in self._device_stats.values())
            
            self.diag_status.values.append(KeyValue(key='total_commands', value=str(cmd_count)))
            self.diag_status.values.append(KeyValue(key='error_count', value=str(error_count)))
            
            # Add info about the most recent active devices (up to 5)
            active_devices = sorted(
                [(addr, stats) for addr, stats in self._device_stats.items()],
                key=lambda x: x[1]['last_time'],
                reverse=True
            )[:5]
            
            for i, (addr, stats) in enumerate(active_devices):
                label = f"device_{i}"
                addr_hex = hex(addr)
                cmd_count = stats['write_count']
                last_cmd = stats['last_command']
                
                self.diag_status.values.append(
                    KeyValue(
                        key=f"{label}_addr", 
                        value=f"{addr_hex} (writes: {cmd_count})"
                    )
                )
                self.diag_status.values.append(
                    KeyValue(
                        key=f"{label}_last_cmd", 
                        value=last_cmd
                    )
                )
        
        # Log the message based on severity
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(message)
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _heartbeat(self) -> None:
        """Publish diagnostic status periodically."""
        # Check how long it's been since last command
        time_since_last_cmd = time.time() - self._last_command_time
        command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        
        if time_since_last_cmd > command_timeout and len(self._device_stats) > 0:
            # Only warn if we've received commands before and now they've stopped
            self.get_logger().debug(
                f"No I2C commands received in {time_since_last_cmd:.1f}s"
            )
            
        # Publish diagnostic status
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