import time
from typing import Optional, Dict, Any, List
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import serial

class SerialNode(Node):
    """
    Node for handling serial communication with hardware devices.
    
    This node provides a ROS interface to a serial device on the robot,
    handling command formatting, error detection, and diagnostics.
    """

    def __init__(self):
        super().__init__('serial_node')
        
        # Set up parameters
        self._declare_parameters()
        
        # Serial connection setup
        self._serial: Optional[serial.Serial] = None
        self._device_stats: Dict[str, Dict[str, Any]] = {}  # Track stats for the serial device
        self._last_command_time = time.time()
        
        # Set up publishers, subscribers, and timers
        self._setup_communications()
        
        # Initialize serial connection
        self._initialize_serial_connection()
        
        self.get_logger().info(f"Serial node initialized on port {self.get_parameter('port').value}")

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters for this node."""
        self.declare_parameter('port', '/dev/ttyUSB0')  # Default serial port
        self.declare_parameter('baud_rate', 115200)     # Default baud rate
        self.declare_parameter('timeout', 1.0)          # Serial timeout in seconds
        self.declare_parameter('line_ending', '\n')     # Line termination character
        self.declare_parameter('heartbeat_interval', 1.0)
        self.declare_parameter('command_timeout', 5.0)  # Timeout for commands

    def _setup_communications(self) -> None:
        """Set up publishers, subscribers, and timers."""
        # Diagnostic publisher
        self.diag_status = DiagnosticStatus(
            name=self.get_name(), 
            level=DiagnosticStatus.OK,
            message="Initializing Serial node"
        )
        self.diag_pub = self.create_publisher(
            DiagnosticStatus, 
            '/serial_node/diag', 
            10
        )
        
        # Command subscriber
        self.create_subscription(
            String, 
            '/serial_node/command', 
            self._command_callback, 
            10
        )
        
        # Heartbeat timer
        heartbeat_interval = self.get_parameter('heartbeat_interval').get_parameter_value().double_value
        self.create_timer(heartbeat_interval, self._heartbeat)

    def _initialize_serial_connection(self) -> None:
        """Initialize the serial connection."""
        try:
            port = self.get_parameter('port').get_parameter_value().string_value
            baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
            timeout = self.get_parameter('timeout').get_parameter_value().double_value
            
            self._serial = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=timeout
            )
            
            self._set_diagnostic_status(
                DiagnosticStatus.OK, 
                f"Serial port {port} open and ready at {baud_rate} baud"
            )
        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Failed to open serial port: {str(e)}"
            )

    def _command_callback(self, msg: String) -> None:
        """
        Process serial command messages.
        
        Format: <command>:<data> where data can be comma-separated values
        Example: "cmd:90,120" - sends values 90 and 120 to the serial device
        
        Args:
            msg: Command message containing command and data
        """
        # Update the last command time
        self._last_command_time = time.time()
        
        try:
            # Parse the command
            parts = msg.data.split(',', 1)
            if len(parts) < 2:
                self._set_diagnostic_status(
                    DiagnosticStatus.WARN, 
                    f"Invalid command format: {msg.data} - should be <command>,<data>"
                )
                return
                
            cmd_str, data_str = parts[0], parts[1]
            
            # Parse data
            if ',' in data_str:
                # Multiple values to send
                try:
                    values = [self._parse_value(v) for v in data_str.split(',')]
                    self._write_serial_block(cmd_str, values)
                except ValueError as e:
                    self._set_diagnostic_status(
                        DiagnosticStatus.WARN, 
                        f"Invalid data format: {str(e)}"
                    )
            else:
                # Single value to send
                try:
                    value = self._parse_value(data_str)
                    self._write_serial_byte(cmd_str, value)
                except ValueError as e:
                    self._set_diagnostic_status(
                        DiagnosticStatus.WARN, 
                        f"Invalid data format: {str(e)}"
                    )
                    
            # Update stats for this command
            if cmd_str not in self._device_stats:
                self._device_stats[cmd_str] = {
                    'write_count': 0,
                    'error_count': 0,
                }
                
            self._device_stats[cmd_str]['write_count'] += 1
            
        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Error processing serial command: {str(e)}"
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
        # Try to parse as int (hex or decimal)
        try:
            return int(value_str, 0)
        except ValueError:
            raise ValueError(f"Cannot parse value: {value_str}")

    def _write_serial_byte(self, cmd_str: str, value: int) -> None:
        """
        Write a single byte to the serial device with command prefix.
        
        Args:
            cmd_str: Command string prefix
            value: Byte value to write
        """
        if self._serial is None:
            self.get_logger().error(f"Could not write to serial port. Would have written byte {value} with command {cmd_str}")
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                "Serial connection not initialized"
            )
            return
            
        try:
            # Format command with value and line ending
            line_ending = self.get_parameter('line_ending').get_parameter_value().string_value
            command = f"{cmd_str},{value}{line_ending}"
            
            # Send as bytes
            self._serial.write(command.encode())
            self.get_logger().debug(f"Wrote byte {value} with command {cmd_str}")
        except Exception as e:
            self._record_error(cmd_str, f"Error writing byte {value} with command {cmd_str}: {str(e)}")

    def _write_serial_block(self, cmd_str: str, values: List[int]) -> None:
        """
        Write a block of data to the serial device.
        
        Args:
            cmd_str: Command string prefix
            values: List of values to write
        """
        if self._serial is None:
            self.get_logger().error(f"Could not write to serial port. Would have written block {values} with command {cmd_str}")
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                "Serial connection not initialized"
            )
            return
            
        try:
            # Format command with values joined by commas and line ending
            line_ending = self.get_parameter('line_ending').get_parameter_value().string_value
            values_str = ",".join(str(v) for v in values)
            command = f"{cmd_str}:{values_str}{line_ending}"
            
            # Send as bytes
            self._serial.write(command.encode())
            self.get_logger().debug(f"Wrote block data {values} with command {cmd_str}")
                
        except Exception as e:
            self._record_error(cmd_str, f"Error writing block {values} with command {cmd_str}: {str(e)}")

    def _record_error(self, cmd_str: str, message: str) -> None:
        """
        Record an error for a specific command.
        
        Args:
            cmd_str: Command where error occurred
            message: Error message
        """
        # Initialize stats for this command if needed
        if cmd_str not in self._device_stats:
            self._device_stats[cmd_str] = {
                'write_count': 0,
                'error_count': 0,
            }
            
        # Update error count
        self._device_stats[cmd_str]['error_count'] += 1
        
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
        
        # Add serial port information
        if self._serial is not None:
            port = self.get_parameter('port').get_parameter_value().string_value
            baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
            self.diag_status.values.append(KeyValue(key='port', value=port))
            self.diag_status.values.append(KeyValue(key='baud_rate', value=str(baud_rate)))
            
            # Add command statistics
            cmd_count = sum(stats['write_count'] for stats in self._device_stats.values())
            error_count = sum(stats['error_count'] for stats in self._device_stats.values())
            
            self.diag_status.values.append(KeyValue(key='total_commands', value=str(cmd_count)))
            self.diag_status.values.append(KeyValue(key='error_count', value=str(error_count)))
            
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
                f"No serial commands received in {time_since_last_cmd:.1f}s"
            )
            
        # Publish diagnostic status
        self.diag_pub.publish(self.diag_status)

    def destroy_node(self) -> None:
        """Clean up resources when node is shutting down."""
        if self._serial is not None:
            try:
                self._serial.close()
                self.get_logger().info("Serial connection closed")
            except Exception as e:
                self.get_logger().error(f"Error closing serial connection: {str(e)}")
                
        super().destroy_node()


def main(args=None):
    """Main entry point for the node."""
    try:
        rclpy.init(args=args)
        node = SerialNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"Error in Serial node: {str(e)}")


if __name__ == '__main__':
    main()