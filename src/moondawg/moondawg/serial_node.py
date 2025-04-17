import time
from threading import Lock
from typing import Optional, Callable, Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String, Header
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from serial import Serial, SerialException

class SerialNode(Node):
    """
    ROS2 node for handling serial communication with hardware.
    
    This node creates a bridge between ROS2 topics and serial devices,
    providing diagnostics and automatic reconnection capabilities.
    """

    def __init__(self):
        super().__init__(node_name='serial_node')
        
        # Initialize serial connection properties
        self._serial: Optional[Serial] = None
        self._serial_lock = Lock()  # Thread safety for serial operations
        self._last_message_time = time.time()
        
        # Set up parameters
        self._declare_parameters()
        
        # Set up publishers, subscribers, and timers
        self._setup_communications()
        
        # Attempt initial serial connection
        self._connect_serial()
        
        self.get_logger().info("Serial node initialized")

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters for this node."""
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('write_timeout', 1.0)
        self.declare_parameter('retry_interval', 5.0)
        self.declare_parameter('heartbeat_interval', 1.0)

    def _setup_communications(self) -> None:
        """Set up publishers, subscribers, and timers."""
        # Diagnostic publisher
        self.diagnostic_status = DiagnosticStatus(
            name=self.get_name(), 
            level=DiagnosticStatus.OK,
            message="Initializing serial node"
        )
        self.diag_publisher = self.create_publisher(
            DiagnosticStatus, 
            '/serial_node/diag', 
            10
        )
        
        # Serial message subscriber
        self.subscription = self.create_subscription(
            String, 
            '/serial_node/serial', 
            self._serial_callback, 
            10
        )
        
        # Timers
        retry_interval = self.get_parameter('retry_interval').get_parameter_value().double_value
        self.serial_retry_timer = self.create_timer(retry_interval, self._connect_serial)
        
        heartbeat_interval = self.get_parameter('heartbeat_interval').get_parameter_value().double_value
        self.heartbeat_timer = self.create_timer(heartbeat_interval, self._heartbeat)
        
        # Parameter callback
        self.add_on_set_parameters_callback(self._handle_parameter_change)

    def _handle_parameter_change(self, params: list) -> rclpy.parameter.SetParametersResult:
        """
        Handle parameter changes.
        
        Args:
            params: List of parameters that have changed
            
        Returns:
            Result indicating success or failure
        """
        reconnect_needed = False
        
        try:
            for param in params:
                if param.name == 'port':
                    reconnect_needed = True
                    self.get_logger().info(f"Serial port changed to {param.value}")
                elif param.name == 'baudrate':
                    reconnect_needed = True
                    self.get_logger().info(f"Baudrate changed to {param.value}")
                elif param.name == 'timeout' or param.name == 'write_timeout':
                    reconnect_needed = True
                    self.get_logger().info(f"{param.name.capitalize()} changed to {param.value}")
            
            # If connection parameters changed, attempt reconnection
            if reconnect_needed:
                result = self._connect_serial()
                if not result:
                    self.get_logger().warn("Failed to reconnect with new parameters")
                    return rclpy.parameter.SetParametersResult(
                        successful=False,
                        reason="Failed to connect with new parameters"
                    )
            
            return rclpy.parameter.SetParametersResult(successful=True)
        
        except Exception as e:
            self.get_logger().error(f"Error processing parameter change: {str(e)}")
            return rclpy.parameter.SetParametersResult(
                successful=False,
                reason=f"Exception: {str(e)}"
            )
    
    def _connect_serial(self) -> bool:
        """
        Attempt to connect to the serial device.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            with self._serial_lock:
                # Close existing connection if open
                if self._serial is not None and self._serial.is_open:
                    self._serial.close()
                    self.get_logger().info("Closed existing serial connection")
                
                # Get connection parameters
                port = self.get_parameter('port').get_parameter_value().string_value
                baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
                timeout = self.get_parameter('timeout').get_parameter_value().double_value
                write_timeout = self.get_parameter('write_timeout').get_parameter_value().double_value
                
                # Open new connection
                self._serial = Serial(
                    port=port, 
                    baudrate=baudrate,
                    timeout=timeout,
                    write_timeout=write_timeout
                )
                
                self._set_diagnostic_status(
                    DiagnosticStatus.OK, 
                    f"Connected to {port} at {baudrate} baud"
                )
                
                # Cancel retry timer if connection successful
                if not self.serial_retry_timer.is_canceled():
                    self.serial_retry_timer.cancel()
                
                return True
                
        except SerialException as e:
            with self._serial_lock:
                self._serial = None
            
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Serial connection error: {str(e)}"
            )
            
            # Ensure retry timer is active if connection failed
            if self.serial_retry_timer.is_canceled():
                retry_interval = self.get_parameter('retry_interval').get_parameter_value().double_value
                self.serial_retry_timer.timer_period_ns = int(retry_interval * 1e9)
                self.serial_retry_timer.reset()
            
            return False
            
        except Exception as e:
            with self._serial_lock:
                self._serial = None
            
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Unexpected error during serial connection: {str(e)}"
            )
            return False

    def _serial_callback(self, message: String) -> None:
        """
        Handle messages to be sent over serial.
        
        Args:
            message: String message to send
        """
        # Check if serial connection is available
        if self._serial is None or not self._serial.is_open:
            self.get_logger().warn("Failed to send packet: Serial connection is not established")
            return

        # C needs the newline character appended, using "read_until('\n')"
        string_to_send = message.data + "\n"
        
        try:
            with self._serial_lock:
                self._serial.write(string_to_send.encode())
                self._last_message_time = time.time()
                
            debug_level = rclpy.logging.LoggingSeverity.DEBUG
            if self.get_logger().get_effective_level() <= debug_level:
                self.get_logger().debug(f"Wrote: {message.data}")
                
        except SerialException as e:
            self._set_diagnostic_status(
                DiagnosticStatus.WARN, 
                f"Serial write error: {str(e)}"
            )
            
            # If we lost connection, try to reconnect
            if not self.serial_retry_timer.is_canceled():
                self.serial_retry_timer.reset()
                
        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR, 
                f"Unexpected error during serial write: {str(e)}"
            )

    def _set_diagnostic_status(self, level: int, message: str) -> None:
        """
        Update diagnostic status and log appropriately.
        
        Args:
            level: Diagnostic level
            message: Status message
        """
        self.diagnostic_status.level = level
        self.diagnostic_status.message = message
        
        # Add hardware details to diagnostics
        self.diagnostic_status.values = []
        
        if self._serial is not None and self._serial.is_open:
            port = self.get_parameter('port').get_parameter_value().string_value
            baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
            
            self.diagnostic_status.values.append(KeyValue(key='port', value=port))
            self.diagnostic_status.values.append(KeyValue(key='baudrate', value=str(baudrate)))
            
            last_msg_age = time.time() - self._last_message_time
            self.diagnostic_status.values.append(
                KeyValue(key='last_message_age', value=f"{last_msg_age:.2f}s")
            )
        
        # Log the message based on severity
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(message)
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
    
    def _heartbeat(self) -> None:
        """Publish diagnostic information periodically."""
        self.diag_publisher.publish(self.diagnostic_status)
        
        # Check if connection is active but not sending/receiving data
        if self._serial is not None and self._serial.is_open:
            last_msg_age = time.time() - self._last_message_time
            if last_msg_age > 30.0:  # No activity for 30 seconds
                self.get_logger().warn(f"No serial activity for {last_msg_age:.1f}s")

    def stop(self) -> None:
        """Clean up resources before shutdown."""
        self.get_logger().info("Stopping serial_node")
        
        with self._serial_lock:
            if self._serial is not None and self._serial.is_open:
                try:
                    self._serial.close()
                    self.get_logger().info("Serial connection closed")
                except Exception as e:
                    self.get_logger().error(f"Error closing serial connection: {str(e)}")


def main(args=None):
    """Main entry point for the node."""
    try:
        rclpy.init(args=args)
        serial_node = SerialNode()
        
        try:
            rclpy.spin(serial_node)
        except KeyboardInterrupt:
            pass
        finally:
            serial_node.stop()
            serial_node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"Error in serial_node: {str(e)}")


if __name__ == '__main__':
    main()
