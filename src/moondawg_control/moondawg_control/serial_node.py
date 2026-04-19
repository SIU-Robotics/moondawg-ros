import time
from typing import Optional, Dict, Any, List
import rclpy
from rclpy.node import Node
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
        self._last_attempt_time = 0.0

        # Stats
        self._device_stats: Dict[str, Dict[str, Any]] = {}
        self._last_command_time = time.time()

        # Communications
        self._setup_communications()

        # Reconnect timer
        self.create_timer(1.0, self._reconnect_loop)

        self.get_logger().info("Serial node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'port',
            '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
        )
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('line_ending', '\n')
        self.declare_parameter('heartbeat_interval', 1.0)
        self.declare_parameter('command_timeout', 5.0)
        self.declare_parameter('reconnect_interval', 2.0)

    def _setup_communications(self) -> None:
        self.diag_status = DiagnosticStatus(
            name=self.get_name(),
            level=DiagnosticStatus.ERROR,
            message="Serial not connected"
        )

        self.diag_pub = self.create_publisher(
            DiagnosticStatus,
            '/serial_node/diag',
            10
        )

        self.create_subscription(
            String,
            '/serial_node/command',
            self._command_callback,
            10
        )

        heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.create_timer(heartbeat_interval, self._heartbeat)

    def _connect(self) -> bool:
        try:
            port = self.get_parameter('port').value
            baud_rate = self.get_parameter('baud_rate').value
            timeout = self.get_parameter('timeout').value

            self._serial = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=timeout
            )

            self._set_diagnostic_status(
                DiagnosticStatus.OK,
                f"Connected to {port} @ {baud_rate}"
            )

            return True

        except Exception as e:
            self._serial = None
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR,
                f"Connection failed: {str(e)}"
            )
            return False

    def _reconnect_loop(self):
        if self._serial is not None and self._serial.is_open:
            return

        now = time.time()
        interval = self.get_parameter('reconnect_interval').value

        if now - self._last_attempt_time < interval:
            return

        self._last_attempt_time = now
        self.get_logger().warn("Attempting serial reconnect...")
        self._connect()

    def _command_callback(self, msg: String) -> None:
        self._last_command_time = time.time()

        try:
            parts = msg.data.split(',', 1)
            if len(parts) < 2:
                self._set_diagnostic_status(
                    DiagnosticStatus.WARN,
                    f"Invalid command format: {msg.data}"
                )
                return

            cmd_str, data_str = parts[0], parts[1]

            if ',' in data_str:
                values = [self._parse_value(v) for v in data_str.split(',')]
                self._write_serial_block(cmd_str, values)
            else:
                value = self._parse_value(data_str)
                self._write_serial_byte(cmd_str, value)

            if cmd_str not in self._device_stats:
                self._device_stats[cmd_str] = {
                    'write_count': 0,
                    'error_count': 0,
                }

            self._device_stats[cmd_str]['write_count'] += 1

        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR,
                f"Command processing error: {str(e)}"
            )

    def _parse_value(self, value_str: str) -> int:
        return int(value_str, 0)

    def _write_serial_byte(self, cmd_str: str, value: int) -> None:
        if self._serial is None or not self._serial.is_open:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR,
                "Serial not connected"
            )
            return

        try:
            line_ending = self.get_parameter('line_ending').value
            command = f"{cmd_str},{value}{line_ending}"
            self._serial.write(command.encode())

        except Exception as e:
            self._record_error(cmd_str, f"Write failed: {str(e)}")
            self._serial = None  # trigger reconnect

    def _write_serial_block(self, cmd_str: str, values: List[int]) -> None:
        if self._serial is None or not self._serial.is_open:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR,
                "Serial not connected"
            )
            return

        try:
            line_ending = self.get_parameter('line_ending').value
            values_str = ",".join(str(v) for v in values)
            command = f"{cmd_str}:{values_str}{line_ending}"
            self._serial.write(command.encode())

        except Exception as e:
            self._record_error(cmd_str, f"Write failed: {str(e)}")
            self._serial = None  # trigger reconnect

    def _record_error(self, cmd_str: str, message: str) -> None:
        if cmd_str not in self._device_stats:
            self._device_stats[cmd_str] = {
                'write_count': 0,
                'error_count': 0,
            }

        self._device_stats[cmd_str]['error_count'] += 1

        self._set_diagnostic_status(DiagnosticStatus.ERROR, message)

    def _set_diagnostic_status(self, level: int, message: str) -> None:
        self.diag_status.level = level
        self.diag_status.message = message
        self.diag_status.values = []

        if self._serial is not None:
            port = self.get_parameter('port').value
            baud_rate = self.get_parameter('baud_rate').value

            self.diag_status.values.append(KeyValue(key='port', value=port))
            self.diag_status.values.append(KeyValue(key='baud_rate', value=str(baud_rate)))

            cmd_count = sum(s['write_count'] for s in self._device_stats.values())
            err_count = sum(s['error_count'] for s in self._device_stats.values())

            self.diag_status.values.append(KeyValue(key='total_commands', value=str(cmd_count)))
            self.diag_status.values.append(KeyValue(key='error_count', value=str(err_count)))

        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(message)
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _heartbeat(self):
        self.diag_pub.publish(self.diag_status)

    def destroy_node(self) -> None:
        """Clean up resources when node is shutting down."""
        if self._serial is not None:
            try:
                self._serial.close()
                self.get_logger().info("Serial closed")
            except Exception as e:
                self.get_logger().error(f"Close error: {str(e)}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()