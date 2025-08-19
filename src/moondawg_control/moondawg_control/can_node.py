import time
from typing import Optional, Dict, Any, List
import rclpy
from rclpy.lifecycle import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import can

class CanNode(Node):
    """
    Node for handling CAN communication with the hardware devices.

    This node provides a ROS interface to a CAN bus device on the robot,
    handling command formatting, error detection, and diagnostics.
    """

    def __init__(self):
        super().__init__('can_node')
        self._declare_parameters()
        self._bus: Optional[can.Bus] = None
        self._device_stats: Dict[int, Dict[str, Any]] = {}
        self._last_command_time = time.time()
        self._setup_communications()
        self._initialize_can_bus()
        self.get_logger().info(f"CAN node initialized on interface {self.get_parameter('interface').value}")

    def _declare_parameters(self) -> None:
        self.declare_parameter('interface', 'can0')
        self.declare_parameter('bustype', 'socketcan')
        self.declare_parameter('heartbeat_interval', 1.0)
        self.declare_parameter('command_timeout', 5.0)

    def _setup_communications(self) -> None:
        self.diag_status = DiagnosticStatus(
            name=self.get_name(),
            level=DiagnosticStatus.OK,
            message="Initializing CAN node"
        )
        self.diag_pub = self.create_publisher(
            DiagnosticStatus,
            '/can_node/diag',
            10
        )
        self.create_subscription(
            String,
            '/can_node/command',
            self._command_callback,
            10
        )
        heartbeat_interval = self.get_parameter('heartbeat_interval').get_parameter_value().double_value
        self.create_timer(heartbeat_interval, self._heartbeat)

    def _initialize_can_bus(self) -> None:
        try:
            interface = self.get_parameter('interface').get_parameter_value().string_value
            bustype = self.get_parameter('bustype').get_parameter_value().string_value
            self._bus = can.Bus(interface=interface, bustype=bustype)
            self._set_diagnostic_status(
                DiagnosticStatus.OK,
                f"CAN interface {interface} open and ready"
            )
        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR,
                f"Failed to open CAN interface: {str(e)}"
            )

    def _command_callback(self, msg: String) -> None:
        """
        Process CAN command messages.

        Format: <can_id>:<data> where data can be comma-separated bytes
        Example: "0x123:01,02,03,04"
        """
        self._last_command_time = time.time()
        try:
            parts = msg.data.split(':', 1)
            if len(parts) < 2:
                self._set_diagnostic_status(
                    DiagnosticStatus.WARN,
                    f"Invalid command format: {msg.data} - should be <can_id>:<data>"
                )
                return
            can_id_str, data_str = parts[0], parts[1]
            try:
                can_id = int(can_id_str, 0)
            except ValueError:
                self._set_diagnostic_status(
                    DiagnosticStatus.WARN,
                    f"Invalid CAN ID format: {can_id_str}"
                )
                return
            try:
                data = [self._parse_value(v) for v in data_str.split(',') if v.strip() != '']
                if not (0 <= len(data) <= 8):
                    raise ValueError("CAN data must be 0-8 bytes")
                self._write_can_frame(can_id, data)
            except ValueError as e:
                self._set_diagnostic_status(
                    DiagnosticStatus.WARN,
                    f"Invalid data format: {str(e)}"
                )
            if can_id not in self._device_stats:
                self._device_stats[can_id] = {
                    'write_count': 0,
                    'error_count': 0,
                }
            self._device_stats[can_id]['write_count'] += 1
        except Exception as e:
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR,
                f"Error processing CAN command: {str(e)}"
            )

    def _parse_value(self, value_str: str) -> int:
        try:
            return int(value_str, 0)
        except ValueError:
            raise ValueError(f"Cannot parse value: {value_str}")

    def _write_can_frame(self, can_id: int, data: List[int]) -> None:
        if self._bus is None:
            self.get_logger().error(f"Could not write to CAN bus. Would have sent {data} to id {hex(can_id)}")
            self._set_diagnostic_status(
                DiagnosticStatus.ERROR,
                "CAN bus not initialized"
            )
            return
        try:
            msg = can.Message(arbitration_id=can_id, data=bytearray(data), is_extended_id=False)
            self._bus.send(msg)
            self.get_logger().debug(f"Wrote CAN frame {data} to id {hex(can_id)}")
        except Exception as e:
            self._record_error(can_id, f"Error writing CAN frame {data} to {hex(can_id)}: {str(e)}")

    def _record_error(self, can_id: int, message: str) -> None:
        if can_id not in self._device_stats:
            self._device_stats[can_id] = {
                'write_count': 0,
                'error_count': 0,
            }
        self._device_stats[can_id]['error_count'] += 1
        self._set_diagnostic_status(
            DiagnosticStatus.ERROR,
            message
        )

    def _set_diagnostic_status(self, level: int, message: str) -> None:
        self.diag_status.level = level
        self.diag_status.message = message
        self.diag_status.values = []
        if self._bus is not None:
            interface = self.get_parameter('interface').get_parameter_value().string_value
            self.diag_status.values.append(KeyValue(key='interface', value=interface))
            cmd_count = sum(stats['write_count'] for stats in self._device_stats.values())
            error_count = sum(stats['error_count'] for stats in self._device_stats.values())
            self.diag_status.values.append(KeyValue(key='total_commands', value=str(cmd_count)))
            self.diag_status.values.append(KeyValue(key='error_count', value=str(error_count)))
        if level == DiagnosticStatus.ERROR:
            self.get_logger().error(message)
        elif level == DiagnosticStatus.WARN:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _heartbeat(self) -> None:
        time_since_last_cmd = time.time() - self._last_command_time
        command_timeout = self.get_parameter('command_timeout').get_parameter_value().double_value
        if time_since_last_cmd > command_timeout and len(self._device_stats) > 0:
            self.get_logger().debug(
                f"No CAN commands received in {time_since_last_cmd:.1f}s"
            )
        self.diag_pub.publish(self.diag_status)

    def destroy_node(self) -> None:
        if self._bus is not None:
            try:
                self._bus.shutdown()
                self.get_logger().info("CAN bus closed")
            except Exception as e:
                self.get_logger().error(f"Error closing CAN bus: {str(e)}")
        super().destroy_node()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CanNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"Error in CAN node: {str(e)}")

if __name__ == '__main__':
    main()


