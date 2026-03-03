#!/usr/bin/env python3
"""
FT Sensor Driver Node

Reads data from the Robotous 6-axis force-torque sensor over serial
and publishes geometry_msgs/WrenchStamped messages at ~100Hz.

Topics Published:
    /ft_sensor/raw (geometry_msgs/WrenchStamped) - Raw sensor readings
    /ft_sensor/status (std_msgs/String) - Sensor status/diagnostics

Parameters:
    See config/ft_sensor_params.yaml under ft_sensor_driver
"""

import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

# Add parent package to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ft_sensor_pipeline'))
try:
    from ft_sensor_pipeline.robotous_protocol import RobotousProtocol, FTReading
except ImportError:
    # Fallback for installed package
    from robotous_protocol import RobotousProtocol, FTReading


class FTSensorDriverNode(Node):
    """ROS2 node for reading Robotous F/T sensor data."""

    def __init__(self):
        super().__init__('ft_sensor_driver')

        # --- Declare Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout_sec', 0.01)
        self.declare_parameter('publish_rate_hz', 200.0)
        self.declare_parameter('frame_id', 'ft_sensor_link')
        self.declare_parameter('sensor_model', 'RFT40-SA01')
        self.declare_parameter('bias_fx', 0.0)
        self.declare_parameter('bias_fy', 0.0)
        self.declare_parameter('bias_fz', 0.0)
        self.declare_parameter('bias_tx', 0.0)
        self.declare_parameter('bias_ty', 0.0)
        self.declare_parameter('bias_tz', 0.0)

        # --- Get Parameters ---
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.sensor_model = self.get_parameter('sensor_model').value

        self.bias = {
            'fx': self.get_parameter('bias_fx').value,
            'fy': self.get_parameter('bias_fy').value,
            'fz': self.get_parameter('bias_fz').value,
            'tx': self.get_parameter('bias_tx').value,
            'ty': self.get_parameter('bias_ty').value,
            'tz': self.get_parameter('bias_tz').value,
        }

        # --- QoS Profile for sensor data (best effort for real-time) ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Publishers ---
        self.wrench_pub = self.create_publisher(
            WrenchStamped, '/ft_sensor/raw', sensor_qos
        )
        self.status_pub = self.create_publisher(
            String, '/ft_sensor/status', 10
        )

        # --- Initialize Sensor ---
        self.protocol = RobotousProtocol(
            port=self.serial_port,
            baud_rate=self.baud_rate,
            timeout=self.timeout,
            sensor_model=self.sensor_model
        )

        self.connected = False
        self._connect_sensor()

        # --- Timer for reading sensor at specified rate ---
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self._timer_callback)

        # --- Diagnostics counters ---
        self.msg_count = 0
        self.error_count = 0
        self.overload_count = 0

        # Status timer (1Hz)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'FT Sensor Driver initialized | Port: {self.serial_port} | '
            f'Rate: {self.publish_rate}Hz | Frame: {self.frame_id}'
        )

    def _connect_sensor(self):
        """Attempt to connect to the sensor."""
        self.get_logger().info(f'Connecting to sensor on {self.serial_port}...')
        self.connected = self.protocol.connect()

        if self.connected:
            self.get_logger().info('Sensor connected. Starting streaming...')
            self.protocol.start_streaming()
        else:
            self.get_logger().error(
                f'Failed to connect to sensor on {self.serial_port}. '
                f'Check connection and permissions (sudo chmod 666 {self.serial_port})'
            )

    def _apply_bias(self, reading: FTReading) -> FTReading:
        """Subtract bias offsets from raw reading."""
        reading.fx -= self.bias['fx']
        reading.fy -= self.bias['fy']
        reading.fz -= self.bias['fz']
        reading.tx -= self.bias['tx']
        reading.ty -= self.bias['ty']
        reading.tz -= self.bias['tz']
        return reading

    def _timer_callback(self):
        """Read sensor data and publish."""
        if not self.connected:
            # Attempt reconnection every call (throttled by timer)
            if self.error_count % 500 == 0:
                self._connect_sensor()
            self.error_count += 1
            return

        reading = self.protocol.read_one()
        if reading is None:
            return

        # Apply bias correction
        reading = self._apply_bias(reading)

        # Check for overload
        overloads = self.protocol.check_overload(reading)
        if overloads:
            self.overload_count += 1
            if self.overload_count % 100 == 1:  # Don't spam logs
                self.get_logger().warn(
                    f'Sensor near overload on axes: {list(overloads.keys())}'
                )

        # Build and publish WrenchStamped message
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.wrench.force.x = reading.fx
        msg.wrench.force.y = reading.fy
        msg.wrench.force.z = reading.fz
        msg.wrench.torque.x = reading.tx
        msg.wrench.torque.y = reading.ty
        msg.wrench.torque.z = reading.tz

        self.wrench_pub.publish(msg)
        self.msg_count += 1

    def _publish_status(self):
        """Publish diagnostic status at 1Hz."""
        status = String()
        status.data = (
            f'{{"connected": {str(self.connected).lower()}, '
            f'"messages": {self.msg_count}, '
            f'"errors": {self.error_count}, '
            f'"overloads": {self.overload_count}, '
            f'"rate_hz": {self.publish_rate}}}'
        )
        self.status_pub.publish(status)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.get_logger().info('Shutting down FT sensor driver...')
        self.protocol.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FTSensorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
