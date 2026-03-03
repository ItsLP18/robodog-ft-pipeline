#!/usr/bin/env python3
"""
FT Mock Sensor Node

Simulates Robotous F/T sensor data for testing the pipeline WITHOUT hardware.
Generates realistic gait-like force patterns for development and debugging.

Simulated patterns:
    - Sinusoidal gait forces (~1Hz walking cadence)
    - Periodic grip force variations
    - Gaussian noise to simulate real sensor behavior

Usage:
    ros2 run ft_sensor_pipeline ft_mock_sensor_node.py

Topics Published:
    /ft_sensor/raw (geometry_msgs/WrenchStamped)
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped


class FTMockSensorNode(Node):
    """Publishes simulated F/T data that mimics co-walking gait patterns."""

    def __init__(self):
        super().__init__('ft_mock_sensor')

        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('frame_id', 'ft_sensor_link')
        self.declare_parameter('noise_level', 0.5)
        self.declare_parameter('gait_frequency_hz', 1.0)  # Walking ~1Hz

        rate = self.get_parameter('publish_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.noise = self.get_parameter('noise_level').value
        self.gait_freq = self.get_parameter('gait_frequency_hz').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(
            WrenchStamped, '/ft_sensor/raw', sensor_qos
        )

        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.t = 0.0
        self.dt = 1.0 / rate

        self.get_logger().info(
            f'Mock FT sensor started | Rate: {rate}Hz | '
            f'Gait freq: {self.gait_freq}Hz | Noise: {self.noise}'
        )

    def _publish(self):
        """Generate and publish simulated F/T data."""
        self.t += self.dt
        w = 2.0 * math.pi * self.gait_freq

        # --- Simulate gait forces ---
        # During walking, the handlebar sees:
        # - Fx (forward/backward): oscillates with gait, ~5-15N
        # - Fy (lateral): smaller oscillation, ~2-5N
        # - Fz (vertical/down): weight support, ~8-20N with gait modulation
        # These approximate an older adult gripping a handlebar while walking

        # Gait phase modulation
        phase = w * self.t
        stride_phase = math.sin(phase)
        stance_phase = math.cos(phase)

        # Simulate occasional increase in grip (e.g., loss of balance)
        balance_event = 3.0 * math.exp(-((self.t % 8.0) - 4.0)**2 / 0.3)

        fx = 8.0 * stride_phase + balance_event + self._noise()
        fy = 3.0 * math.sin(phase * 2.0) + self._noise()  # 2x frequency (bilateral)
        fz = -12.0 + 5.0 * stance_phase + balance_event * 1.5 + self._noise()

        # Torques (smaller, from wrist rotation during gait)
        tx = 0.3 * stride_phase + self._noise() * 0.1
        ty = 0.2 * math.cos(phase + 0.5) + self._noise() * 0.1
        tz = 0.1 * math.sin(phase * 0.5) + self._noise() * 0.05

        # --- Build message ---
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        msg.wrench.torque.x = tx
        msg.wrench.torque.y = ty
        msg.wrench.torque.z = tz

        self.pub.publish(msg)

    def _noise(self) -> float:
        """Generate Gaussian noise."""
        return random.gauss(0, self.noise)


def main(args=None):
    rclpy.init(args=args)
    node = FTMockSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
