#!/usr/bin/env python3
"""
FT Data Processor Node

Subscribes to raw F/T sensor data, applies filtering, and computes
derived metrics for force analysis.

Topics Subscribed:
    /ft_sensor/raw (geometry_msgs/WrenchStamped)

Topics Published:
    /ft_sensor/filtered (geometry_msgs/WrenchStamped) - Filtered F/T data
    /ft_sensor/force_magnitude (std_msgs/Float64) - Force vector magnitude
    /ft_sensor/torque_magnitude (std_msgs/Float64) - Torque vector magnitude
    /ft_sensor/grip_detected (std_msgs/Bool) - Force exceeds threshold

Parameters:
    See config/ft_sensor_params.yaml under ft_data_processor
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64, Bool

import numpy as np


class ButterworthFilter:
    """
    Real-time 2nd-order Butterworth low-pass filter.
    Implements a biquad (second-order section) IIR filter.
    """

    def __init__(self, cutoff_hz: float, sample_rate_hz: float, order: int = 2):
        """
        Initialize Butterworth filter.

        Args:
            cutoff_hz: Cutoff frequency in Hz
            sample_rate_hz: Sampling rate in Hz
            order: Filter order (only 2 supported for real-time biquad)
        """
        self.cutoff = cutoff_hz
        self.fs = sample_rate_hz

        # Pre-warp analog frequency
        wc = math.tan(math.pi * cutoff_hz / sample_rate_hz)
        wc2 = wc * wc

        # 2nd-order Butterworth coefficients
        k = math.sqrt(2.0) * wc
        norm = 1.0 / (1.0 + k + wc2)

        self.b0 = wc2 * norm
        self.b1 = 2.0 * self.b0
        self.b2 = self.b0
        self.a1 = 2.0 * (wc2 - 1.0) * norm
        self.a2 = (1.0 - k + wc2) * norm

        # State variables (for 6 channels)
        self.x1 = {}  # x[n-1]
        self.x2 = {}  # x[n-2]
        self.y1 = {}  # y[n-1]
        self.y2 = {}  # y[n-2]

    def filter(self, channel: str, value: float) -> float:
        """Apply filter to one sample of a named channel."""
        if channel not in self.x1:
            # Initialize state
            self.x1[channel] = value
            self.x2[channel] = value
            self.y1[channel] = value
            self.y2[channel] = value
            return value

        # Direct Form II transposed
        y = (self.b0 * value +
             self.b1 * self.x1[channel] +
             self.b2 * self.x2[channel] -
             self.a1 * self.y1[channel] -
             self.a2 * self.y2[channel])

        # Update state
        self.x2[channel] = self.x1[channel]
        self.x1[channel] = value
        self.y2[channel] = self.y1[channel]
        self.y1[channel] = y

        return y


class FTDataProcessorNode(Node):
    """Processes raw F/T data: filtering, magnitude, threshold detection."""

    def __init__(self):
        super().__init__('ft_data_processor')

        # --- Declare Parameters ---
        self.declare_parameter('filter_enabled', True)
        self.declare_parameter('filter_order', 2)
        self.declare_parameter('filter_cutoff_hz', 10.0)
        self.declare_parameter('sample_rate_hz', 100.0)
        self.declare_parameter('force_magnitude_threshold_n', 2.0)
        self.declare_parameter('torque_magnitude_threshold_nm', 0.1)
        self.declare_parameter('moving_avg_window', 5)

        # --- Get Parameters ---
        self.filter_enabled = self.get_parameter('filter_enabled').value
        filter_order = self.get_parameter('filter_order').value
        cutoff = self.get_parameter('filter_cutoff_hz').value
        sample_rate = self.get_parameter('sample_rate_hz').value
        self.force_threshold = self.get_parameter('force_magnitude_threshold_n').value
        self.torque_threshold = self.get_parameter('torque_magnitude_threshold_nm').value
        avg_window = self.get_parameter('moving_avg_window').value

        # --- Initialize Filter ---
        if self.filter_enabled:
            self.lpf = ButterworthFilter(cutoff, sample_rate, filter_order)
            self.get_logger().info(
                f'Butterworth filter enabled: order={filter_order}, '
                f'cutoff={cutoff}Hz, fs={sample_rate}Hz'
            )

        # --- Moving average buffers ---
        self.force_mag_buffer = deque(maxlen=avg_window)
        self.torque_mag_buffer = deque(maxlen=avg_window)

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Subscriber ---
        self.raw_sub = self.create_subscription(
            WrenchStamped, '/ft_sensor/raw', self._raw_callback, sensor_qos
        )

        # --- Publishers ---
        self.filtered_pub = self.create_publisher(
            WrenchStamped, '/ft_sensor/filtered', sensor_qos
        )
        self.force_mag_pub = self.create_publisher(
            Float64, '/ft_sensor/force_magnitude', sensor_qos
        )
        self.torque_mag_pub = self.create_publisher(
            Float64, '/ft_sensor/torque_magnitude', sensor_qos
        )
        self.grip_pub = self.create_publisher(
            Bool, '/ft_sensor/grip_detected', sensor_qos
        )

        self.get_logger().info('FT Data Processor initialized')

    def _raw_callback(self, msg: WrenchStamped):
        """Process incoming raw F/T data."""
        # Extract raw values
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z

        # Apply Butterworth filter
        if self.filter_enabled:
            fx = self.lpf.filter('fx', fx)
            fy = self.lpf.filter('fy', fy)
            fz = self.lpf.filter('fz', fz)
            tx = self.lpf.filter('tx', tx)
            ty = self.lpf.filter('ty', ty)
            tz = self.lpf.filter('tz', tz)

        # Publish filtered wrench
        filtered_msg = WrenchStamped()
        filtered_msg.header = msg.header
        filtered_msg.wrench.force.x = fx
        filtered_msg.wrench.force.y = fy
        filtered_msg.wrench.force.z = fz
        filtered_msg.wrench.torque.x = tx
        filtered_msg.wrench.torque.y = ty
        filtered_msg.wrench.torque.z = tz
        self.filtered_pub.publish(filtered_msg)

        # Compute and publish force magnitude
        force_mag = math.sqrt(fx**2 + fy**2 + fz**2)
        self.force_mag_buffer.append(force_mag)
        avg_force = sum(self.force_mag_buffer) / len(self.force_mag_buffer)

        force_msg = Float64()
        force_msg.data = avg_force
        self.force_mag_pub.publish(force_msg)

        # Compute and publish torque magnitude
        torque_mag = math.sqrt(tx**2 + ty**2 + tz**2)
        self.torque_mag_buffer.append(torque_mag)
        avg_torque = sum(self.torque_mag_buffer) / len(self.torque_mag_buffer)

        torque_msg = Float64()
        torque_msg.data = avg_torque
        self.torque_mag_pub.publish(torque_msg)

        # Threshold detection
        grip_msg = Bool()
        grip_msg.data = (avg_force > self.force_threshold or
                         avg_torque > self.torque_threshold)
        self.grip_pub.publish(grip_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FTDataProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
