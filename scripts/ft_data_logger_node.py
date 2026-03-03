#!/usr/bin/env python3
"""
FT Data Logger Node

Logs raw and filtered F/T sensor data to CSV files with session metadata.
Also provides rosbag2 recording coordination via service calls.

Topics Subscribed:
    /ft_sensor/raw (geometry_msgs/WrenchStamped)
    /ft_sensor/filtered (geometry_msgs/WrenchStamped)
    /ft_sensor/force_magnitude (std_msgs/Float64)
    /ft_sensor/grip_detected (std_msgs/Bool)

Output:
    CSV files: ~/robodog_data/csv/ft_session_YYYYMMDD_HHMMSS.csv
    
Parameters:
    See config/ft_sensor_params.yaml under ft_data_logger
"""

import os
import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64, Bool, String


class FTDataLoggerNode(Node):
    """Logs F/T sensor data to CSV with session metadata."""

    def __init__(self):
        super().__init__('ft_data_logger')

        # --- Declare Parameters ---
        self.declare_parameter('csv_enabled', True)
        self.declare_parameter('csv_output_dir', '~/robodog_data/csv')
        self.declare_parameter('csv_prefix', 'ft_session')
        self.declare_parameter('rosbag_enabled', True)
        self.declare_parameter('rosbag_output_dir', '~/robodog_data/rosbags')
        self.declare_parameter('participant_id', 'P000')
        self.declare_parameter('trial_id', 'T00')
        self.declare_parameter('condition', 'with_robot')

        # --- Get Parameters ---
        self.csv_enabled = self.get_parameter('csv_enabled').value
        csv_dir = self.get_parameter('csv_output_dir').value
        csv_prefix = self.get_parameter('csv_prefix').value
        self.participant_id = self.get_parameter('participant_id').value
        self.trial_id = self.get_parameter('trial_id').value
        self.condition = self.get_parameter('condition').value

        # --- Setup CSV output ---
        self.csv_file = None
        self.csv_writer = None
        self.is_logging = False
        self.sample_count = 0

        if self.csv_enabled:
            csv_path = Path(os.path.expanduser(csv_dir))
            csv_path.mkdir(parents=True, exist_ok=True)

            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'{csv_prefix}_{self.participant_id}_{self.trial_id}_{timestamp}.csv'
            filepath = csv_path / filename

            self.csv_file = open(filepath, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # Write header
            self.csv_writer.writerow([
                'timestamp_sec',
                'timestamp_nsec',
                'raw_fx', 'raw_fy', 'raw_fz',
                'raw_tx', 'raw_ty', 'raw_tz',
                'filt_fx', 'filt_fy', 'filt_fz',
                'filt_tx', 'filt_ty', 'filt_tz',
                'force_magnitude',
                'grip_detected',
                'participant_id', 'trial_id', 'condition'
            ])

            self.is_logging = True
            self.get_logger().info(f'CSV logging to: {filepath}')

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- State for syncing data ---
        self._latest_raw = None
        self._latest_filtered = None
        self._latest_force_mag = 0.0
        self._latest_grip = False

        # --- Subscribers ---
        self.create_subscription(
            WrenchStamped, '/ft_sensor/raw',
            self._raw_callback, sensor_qos
        )
        self.create_subscription(
            WrenchStamped, '/ft_sensor/filtered',
            self._filtered_callback, sensor_qos
        )
        self.create_subscription(
            Float64, '/ft_sensor/force_magnitude',
            self._force_mag_callback, sensor_qos
        )
        self.create_subscription(
            Bool, '/ft_sensor/grip_detected',
            self._grip_callback, sensor_qos
        )

        # --- Status publisher ---
        self.status_pub = self.create_publisher(String, '/ft_logger/status', 10)
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f'FT Data Logger initialized | Participant: {self.participant_id} | '
            f'Trial: {self.trial_id} | Condition: {self.condition}'
        )

    def _raw_callback(self, msg: WrenchStamped):
        """Store latest raw data and trigger CSV write."""
        self._latest_raw = msg
        self._write_csv_row()

    def _filtered_callback(self, msg: WrenchStamped):
        """Store latest filtered data."""
        self._latest_filtered = msg

    def _force_mag_callback(self, msg: Float64):
        """Store latest force magnitude."""
        self._latest_force_mag = msg.data

    def _grip_callback(self, msg: Bool):
        """Store latest grip detection."""
        self._latest_grip = msg.data

    def _write_csv_row(self):
        """Write a synchronized row of data to CSV."""
        if not self.is_logging or self._latest_raw is None:
            return

        raw = self._latest_raw
        filt = self._latest_filtered

        row = [
            raw.header.stamp.sec,
            raw.header.stamp.nanosec,
            raw.wrench.force.x,
            raw.wrench.force.y,
            raw.wrench.force.z,
            raw.wrench.torque.x,
            raw.wrench.torque.y,
            raw.wrench.torque.z,
        ]

        # Add filtered data (or zeros if not yet available)
        if filt:
            row.extend([
                filt.wrench.force.x,
                filt.wrench.force.y,
                filt.wrench.force.z,
                filt.wrench.torque.x,
                filt.wrench.torque.y,
                filt.wrench.torque.z,
            ])
        else:
            row.extend([0.0] * 6)

        row.extend([
            self._latest_force_mag,
            int(self._latest_grip),
            self.participant_id,
            self.trial_id,
            self.condition,
        ])

        self.csv_writer.writerow(row)
        self.sample_count += 1

        # Flush periodically (every 100 samples = ~1 sec at 100Hz)
        if self.sample_count % 100 == 0:
            self.csv_file.flush()

    def _publish_status(self):
        """Publish logger status."""
        status = String()
        status.data = (
            f'{{"logging": {str(self.is_logging).lower()}, '
            f'"samples": {self.sample_count}, '
            f'"participant": "{self.participant_id}", '
            f'"trial": "{self.trial_id}"}}'
        )
        self.status_pub.publish(status)

    def destroy_node(self):
        """Clean up CSV file on shutdown."""
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.get_logger().info(
                f'CSV file closed. Total samples: {self.sample_count}'
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FTDataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
