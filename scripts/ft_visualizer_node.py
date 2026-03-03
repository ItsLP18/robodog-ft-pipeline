#!/usr/bin/env python3
"""
FT Visualizer Node

Real-time visualization of force-torque sensor data using matplotlib.
Displays 6-axis forces/torques, magnitudes, and grip state.

Topics Subscribed:
    /ft_sensor/raw (geometry_msgs/WrenchStamped)
    /ft_sensor/filtered (geometry_msgs/WrenchStamped)
    /ft_sensor/force_magnitude (std_msgs/Float64)
    /ft_sensor/torque_magnitude (std_msgs/Float64)
    /ft_sensor/grip_detected (std_msgs/Bool)

Parameters:
    See config/ft_sensor_params.yaml under ft_visualizer
"""

from collections import deque
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64, Bool

import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for interactive display
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


class FTVisualizerNode(Node):
    """Real-time F/T data visualization with matplotlib."""

    def __init__(self):
        super().__init__('ft_visualizer')

        # --- Parameters ---
        self.declare_parameter('plot_window_sec', 10.0)
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('show_raw', True)
        self.declare_parameter('show_filtered', True)

        window_sec = self.get_parameter('plot_window_sec').value
        self.update_rate = self.get_parameter('update_rate_hz').value
        self.show_raw = self.get_parameter('show_raw').value
        self.show_filtered = self.get_parameter('show_filtered').value

        # Buffer size: window_sec * 100Hz
        self.buf_size = int(window_sec * 100)

        # --- Data Buffers ---
        self.time_buf = deque(maxlen=self.buf_size)

        # Raw data buffers
        self.raw_fx = deque(maxlen=self.buf_size)
        self.raw_fy = deque(maxlen=self.buf_size)
        self.raw_fz = deque(maxlen=self.buf_size)
        self.raw_tx = deque(maxlen=self.buf_size)
        self.raw_ty = deque(maxlen=self.buf_size)
        self.raw_tz = deque(maxlen=self.buf_size)

        # Filtered data buffers
        self.filt_fx = deque(maxlen=self.buf_size)
        self.filt_fy = deque(maxlen=self.buf_size)
        self.filt_fz = deque(maxlen=self.buf_size)
        self.filt_tx = deque(maxlen=self.buf_size)
        self.filt_ty = deque(maxlen=self.buf_size)
        self.filt_tz = deque(maxlen=self.buf_size)

        # Magnitude buffers
        self.force_mag_buf = deque(maxlen=self.buf_size)
        self.torque_mag_buf = deque(maxlen=self.buf_size)
        self.grip_buf = deque(maxlen=self.buf_size)

        self._start_time = None
        self._lock = threading.Lock()

        # --- QoS ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

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
            Float64, '/ft_sensor/torque_magnitude',
            self._torque_mag_callback, sensor_qos
        )
        self.create_subscription(
            Bool, '/ft_sensor/grip_detected',
            self._grip_callback, sensor_qos
        )

        self.get_logger().info('FT Visualizer initialized')

    def _get_time(self, stamp):
        """Convert ROS stamp to relative seconds."""
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._start_time is None:
            self._start_time = t
        return t - self._start_time

    def _raw_callback(self, msg):
        with self._lock:
            t = self._get_time(msg.header.stamp)
            self.time_buf.append(t)
            self.raw_fx.append(msg.wrench.force.x)
            self.raw_fy.append(msg.wrench.force.y)
            self.raw_fz.append(msg.wrench.force.z)
            self.raw_tx.append(msg.wrench.torque.x)
            self.raw_ty.append(msg.wrench.torque.y)
            self.raw_tz.append(msg.wrench.torque.z)

    def _filtered_callback(self, msg):
        with self._lock:
            self.filt_fx.append(msg.wrench.force.x)
            self.filt_fy.append(msg.wrench.force.y)
            self.filt_fz.append(msg.wrench.force.z)
            self.filt_tx.append(msg.wrench.torque.x)
            self.filt_ty.append(msg.wrench.torque.y)
            self.filt_tz.append(msg.wrench.torque.z)

    def _force_mag_callback(self, msg):
        with self._lock:
            self.force_mag_buf.append(msg.data)

    def _torque_mag_callback(self, msg):
        with self._lock:
            self.torque_mag_buf.append(msg.data)

    def _grip_callback(self, msg):
        with self._lock:
            self.grip_buf.append(1.0 if msg.data else 0.0)

    def setup_plot(self):
        """Create the matplotlib figure with subplots."""
        self.fig, self.axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
        self.fig.suptitle(
            'Robotous F/T Sensor — RoboDog Co-Walking',
            fontsize=14, fontweight='bold'
        )

        # Color scheme
        colors_raw = ['#e74c3c', '#2ecc71', '#3498db']   # R, G, B for x, y, z
        colors_filt = ['#c0392b', '#27ae60', '#2980b9']   # Darker for filtered

        # --- Subplot 0: Forces ---
        ax = self.axes[0]
        ax.set_ylabel('Force (N)')
        ax.set_title('Forces (Fx, Fy, Fz)', fontsize=10)
        self.lines_raw_f = []
        self.lines_filt_f = []
        for i, (label, color) in enumerate(zip(['Fx', 'Fy', 'Fz'], colors_raw)):
            if self.show_raw:
                line, = ax.plot([], [], color=color, alpha=0.4, linewidth=0.8,
                               label=f'{label} raw')
                self.lines_raw_f.append(line)
            if self.show_filtered:
                line, = ax.plot([], [], color=colors_filt[i], linewidth=1.5,
                               label=f'{label} filt')
                self.lines_filt_f.append(line)
        ax.legend(loc='upper right', fontsize=7, ncol=2)
        ax.set_ylim(-50, 50)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linewidth=0.5)

        # --- Subplot 1: Torques ---
        ax = self.axes[1]
        ax.set_ylabel('Torque (Nm)')
        ax.set_title('Torques (Tx, Ty, Tz)', fontsize=10)
        self.lines_raw_t = []
        self.lines_filt_t = []
        for i, (label, color) in enumerate(zip(['Tx', 'Ty', 'Tz'], colors_raw)):
            if self.show_raw:
                line, = ax.plot([], [], color=color, alpha=0.4, linewidth=0.8,
                               label=f'{label} raw')
                self.lines_raw_t.append(line)
            if self.show_filtered:
                line, = ax.plot([], [], color=colors_filt[i], linewidth=1.5,
                               label=f'{label} filt')
                self.lines_filt_t.append(line)
        ax.legend(loc='upper right', fontsize=7, ncol=2)
        ax.set_ylim(-2, 2)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linewidth=0.5)

        # --- Subplot 2: Magnitudes ---
        ax = self.axes[2]
        ax.set_ylabel('Magnitude')
        ax.set_title('Force & Torque Magnitudes', fontsize=10)
        self.line_force_mag, = ax.plot([], [], color='#e67e22', linewidth=2,
                                        label='|F| (N)')
        self.line_torque_mag, = ax.plot([], [], color='#9b59b6', linewidth=2,
                                         label='|T| (Nm)')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_ylim(0, 60)
        ax.grid(True, alpha=0.3)

        # --- Subplot 3: Grip Detection ---
        ax = self.axes[3]
        ax.set_ylabel('Grip')
        ax.set_xlabel('Time (s)')
        ax.set_title('Grip Detection', fontsize=10)
        self.line_grip, = ax.plot([], [], color='#1abc9c', linewidth=2)
        ax.fill_between([], [], alpha=0.3, color='#1abc9c')
        ax.set_ylim(-0.1, 1.1)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(['No Grip', 'Gripping'])
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        return self.fig

    def update_plot(self, frame):
        """Animation update function."""
        with self._lock:
            t = list(self.time_buf)
            n = len(t)
            if n < 2:
                return []

            updated = []

            # --- Forces ---
            if self.show_raw and len(self.raw_fx) == n:
                raw_data = [list(self.raw_fx), list(self.raw_fy), list(self.raw_fz)]
                for i, line in enumerate(self.lines_raw_f):
                    line.set_data(t, raw_data[i])
                    updated.append(line)

            if self.show_filtered and len(self.filt_fx) >= n:
                filt_data = [list(self.filt_fx)[-n:], list(self.filt_fy)[-n:],
                            list(self.filt_fz)[-n:]]
                for i, line in enumerate(self.lines_filt_f):
                    line.set_data(t, filt_data[i])
                    updated.append(line)

            # --- Torques ---
            if self.show_raw and len(self.raw_tx) == n:
                raw_data = [list(self.raw_tx), list(self.raw_ty), list(self.raw_tz)]
                for i, line in enumerate(self.lines_raw_t):
                    line.set_data(t, raw_data[i])
                    updated.append(line)

            if self.show_filtered and len(self.filt_tx) >= n:
                filt_data = [list(self.filt_tx)[-n:], list(self.filt_ty)[-n:],
                            list(self.filt_tz)[-n:]]
                for i, line in enumerate(self.lines_filt_t):
                    line.set_data(t, filt_data[i])
                    updated.append(line)

            # --- Magnitudes ---
            if len(self.force_mag_buf) >= n:
                fm = list(self.force_mag_buf)[-n:]
                self.line_force_mag.set_data(t, fm)
                updated.append(self.line_force_mag)

            if len(self.torque_mag_buf) >= n:
                tm = list(self.torque_mag_buf)[-n:]
                self.line_torque_mag.set_data(t, tm)
                updated.append(self.line_torque_mag)

            # --- Grip ---
            if len(self.grip_buf) >= n:
                g = list(self.grip_buf)[-n:]
                self.line_grip.set_data(t, g)
                updated.append(self.line_grip)

            # Update x-axis limits
            if t:
                xmin = max(0, t[-1] - 10.0)
                xmax = t[-1] + 0.5
                for ax in self.axes:
                    ax.set_xlim(xmin, xmax)

        return updated


def main(args=None):
    rclpy.init(args=args)
    node = FTVisualizerNode()

    # Setup matplotlib
    fig = node.setup_plot()

    # Create animation
    ani = animation.FuncAnimation(
        fig,
        node.update_plot,
        interval=int(1000 / node.update_rate),  # ms
        blit=False,
        cache_frame_data=False
    )

    # Run ROS2 spinning in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run matplotlib main loop (must be in main thread)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
