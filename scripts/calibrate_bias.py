#!/usr/bin/env python3
"""
FT Sensor Bias Calibration Script

Run this with NO LOAD on the sensor to determine bias offsets.
Collects N samples and computes the mean for each axis.
Updates the config YAML with the calibrated bias values.

Usage:
    python3 calibrate_bias.py --port /dev/ttyUSB0 --samples 500
"""

import argparse
import sys
import os
import time
import yaml
import numpy as np

# Add parent to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ft_sensor_pipeline'))
from ft_sensor_pipeline.robotous_protocol import RobotousProtocol


def calibrate(port: str, baud: int, num_samples: int, config_path: str = None):
    """Collect samples and compute bias."""
    print("=" * 60)
    print("  Robotous F/T Sensor Bias Calibration")
    print("  Project: RoboDog Co-Walking")
    print("=" * 60)
    print()
    print("⚠️  IMPORTANT: Remove all loads from the sensor!")
    print("    The sensor should be in its mounted position")
    print("    with ONLY gravity acting on it.")
    print()
    input("Press ENTER when ready to begin calibration...")
    print()

    # Connect
    protocol = RobotousProtocol(port=port, baud_rate=baud)
    if not protocol.connect():
        print(f"❌ Failed to connect on {port}")
        sys.exit(1)

    print(f"✅ Connected to sensor on {port}")
    protocol.start_streaming()
    time.sleep(0.5)  # Let sensor stabilize

    # Collect samples
    data = {'fx': [], 'fy': [], 'fz': [], 'tx': [], 'ty': [], 'tz': []}
    collected = 0
    print(f"Collecting {num_samples} samples...")

    while collected < num_samples:
        reading = protocol.read_one()
        if reading:
            data['fx'].append(reading.fx)
            data['fy'].append(reading.fy)
            data['fz'].append(reading.fz)
            data['tx'].append(reading.tx)
            data['ty'].append(reading.ty)
            data['tz'].append(reading.tz)
            collected += 1

            if collected % 100 == 0:
                print(f"  {collected}/{num_samples} samples collected...")

    protocol.stop_streaming()
    protocol.disconnect()

    # Compute statistics
    print()
    print("=" * 60)
    print("  CALIBRATION RESULTS")
    print("=" * 60)
    print()
    print(f"{'Axis':<8} {'Mean (Bias)':<14} {'Std Dev':<12} {'Min':<12} {'Max':<12}")
    print("-" * 58)

    biases = {}
    for axis in ['fx', 'fy', 'fz', 'tx', 'ty', 'tz']:
        arr = np.array(data[axis])
        mean = np.mean(arr)
        std = np.std(arr)
        biases[axis] = float(mean)
        unit = 'N' if axis.startswith('f') else 'Nm'
        print(f"{axis:<8} {mean:>10.4f} {unit}  {std:>8.4f}     {arr.min():>8.4f}     {arr.max():>8.4f}")

    print()
    print("Noise levels (std dev) should be small relative to expected signals.")
    print("For gait analysis, expect grip forces of 5-50N.")
    print()

    # Output YAML snippet
    print("Add these to your ft_sensor_params.yaml:")
    print()
    print("    bias_fx: {:.6f}".format(biases['fx']))
    print("    bias_fy: {:.6f}".format(biases['fy']))
    print("    bias_fz: {:.6f}".format(biases['fz']))
    print("    bias_tx: {:.6f}".format(biases['tx']))
    print("    bias_ty: {:.6f}".format(biases['ty']))
    print("    bias_tz: {:.6f}".format(biases['tz']))
    print()

    # Optionally update config file
    if config_path:
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            params = config.get('ft_sensor_driver', {}).get('ros__parameters', {})
            params['bias_fx'] = biases['fx']
            params['bias_fy'] = biases['fy']
            params['bias_fz'] = biases['fz']
            params['bias_tx'] = biases['tx']
            params['bias_ty'] = biases['ty']
            params['bias_tz'] = biases['tz']

            with open(config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)

            print(f"✅ Config updated: {config_path}")
        except Exception as e:
            print(f"⚠️  Could not update config: {e}")
            print("   Please update manually with the values above.")

    print()
    print("✅ Calibration complete!")


def main():
    parser = argparse.ArgumentParser(description='Calibrate F/T sensor bias')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--samples', type=int, default=500, help='Number of samples')
    parser.add_argument('--config', default=None,
                        help='Path to ft_sensor_params.yaml to auto-update')
    args = parser.parse_args()

    calibrate(args.port, args.baud, args.samples, args.config)


if __name__ == '__main__':
    main()
