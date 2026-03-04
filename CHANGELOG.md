# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-03-02

### Added
- Complete UART protocol handler for Robotous RFT Series sensors
- Protocol verified against official manual v1.8 and real RFT40-SA01-D hardware
- ROS2 Jazzy driver node with 200Hz real-time streaming
- 2nd-order Butterworth low-pass filter (configurable cutoff)
- Force/torque magnitude computation
- Force threshold detection with configurable threshold
- 4-panel real-time matplotlib visualization dashboard
- CSV session logger with participant/trial metadata
- rosbag2 recording integration
- Mock sensor node with simulated force patterns for offline development
- Interactive bias calibration utility
- Full launch system with configurable parameters

### Technical Details
- Packet structure: `SOP(0x55) + Data(8/16B) + Checksum + EOP(0xAA)`
- Data format: 6× signed int16 → Force = raw/DF, Torque = raw/DT
- RFT40-SA01 dividers: DF=50, DT=2000
- Validated sensor: Model RFT40-SA01-D, S/N D00100166, FW RFT40-01-2-05
