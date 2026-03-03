# Calibration Guide

## Bias Calibration (Zeroing)

The sensor outputs non-zero readings even with no applied load due to gravity, mounting orientation, and electrical offset. Bias calibration zeros out these offsets.

### Quick Bias (Via Sensor Hardware)

The Robotous sensor has a built-in bias command that zeros the internal readings. This is the fastest method and works during streaming:

```bash
python3 -c "
from src.ft_sensor_pipeline.robotous_protocol import RobotousProtocol
s = RobotousProtocol(port='/dev/ttyUSB0', sensor_model='RFT40-SA01')
s.connect()
s.set_bias(enable=True)
print('Sensor biased.')
s.disconnect()
"
```

> **Note**: The sensor does not persist bias across power cycles. Re-bias after every power-on.

### Software Bias (Via Config File)

The calibration script collects multiple samples and writes averaged offsets to the config file. These offsets are applied by the ROS2 driver node:

```bash
python3 scripts/calibrate_bias.py \
    --port /dev/ttyUSB0 \
    --samples 500 \
    --config config/ft_sensor_params.yaml
```

**Procedure:**

1. Mount the sensor in its operating orientation
2. Ensure **no external load** is applied
3. Wait 30 seconds for the sensor to thermally stabilize
4. Run the calibration script
5. Verify near-zero readings: `ros2 topic echo /ft_sensor/raw --once`

### When to Recalibrate

- After remounting the sensor
- After significant temperature changes
- At the start of each data collection session
- If readings drift beyond expected noise levels (>0.5N)

## Validating Calibration

```bash
# Check noise floor (should be <200mN for forces, <8mNm for torques)
ros2 launch ft_sensor_pipeline ft_pipeline_launch.py

# In another terminal:
ros2 topic echo /ft_sensor/filtered --field wrench.force
```

Expected output with no load:
```
x: 0.02   # ±0.2N is within spec
y: -0.05
z: 0.12
```
