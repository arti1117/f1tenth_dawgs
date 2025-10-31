# Experimental Data

This directory contains experimental data, calibration results, and performance measurements for the F1TENTH DAWGS system.

## Directory Structure

### calibration/
**Vehicle and Sensor Calibration Data**

Contains calibration data for:
- VESC motor controller calibration
- Steering servo calibration
- Sensor calibration (LiDAR, IMU)
- Speed-to-ERPM mappings
- Steering angle-to-servo mappings

**Usage**: Reference these when tuning `vesc_driver` and `vesc_ackermann` configuration files.

**Related Config Files**:
- `src/base_system/f1tenth_system/vesc/vesc_driver/config/vesc_config.yaml`
- `src/base_system/f1tenth_system/vesc/vesc_ackermann/config/ackermann_to_vesc.yaml`

---

### friction_circle/
**Friction Circle Analysis Data**

Performance envelope data showing vehicle dynamics limits:
- Maximum lateral acceleration vs. speed
- Braking performance curves
- Combined acceleration limits
- Tire friction characteristics

**What is a Friction Circle?**

A friction circle represents the vehicle's performance limits in terms of lateral and longitudinal acceleration. It shows the maximum combined acceleration the vehicle can achieve at any point.

**Usage**:
- Tune path planner acceleration limits (`frenet_max_accel`)
- Validate controller performance
- Set safe operating bounds
- Benchmark performance improvements

**Data Format**: Typically CSV or plots showing acceleration envelopes

---

## Adding New Data

### Best Practices

1. **Organize by category**: Create subdirectories for different data types
2. **Document format**: Add README in subdirectory explaining data format
3. **Include metadata**: Date, vehicle config, test conditions
4. **Version control**: Track significant calibration changes
5. **Backup important data**: Don't rely solely on git

### Recommended Subdirectories

```
data/
├── calibration/          # Vehicle calibration data
├── friction_circle/      # Performance envelope
├── lap_times/           # Racing performance (add if needed)
├── sensor_logs/         # Rosbag recordings (add if needed)
└── experiments/         # Ad-hoc experimental data (add if needed)
```

---

## .gitignore Considerations

**Tracked** (small, important):
- Calibration parameters (YAML, CSV)
- Friction circle measurements
- Configuration baselines

**Not Tracked** (large, regenerable):
- Rosbag files (`*.bag`)
- Large sensor logs (`*.pcap`, `*.raw`)
- Point cloud data (`*.pcd`, `*.ply`)

See `.gitignore` for current exclusions.

---

## Data Collection Guidelines

### Calibration Data
```bash
# Example: Collect speed calibration
# 1. Command known speed
ros2 topic pub /commands/motor/speed std_msgs/Float64 "{data: 5000.0}"

# 2. Measure actual speed (manual or via GPS)
# 3. Record: commanded_erpm, measured_speed_m_s
# 4. Calculate gain: speed_to_erpm_gain = erpm / speed
```

### Performance Data
```bash
# Record vehicle performance during testing
ros2 bag record /odom /sensors/imu/raw /drive -o performance_test

# Post-process for friction circle analysis
# Extract: lateral_accel, longitudinal_accel, speed
```

---

## Calibration Workflow

1. **Initial Calibration**: Use manufacturer specs as baseline
2. **Data Collection**: Run calibration procedures, collect data here
3. **Analysis**: Process data, calculate parameters
4. **Update Configs**: Apply calibrated values to config files
5. **Validation**: Test with new parameters, iterate if needed
6. **Document**: Record final values and test conditions

---

**Related Documentation**:
- Calibration guide: `docs/index/QUICK_REFERENCE.md#calibration`
- VESC parameters: `docs/index/PACKAGE_REFERENCE.md#vesc_driver`
- Performance tuning: `docs/index/QUICK_REFERENCE.md#performance-tuning-cheatsheet`
