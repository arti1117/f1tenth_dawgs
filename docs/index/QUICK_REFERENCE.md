# F1TENTH DAWGS - Quick Reference Guide

**Fast lookup for commands, workflows, and troubleshooting**

---

## 🚀 Quick Start Workflows

### Hardware Racing (Complete Workflow)

```bash
# Terminal 1: Hardware Stack
ros2 launch f1tenth_stack bringup_launch.py

# Terminal 2: Localization
ros2 launch particle_filter localize_launch.py

# Terminal 3: Path Planner
ros2 launch path_planner path_planner.launch.py \
  csv_file_path:=/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/icheon/track.csv

# Terminal 4: Path Tracker
ros2 launch path_tracker path_tracker.launch.py

# Control:
# - Hold LB button: Teleop mode
# - Hold RB button: Autonomous mode
```

---

### Simulation Racing

```bash
# Terminal 1: Simulator
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: Path Planner
ros2 launch path_planner path_planner.launch.py \
  csv_file_path:=/path/to/track.csv

# Terminal 3: Path Tracker
ros2 launch path_tracker path_tracker.launch.py
```

---

### Mapping New Track

```bash
# Terminal 1: Hardware
ros2 launch f1tenth_stack bringup_launch.py

# Terminal 2: SLAM
ros2 launch agent_dawgs mapping_cartographer_pure.launch.py

# Terminal 3: Drive around with LB held (teleop)

# Terminal 4: Save map when done
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: map, map_url: ~/my_track, image_format: pgm, map_mode: trinary}"
```

---

## 📦 Build Commands

```bash
# Full build
cd ~/f1tenth_dawgs
colcon build
source install/setup.bash

# Single package (faster)
colcon build --packages-select path_planner
source install/setup.bash

# With dependencies
colcon build --packages-up-to path_planner

# Clean build
rm -rf build/ install/ log/
colcon build

# Debug build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Parallel build (faster)
colcon build --parallel-workers 4
```

---

## 🔍 Topic Commands

### List and Info
```bash
# List all topics
ros2 topic list

# Topic details
ros2 topic info /scan
ros2 topic info /scan --verbose  # Show publishers/subscribers

# Message type definition
ros2 interface show sensor_msgs/LaserScan
```

---

### Monitor and Debug
```bash
# Topic rate (Hz)
ros2 topic hz /scan
ros2 topic hz /planned_path

# Topic bandwidth
ros2 topic bw /scan

# Echo data (view messages)
ros2 topic echo /drive
ros2 topic echo /scan --once
ros2 topic echo /odom --field pose.pose.position

# Monitor multiple
ros2 topic hz /scan & ros2 topic hz /odom &
```

---

### Publish Test Commands
```bash
# Publish Ackermann command
ros2 topic pub /drive ackermann_msgs/AckermannDriveStamped \
  "{header: {frame_id: base_link}, drive: {speed: 1.0, steering_angle: 0.1}}"

# One-shot publish
ros2 topic pub --once /drive ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 2.0, steering_angle: 0.0}}"

# Publish at rate
ros2 topic pub --rate 50 /drive ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 1.5, steering_angle: 0.05}}"
```

---

## 🎛️ Parameter Commands

```bash
# List all parameters for node
ros2 param list /path_planner

# Get parameter value
ros2 param get /path_planner log_level
ros2 param get /path_planner frenet_target_speed

# Set parameter (runtime)
ros2 param set /path_planner log_level 4
ros2 param set /path_planner frenet_target_speed 5.0

# Dump all parameters
ros2 param dump /path_planner > planner_params_dump.yaml

# Load parameters from file
ros2 param load /path_planner planner_params.yaml
```

---

## 🤖 Node Commands

```bash
# List all nodes
ros2 node list

# Node info (topics, services, actions)
ros2 node info /path_planner

# Node details
ros2 node info /path_planner --verbose
```

---

## 🛠️ Service Commands

```bash
# List services
ros2 service list

# Service type
ros2 service type /map_saver/save_map

# Call service
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: map, map_url: ~/my_map, image_format: pgm}"

# Service definition
ros2 interface show nav2_msgs/srv/SaveMap
```

---

## 📊 RViz2 Visualization

```bash
# Launch RViz
rviz2

# With config file
rviz2 -d ~/f1tenth_dawgs/src/agent_dawgs/rviz/racing.rviz
```

### Essential Displays to Add

1. **LaserScan** (`/scan`)
   - Topic: `/scan`
   - Size: 0.05
   - Color: White

2. **Map** (`/map`)
   - Topic: `/map`
   - Color Scheme: map

3. **Path** (`/planned_path`)
   - Topic: `/planned_path`
   - Color: Green
   - Line Width: 0.05

4. **Path** (`/global_centerline`)
   - Topic: `/global_centerline`
   - Color: Blue
   - Line Width: 0.03

5. **PoseArray** (`/particle_cloud`)
   - Topic: `/particle_cloud`
   - Arrow Length: 0.3
   - Color: Red

6. **RobotModel** or **TF**
   - Shows vehicle position

---

## 🐛 Debugging Tools

### Enable Verbose Logging

**Path Planner**:
```yaml
# Edit: src/controller/path_planner/config/planner_params.yaml
log_level: 4  # DEBUG
log_level: 5  # VERBOSE
```

**ROS2 Logging**:
```bash
# Set logger level at runtime
ros2 run rqt_logger_level rqt_logger_level

# Or via command line
ros2 param set /path_planner ros__log_level DEBUG
```

---

### Check System Status

```bash
# All nodes running?
ros2 node list

# All expected topics?
ros2 topic list | grep -E "(scan|odom|planned_path|drive)"

# Topic rates okay?
ros2 topic hz /scan        # Should be ~40Hz
ros2 topic hz /odom        # Should be ~50Hz
ros2 topic hz /planned_path # Should be ~20Hz

# Any warnings/errors?
ros2 topic echo /rosout | grep -E "(WARN|ERROR)"
```

---

### Record and Playback

```bash
# Record topics for debugging
ros2 bag record /scan /odom /planned_path /drive -o my_run

# Record all topics
ros2 bag record -a -o full_run

# Playback
ros2 bag play my_run

# Playback with rate control
ros2 bag play my_run --rate 0.5  # Half speed

# Playback specific topics
ros2 bag play my_run --topics /scan /odom

# Bag info
ros2 bag info my_run
```

---

## ⚠️ Common Issues & Fixes

### LiDAR Not Working

**Symptoms**: No `/scan` topic

**Checks**:
```bash
# 1. Network connectivity
ping 192.168.1.111

# 2. Port accessibility
nc -zv 192.168.1.111 10940

# 3. Host IP (must be on 192.168.1.x, NOT .111)
ip addr show | grep 192.168.1

# 4. Run diagnostic
./test_urg_connection.sh
```

**Fixes**:
```bash
# Configure host network
sudo ip addr add 192.168.1.100/24 dev eth0

# Or via NetworkManager
nmcli con modify "Wired connection 1" ipv4.addresses 192.168.1.100/24
nmcli con modify "Wired connection 1" ipv4.method manual
nmcli con up "Wired connection 1"
```

---

### No Planned Path

**Symptoms**: `/planned_path` not publishing

**Checks**:
```bash
# 1. Is planner node running?
ros2 node list | grep path_planner

# 2. Is global centerline loaded?
ros2 topic echo /global_centerline --once

# 3. Is odometry publishing?
ros2 topic hz /odom

# 4. Check planner logs
ros2 param set /path_planner log_level 4  # Enable DEBUG
```

**Fixes**:
```bash
# Verify CSV file path
ls -l /path/to/track.csv

# Check planner config
ros2 param get /path_planner csv_file_path

# Restart planner with correct path
ros2 launch path_planner path_planner.launch.py \
  csv_file_path:=/correct/path/to/track.csv
```

---

### Slow Planning Performance

**Symptoms**: Low `/planned_path` rate, high computation time

**Fixes**:
```yaml
# Edit: config/planner_params.yaml

# Increase downsampling (trade accuracy for speed)
scan_downsample_factor: 5  # 5x speedup

# Reduce trajectory candidates
frenet_d_samples: [-0.5, 0.0, 0.5]  # 3 instead of 9
frenet_t_samples: [2.0, 3.0]        # 2 instead of 5

# Reduce planning horizon
frenet_time_horizon: 2.0  # Instead of 3.0
```

---

### Car Not Moving (Autonomous)

**Symptoms**: No motion when RB held

**Checks**:
```bash
# 1. Is drive command publishing?
ros2 topic echo /drive

# 2. Is mux selecting autonomous?
ros2 topic echo /mux/ackermann_cmd

# 3. Is VESC receiving commands?
ros2 topic echo /commands/motor/speed
ros2 topic echo /commands/servo/position

# 4. Is VESC responding?
ros2 topic echo /sensors/core
```

**Fixes**:
```bash
# Check ackermann_mux priorities
ros2 param dump /ackermann_mux

# Verify deadman switch (RB button = button 5)
ros2 topic echo /joy

# Test VESC directly
ros2 topic pub /commands/motor/speed std_msgs/Float64 "{data: 5000.0}"
```

---

### DDS Communication Issues

**Symptoms**: Topics not visible across machines

**Fixes**:
```bash
# Use DDS configuration
./launch_with_dds.sh

# Or manually:
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/cyclonedds_jetson.xml
export ROS_LOCALHOST_ONLY=0

# Verify settings
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION

# Check DDS log
cat cyclonedds.log
```

---

## 🔧 Calibration

### VESC Motor Calibration

**Location**: `src/base_system/f1tenth_system/vesc/vesc_driver/config/vesc_config.yaml`

```yaml
# Test different speeds
speed_to_erpm_gain: 4614   # Adjust if speed inaccurate
speed_to_erpm_offset: 0.0

# Test with:
ros2 topic pub /commands/motor/speed std_msgs/Float64 "{data: 5000.0}"
# Measure actual speed, adjust gain
```

---

### Steering Calibration

**Location**: `src/base_system/f1tenth_system/vesc/vesc_ackermann/config/ackermann_to_vesc.yaml`

```yaml
# Center steering
steering_angle_to_servo_offset: 0.5  # Adjust if not centered

# Steering gain
steering_angle_to_servo_gain: -1.2135  # Adjust if range wrong

# Test with:
ros2 topic pub /mux/ackermann_cmd ackermann_msgs/AckermannDriveStamped \
  "{drive: {steering_angle: 0.3}}"  # Should turn left ~17°
```

---

## 📁 Important File Locations

```
f1tenth_dawgs/
├── CLAUDE.md                  # Primary project guide
├── claudedocs/index/          # Generated documentation
│   ├── PROJECT_INDEX.md       # Complete project index
│   ├── PACKAGE_REFERENCE.md   # All 37 packages
│   ├── ROS2_INTERFACES.md     # Topics and messages
│   └── QUICK_REFERENCE.md     # This file
│
├── src/controller/
│   ├── path_planner/
│   │   └── config/planner_params.yaml  # 70+ planning parameters ⭐
│   └── path_tracker/
│       └── config/tracker_params.yaml  # Tracking parameters
│
├── src/base_system/f1tenth_system/
│   ├── vesc/vesc_driver/
│   │   └── config/vesc_config.yaml     # Motor limits
│   ├── vesc/vesc_ackermann/
│   │   └── config/ackermann_to_vesc.yaml  # Steering calibration
│   └── ackermann_mux/
│       └── config/mux.yaml             # Command multiplexer
│
├── src/peripheral/
│   ├── maps/                  # Production tracks
│   │   ├── icheon/track.csv
│   │   └── mohyun_1017/track.csv
│   └── racetracks/            # Reference F1 tracks
│
├── cyclonedds_jetson.xml      # DDS configuration
├── launch_with_dds.sh         # DDS launcher script
└── test_urg_connection.sh     # LiDAR diagnostic
```

---

## 🎯 Performance Tuning Cheatsheet

### Path Planner Speed vs Quality

| Parameter | Fast (Low Quality) | Balanced | High Quality (Slow) |
|-----------|-------------------|----------|---------------------|
| `scan_downsample_factor` | 5 | 3 | 1 |
| `frenet_d_samples` | 3 | 9 | 15 |
| `frenet_t_samples` | 2 | 5 | 7 |
| Computation | ~2ms | ~5ms | ~15ms |

---

### QoS Trade-offs

| QoS | Latency | Reliability | Use For |
|-----|---------|-------------|---------|
| Best Effort | Low | Medium | Sensors, control |
| Reliable | Medium | High | Planning, maps |

---

## 📞 Emergency Commands

### Emergency Stop
```bash
# Publish zero command
ros2 topic pub --once /drive ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"

# Kill all nodes
killall ros2
killall path_planner
killall vesc_driver
```

---

### Kill Specific Nodes
```bash
# Find node PID
ros2 node list
ps aux | grep path_planner

# Kill by PID
kill <PID>

# Force kill
kill -9 <PID>
```

---

## 🔑 Keyboard Shortcuts

### RViz2
- `Ctrl+S`: Save config
- `Ctrl+O`: Open config
- `R`: Reset view
- `G`: Toggle grid
- Mouse drag: Rotate view
- Shift+drag: Pan view
- Scroll: Zoom

---

## 🚦 Safety Checklist

- [ ] Emergency stop accessible
- [ ] LB/RB deadman switches tested
- [ ] Battery voltage >11V
- [ ] LiDAR publishing `/scan` at 40Hz
- [ ] Odometry publishing `/odom` at 50Hz
- [ ] Map loaded (if using localization)
- [ ] Ackermann mux operational
- [ ] Clear operating area
- [ ] RViz monitoring active
- [ ] Teleop tested before autonomous

---

## 📚 Related Documentation

- **Project Overview**: `claudedocs/index/PROJECT_INDEX.md`
- **All Packages**: `claudedocs/index/PACKAGE_REFERENCE.md`
- **ROS2 Topics**: `claudedocs/index/ROS2_INTERFACES.md`
- **Primary Guide**: `/home/arti/Documents/RoboRacer/CLAUDE.md`
- **Controller Changes**: `src/controller/CONTROLLER.md`

---

**Last Updated**: 2025-10-31
**Version**: F1TENTH DAWGS v1.0
