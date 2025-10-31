# F1TENTH DAWGS - ROS2 Interfaces Reference

**Complete reference for topics, message types, services, and parameters**

---

## 📡 Topic Architecture

### Communication Patterns

```
┌──────────────────┐
│  Sensor Layer    │ → Best Effort QoS (Low Latency)
├──────────────────┤
│  /scan (40Hz)    │────┐
│  /odom (50Hz)    │────┤
│  /imu (100Hz)    │────┤→ Planning Layer
└──────────────────┘    │
                        │
┌──────────────────┐    │
│  Planning Layer  │ ← ─┘
├──────────────────┤
│ /planned_path    │──→ Reliable QoS (Data Integrity)
│ (20Hz)           │
└──────────────────┘
         │
         ↓
┌──────────────────┐
│  Control Layer   │ → Best Effort QoS (Low Latency)
├──────────────────┤
│ /drive (50Hz)    │
│ /teleop (50Hz)   │
└──────────────────┘
         │
         ↓
┌──────────────────┐
│  Actuation       │
├──────────────────┤
│ VESC Motor       │
│ Servo Steering   │
└──────────────────┘
```

---

## Core Topics Reference

### Sensor Topics

#### /scan
**Type**: `sensor_msgs/LaserScan`
**Rate**: 40Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `urg_node` (Hokuyo LiDAR driver)

**Subscribers**:
- `path_planner` - Obstacle detection
- `gap_follow` - Reactive navigation
- `particle_filter` - Localization
- `cartographer` - SLAM

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id: "laser"

float32 angle_min: -2.35619   # -135° in radians
float32 angle_max: 2.35619    # +135° in radians
float32 angle_increment: 0.00436  # ~0.25°
float32 time_increment: 0.0
float32 scan_time: 0.025      # 40Hz
float32 range_min: 0.02       # 2cm
float32 range_max: 30.0       # 30m

float32[] ranges              # 1080 distance measurements [m]
float32[] intensities         # 1080 intensity values
```

**Usage Notes**:
- **1080 points** per scan (270° FOV)
- **Downsampled** to 360 points in path_planner (scan_downsample_factor: 3)
- Invalid ranges marked as `inf` or `nan`
- Frame: `laser` (LiDAR sensor frame)

---

#### /odom
**Type**: `nav_msgs/Odometry`
**Rate**: 50Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `vesc_driver` (computed from motor encoder)

**Subscribers**:
- `path_planner` - **Triggers planning** (odometry callback)
- `path_tracker` - Current pose for tracking
- `pure_pursuit` - Waypoint following
- `particle_filter` - Motion model
- `robot_localization` (EKF) - Sensor fusion

**Message Structure**:
```
std_msgs/Header header
  time stamp
  string frame_id: "odom"

string child_frame_id: "base_link"

geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x  # meters
      float64 y  # meters
      float64 z: 0.0
    geometry_msgs/Quaternion orientation
      float64 x, y, z, w  # Orientation quaternion
  float64[36] covariance  # 6x6 pose covariance

geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x  # Forward velocity [m/s]
      float64 y: 0.0
      float64 z: 0.0
    geometry_msgs/Vector3 angular
      float64 x: 0.0
      float64 y: 0.0
      float64 z  # Yaw rate [rad/s]
  float64[36] covariance  # 6x6 twist covariance
```

**Usage Notes**:
- Derived from **wheel encoder** on VESC motor
- **Triggers path planning** (not timer-based)
- **Dead reckoning** - accumulates drift over time
- Fused with LiDAR/IMU in particle_filter or EKF for accuracy
- Frame: `odom` → `base_link`

---

#### /sensors/imu/raw
**Type**: `sensor_msgs/Imu`
**Rate**: 100Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `iahrs_driver` (iAHRS IMU)

**Subscribers**:
- `robot_localization` (EKF) - Orientation and angular velocity
- `particle_filter` - Heading estimate

**Message Structure**:
```
std_msgs/Header header
  time stamp
  string frame_id: "imu"

geometry_msgs/Quaternion orientation
  float64 x, y, z, w  # Orientation quaternion
float64[9] orientation_covariance

geometry_msgs/Vector3 angular_velocity
  float64 x  # Roll rate [rad/s]
  float64 y  # Pitch rate [rad/s]
  float64 z  # Yaw rate [rad/s]
float64[9] angular_velocity_covariance

geometry_msgs/Vector3 linear_acceleration
  float64 x  # Forward accel [m/s²]
  float64 y  # Lateral accel [m/s²]
  float64 z  # Vertical accel [m/s²]
float64[9] linear_acceleration_covariance
```

---

#### /sensors/core
**Type**: `vesc_msgs/VescStateStamped`
**Rate**: 50Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `vesc_driver`

**Subscribers**:
- `vehicle_calibration` - Motor characterization
- Monitoring/logging nodes

**Message Structure** (see PACKAGE_REFERENCE.md for full details):
```
std_msgs/Header header
float64 voltage_input      # Battery voltage [V]
float64 temperature_pcb    # PCB temp [°C]
float64 current_motor      # Motor current [A]
float64 speed              # Motor speed [ERPM]
float64 duty_cycle         # Duty cycle [-1, 1]
float64 displacement       # Meters traveled
float64 distance_traveled  # Total distance [m]
int32 fault_code           # Fault status
...
```

---

### Planning Topics

#### /global_centerline
**Type**: `nav_msgs/Path`
**Rate**: 1Hz (static, published once or on reload)
**QoS**: Reliable, Keep Last 10

**Publisher**: CSV loader in `path_planner` or `pure_pursuit`

**Subscribers**:
- `path_planner` - Reference centerline for Frenet planning

**Message Structure**:
```
std_msgs/Header header
  string frame_id: "map"

geometry_msgs/PoseStamped[] poses
  std_msgs/Header header
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x  # Waypoint x [m]
      float64 y  # Waypoint y [m]
      float64 z  # TARGET VELOCITY [m/s] (special usage)
    geometry_msgs/Quaternion orientation
      # Encodes heading and curvature
```

**Special Convention**:
- `pose.position.z` stores **target velocity** (z unused in 2D)
- Curvature (kappa) encoded in orientation or separate field
- CSV format: `x,y,v,kappa`

---

#### /planned_path
**Type**: `nav_msgs/Path`
**Rate**: 20Hz
**QoS**: Reliable, Keep Last 10

**Publisher**: `path_planner` (Frenet optimal planner)

**Subscribers**:
- `path_tracker` - Pure pursuit controller
- RViz visualization

**Message Structure**:
```
std_msgs/Header header
  time stamp
  string frame_id: "map"

geometry_msgs/PoseStamped[] poses  # ~200 poses (interpolated to 0.1m)
  # Each pose:
  geometry_msgs/Point position (x, y, z)
  geometry_msgs/Quaternion orientation (heading)
```

**Generation**:
1. Frenet planner generates 45 candidate trajectories
2. Selects best based on cost function
3. Converts from Frenet (s, d) to Cartesian (x, y)
4. Interpolates to high resolution (0.1m spacing)
5. Publishes at 20Hz

---

#### /frenet_path, /lut_path
**Type**: `visualization_msgs/Marker`
**Rate**: 20Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `path_planner`

**Subscribers**: RViz

**Purpose**: Visualization of Frenet and LUT trajectories for debugging

---

### Control Topics

#### /drive
**Type**: `ackermann_msgs/AckermannDriveStamped`
**Rate**: 50Hz
**QoS**: Best Effort, Keep Last 5

**Publishers**:
- `path_tracker` - Autonomous mode
- `pure_pursuit` - Static waypoint mode
- `gap_follow` - Reactive mode

**Subscribers**:
- `ackermann_mux` - Command multiplexer

**Message Structure**:
```
std_msgs/Header header
  time stamp
  string frame_id: "base_link"

ackermann_msgs/AckermannDrive drive
  float32 steering_angle        # [rad] positive = left
  float32 steering_angle_velocity: 0.0
  float32 speed                 # [m/s] forward velocity
  float32 acceleration: 0.0
  float32 jerk: 0.0
```

**Constraints**:
- `steering_angle`: -0.5 to +0.5 rad (-28° to +28°)
- `speed`: 0 to 8.0 m/s
- Negative speeds for reverse (rarely used)

---

#### /teleop/drive
**Type**: `ackermann_msgs/AckermannDriveStamped`
**Rate**: 50Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `joy_teleop` (joystick teleoperation)

**Subscriber**: `ackermann_mux`

**Requires**: LB deadman button held on F-710 controller

---

#### /mux/ackermann_cmd
**Type**: `ackermann_msgs/AckermannDriveStamped`
**Rate**: 50Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `ackermann_mux` (command multiplexer)

**Subscriber**: `vesc_ackermann` (Ackermann to VESC converter)

**Selection Logic**:
1. Check deadman status for all inputs
2. Select highest priority active input
3. Publish selected command
4. Timeout if no valid inputs

**Priority Order**:
1. Emergency (priority 100) - Not typically used
2. Teleop (priority 10) - LB button held
3. Autonomous (priority 5) - RB button held or auto mode

---

#### /commands/motor/speed
**Type**: `std_msgs/Float64`
**Rate**: 50Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `vesc_ackermann` (after Ackermann→ERPM conversion)

**Subscriber**: `vesc_driver`

**Units**: ERPM (Electrical RPM)

**Conversion**:
```
ERPM = speed_m_s * speed_to_erpm_gain + speed_to_erpm_offset
# Typical: gain = 4614, offset = 0.0
```

---

#### /commands/servo/position
**Type**: `std_msgs/Float64`
**Rate**: 50Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `vesc_ackermann`

**Subscriber**: `vesc_driver`

**Units**: Normalized [0, 1]
- 0.5 = center (straight)
- 0.1 = full left
- 0.9 = full right

**Conversion**:
```
servo = steering_angle * steering_angle_to_servo_gain + steering_angle_to_servo_offset
# Typical: gain = -1.2135, offset = 0.5
```

---

### Localization Topics

#### /map
**Type**: `nav_msgs/OccupancyGrid`
**Rate**: 1Hz (static, or during SLAM)
**QoS**: Reliable, Transient Local, Keep Last 1

**Publishers**:
- `map_server` - Static map loading
- `cartographer_node` - During SLAM
- `slam_toolbox` - During SLAM

**Subscribers**:
- `particle_filter` - Localization
- RViz - Visualization

**Message Structure**:
```
std_msgs/Header header
  string frame_id: "map"

nav_msgs/MapMetaData info
  time map_load_time
  float32 resolution    # meters/pixel (e.g., 0.05)
  uint32 width          # pixels
  uint32 height         # pixels
  geometry_msgs/Pose origin  # Map origin in world

int8[] data  # Occupancy values: -1=unknown, 0=free, 100=occupied
```

---

#### /particle_filter/pose
**Type**: `geometry_msgs/PoseStamped`
**Rate**: 50Hz
**QoS**: Reliable, Keep Last 10

**Publisher**: `particle_filter`

**Subscribers**:
- `path_planner` - Localized position for planning
- `path_tracker` - Can substitute for /odom
- Monitoring/logging

**Message Structure**:
```
std_msgs/Header header
  time stamp
  string frame_id: "map"

geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x, y, z
  geometry_msgs/Quaternion orientation
    float64 x, y, z, w
```

---

#### /particle_cloud
**Type**: `geometry_msgs/PoseArray`
**Rate**: 10Hz
**QoS**: Best Effort, Keep Last 5

**Publisher**: `particle_filter`

**Subscriber**: RViz (visualization only)

**Purpose**: Visualize particle distribution for debugging localization

---

### Visualization Topics

#### /obstacle_points
**Type**: `visualization_msgs/Marker` or `visualization_msgs/MarkerArray`
**Rate**: 20Hz

**Publisher**: `path_planner`

**Subscriber**: RViz

**Purpose**: Visualize detected obstacles from LiDAR scan

---

## TF Tree

### Frame Hierarchy

```
map
 └─ odom
     └─ base_link
         ├─ base_footprint
         ├─ laser
         └─ imu
```

### Frame Descriptions

#### map
**Type**: Fixed world frame
**Origin**: Map corner (from map YAML)
**Usage**: Global planning, localization reference

**Published by**:
- `particle_filter`: map → odom transform
- `cartographer`: map → odom transform
- `slam_toolbox`: map → odom transform

---

#### odom
**Type**: Continuous odometry frame
**Origin**: Robot start position (or EKF-fused estimate)
**Usage**: Local odometry, accumulates drift

**Published by**:
- `vesc_driver`: odom → base_link transform (dead reckoning)
- `robot_localization` (EKF): odom → base_link transform (sensor fusion)

**Note**: `map → odom` transform corrects for accumulated drift

---

#### base_link
**Type**: Robot center frame
**Origin**: Center of rear axle (typical convention)
**Usage**: Robot-centric planning and control

**Published by**: odom or EKF node

**Children**: All sensor frames

---

#### laser
**Type**: LiDAR sensor frame
**Origin**: LiDAR sensor center
**Transform from base_link**: Static, configured in URDF or launch

**Typical**:
```yaml
x: 0.27  # 27cm forward from base_link
y: 0.0
z: 0.1   # 10cm above ground
```

---

#### imu
**Type**: IMU sensor frame
**Origin**: IMU sensor center
**Transform from base_link**: Static

---

## Message Type Reference

### Standard ROS2 Messages

#### sensor_msgs
- `sensor_msgs/LaserScan` - LiDAR scans
- `sensor_msgs/Imu` - IMU data
- `sensor_msgs/Joy` - Joystick inputs

#### nav_msgs
- `nav_msgs/Odometry` - Odometry estimates
- `nav_msgs/Path` - Waypoint sequences
- `nav_msgs/OccupancyGrid` - Map representation

#### geometry_msgs
- `geometry_msgs/PoseStamped` - Stamped pose
- `geometry_msgs/PoseArray` - Array of poses (particles)
- `geometry_msgs/TwistStamped` - Velocity commands
- `geometry_msgs/Point`, `Quaternion`, `Vector3` - Primitives

#### std_msgs
- `std_msgs/Header` - Timestamp and frame_id
- `std_msgs/Float32`, `Float64` - Scalar values

#### visualization_msgs
- `visualization_msgs/Marker` - Single visualization
- `visualization_msgs/MarkerArray` - Multiple markers

---

### Custom Messages

#### ackermann_msgs
**Package**: `ackermann_msgs`
**Install**: `sudo apt install ros-foxy-ackermann-msgs`

**Messages**:
- `ackermann_msgs/AckermannDrive` - Ackermann command (no timestamp)
- `ackermann_msgs/AckermannDriveStamped` - With header

---

#### vesc_msgs
**Package**: `vesc_msgs`
**Location**: `src/base_system/f1tenth_system/vesc/vesc_msgs/`

**Messages**:
- `vesc_msgs/VescState` - Motor state (no timestamp)
- `vesc_msgs/VescStateStamped` - With header
- `vesc_msgs/VescImuStamped` - VESC IMU (if equipped)

---

#### teleop_tools_msgs
**Package**: `teleop_tools_msgs`
**Location**: `src/base_system/f1tenth_system/teleop_tools/teleop_tools_msgs/`

**Actions**:
- `teleop_tools_msgs/action/Increment` - Incremental adjustment
- `teleop_tools_msgs/action/ConfigureJoy` - Runtime joystick config

---

## QoS Policies

### Best Effort (Sensor Topics)
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)
```

**Topics**: `/scan`, `/odom`, `/sensors/imu/raw`, `/drive`

**Rationale**: Minimize latency for real-time control, occasional packet loss acceptable

---

### Reliable (Planning Topics)
```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Topics**: `/planned_path`, `/global_centerline`, `/particle_filter/pose`

**Rationale**: Ensure data integrity for planning, avoid control errors from missing paths

---

### Transient Local (Map Topics)
```python
qos_profile = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

**Topics**: `/map`

**Rationale**: Late-joining subscribers receive last map (no re-publishing needed)

---

## Services

### map_server Services

#### /map_saver/save_map
**Type**: `nav2_msgs/srv/SaveMap`

**Usage**:
```bash
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: map, map_url: ~/my_map, image_format: pgm, map_mode: trinary}"
```

---

### cartographer Services

#### /finish_trajectory
**Type**: `cartographer_ros_msgs/srv/FinishTrajectory`

**Usage**: Finalize SLAM trajectory after mapping

---

#### /write_state
**Type**: `cartographer_ros_msgs/srv/WriteState`

**Usage**: Save SLAM state to file

---

## Parameters

### Path Planner Parameters

See `src/controller/path_planner/config/planner_params.yaml` for 70+ parameters.

**Key parameters**:
- `log_level`: 0-5 (NONE to VERBOSE)
- `csv_file_path`: Track file path
- `frenet_target_speed`: Target speed [m/s]
- `frenet_max_accel`: Max lateral accel [m/s²]
- `frenet_d_samples`: Lateral sampling array
- `frenet_t_samples`: Time horizon array
- `frenet_k_jerk`: Jerk cost weight
- `frenet_safety_radius`: Obstacle safety [m]
- `scan_downsample_factor`: LiDAR downsampling

**Runtime modification**:
```bash
ros2 param set /path_planner log_level 4
ros2 param get /path_planner frenet_target_speed
```

---

### VESC Parameters

**Key parameters**:
- `port`: Serial port (e.g., `/dev/ttyACM0`)
- `duty_cycle_max`: Max duty cycle limit
- `current_max`: Max current [A]
- `speed_to_erpm_gain`: Speed conversion factor
- `steering_angle_to_servo_gain`: Steering conversion

---

### Particle Filter Parameters

**Key parameters**:
- `num_particles`: Particle count (default: 1000)
- `resample_interval`: Resampling frequency
- `update_min_d`: Min distance for update [m]
- `update_min_a`: Min angle for update [rad]

---

## Topic Debugging Commands

```bash
# List all topics
ros2 topic list

# Topic info
ros2 topic info /scan --verbose

# Topic rate
ros2 topic hz /scan
ros2 topic hz /planned_path

# Topic echo (view data)
ros2 topic echo /drive
ros2 topic echo /scan --once

# Topic bandwidth
ros2 topic bw /scan

# Publish test command
ros2 topic pub /drive ackermann_msgs/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.1}}"
```

---

**Last Updated**: 2025-10-31
**For Package Details**: See `claudedocs/index/PACKAGE_REFERENCE.md`
**For System Overview**: See `claudedocs/index/PROJECT_INDEX.md`
