# F1TENTH DAWGS - Package Reference Guide

**Complete reference for all 37 ROS2 packages in the system**

---

## 📦 Package Categories

1. [Agent Integration (1)](#agent-integration)
2. [Base System - Hardware (13)](#base-system---hardware)
3. [Controllers (7)](#controllers)
4. [Peripherals (2)](#peripherals)
5. [Utilities (14)](#utilities)

---

## Agent Integration

### agent_dawgs
**Complete racing agent with SLAM and localization integration**

**Location**: `src/agent_dawgs/`
**Type**: Python + Launch
**Dependencies**: cartographer_ros, slam_toolbox, particle_filter, robot_localization

**Description**:
Integrates perception, planning, and control into complete autonomous racing agent. Provides 15+ launch file variants for mapping and localization with different SLAM backends.

**Key Files**:
- `launch/mapping_cartographer_pure.launch.py` - **Recommended** SLAM
- `launch/localize_particle_pure.launch.py` - **Fast** localization
- `config/dawgs/` - Agent-specific configurations
- `scripts/` - Utility scripts for agent management

**Topics Subscribed**:
- `/scan` - LiDAR data
- `/odom` - Odometry
- `/sensors/imu/raw` - IMU data

**Topics Published**:
- `/map` - Occupancy grid (during mapping)
- `/particle_filter/pose` - Localized pose

**Parameters**:
- SLAM backend selection (Cartographer vs SLAM Toolbox)
- Odometry source (pure, EKF, wheel odom)
- Particle filter count and resampling

---

## Base System - Hardware

### f1tenth_gym_ros
**F1TENTH Gym simulation interface**

**Location**: `src/base_system/f1tenth_gym_ros/` (submodule)
**Type**: Python
**Dependencies**: f1tenth_gym (external Python package)

**Description**:
ROS2 bridge to F1TENTH Gym simulator. Enables hardware-free testing of autonomous algorithms. Publishes simulated sensor data and accepts control commands.

**Key Files**:
- `f1tenth_gym_ros/gym_bridge.py` - Main simulation bridge
- `launch/gym_bridge_launch.py` - Simulation launcher
- `config/sim_config.yaml` - Simulation parameters
- `maps/` - Simulation track maps

**Topics Published**:
- `/scan` - Simulated LiDAR (sensor_msgs/LaserScan)
- `/odom` - Simulated odometry (nav_msgs/Odometry)
- `/ego_racecar/odom` - Per-car odometry

**Topics Subscribed**:
- `/drive` - Ackermann commands
- `/reset` - Reset simulation

**Parameters**:
```yaml
map_path: path/to/map.yaml
num_agents: 1
scan_beams: 1080
scan_fov: 4.7  # radians
update_pose_rate: 0.01  # seconds
```

---

### f1tenth_stack
**Hardware stack launcher and configuration**

**Location**: `src/base_system/f1tenth_system/f1tenth_stack/`
**Type**: Launch + Config
**Dependencies**: vesc_driver, ackermann_mux, joy_teleop, urg_node

**Description**:
Meta-package that launches complete F1TENTH hardware stack. Brings up VESC motor controller, LiDAR, joystick, and command multiplexer.

**Key Files**:
- `launch/bringup_launch.py` - **Main hardware launcher**
- `config/sensors.yaml` - Sensor configurations
- `config/mux.yaml` - Command multiplexer setup

**Launch Arguments**:
```bash
ros2 launch f1tenth_stack bringup_launch.py \
  joy_dev:=/dev/input/js0 \
  lidar_ip:=192.168.1.111 \
  vesc_port:=/dev/ttyACM0
```

---

### ackermann_mux
**Ackermann command multiplexer with priority and deadman switches**

**Location**: `src/base_system/f1tenth_system/ackermann_mux/`
**Type**: C++ node
**Dependencies**: ackermann_msgs

**Description**:
Multiplexes multiple Ackermann command sources (teleop, autonomous, emergency) based on priority and deadman switch status. Critical safety component.

**Key Files**:
- `src/ackermann_mux_node.cpp` - Multiplexer logic
- `config/mux.yaml` - Priority and timeout configuration

**Topics Subscribed**:
- `/teleop/drive` - Teleop commands (priority 10)
- `/drive` - Autonomous commands (priority 5)
- `/emergency/drive` - Emergency override (priority 100)

**Topics Published**:
- `/mux/ackermann_cmd` - Selected command

**Configuration** (`config/mux.yaml`):
```yaml
topics:
  - name: teleop
    topic: /teleop/drive
    timeout: 0.5  # seconds
    priority: 10  # Higher = higher priority

  - name: autonomous
    topic: /drive
    timeout: 1.0
    priority: 5
```

**Safety Logic**:
- Selects highest priority command with active deadman
- Falls back to lower priority if timeout occurs
- Emergency stop if all inputs timeout

---

### vesc (Meta-package)
**VESC motor controller stack**

**Location**: `src/base_system/f1tenth_system/vesc/`
**Sub-packages**: vesc_driver, vesc_ackermann, vesc_msgs
**Type**: Meta

**Description**:
Complete VESC (Vedder Electronic Speed Controller) integration. Handles low-level motor control, Ackermann command conversion, and telemetry.

---

### vesc_driver
**VESC motor controller driver**

**Location**: `src/base_system/f1tenth_system/vesc/vesc_driver/`
**Type**: C++ node
**Dependencies**: vesc_msgs, serial

**Description**:
Low-level driver for VESC motor controller. Communicates via serial port, publishes motor telemetry, accepts duty cycle/current/speed commands.

**Key Files**:
- `src/vesc_driver_node.cpp` - Driver implementation
- `config/vesc_config.yaml` - Motor limits and calibration

**Topics Published**:
- `/sensors/core` - vesc_msgs/VescStateStamped (50Hz)
  - Motor RPM, current, voltage, temperature
  - Duty cycle, fault codes

**Topics Subscribed**:
- `/commands/motor/duty_cycle` - Duty cycle command [-1, 1]
- `/commands/motor/current` - Current command [A]
- `/commands/motor/speed` - Speed command [ERPM]

**Parameters** (`config/vesc_config.yaml`):
```yaml
port: /dev/ttyACM0
duty_cycle_min: 0.0
duty_cycle_max: 0.5
current_min: 0.0
current_max: 40.0  # Amps
erpm_speed_limit: 30000
```

---

### vesc_ackermann
**Ackermann to VESC command converter**

**Location**: `src/base_system/f1tenth_system/vesc/vesc_ackermann/`
**Type**: C++ node
**Dependencies**: vesc_driver, ackermann_msgs

**Description**:
Converts high-level Ackermann commands (steering angle, speed) to low-level VESC commands (servo position, ERPM). Handles calibration and limiting.

**Key Files**:
- `src/ackermann_to_vesc_node.cpp` - Conversion logic
- `config/ackermann_to_vesc.yaml` - Calibration parameters

**Topics Subscribed**:
- `/mux/ackermann_cmd` - ackermann_msgs/AckermannDriveStamped

**Topics Published**:
- `/commands/motor/speed` - ERPM command
- `/commands/servo/position` - Servo position [0, 1]

**Calibration** (`config/ackermann_to_vesc.yaml`):
```yaml
speed_to_erpm_gain: 4614  # ERPM per m/s
speed_to_erpm_offset: 0.0

steering_angle_to_servo_gain: -1.2135
steering_angle_to_servo_offset: 0.5

servo_min: 0.1
servo_max: 0.9
speed_min: 0.0  # m/s
speed_max: 8.0  # m/s
```

---

### vesc_msgs
**VESC message definitions**

**Location**: `src/base_system/f1tenth_system/vesc/vesc_msgs/`
**Type**: Message definitions

**Messages**:
- `VescState.msg` - Motor state without timestamp
- `VescStateStamped.msg` - Motor state with timestamp
- `VescImuStamped.msg` - IMU from VESC (if equipped)

**VescStateStamped fields**:
```
std_msgs/Header header
float64 voltage_input        # Input voltage [V]
float64 temperature_pcb      # PCB temp [°C]
float64 current_motor        # Motor current [A]
float64 current_input        # Input current [A]
float64 speed                # Motor speed [ERPM]
float64 duty_cycle           # Duty cycle [-1, 1]
float64 charge_drawn         # Ah drawn
float64 charge_regen         # Ah regenerated
float64 energy_drawn         # Wh drawn
float64 energy_regen         # Wh regenerated
float64 displacement         # Meters traveled
float64 distance_traveled    # Total distance [m]
int32 fault_code             # Fault status
```

---

### joy_teleop
**Joystick teleoperation with configurable mappings**

**Location**: `src/base_system/f1tenth_system/teleop_tools/joy_teleop/`
**Type**: Python node
**Dependencies**: sensor_msgs (Joy), ackermann_msgs

**Description**:
Translates joystick inputs to Ackermann commands. Supports Logitech F-710 with deadman switches (LB for teleop mode).

**Key Files**:
- `joy_teleop/joy_teleop.py` - Teleop logic
- `config/teleop.yaml` - Button/axis mappings

**Topics Subscribed**:
- `/joy` - sensor_msgs/Joy

**Topics Published**:
- `/teleop/drive` - ackermann_msgs/AckermannDriveStamped

**Configuration** (`config/teleop.yaml`):
```yaml
# F-710 Controller Configuration
teleop:
  drive:
    type: topic
    message_type: ackermann_msgs/AckermannDriveStamped
    topic_name: /teleop/drive
    deadman_buttons: [4]  # LB button
    axis_mappings:
      - axis: 1          # Left stick Y
        target: speed
        scale: 3.0       # Max speed m/s
      - axis: 3          # Right stick X
        target: steering_angle
        scale: 0.35      # Max angle rad
```

---

### key_teleop
**Keyboard teleoperation**

**Location**: `src/base_system/f1tenth_system/teleop_tools/key_teleop/`
**Type**: Python node
**Dependencies**: ackermann_msgs

**Description**:
Simple keyboard-based teleoperation for testing. WASD controls, spacebar for stop.

---

### mouse_teleop
**Mouse teleoperation**

**Location**: `src/base_system/f1tenth_system/teleop_tools/mouse_teleop/`
**Type**: Python node
**Dependencies**: ackermann_msgs

**Description**:
Mouse-based teleoperation. X-axis controls steering, Y-axis controls speed.

---

### teleop_tools (Meta-package)
**Teleoperation tools collection**

**Location**: `src/base_system/f1tenth_system/teleop_tools/teleop_tools/`
**Sub-packages**: joy_teleop, key_teleop, mouse_teleop
**Type**: Meta

---

### teleop_tools_msgs
**Teleoperation message definitions**

**Location**: `src/base_system/f1tenth_system/teleop_tools/teleop_tools_msgs/`
**Type**: Message and action definitions

**Actions**:
- `Increment.action` - Incremental value adjustment
- `ConfigureJoy.action` - Runtime joystick reconfiguration

---

## Controllers

### path_planner ⭐ PRIMARY
**Two-stage Frenet optimal trajectory planner**

**Location**: `src/controller/path_planner/`
**Type**: C++ node
**Dependencies**: nanoflann, Eigen3

**Description**:
**Primary planning system**. Generates optimal trajectories using two-stage Frenet planning: (1) Lattice sampling with 45 candidates for obstacle avoidance, (2) LUT spirals for smooth racing lines. Operates at 20Hz+ with aggressive optimizations.

**Key Files**:
- `src/path_planner_node.cpp` - Main planner (1500+ lines)
- `include/path_planner/frenet.hpp` - Frenet coordinate conversion
- `include/path_planner/lattice_planner.hpp` - Lattice sampling
- `include/path_planner/lut_planner.hpp` - LUT spiral generation
- `config/planner_params.yaml` - **70+ configuration parameters**

**Topics Subscribed**:
- `/global_centerline` - nav_msgs/Path (reference path from CSV)
- `/odom` - nav_msgs/Odometry (triggers planning)
- `/scan` - sensor_msgs/LaserScan (obstacle detection)

**Topics Published**:
- `/planned_path` - nav_msgs/Path (20Hz, final trajectory)
- `/frenet_path` - visualization_msgs/Marker (Frenet visualization)
- `/lut_path` - visualization_msgs/Marker (LUT visualization)
- `/obstacle_points` - visualization_msgs/Marker (detected obstacles)

**Key Parameters** (`config/planner_params.yaml`):
```yaml
# Logging
log_level: 1  # 0=NONE, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG, 5=VERBOSE

# Input
csv_file_path: "/path/to/track.csv"  # Track waypoints (x,y,v,kappa)

# Planning modes
use_lattice: true   # Enable lattice planner
use_frenet: true    # Enable Frenet planner
planner_horizon: 3.0  # Planning horizon [s]

# Frenet trajectory generation
frenet_time_horizon: 3.0    # Max time horizon [s]
frenet_min_time: 1.0        # Min time horizon [s]
frenet_target_speed: 3.0    # Target speed [m/s]
frenet_dt: 0.05             # Time step [s]
frenet_max_speed: 15.0      # Speed limit [m/s]
frenet_max_accel: 4.0       # Max lateral accel [m/s²]

# Trajectory sampling
frenet_d_samples: [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]  # 9 lateral
frenet_t_samples: [1.5, 2.0, 2.5, 3.0, 3.5]  # 5 time horizons
# Total candidates: 9 × 5 = 45 trajectories

# Cost function weights
frenet_k_jerk: 0.1          # Jerk (smoothness)
frenet_k_time: 0.1          # Time (speed preference)
frenet_k_deviation: 1.0     # Centerline deviation
frenet_k_velocity: 1.0      # Speed tracking
frenet_k_proximity: 0.5     # Obstacle proximity

# Safety parameters
frenet_safety_radius: 0.9       # Base safety bubble [m]
frenet_road_half_width: 1.2     # Drivable width [m]
frenet_vehicle_radius: 0.5      # Vehicle footprint [m]
frenet_obstacle_radius: 0.5     # Obstacle radius [m]
frenet_k_velocity_safety: 0.15  # Velocity-dependent gain [s]
frenet_min_safety_margin: 0.25  # Min safety margin [m]
frenet_proximity_threshold: 1.5 # Proximity cost threshold [m]
frenet_interpolation_checks: 3  # Collision check density

# Performance optimization
scan_downsample_factor: 3       # LiDAR downsampling (3x speedup)
expected_computation_time: 0.01 # Position compensation [s]
adaptive_compensation: true     # Adaptive timing
```

**Algorithm Flow**:
```
1. Odometry callback → Current position (x, y, θ)
2. KD-tree search → Nearest waypoint on centerline
3. Cartesian → Frenet conversion (s, d)
4. Generate 45 trajectory candidates:
   - 9 lateral offsets × 5 time horizons
   - Quintic polynomials for smooth motion
5. Cost evaluation for each:
   - Jerk, time, deviation, velocity, obstacles
6. Select minimum cost trajectory
7. Frenet → Cartesian conversion
8. Interpolate to 0.1m resolution
9. Publish /planned_path at 20Hz
```

**Performance**:
- Planning frequency: 20Hz+
- Computation time: 4-6ms (typical)
- LiDAR processing: 360 points (downsampled from 1080)
- KD-tree search: O(log n) for n waypoints

---

### path_tracker
**Pure pursuit controller for dynamic trajectories**

**Location**: `src/controller/path_tracker/`
**Type**: C++ node
**Dependencies**: Eigen3

**Description**:
Pure pursuit path tracking for dynamically generated trajectories. Interpolates received paths to high resolution (0.1m), calculates lookahead points, and generates Ackermann commands.

**Key Files**:
- `src/path_tracker_node.cpp` - Tracker implementation
- `include/path_tracker/pure_pursuit.hpp` - Algorithm
- `config/tracker_params.yaml` - Configuration

**Topics Subscribed**:
- `/planned_path` - nav_msgs/Path (from path_planner)
- `/odom` - nav_msgs/Odometry (current pose)

**Topics Published**:
- `/drive` - ackermann_msgs/AckermannDriveStamped

**Parameters** (`config/tracker_params.yaml`):
```yaml
lookahead_distance: 1.5  # Base lookahead [m]
lookahead_gain: 0.3      # Velocity-dependent gain [s]
max_lookahead: 3.0       # Max lookahead [m]
min_lookahead: 0.5       # Min lookahead [m]

wheelbase: 0.33          # Vehicle wheelbase [m]
max_steering_angle: 0.5  # Max steering [rad]

path_timeout: 1.0        # Path timeout [s]
interpolation_resolution: 0.1  # Path interpolation [m]
```

**Pure Pursuit Algorithm**:
```
1. Receive path → Interpolate to 0.1m resolution
2. Current position from odometry
3. Calculate lookahead distance:
   L = base + gain * velocity
4. Find lookahead point on path
5. Calculate curvature:
   κ = 2 * lateral_offset / L²
6. Calculate steering angle:
   δ = atan(wheelbase * κ)
7. Publish Ackermann command
```

**Recent Improvements**:
- Path interpolation (5x denser) for smoother tracking
- Last command buffering (graceful degradation on timeout)
- Velocity-dependent lookahead for dynamic adaptation

---

### pure_pursuit
**Static waypoint follower with lap timing**

**Location**: `src/controller/pure_pursuit/`
**Type**: C++ node

**Description**:
Pure pursuit for static waypoint following. Includes lap timing, waypoint progress tracking. Simpler than path_tracker, used for testing and validation.

**Key Files**:
- `src/pure_pursuit_node.cpp`
- `config/pure_pursuit.yaml`

**Topics Subscribed**:
- `/global_centerline` - nav_msgs/Path (static waypoints)
- `/odom` - nav_msgs/Odometry

**Topics Published**:
- `/drive` - ackermann_msgs/AckermannDriveStamped
- `/lap_time` - std_msgs/Float32 (lap completion time)

---

### gap_follow
**Reactive gap following for obstacle avoidance**

**Location**: `src/controller/gap_follow/`
**Type**: Python + C++
**Dependencies**: LiDAR processing

**Description**:
Reactive navigation using "Follow the Gap" method. Identifies largest gap in LiDAR scan, aims vehicle toward gap center. Backup system when planner fails.

**Key Files**:
- `gap_follow/reactive_node.py` - Python implementation
- `src/reactive_node.cpp` - C++ implementation
- `config/gap_follow.yaml` - Gap detection parameters
- `launch/reactive_launch.py`

**Topics Subscribed**:
- `/scan` - sensor_msgs/LaserScan

**Topics Published**:
- `/drive` - ackermann_msgs/AckermannDriveStamped

**Parameters**:
```yaml
bubble_radius: 0.8        # Safety bubble [m]
min_gap_width: 0.5        # Min gap width [m]
max_speed: 3.0            # Max speed [m/s]
lookahead_distance: 1.5   # Lookahead [m]
```

**Algorithm**:
```
1. Receive LiDAR scan
2. Identify obstacles (< threshold)
3. Find largest gap in scan
4. Calculate gap center angle
5. Set target steering toward gap
6. Scale speed by gap width
7. Publish Ackermann command
```

---

### frenet_follower
**Frenet coordinate path follower**

**Location**: `src/controller/frenet_follower/`
**Type**: C++ node

**Description**:
Path follower that operates in Frenet coordinates. Tracks (s, d) trajectory rather than (x, y). Useful for testing Frenet planning systems.

---

### lqr_path_follower
**Linear Quadratic Regulator path follower**

**Location**: `src/controller/lqr_path_follower/`
**Type**: C++ node
**Dependencies**: Eigen3

**Description**:
Optimal control using LQR (Linear Quadratic Regulator). Minimizes cost function balancing path deviation and control effort. More sophisticated than pure pursuit.

**Key Files**:
- `src/lqr_node.cpp`
- `config/lqr_params.yaml`

**Parameters**:
```yaml
# LQR cost matrices
Q_lateral: 10.0     # Lateral error weight
Q_heading: 5.0      # Heading error weight
R_steering: 1.0     # Steering effort weight

prediction_horizon: 20  # Timesteps
dt: 0.05               # Timestep [s]
```

---

### mpc_path_follower
**Model Predictive Control path follower**

**Location**: `src/controller/mpc_path_follower/`
**Type**: C++ node
**Dependencies**: Eigen3, optimization library

**Description**:
Advanced control using MPC (Model Predictive Control). Solves optimization problem at each timestep considering constraints and future predictions.

**Key Files**:
- `src/mpc_node.cpp`
- `config/mpc_params.yaml`

**Parameters**:
```yaml
prediction_horizon: 30  # Prediction steps
control_horizon: 10     # Control steps
dt: 0.05               # Timestep [s]

# Cost weights
Q_lateral: 10.0
Q_heading: 5.0
Q_velocity: 1.0
R_steering: 1.0
R_throttle: 1.0

# Constraints
max_steering: 0.5      # rad
max_steering_rate: 2.0 # rad/s
max_acceleration: 4.0  # m/s²
```

---

## Peripherals

### maps
**Production track data**

**Location**: `src/peripheral/maps/`
**Type**: Data files (CSV, PNG, YAML)

**Description**:
Production track maps and waypoint files. Organized by track name with map images, YAML metadata, and CSV waypoints.

**Structure**:
```
maps/
├── icheon/
│   ├── track.csv        # Waypoints (x,y,v,kappa)
│   ├── map.png          # Occupancy grid image
│   └── map.yaml         # Map metadata
├── mohyun_1017/
│   ├── track.csv
│   ├── map.png
│   └── map.yaml
└── ...
```

**CSV Format**:
```
x,y,v,kappa
12.5,8.3,3.5,0.05
12.6,8.4,3.6,0.06
...
```
- `x, y`: Position in map frame [m]
- `v`: Target velocity [m/s]
- `kappa`: Curvature [1/m]

---

### racetracks
**Reference F1 circuit tracks**

**Location**: `src/peripheral/racetracks/`
**Type**: Data files (CSV)

**Description**:
Reference tracks from various F1 circuits. Includes centerline and speed-optimized racing lines for testing and benchmarking.

**Tracks**:
- Monaco
- Silverstone
- Monza
- Spa-Francorchamps
- And more...

---

## Utilities

### particle_filter
**Monte Carlo localization**

**Location**: `src/utilities/particle_filter/` (submodule)
**Type**: C++ node
**Dependencies**: nav_msgs, sensor_msgs, tf2

**Description**:
Particle filter for robot localization. Uses Monte Carlo sampling to estimate pose from LiDAR scans and map. Fast and robust for real-time racing.

**Key Files**:
- `src/particle_filter_node.cpp`
- `config/localize.yaml`

**Topics Subscribed**:
- `/scan` - sensor_msgs/LaserScan
- `/odom` - nav_msgs/Odometry
- `/map` - nav_msgs/OccupancyGrid

**Topics Published**:
- `/particle_filter/pose` - geometry_msgs/PoseStamped
- `/particle_cloud` - geometry_msgs/PoseArray (visualization)
- `/tf` - map → base_link transform

**Parameters**:
```yaml
num_particles: 1000
resample_interval: 1
update_min_d: 0.2     # Min distance for update [m]
update_min_a: 0.5     # Min angle for update [rad]
motion_model: odometry
sensor_model: likelihood_field
```

---

### iahrs_driver
**IMU driver for iAHRS**

**Location**: `src/utilities/iahrs_driver/`
**Type**: C++ node
**Dependencies**: serial, sensor_msgs

**Description**:
Driver for iAHRS IMU. Publishes orientation, angular velocity, linear acceleration.

**Topics Published**:
- `/sensors/imu/raw` - sensor_msgs/Imu (100Hz)

---

### global_planner
**TUM global race trajectory optimization**

**Location**: `src/utilities/global_planner/`
**Type**: Python package
**Dependencies**: CVXPY, NumPy, SciPy, Matplotlib

**Description**:
Integration of TUM (Technical University of Munich) global race trajectory optimization. Generates optimal racing lines from occupancy grid maps using minimum curvature and minimum time optimization.

**Key Files**:
- `global_planner/offline_trajectory_generator.py` - Offline generation
- `global_planner/global_racetrajectory_optimization/` - TUM algorithm

**Usage**:
```bash
python3 src/utilities/global_planner/global_planner/offline_trajectory_generator.py \
  --map_name my_track \
  --map_dir src/peripheral/maps/my_track/
```

**Output**:
- Optimized centerline CSV (x, y, v, kappa)
- Racing line visualization
- Speed profile plots

---

### frenet2speedopt
**Speed optimization for Frenet trajectories**

**Location**: `src/utilities/frenet2speedopt/`
**Type**: Python package
**Dependencies**: CVXPY, NumPy

**Description**:
Speed optimization algorithms for Frenet coordinate trajectories. Implements Lipp-Boyd and forward-backward optimization.

**Key Files**:
- `frenet2speedopt/lipp_boyd.py` - Lipp-Boyd optimizer
- `frenet2speedopt/forward_backward.py` - Forward-backward optimizer

---

### frenet_conversion
**Frenet coordinate conversion utilities**

**Location**: `src/utilities/frenet_conversion/`
**Type**: C++ library
**Dependencies**: Eigen3, nanoflann

**Description**:
Standalone library for Frenet coordinate conversions. Header-only library for converting between Cartesian (x, y) and Frenet (s, d) coordinates.

**Key Files**:
- `include/frenet_conversion/frenet.hpp` - Conversion functions
- `include/frenet_conversion/spline.hpp` - Spline interpolation

**Functions**:
```cpp
// Cartesian → Frenet
FrenetCoord cart2frenet(const CartesianPoint& point,
                        const std::vector<Waypoint>& path);

// Frenet → Cartesian
CartesianPoint frenet2cart(const FrenetCoord& frenet,
                           const std::vector<Waypoint>& path);
```

---

### tunercar
**Performance tuning and evaluation tools**

**Location**: `src/utilities/tunercar/`
**Type**: Python scripts

**Description**:
Collection of tools for performance tuning, parameter sweeps, and evaluation metrics.

---

### nanoflann (Third-party)
**Fast KD-tree library**

**Location**: `src/utilities/third_party/nanoflann/`
**Type**: C++ header-only library

**Description**:
Fast KD-tree implementation for nearest neighbor search. Used extensively in path planning for O(log n) waypoint lookup.

**Usage in path_planner**:
```cpp
#include <nanoflann.hpp>

// Build KD-tree from waypoints
KDTree tree(waypoints);

// Fast nearest neighbor search
int nearest_idx = tree.findNearest(query_point);
```

---

## Summary Statistics

| Category | Count | Primary Packages |
|----------|-------|------------------|
| **Agent** | 1 | agent_dawgs |
| **Hardware** | 13 | f1tenth_stack, vesc_driver, ackermann_mux, joy_teleop |
| **Controllers** | 7 | path_planner ⭐, path_tracker, pure_pursuit |
| **Peripherals** | 2 | maps, racetracks |
| **Utilities** | 14 | particle_filter, global_planner, frenet_conversion |
| **Total** | **37** | |

---

**Last Updated**: 2025-10-31
**For System Overview**: See `claudedocs/index/PROJECT_INDEX.md`
**For Claude Code**: See `/home/arti/Documents/RoboRacer/CLAUDE.md`
