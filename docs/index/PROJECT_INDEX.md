# F1TENTH DAWGS - Project Documentation Index

**Generated**: 2025-10-31
**Repository**: F1TENTH DAWGS Autonomous Racing System
**ROS Version**: ROS2 Foxy
**Build System**: Colcon

---

## 📋 Quick Navigation

- [System Overview](#system-overview)
- [Package Reference](#package-reference)
- [ROS2 Topics & Interfaces](#ros2-topics--interfaces)
- [Launch Files](#launch-files)
- [Configuration Files](#configuration-files)
- [Development Guides](#development-guides)
- [Troubleshooting](#troubleshooting)

---

## System Overview

### Architecture

```
┌──────────────────────────────────────────────────────────┐
│                   PERCEPTION LAYER                        │
│  LiDAR + IMU + Odometry → Localization (Particle Filter) │
└──────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────┐
│                    PLANNING LAYER                         │
│  Two-Stage Frenet Planner (Lattice + LUT)                │
└──────────────────────────────────────────────────────────┘
                          ↓
┌──────────────────────────────────────────────────────────┐
│                    CONTROL LAYER                          │
│  Pure Pursuit → Ackermann Commands → VESC                │
└──────────────────────────────────────────────────────────┘
```

### Project Statistics

- **Total ROS2 Packages**: 37
- **Launch Files**: 37
- **Configuration Files**: 137
- **Main Components**: 5 (agent, base_system, controller, peripheral, utilities)

---

## Package Reference

### 📦 Core Packages (37 Total)

#### Agent Integration (1 package)
| Package | Description | Key Files |
|---------|-------------|-----------|
| `agent_dawgs` | Complete racing agent with SLAM integration | `launch/`, `config/dawgs/` |

#### Base System (13 packages)
| Package | Description | Key Files |
|---------|-------------|-----------|
| `f1tenth_gym_ros` | Simulation interface | `f1tenth_gym_ros/gym_bridge.py` |
| `f1tenth_stack` | Hardware stack launcher | `launch/bringup_launch.py` |
| `ackermann_mux` | Command multiplexer (teleop/autonomous) | `config/mux.yaml` |
| `vesc` | Meta-package for VESC motor controller | - |
| `vesc_driver` | VESC motor controller driver | `config/vesc_config.yaml` |
| `vesc_ackermann` | Ackermann conversion for VESC | `config/ackermann_to_vesc.yaml` |
| `vesc_msgs` | VESC message definitions | `msg/` |
| `joy_teleop` | Joystick teleoperation (F-710) | `config/teleop.yaml` |
| `key_teleop` | Keyboard teleoperation | - |
| `mouse_teleop` | Mouse teleoperation | - |
| `teleop_tools` | Teleoperation meta-package | - |
| `teleop_tools_msgs` | Teleoperation messages | `action/`, `msg/` |

#### Controllers (7 packages)
| Package | Description | Type | Key Files |
|---------|-------------|------|-----------|
| `path_planner` | **PRIMARY** Two-stage Frenet planner | Planning | `config/planner_params.yaml` |
| `path_tracker` | Pure pursuit for dynamic trajectories | Control | `config/tracker_params.yaml` |
| `pure_pursuit` | Static waypoint follower | Control | `config/pure_pursuit.yaml` |
| `gap_follow` | Reactive gap following | Planning | `config/gap_follow.yaml` |
| `frenet_follower` | Frenet path follower | Control | `config/frenet_follower.yaml` |
| `lqr_path_follower` | LQR optimal control | Control | `config/lqr_params.yaml` |
| `mpc_path_follower` | Model Predictive Control | Control | `config/mpc_params.yaml` |

#### Utilities (16 packages)
| Package | Description | Category | Key Files |
|---------|-------------|----------|-----------|
| `particle_filter` | Monte Carlo localization | Localization | `config/localize.yaml` |
| `iahrs_driver` | IMU driver | Hardware | `config/iahrs_param.yaml` |
| `global_planner` | TUM trajectory optimization | Planning | `config/` |
| `frenet2speedopt` | Speed optimization (CVXPY) | Planning | Python modules |
| `frenet_conversion` | Frenet coordinate utilities | Math | `include/frenet_conversion/` |
| `tunercar` | Performance tuning tools | Tools | `scripts/` |
| `nanoflann` | KD-tree library (third-party) | Math | `include/nanoflann.hpp` |

### 📁 Package Organization

```
src/
├── agent_dawgs/           # Complete agent with SLAM
│   ├── config/            # Agent configurations
│   ├── launch/            # 15+ launch variants
│   └── scripts/           # Helper scripts
│
├── base_system/           # Hardware drivers
│   ├── f1tenth_gym_ros/   # Simulation (submodule)
│   └── f1tenth_system/    # Hardware stack (submodule)
│       ├── ackermann_mux/ # Command multiplexer
│       ├── f1tenth_stack/ # Hardware launcher
│       ├── teleop_tools/  # 5 teleoperation packages
│       └── vesc/          # 4 VESC packages
│
├── controller/            # Control algorithms (7 packages)
│   ├── path_planner/      # PRIMARY: Frenet optimal planner
│   ├── path_tracker/      # Dynamic trajectory tracker
│   ├── pure_pursuit/      # Static waypoint follower
│   ├── gap_follow/        # Reactive navigation
│   ├── frenet_follower/   # Frenet path follower
│   ├── lqr_path_follower/ # LQR control
│   └── mpc_path_follower/ # MPC control
│
├── peripheral/            # Track data and maps
│   ├── maps/              # Production track files
│   └── racetracks/        # Reference F1 circuits
│
└── utilities/             # Supporting tools (16 packages)
    ├── particle_filter/   # Localization
    ├── iahrs_driver/      # IMU driver
    ├── global_planner/    # TUM trajectory optimizer
    ├── frenet2speedopt/   # Speed optimization
    ├── frenet_conversion/ # Coordinate conversion
    ├── tunercar/          # Tuning tools
    └── third_party/       # External libraries
```

---

## ROS2 Topics & Interfaces

### Core Topics

#### Sensor Topics (Best Effort QoS)
| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 40Hz | urg_node | path_planner, gap_follow |
| `/odom` | `nav_msgs/Odometry` | 50Hz | vesc_driver | path_planner, path_tracker, particle_filter |
| `/sensors/imu/raw` | `sensor_msgs/Imu` | 100Hz | iahrs_driver | particle_filter, EKF |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | 50Hz | vesc_driver | calibration, monitoring |

#### Planning Topics (Reliable QoS)
| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/global_centerline` | `nav_msgs/Path` | 1Hz | CSV loader | path_planner |
| `/planned_path` | `nav_msgs/Path` | 20Hz | path_planner | path_tracker |
| `/frenet_path` | `visualization_msgs/Marker` | 20Hz | path_planner | RViz |
| `/lut_path` | `visualization_msgs/Marker` | 20Hz | path_planner | RViz |

#### Control Topics (Best Effort QoS)
| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | 50Hz | path_tracker | ackermann_mux |
| `/teleop/drive` | `ackermann_msgs/AckermannDriveStamped` | 50Hz | joy_teleop | ackermann_mux |
| `/mux/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | 50Hz | ackermann_mux | vesc_ackermann |

#### Localization Topics (Reliable QoS)
| Topic | Type | Rate | Publisher | Subscribers |
|-------|------|------|-----------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | 1Hz | map_server, SLAM | particle_filter |
| `/particle_filter/pose` | `geometry_msgs/PoseStamped` | 50Hz | particle_filter | path_planner |
| `/particle_cloud` | `geometry_msgs/PoseArray` | 10Hz | particle_filter | RViz |

### Message Types Used

#### Standard ROS2 Messages
- `sensor_msgs/LaserScan`: LiDAR data (1080 points)
- `sensor_msgs/Imu`: IMU orientation and angular velocity
- `nav_msgs/Odometry`: Position, velocity estimates
- `nav_msgs/Path`: Waypoint sequences with poses
- `nav_msgs/OccupancyGrid`: Map representation
- `geometry_msgs/PoseStamped`: Localized position
- `geometry_msgs/PoseArray`: Particle filter cloud

#### Custom Messages
- `ackermann_msgs/AckermannDriveStamped`: Steering + speed commands
- `vesc_msgs/VescStateStamped`: Motor controller state
- `teleop_tools_msgs/*`: Teleoperation commands

### TF Frames
```
map
 └─ odom
     └─ base_link
         ├─ laser  (LiDAR sensor frame)
         ├─ imu    (IMU sensor frame)
         └─ base_footprint
```

---

## Launch Files

### Agent Launches (`src/agent_dawgs/launch/`)

#### Mapping (SLAM)
| File | SLAM Backend | Odometry Source | Description |
|------|--------------|-----------------|-------------|
| `mapping_cartographer_pure.launch.py` | Cartographer | LiDAR-only | **Recommended** - Pure LiDAR SLAM |
| `mapping_cartographer_ekf.launch.py` | Cartographer | EKF fusion | LiDAR + IMU + Odom fusion |
| `mapping_cartographer_odom.launch.py` | Cartographer | Wheel odom | Wheel odometry integration |
| `mapping_slamtoolbox_pure.launch.py` | SLAM Toolbox | LiDAR-only | Alternative SLAM |
| `mapping_slamtoolbox_ekf.launch.py` | SLAM Toolbox | EKF fusion | EKF with SLAM Toolbox |
| `mapping_slamtoolbox_odom.launch.py` | SLAM Toolbox | Wheel odom | SLAM Toolbox + wheel odom |

#### Localization
| File | Backend | Description |
|------|---------|-------------|
| `localize_cartographer_pure.launch.py` | Cartographer | Pure LiDAR localization |
| `localize_cartographer_ekf.launch.py` | Cartographer | EKF-fused localization |
| `localize_slamtoolbox_pure.launch.py` | SLAM Toolbox | SLAM Toolbox localization |
| `localize_particle_pure.launch.py` | Particle Filter | **Fast** - Monte Carlo localization |
| `localize_particle_ekf.launch.py` | Particle Filter + EKF | Sensor-fused particle filter |

### Controller Launches

| Package | File | Description |
|---------|------|-------------|
| `path_planner` | `path_planner.launch.py` | **PRIMARY** - Frenet optimal planner |
| `path_tracker` | `path_tracker.launch.py` | Dynamic pure pursuit tracker |
| `pure_pursuit` | `pure_pursuit.launch.py` | Static waypoint follower |
| `gap_follow` | `reactive_launch.py` | Reactive gap following |

### Base System Launches

| Package | File | Description |
|---------|------|-------------|
| `f1tenth_stack` | `bringup_launch.py` | Complete hardware stack |
| `f1tenth_gym_ros` | `gym_bridge_launch.py` | Simulation interface |
| `particle_filter` | `localize_launch.py` | Standalone particle filter |

---

## Configuration Files

### Critical Configuration Files

#### Path Planner (`src/controller/path_planner/config/planner_params.yaml`)
```yaml
# Primary planning configuration - 70+ parameters
log_level: 1                    # 0=NONE to 5=VERBOSE
csv_file_path: "/path/to/track.csv"
use_lattice: true               # Enable lattice planner
use_frenet: true                # Enable Frenet planner

# Frenet parameters
frenet_target_speed: 3.0        # Target speed [m/s]
frenet_max_accel: 4.0           # Max lateral accel [m/s²]
frenet_d_samples: [-1.0, ..., 1.0]  # Lateral sampling
frenet_t_samples: [1.5, 2.0, 2.5, 3.0]  # Time horizons

# Cost weights
frenet_k_jerk: 0.1              # Smoothness
frenet_k_time: 0.1              # Speed preference
frenet_k_deviation: 1.0         # Centerline adherence
frenet_k_velocity: 1.0          # Speed tracking
frenet_k_proximity: 0.5         # Obstacle avoidance

# Safety parameters
frenet_safety_radius: 0.9       # Base safety bubble [m]
frenet_k_velocity_safety: 0.15  # Velocity-dependent gain [s]
scan_downsample_factor: 3       # LiDAR downsampling (3x speedup)
```

#### VESC Driver (`src/base_system/f1tenth_system/vesc/vesc_driver/config/vesc_config.yaml`)
```yaml
# Motor controller configuration
speed_min: 0.0                  # Min speed [m/s]
speed_max: 8.0                  # Max speed [m/s]
servo_min: 0.1                  # Min servo value
servo_max: 0.9                  # Max servo value
duty_cycle_min: 0.0
duty_cycle_max: 0.5
current_min: 0.0
current_max: 40.0
```

#### Ackermann Mux (`src/base_system/f1tenth_system/ackermann_mux/config/mux.yaml`)
```yaml
# Command multiplexer - deadman switch configuration
topics:
  - name: teleop              # Teleop commands
    topic: /teleop/drive
    timeout: 0.5
    priority: 10
  - name: autonomous          # Autonomous commands
    topic: /drive
    timeout: 1.0
    priority: 5
```

#### Particle Filter (`src/utilities/particle_filter/config/localize.yaml`)
```yaml
# Monte Carlo localization
num_particles: 1000
resample_interval: 1
update_min_d: 0.2              # Min distance for update [m]
update_min_a: 0.5              # Min angle for update [rad]
```

### Configuration Directory Structure
```
config/
├── planner_params.yaml        # Path planner (70+ params)
├── tracker_params.yaml        # Path tracker
├── pure_pursuit.yaml          # Pure pursuit
├── vesc_config.yaml           # VESC motor
├── ackermann_to_vesc.yaml     # Ackermann conversion
├── mux.yaml                   # Command multiplexer
├── localize.yaml              # Particle filter
├── cartographer_*.lua         # Cartographer configs
└── slamtoolbox_*.yaml         # SLAM Toolbox configs
```

---

## Development Guides

### Building the System

#### Full Build
```bash
cd /home/arti/Documents/RoboRacer/f1tenth_dawgs
colcon build
source install/setup.bash
```

#### Package-Specific Build (Faster)
```bash
# Build only path planner
colcon build --packages-select path_planner
source install/setup.bash

# Build with dependencies
colcon build --packages-up-to path_planner
```

#### Clean Build
```bash
# Remove build artifacts
rm -rf build/ install/ log/

# Rebuild everything
colcon build
```

### Running the System

#### Hardware Workflow
```bash
# Terminal 1: Hardware stack
ros2 launch f1tenth_stack bringup_launch.py

# Terminal 2: Localization
ros2 launch particle_filter localize_launch.py

# Terminal 3: Path planner
ros2 launch path_planner path_planner.launch.py \
  csv_file_path:=/path/to/track.csv

# Terminal 4: Path tracker
ros2 launch path_tracker path_tracker.launch.py

# Hold RB button on F-710 for autonomous mode
```

#### Simulation Workflow
```bash
# Terminal 1: Simulation
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: Path planner
ros2 launch path_planner path_planner.launch.py \
  csv_file_path:=/path/to/track.csv

# Terminal 3: Path tracker
ros2 launch path_tracker path_tracker.launch.py
```

### Debugging Tools

#### Enable Verbose Logging
```yaml
# In planner_params.yaml
log_level: 4  # DEBUG level
log_level: 5  # VERBOSE level
```

#### RViz2 Visualization
```bash
rviz2
# Add displays:
# - /scan (LaserScan)
# - /map (OccupancyGrid)
# - /planned_path (Path)
# - /frenet_path (Marker)
# - /particle_cloud (PoseArray)
```

#### Topic Monitoring
```bash
# List all topics
ros2 topic list

# Monitor topic rate
ros2 topic hz /scan
ros2 topic hz /planned_path

# Echo topic data
ros2 topic echo /drive

# Check topic info
ros2 topic info /scan --verbose
```

---

## Troubleshooting

### Common Issues

#### 1. Build Failures

**CMake Cache Issues**:
```bash
rm -rf build/ install/ log/
colcon build
```

**Missing Dependencies**:
```bash
sudo apt install ros-foxy-ackermann-msgs
sudo apt install ros-foxy-cartographer-ros
pip3 install cvxpy numpy scipy matplotlib
```

#### 2. LiDAR Connection

**No `/scan` topic**:
```bash
# Check network
ping 192.168.1.111

# Verify port
nc -zv 192.168.1.111 10940

# Run diagnostic script
./test_urg_connection.sh

# Check host IP (must be on 192.168.1.x subnet, not .111)
ip addr show
```

#### 3. DDS Communication

**Topics not visible across machines**:
```bash
# Use DDS launch script
./launch_with_dds.sh

# Or manually set environment
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/cyclonedds_jetson.xml
export ROS_LOCALHOST_ONLY=0
```

#### 4. Path Planner Issues

**No planned path published**:
```yaml
# Check log level in planner_params.yaml
log_level: 4  # Enable DEBUG logs

# Check topics
ros2 topic list | grep path
ros2 topic hz /planned_path
ros2 topic echo /global_centerline --once
```

**Slow planning performance**:
```yaml
# Increase downsampling in planner_params.yaml
scan_downsample_factor: 5  # More aggressive (5x speedup)

# Reduce trajectory candidates
frenet_d_samples: [-0.5, 0.0, 0.5]  # 3 instead of 9
frenet_t_samples: [2.0, 3.0]        # 2 instead of 5
```

### Performance Tuning

#### Path Planner Optimization
```yaml
# Balance quality vs speed
scan_downsample_factor: 3    # 3x speedup (recommended)
frenet_d_samples: 9 samples  # More = better obstacle avoidance
frenet_t_samples: 5 samples  # More = smoother long-term planning
```

#### QoS Tuning
```yaml
# Sensor topics (latency priority)
qos_profile: BEST_EFFORT
history_depth: 5

# Path topics (reliability priority)
qos_profile: RELIABLE
history_depth: 10
```

---

## File References

### Key Source Files

#### Path Planner
- `src/controller/path_planner/src/path_planner_node.cpp` - Main planner node
- `src/controller/path_planner/include/path_planner/frenet.hpp` - Frenet conversion
- `src/controller/path_planner/include/path_planner/lattice_planner.hpp` - Lattice sampling
- `src/controller/path_planner/config/planner_params.yaml` - Configuration (70+ params)

#### Path Tracker
- `src/controller/path_tracker/src/path_tracker_node.cpp` - Pure pursuit controller
- `src/controller/path_tracker/include/path_tracker/pure_pursuit.hpp` - Algorithm
- `src/controller/path_tracker/config/tracker_params.yaml` - Configuration

#### VESC Driver
- `src/base_system/f1tenth_system/vesc/vesc_driver/src/vesc_driver_node.cpp` - Motor driver
- `src/base_system/f1tenth_system/vesc/vesc_ackermann/src/ackermann_to_vesc_node.cpp` - Ackermann conversion

### Documentation Files
- `/home/arti/Documents/RoboRacer/CLAUDE.md` - Root project guide (495 lines)
- `src/controller/CONTROLLER.md` - Controller change history
- `src/agent_dawgs/README.md` - Agent configuration notes
- This file: `claudedocs/index/PROJECT_INDEX.md` - Complete project index

---

## Additional Resources

### Git Submodules
```bash
# Update all submodules
git submodule update --init --recursive

# Submodule locations:
# - src/base_system/f1tenth_system/
# - src/base_system/f1tenth_gym_ros/
# - src/utilities/particle_filter/
```

### External Libraries
- **nanoflann**: Fast KD-tree for nearest neighbor search
- **TUM Trajectory Optimization**: Global racing line optimization
- **CVXPY**: Convex optimization for speed planning
- **Cartographer**: Google's SLAM library
- **SLAM Toolbox**: ROS2 native SLAM

### Network Configuration
- **LiDAR IP**: `192.168.1.111:10940`
- **DDS Config**: `cyclonedds_jetson.xml`
- **ROS Domain**: `ROS_DOMAIN_ID=0`

---

## Quick Reference Cards

### Command Cheatsheet
```bash
# Build
colcon build --packages-select <package>
source install/setup.bash

# Run
ros2 launch <package> <launch_file>

# Debug
ros2 topic list
ros2 topic hz <topic>
ros2 topic echo <topic>
ros2 node list
ros2 node info <node>

# Map
ros2 run nav2_map_server map_saver_cli -f <map_name>

# Param
ros2 param list
ros2 param get <node> <param>
ros2 param set <node> <param> <value>
```

### Safety Checklist
- [ ] Hardware emergency stop accessible
- [ ] F-710 deadman switches functional (LB=teleop, RB=auto)
- [ ] Battery voltage sufficient (>11V)
- [ ] LiDAR connected and publishing `/scan`
- [ ] Odometry publishing `/odom`
- [ ] Map loaded (if using localization)
- [ ] Ackermann mux operational
- [ ] Clear operating area (no obstacles/people)

---

**Last Updated**: 2025-10-31
**Maintainer**: F1TENTH DAWGS Team
**For Updates**: See `/home/arti/Documents/RoboRacer/CLAUDE.md`
