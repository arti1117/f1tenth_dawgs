# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## F1TENTH DAWGS Racing System

This is a ROS2 Foxy-based autonomous racing system for F1TENTH scale vehicles. The codebase implements a complete autonomous racing stack including perception, planning, control, and simulation components.

## Project Structure

The repository is organized as follows:

```
f1tenth_dawgs/
├── config/                     # System configurations
│   ├── cyclonedds_jetson.xml   # DDS network configuration
│   └── README.md
│
├── data/                       # Experimental data and calibration
│   ├── calibration/            # Vehicle/sensor calibration data
│   ├── friction_circle/        # Performance envelope data
│   └── README.md
│
├── docs/                       # All project documentation
│   ├── index/                  # Quick reference (88KB, 5 files)
│   │   ├── README.md           # Documentation navigation
│   │   ├── PROJECT_INDEX.md    # Complete system overview
│   │   ├── PACKAGE_REFERENCE.md # All 37 packages detailed
│   │   ├── ROS2_INTERFACES.md  # Topics, messages, services
│   │   └── QUICK_REFERENCE.md  # Commands and troubleshooting
│   ├── technical/              # Technical deep-dives (8 files)
│   ├── resources/              # PDFs and presentations
│   └── README.md               # Documentation index
│
├── scripts/                    # Utility scripts
│   ├── launch_with_dds.sh      # DDS environment setup
│   ├── test_urg_connection.sh  # LiDAR diagnostic
│   └── README.md
│
├── src/                        # ROS2 packages (37 total)
│   ├── agent_dawgs/            # Complete racing agent
│   ├── base_system/            # Hardware drivers (13 packages)
│   ├── controller/             # Control algorithms (7 packages)
│   ├── peripheral/             # Track data and maps
│   └── utilities/              # Supporting tools (14 packages)
│
├── .gitignore
├── CLAUDE.md                   # This file
└── README.md                   # Project overview
```

**Key Directories**:
- `docs/` - **Start here** for documentation (organized into index/, technical/, resources/)
- `src/` - ROS2 packages organized by function
- `config/` - System-level configuration files
- `scripts/` - Utility and diagnostic scripts
- `data/` - Calibration and experimental data

## Architecture Overview

### Core System Components

- **Base System** (`src/base_system/`): Core F1TENTH hardware drivers and communication
  - `f1tenth_system/`: Hardware drivers (VESC motor controller, Hokuyo LiDAR, Logitech F-710 joystick)
  - `f1tenth_gym_ros/`: Simulation interface for F1TENTH Gym environment

- **Controllers** (`src/controller/`): High-level control algorithms
  - `path_planner/`: Frenet optimal trajectory and lattice planner with real-time obstacle avoidance
  - `path_tracker/`: Pure pursuit controller for dynamic Frenet trajectories
  - `pure_pursuit/`: Static waypoint follower with lap timing
  - `gap_follow/`: Reactive gap following algorithm
  - `simple_path_tracker/`: Simplified path tracking implementation
  - `vehicle_calibration/`: VESC calibration, steering calibration, friction circle analysis

- **Utilities** (`src/utilities/`): Supporting components
  - `particle_filter/`: Monte Carlo localization (git submodule)
  - `iahrs_driver/`: IMU driver for attitude estimation
  - `frenet2speedopt/`: Speed optimization utilities (CVXPY, Lipp-Boyd, forward-backward)
  - `global_planner/`: TUM global race trajectory optimization integration
  - `frenet_conversion/`: Frenet coordinate system conversion utilities
  - `tunercar/`: Performance tuning and evaluation tools
  - `third_party/nanoflann/`: KD-tree library for fast nearest-neighbor search

- **Agent Integration** (`src/agent_dawgs/`): Complete racing agent with SLAM (Cartographer/SLAM Toolbox) and localization (particle filter, EKF)

### Data Flow Architecture

1. **Perception Layer**: LiDAR (`/scan`) + IMU (`/sensors/imu/raw`) + Odometry (`/odom`) → Localization (`particle_filter`)
2. **Planning Layer**: Global path (`/global_centerline`) + obstacles (`/scan`) → Local trajectory (`/frenet_path`, `/lut_path`)
3. **Control Layer**: Trajectory (`/planned_path`) → Pure pursuit controller → Ackermann commands (`/drive`)
4. **Safety Layer**: Ackermann multiplexer (`ackermann_mux`) for teleop/autonomous switching with deadman switches

### Planning Architecture Deep Dive

The path planner uses a two-stage Frenet frame planning approach:

**Stage 1: Frenet Lattice Sampling** (Obstacle Avoidance)
- Samples 45 trajectory candidates (9 lateral × 5 time horizons)
- Lateral samples: -1.0m to +1.0m from centerline
- Time horizons: 1.5s to 3.0s
- Cost function: jerk + time + deviation + velocity error + obstacle collision
- Uses quintic polynomials for smooth lateral motion
- Real-time obstacle detection from `/scan` transformed to map frame

**Stage 2: Lattice LUT** (Spiral Trajectories)
- Pre-computed lookup table of spiral trajectories
- Curvature-aware path generation based on centerline curvature
- Frenet-to-Cartesian conversion with KD-tree acceleration
- Closed-loop track support with automatic wrapping

**Coordinate System Flow**:
```
CSV waypoints (x,y,v,kappa) → Global path → Frenet frame (s,d) →
Lattice sampling → Cost evaluation → Best trajectory →
Frenet-to-Cartesian → Published path → Pure pursuit tracker
```

**Key Implementation Details**:
- Triggered by odometry callback (not timer) for minimal latency
- Nanoflann KD-tree for O(log n) nearest waypoint search
- Closed-loop detection: first/last waypoint within 2m
- Position compensation for computation time (~10ms)
- Adaptive log levels (0=NONE to 5=VERBOSE) in `planner.yaml`

### CSV Data Format for Path Planning

Path CSV files follow the format: `x,y,v,kappa` where:
- `x,y`: Track waypoint coordinates in map frame (meters)
- `v`: Target velocity at waypoint (m/s)
- `kappa`: Curvature at waypoint (1/m, sign indicates turn direction)

Available tracks in `src/peripheral/racetracks/` include centerline and speed-optimized racing lines.

### Frenet Coordinate System

The system uses Frenet coordinates for local planning:
- **s (longitudinal)**: Arc length along centerline from start (meters)
- **d (lateral)**: Signed perpendicular distance from centerline (meters, + = right, - = left)

**Conversion Process** (`frenet.cpp`, `frenet.hpp`):
1. **Cart2Frenet**: KD-tree nearest neighbor → project onto segment → calculate (s,d)
2. **Frenet2Cart**: Find segment containing s → interpolate centerline position → offset by d perpendicular to path
3. **Closed-loop handling**: Automatic s-coordinate wrapping for circular tracks (detected when first/last point < 2m apart)

## Build and Development Commands

### Build System
```bash
# Build all packages
colcon build
source install/setup.bash

# Build specific package (faster for development)
colcon build --packages-select <package_name>
source install/setup.bash

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build (when facing CMake cache issues)
rm -rf build/ install/ log/
colcon build
```

### Running Components

#### Hardware Bringup
```bash
# Launch complete F1TENTH stack
ros2 launch f1tenth_stack bringup_launch.py

# Individual hardware components
ros2 launch vesc_driver vesc_driver_node.launch.py
ros2 launch ackermann_mux ackermann_mux_launch.py
```

#### SLAM and Localization
```bash
# Mapping with Cartographer
ros2 launch agent_dawgs mapping_cartographer_pure.launch.py

# Localization with particle filter
ros2 launch particle_filter localize_launch.py
```

#### Controllers
```bash
# Pure pursuit with path following
ros2 launch pure_pursuit pure_pursuit.launch.py

# Reactive gap following
ros2 launch gap_follow reactive_launch.py
```

#### Simulation
```bash
# F1TENTH Gym simulation
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Development Workflow

1. **Testing**: No unified test framework - each package has custom testing
2. **Configuration**: Parameters in `config/` directories, typically YAML format
3. **Coordinate Frames**: Uses `map` frame for global planning, `base_link` for vehicle frame
4. **Topic Naming**: Follows ROS2 conventions with namespacing

## Key Configuration Files

- **Path Planner**: `src/controller/path_planner/config/planner.yaml`
  - Global path topic: `/global_centerline`
  - Planning parameters: horizon, lattice/frenet usage

- **F1TENTH Stack**: Parameter files in `src/base_system/f1tenth_system/f1tenth_stack/`
  - VESC motor parameters
  - Sensor configurations
  - Safety limits

## Important Notes

- **ROS2 Foxy**: System designed for ROS2 Foxy distribution
- **Hardware Dependencies**: Requires VESC motor controller and Hokuyo LiDAR
- **Safety**: Always use deadman switches (LB for teleop, RB for autonomous on F-710 joystick)
- **Speed Optimization**: Uses CVXPY for trajectory optimization - requires Python environment setup
- **Build Dependencies**: Some packages require ackermann_msgs - install with `sudo apt install ros-foxy-ackermann-msgs`

## Git Submodules

The repository contains git submodules for:
- `src/base_system/f1tenth_system/`: F1TENTH driver stack
- `src/base_system/f1tenth_gym_ros/`: Simulation environment
- `src/utilities/particle_filter/`: Localization package

Update submodules after clone: `git submodule update --init --recursive`

## Comprehensive Documentation

This CLAUDE.md file provides a quick reference for working with the codebase. For comprehensive documentation, see the `docs/` directory:

### Quick Reference Documentation (`docs/index/`)
- **[README.md](docs/index/README.md)** - Documentation navigation and learning paths
- **[PROJECT_INDEX.md](docs/index/PROJECT_INDEX.md)** - Complete system overview (21KB, 701 lines)
  - System architecture and data flow
  - All 37 packages organized by category
  - Launch files and configuration reference
  - Development guides and troubleshooting

- **[PACKAGE_REFERENCE.md](docs/index/PACKAGE_REFERENCE.md)** - Detailed package documentation (25KB, 925 lines)
  - Complete reference for all 37 ROS2 packages
  - Message types, topics, parameters for each package
  - Configuration examples and usage patterns

- **[ROS2_INTERFACES.md](docs/index/ROS2_INTERFACES.md)** - ROS2 communication reference (18KB, 777 lines)
  - Complete topic architecture with message structures
  - TF tree and coordinate frames
  - QoS policies explained
  - Services and parameter documentation

- **[QUICK_REFERENCE.md](docs/index/QUICK_REFERENCE.md)** - Commands and troubleshooting (13KB, 418 lines)
  - Quick start workflows (hardware, simulation, mapping)
  - Build, topic, node, and parameter commands
  - Common issues with solutions
  - Performance tuning cheatsheet

### Technical Documentation (`docs/technical/`)
- **PATH_PLANNER_STRUCTURE.md** - Planner architecture deep-dive
- **FRICTION_CIRCLE_IMPLEMENTATION.md** - Vehicle dynamics and limits
- **QOS_POLICY_GUIDE.md** - Complete QoS policy guide
- **SAFETY_DISTANCE_IMPROVEMENTS.md** - Obstacle avoidance details
- And 4 more technical documents

### Resources (`docs/resources/`)
- **F1TENTH_KOREA.pdf** - F1TENTH Korea presentation

**Total Documentation**: 88KB quick reference + ~120KB technical docs covering all 37 packages, 30+ topics, 100+ parameters.

**Start Here**:
- New to the project? → [docs/index/PROJECT_INDEX.md](docs/index/PROJECT_INDEX.md)
- Quick task? → [docs/index/QUICK_REFERENCE.md](docs/index/QUICK_REFERENCE.md)
- Package details? → [docs/index/PACKAGE_REFERENCE.md](docs/index/PACKAGE_REFERENCE.md)
- ROS2 integration? → [docs/index/ROS2_INTERFACES.md](docs/index/ROS2_INTERFACES.md)