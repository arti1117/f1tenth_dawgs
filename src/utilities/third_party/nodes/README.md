# Third Party Nodes - Utility Tools

This directory contains utility packages developed by the ForzaETH racing team for F1TENTH autonomous racing. These tools provide essential functionality for testing, tuning, and analyzing racing performance.

## Package Overview

### 1. opponent_publisher

**Purpose**: Simulates dynamic obstacles and detects collisions in the F1TENTH racing environment.

**Description**: Creates virtual or LiDAR-based dynamic obstacles that follow predefined trajectories around the track. Useful for testing obstacle avoidance algorithms and multi-agent racing scenarios.

**Maintainer**: ltognoni@ethz.ch

**License**: MIT

#### Nodes

##### obstacle_publisher
- **Topics Subscribed**:
  - `/car_state/odom_frenet` (Odometry): Vehicle odometry in Frenet frame
  - `/global_waypoints` (WpntArray): Global racing line waypoints
  - `/map` (OccupancyGrid): Map for LiDAR-based obstacles

- **Topics Published**:
  - `/perception/obstacles` (ObstacleArray): Detected/simulated obstacles
  - `/dummy_obstacle_markers` (MarkerArray): Visualization markers
  - `/opponent_waypoints` (OpponentTrajectory): Opponent trajectory
  - `/map` (OccupancyGrid): Modified map with obstacles

- **Parameters**:
  - `speed_scaler` (double, default: 0.5): Speed multiplier for opponent
  - `constant_speed` (bool, default: false): Use constant speed vs scaled speed
  - `trajectory` (string, default: "min_curv"): Trajectory type (min_curv/shortest_path/centerline)
  - `start_s` (double, default: 0.0): Starting position on track (meters along centerline)
  - `type` (string, default: "lidar"): Obstacle type ("lidar" or "virtual")

**Usage**:
```bash
ros2 run opponent_publisher obstacle_publisher --ros-args \
  -p speed_scaler:=0.7 \
  -p trajectory:=centerline \
  -p type:=virtual
```

##### collision_detector
- **Topics Subscribed**:
  - `/perception/obstacles` (ObstacleArray): Obstacle information
  - `/car_state/odom_frenet` (Odometry): Vehicle odometry in Frenet frame
  - `/global_waypoints` (WpntArray): Global waypoints for track length

- **Topics Published**:
  - `/opponent_collision` (Bool): Collision flag
  - `/opponent_dist` (Float32): Distance to nearest opponent
  - `/collision_marker` (MarkerArray): Visualization of collision events

**Usage**:
```bash
ros2 run opponent_publisher collision_detector
```

#### Use Cases
- Testing obstacle avoidance algorithms
- Multi-agent racing simulation
- Collision detection validation
- Overtaking maneuver development

---

### 2. frenet_odom_republisher

**Purpose**: Converts vehicle odometry from Cartesian coordinates to Frenet frame coordinates.

**Description**: Subscribes to standard odometry and global waypoints, then republishes odometry in the Frenet coordinate system (s: longitudinal, d: lateral). Essential for path planning and control algorithms that operate in Frenet frame.

**Maintainer**: nimholz@ethz.ch

**License**: Apache License 2.0

#### Node: frenet_odom_republisher_node

- **Topics Subscribed**:
  - `/global_waypoints` (WpntArray): Reference trajectory for Frenet conversion
  - `car_state/odom` (Odometry): Vehicle odometry in Cartesian coordinates

- **Topics Published**:
  - `/car_state/frenet/odom` (Odometry): Odometry in Frenet frame
  - `/car_state/frenet/pose` (PoseStamped): Pose in Frenet frame

**Usage**:
```bash
ros2 run frenet_odom_republisher frenet_odom_republisher_node
```

**Key Features**:
- Real-time coordinate transformation using FrenetConverter
- Velocity transformation to Frenet frame
- Closest waypoint index tracking (stored in child_frame_id)
- Automatic initialization when global waypoints are received

---

### 3. sector_tuner

**Purpose**: Dynamic velocity tuning for different track sectors through runtime parameter adjustment.

**Description**: Divides the racing track into configurable sectors and allows real-time adjustment of velocity scaling per sector. Provides interactive GUI tools for sector definition and parameter tuning.

**Maintainer**: kuehnej@ethz.ch

#### Nodes

##### sector_tuner
- **Topics Subscribed**:
  - `/global_waypoints` (WpntArray): Main trajectory waypoints
  - `/global_waypoints/shortest_path` (WpntArray): Alternative trajectory

- **Topics Published**:
  - `/global_waypoints_scaled` (WpntArray): Velocity-scaled waypoints
  - `/global_waypoints_scaled/shortest_path` (WpntArray): Scaled alternative trajectory
  - `/sector_markers` (MarkerArray): Sector boundary visualization

- **Dynamic Parameters**:
  - `n_sectors` (int): Number of track sectors
  - `global_limit` (double, range: 0.0-1.0): Global velocity limit
  - `Sector{N}.scaling` (double, range: 0.0-1.0): Per-sector velocity scaling
  - `Sector{N}.start` (int): Starting waypoint index
  - `Sector{N}.end` (int): Ending waypoint index

**Usage**:
```bash
ros2 run sector_tuner sector_tuner --ros-args \
  --params-file config/speed_scaling.yaml
```

##### sector_slicer
Interactive GUI tool for defining track sectors.

- **Topics Subscribed**:
  - `/global_waypoints` (WpntArray): Track waypoints
  - `/trackbounds/markers` (MarkerArray): Track boundaries

- **Parameters**:
  - `map_name` (string, default: "hangar_1905_v0"): Map identifier

**Usage**:
```bash
ros2 run sector_tuner sector_slicer --ros-args -p map_name:=your_map_name
```

**Interactive GUI**:
- Visualize track with boundaries
- Slider to navigate through waypoints
- Click "Select S" to mark sector boundaries
- Click "Done" to export configuration
- Automatically generates `speed_scaling.yaml` in map directory

##### ot_interpolator & ot_sector_slicer
Optimal trajectory interpolation and sector tools.

**Key Features**:
- Linear interpolation between adjacent sectors (10m transition zones)
- Real-time parameter updates via dynamic reconfigure
- Visualization of sector boundaries and scaling values
- YAML configuration export/import

---

### 4. lap_analyser

**Purpose**: Comprehensive lap time analysis and performance metrics tracking.

**Description**: Monitors vehicle performance during racing, computing lap times, lateral errors, and distance to track boundaries. Provides statistical analysis over multiple laps and logs data for post-race analysis.

**Maintainer**: nicolas.baumann@pbl.ee.ethz.ch

**License**: MIT

#### Node: lap_analyser

- **Topics Subscribed**:
  - `/car_state/frenet/odom` (Odometry): Vehicle position in Frenet frame
  - `/global_waypoints` (WpntArray): Reference trajectory
  - `/state_marker` (Marker): State machine visualization marker
  - `/lap_analyser/start` (Empty): Start/reset logging trigger

- **Topics Published**:
  - `lap_data` (LapData): Complete lap statistics (lap time, errors)
  - `min_car_distance_to_boundary` (Float32): Minimum track boundary distance
  - `lap_data_vis` (Marker): Lap data visualization in RViz

**Usage**:
```bash
ros2 run lap_analyser lap_analyser
```

**Start logging**:
```bash
ros2 topic pub /lap_analyser/start std_msgs/Empty
```

**Data Output**:
- **Log File**: `data/lap_analyser/lap_analyzer_DDMM_HHMM.txt`
  - Individual lap times
  - Statistics every N laps (mean, std)
  - Average and maximum lateral errors

**Metrics Tracked**:
- **Lap Time**: Time to complete one full lap
- **Average Lateral Error**: Mean deviation from global waypoints
- **Maximum Lateral Error**: Peak deviation during lap
- **Distance to Boundary**: Minimum clearance to track edges
- **Lap Count**: Sequential lap numbering

**Configuration**:
- `NUM_LAPS_ANALYSED` (default: 10): Number of laps for statistical analysis

**Key Features**:
- Automatic lap detection via Frenet s-coordinate wrapping
- Rolling statistics computation
- Real-time RViz visualization
- Persistent logging to timestamped files
- Optional map saving for SLAM validation

---

### 5. map_editor

**Purpose**: Interactive map editing tools for F1TENTH tracks.

**Description**: Provides launch files and RViz configurations for editing racing maps. Package structure supports map manipulation but requires external tools.

**Maintainer**: tialim@student.ethz.ch

**License**: MIT

**Contents**:
- Launch files for map editing workflows
- RViz configurations for visualization
- No direct executable nodes (tooling framework)

**Usage**:
```bash
ros2 launch map_editor <launch_file>.launch.py
```

**Dependencies**:
- f110_msgs
- stack_master
- global_planner

---

### 6. slam_tuner

**Purpose**: Offline SLAM parameter tuning using rosbag recordings.

**Description**: Facilitates iterative tuning of SLAM algorithms by replaying recorded data and computing localization accuracy metrics. Generates error plots and trajectory comparisons for parameter optimization.

**Maintainer**: bastuckn@student.ethz.ch

**License**: MIT

#### Main Components

##### reconstruction_error
**ROS1 Node** (note: uses rospy, not rclpy) for computing localization error metrics.

- **Topics Subscribed**:
  - `/car_state/pose` (PoseStamped): Ground truth pose
  - `/tracked_pose` (PoseStamped): SLAM-estimated pose
  - `/create_plot` (Empty): Trigger for plot generation

- **Parameters**:
  - `loc_algo` (string): Localization algorithm name
  - `timeout` (float, default: 15.0): Auto-plot timeout when stationary
  - `pose_topic` (string, default: "/car_state/pose"): Ground truth topic
  - `n_pose_msgs` (int, default: 20): Pose message averaging window

- **Output Files**:
  - `{LOC_ALGO}_err_metrics_TIMESTAMP.csv`: Raw error data
  - `{LOC_ALGO}_err_metrics_TIMESTAMP.png`: Error plots
  - `{LOC_ALGO}_err_metrics_TIMESTAMP_traj_hist.png`: Trajectory history
  - Multiple CSV files with TF transformations

**Metrics Computed**:
- **Position Error**: Euclidean distance between ground truth and estimate
- **Angular Error**: Absolute orientation difference
- **Mean Position Error**: Average over entire run
- **Trajectory Comparison**: Visual overlay of paths

**Usage Workflow**:
1. Record rosbag with ground truth and SLAM poses
2. Play back rosbag
3. Run reconstruction_error node
4. Trigger plot generation (automatic after timeout or manual via topic)

##### ekf_sim_launch.py
Launch file for Extended Kalman Filter (EKF) node in simulation.

**Configuration**:
- Uses `robot_localization` package
- Loads EKF parameters from `state_estimation/config/ekf.yaml`
- Remaps output to `/early_fusion/odom`
- Enables sim_time for rosbag playback

**Usage**:
```bash
ros2 launch slam_tuner ekf_sim_launch.py
```

**Package Contents**:
- Configuration files (*.lua for Cartographer)
- Launch files for SLAM tuning workflows
- RViz configurations for visualization
- Data directory for storing results

---

### 7. sombrero_visualization

**Purpose**: Placeholder package for visualization tools.

**Status**: Empty package (only .gitkeep file)

---

## Common Dependencies

All packages require:
- ROS2 Foxy
- Python 3.8+
- Standard ROS2 Python packages (rclpy)

Additional dependencies by package:
- **opponent_publisher**: f110_msgs, frenet_conversion, geometry_msgs, nav_msgs
- **frenet_odom_republisher**: f110_msgs, frenet_conversion
- **sector_tuner**: f110_msgs, stack_master (for parameter handling)
- **lap_analyser**: f110_msgs, ament_index_python
- **map_editor**: f110_msgs, stack_master, global_planner
- **slam_tuner**: rosbag2, state_estimation, matplotlib, scipy

## Build Instructions

```bash
# Build all packages
cd /path/to/f1tenth_dawgs
colcon build --packages-select opponent_publisher frenet_odom_republisher sector_tuner lap_analyser map_editor slam_tuner

# Build specific package
colcon build --packages-select <package_name>

# Source the workspace
source install/setup.bash
```

## Typical Workflow

### Race Performance Analysis
1. **Start lap analyzer**:
   ```bash
   ros2 run lap_analyser lap_analyser
   ```
2. **Begin logging**:
   ```bash
   ros2 topic pub /lap_analyser/start std_msgs/Empty
   ```
3. **Run racing stack** and complete laps
4. **Review logs** in `data/lap_analyser/`

### Velocity Tuning
1. **Define sectors** (first time):
   ```bash
   ros2 run sector_tuner sector_slicer --ros-args -p map_name:=your_map
   ```
2. **Launch sector tuner**:
   ```bash
   ros2 run sector_tuner sector_tuner --ros-args --params-file config/speed_scaling.yaml
   ```
3. **Adjust parameters** via dynamic reconfigure
4. **Test and iterate** velocity profiles

### Obstacle Avoidance Testing
1. **Start obstacle publisher**:
   ```bash
   ros2 run opponent_publisher obstacle_publisher --ros-args \
     -p speed_scaler:=0.8 -p trajectory:=centerline
   ```
2. **Start collision detector**:
   ```bash
   ros2 run opponent_publisher collision_detector
   ```
3. **Monitor** `/opponent_collision` and `/opponent_dist` topics
4. **Visualize** in RViz with marker displays

### SLAM Parameter Tuning
1. **Record rosbag** with sensor data
2. **Play back rosbag** with different SLAM parameters
3. **Run reconstruction_error** node to compute metrics
4. **Compare plots** to select optimal parameters
5. **Iterate** until desired accuracy achieved

## Visualization in RViz

Key visualization topics:
- `/dummy_obstacle_markers`: Obstacle positions
- `/sector_markers`: Sector boundaries and labels
- `/collision_marker`: Collision events
- `/lap_data_vis`: Lap time and count display
- `/trackbounds/markers`: Track boundary markers

## Troubleshooting

### frenet_odom_republisher not publishing
- Ensure `/global_waypoints` topic is being published
- Check that waypoint message contains valid x, y, psi data
- Verify FrenetConverter initialization

### sector_tuner not scaling velocities
- Confirm YAML parameter file is loaded correctly
- Check dynamic parameter updates with `ros2 param list`
- Verify `/global_waypoints` topic is active

### lap_analyser missing laps
- Ensure Frenet odometry is publishing on `/car_state/frenet/odom`
- Verify global waypoints are loaded
- Check s-coordinate wrapping threshold (default: 1.0m)

### obstacle_publisher no obstacles appearing
- For "lidar" type: ensure `/map` topic is publishing
- For "virtual" type: check `/perception/obstacles` subscribers
- Verify trajectory topic matches configuration

## Contact

For questions or issues with specific packages, contact the respective maintainers listed in each package section.

## Attribution

These tools were developed by the ForzaETH racing team at ETH Zurich for F1TENTH autonomous racing competitions.

-- super summary
● 완료했습니다! src/utilities/third_party/nodes/README.md 파일을 생성했습니다.

  분석 요약

  총 7개 패키지를 분석했습니다:

  주요 패키지들

  1. opponent_publisher - 동적 장애물 시뮬레이션 및 충돌 감지
    - 가상 또는 LiDAR 기반 장애물 생성
    - 실시간 충돌 감지
  2. frenet_odom_republisher - Frenet 좌표계 변환
    - Cartesian → Frenet 좌표 변환
    - 경로 계획에 필수적
  3. sector_tuner - 트랙 섹터별 속도 조정
    - GUI 기반 섹터 정의 도구
    - 실시간 속도 프로파일 조정
  4. lap_analyser - 랩 타임 및 성능 분석
    - 자동 랩 감지 및 시간 측정
    - 횡방향 오차 및 통계 분석
  5. map_editor - 맵 편집 도구 프레임워크
    - Launch 및 RViz 설정만 포함
  6. slam_tuner - SLAM 파라미터 오프라인 튜닝
    - Rosbag 재생을 통한 파라미터 최적화
    - 오차 메트릭 및 플롯 생성
  7. sombrero_visualization - 빈 패키지

  README에는 각 패키지의 다음 정보가 포함되어 있습니다:
  - 목적 및 설명
  - 노드별 토픽 구독/발행
  - 파라미터 설정
  - 사용 예시
  - 일반적인 워크플로우
  - 문제 해결 팁
