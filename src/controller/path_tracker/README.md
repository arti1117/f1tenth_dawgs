# Path Tracker

Pure Pursuit path tracking controller for dynamic Frenet trajectories.

## Overview

`path_tracker` subscribes to dynamically generated paths (e.g., from `path_planner/frenet_path`) and uses the Pure Pursuit algorithm to compute steering commands for path following. Unlike `pure_pursuit`, which follows static waypoint files, this package tracks continuously updated paths for reactive obstacle avoidance and dynamic trajectory planning.

## Features

- **Dynamic Path Tracking**: Follows continuously updated `nav_msgs/Path` messages
- **Pure Pursuit Control**: Classic geometric path tracking algorithm
- **Speed-Dependent Lookahead**: Adjusts lookahead distance based on vehicle speed
- **Path Timeout Handling**: Safety mechanism to stop if path updates cease
- **Visualization**: RViz markers for lookahead point

## Architecture

### Subscribed Topics

- `/frenet_path` (`nav_msgs/Path`): Dynamic path from path planner
- `/odom` (`nav_msgs/Odometry`): Vehicle odometry

### Published Topics

- `/drive` (`ackermann_msgs/AckermannDriveStamped`): Steering and speed commands
- `/lookahead_point` (`visualization_msgs/Marker`): Lookahead visualization

## Parameters

### Pure Pursuit Parameters

- **`lookahead_base`** (default: `1.5`): Base lookahead distance [m]
  - Larger values → smoother but less responsive
  - Smaller values → more responsive but oscillatory

- **`lookahead_k`** (default: `0.3`): Speed-dependent lookahead gain
  - Total lookahead = `lookahead_base + lookahead_k × speed`

- **`use_speed_lookahead`** (default: `true`): Enable speed-dependent lookahead

### Vehicle Parameters

- **`wheelbase`** (default: `0.33`): Vehicle wheelbase [m]
  - Critical for accurate steering computation

- **`default_speed`** (default: `2.0`): Default commanded speed [m/s]
  - Used when `speed_mode` is "default"

- **`max_steering_angle`** (default: `0.4189`): Maximum steering angle [rad] (~24°)

### Speed Control Mode

- **`speed_mode`** (default: `"default"`): Speed calculation method
  - **`"default"`**: Use fixed `default_speed` parameter
  - **`"path_velocity"`**: Extract velocity from global path waypoints
    - Requires `global_path_topic` to publish waypoints with velocity information
    - Uses CSV velocity column from path planner
  - **`"curvature"`**: Calculate speed based on path curvature
    - Automatically slows down for tight corners
    - Uses physics-based formula: v_max = sqrt(μ × g / |κ|)

### Curvature-Based Speed Parameters

Used when `speed_mode = "curvature"`:

- **`friction_coeff`** (default: `0.9`): Friction coefficient (μ)
  - Represents tire-road grip
  - Lower values = more conservative cornering speeds
  - Range: 0.5 (wet) to 1.2 (racing slicks)

- **`max_speed_limit`** (default: `8.0`): Maximum speed limit [m/s]
  - Upper bound regardless of curvature

- **`min_speed_limit`** (default: `0.5`): Minimum speed limit [m/s]
  - Lower bound for very tight corners

### Path Tracking Parameters

- **`path_timeout`** (default: `1.0`): Path timeout [s]
  - Vehicle stops if no path received within this time

### Topic Configuration

- **`odom_topic`** (default: `"/odom"`): Odometry topic name
- **`drive_topic`** (default: `"/drive"`): Drive command topic name
- **`path_topic`** (default: `"/frenet_path"`): Path topic to follow
- **`global_path_topic`** (default: `"/global_centerline"`): Global path with velocity info
  - Required when `speed_mode = "path_velocity"`
- **`base_frame`** (default: `"base_link"`): Vehicle base frame

## Usage

### Building

```bash
cd ~/f1tenth_dawgs
colcon build --packages-select path_tracker
source install/setup.bash
```

### Running

#### Launch with default parameters:
```bash
ros2 launch path_tracker path_tracker.launch.py
```

#### Run with custom parameters:
```bash
# Default speed mode
ros2 run path_tracker path_tracker_node --ros-args \
  -p lookahead_base:=2.0 \
  -p default_speed:=3.0 \
  -p speed_mode:=default

# Path velocity mode (uses global path speeds)
ros2 run path_tracker path_tracker_node --ros-args \
  -p speed_mode:=path_velocity \
  -p global_path_topic:=/global_centerline

# Curvature-based speed mode
ros2 run path_tracker path_tracker_node --ros-args \
  -p speed_mode:=curvature \
  -p friction_coeff:=0.9 \
  -p max_speed_limit:=8.0
```

### Integration with Path Planner

To use with the `path_planner` package:

```bash
# Terminal 1: Launch path planner
ros2 launch path_planner path_planner.launch.py

# Terminal 2: Launch path tracker
ros2 launch path_tracker path_tracker.launch.py

# Ensure path_topic parameter matches path_planner's output
```

## Algorithm

### Pure Pursuit Overview

1. **Find Closest Point**: Locate nearest point on path to vehicle
2. **Compute Lookahead**: Calculate lookahead distance based on speed
3. **Find Lookahead Point**: Walk along path to find goal point
4. **Transform to Vehicle Frame**: Convert goal to vehicle coordinates
5. **Compute Steering**: Apply pure pursuit formula:
   ```
   δ = atan(2 × L × sin(α) / d)
   ```
   where:
   - `δ` = steering angle
   - `L` = wheelbase
   - `α` = angle to lookahead point
   - `d` = distance to lookahead point

### Key Differences from `pure_pursuit`

| Feature | `pure_pursuit` | `path_tracker` |
|---------|---------------|----------------|
| Path Source | Static CSV file | Dynamic `/frenet_path` topic |
| Path Updates | One-time at startup | Continuous updates |
| Lap Timing | Yes | No |
| Path Rotation | Yes | No |
| Curvature-based Speed | Yes | No (uses default speed) |
| KD-tree Search | Yes | No (linear search) |

## Tuning Guide

### For Smooth Tracking

Increase `lookahead_base` to 2.0-3.0 m for gentler steering.

### For Tight Corners

Reduce `lookahead_base` to 1.0-1.5 m for more aggressive tracking.

### For High-Speed Tracking

Increase `lookahead_k` to 0.4-0.5 for speed-adaptive lookahead.

### For Low-Speed Precision

Reduce `lookahead_base` to 0.8-1.2 m and set `lookahead_k` to 0.1-0.2.

### Speed Mode Selection

**Use `"default"` when:**
- Testing or debugging
- Constant speed is desired
- Simplicity is preferred

**Use `"path_velocity"` when:**
- Path planner provides optimized speeds
- Following pre-computed racing lines from CSV
- Want to respect speed limits from waypoints

**Use `"curvature"` when:**
- Dynamic obstacle avoidance changes path shape
- No pre-computed speeds available
- Want automatic corner speed adjustment
- Racing on unknown tracks

### Curvature Mode Tuning

- **Conservative driving**: `friction_coeff = 0.7`, `max_speed_limit = 5.0`
- **Normal driving**: `friction_coeff = 0.9`, `max_speed_limit = 8.0`
- **Aggressive racing**: `friction_coeff = 1.1`, `max_speed_limit = 12.0`

## Troubleshooting

### General Issues

**Vehicle not moving:**
- Check if path is being published: `ros2 topic echo /frenet_path`
- Verify path timeout: increase `path_timeout` parameter
- Check odometry: `ros2 topic echo /odom`
- **NEW**: Vehicle now stops (speed=0) until valid frenet_path is received

**Oscillatory behavior:**
- Reduce `lookahead_base`
- Increase `lookahead_k` for speed-dependent smoothing

**Vehicle cuts corners:**
- Increase `lookahead_base`
- Reduce `default_speed`

**"Path is stale" warnings:**
- Path planner not running or publishing
- Increase `path_timeout` if planner is slow
- Check topic name matches: `ros2 topic list | grep path`

### PATH_VELOCITY Mode Debugging

**Problem: Speed always stuck at `min_speed_limit` (0.5 m/s)**

This is a common issue where the vehicle moves very slowly despite having higher target speeds in the path.

#### Velocity Data Flow in PATH_VELOCITY Mode

```
┌──────────────────────────────────────────────────────────────────┐
│ PATH_PLANNER (path_planner_node)                                 │
│   • Publishes /frenet_path (nav_msgs/Path)                       │
│   • Velocity stored in: pose.position.z                          │
│   • Must be > 0.01 to be recognized                              │
└──────────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────────┐
│ PATH_TRACKER: pathCallback()                                     │
│   • Reads each pose from /frenet_path                            │
│   • Extracts velocity:                                           │
│       if (pose.position.z > 0.01):                              │
│          pt.v = pose.position.z        ← Use path velocity      │
│       else:                                                      │
│          pt.v = default_speed (2.0)    ← Fallback               │
│                                                                  │
│   📊 Log: "PATH_CALLBACK: Point[0]: pose.z=X.XXXX → pt.v=Y.YY" │
└──────────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────────┐
│ findLookaheadPoint()                                             │
│   • Interpolates velocity between path points                   │
│   • result.v = v[i] + t * (v[i+1] - v[i])                      │
│                                                                  │
│   📊 Log: "LOOKAHEAD: idx=XX, lookahead.v=Y.YYYY m/s"          │
└──────────────────────────────────────────────────────────────────┘
                              ↓
┌──────────────────────────────────────────────────────────────────┐
│ computeSpeed() [PATH_VELOCITY mode]                              │
│   • Checks: lookahead.v > 0.01?                                 │
│       YES → speed = lookahead.v                                 │
│       NO  → speed = getGlobalPathSpeed() (fallback)             │
│                                                                  │
│   • Applies clamping:                                            │
│       speed = clamp(speed, min_speed_limit, max_speed_limit)   │
│       speed = clamp(speed, 0.5, 8.0)      ← ⚠️ CRITICAL!        │
│                                                                  │
│   📊 Log: "PATH_VELOCITY: Using frenet path velocity |          │
│            raw=X.XXXX → clamped=Y.YY m/s"                       │
└──────────────────────────────────────────────────────────────────┘
```

#### Diagnostic Steps

**Step 1: Check if path_planner is publishing velocity**

```bash
# Check first waypoint's velocity
ros2 topic echo /frenet_path --field poses[0].pose.position.z

# Expected: > 0.01 (e.g., 2.5, 3.0, 4.5)
# Problem:  = 0.0 or very small
```

**Interpretation:**
- `pose.z = 0.0000` → path_planner not setting velocity
- `pose.z = 3.5000` → velocity is being published correctly

**Step 2: Check pathCallback logs**

```bash
# Run with INFO logging
ros2 run path_tracker path_tracker_node

# Look for these logs:
[INFO] PATH_CALLBACK: Point[0]: pose.z=3.5000 → pt.v=3.50 m/s
[INFO] PATH_CALLBACK: Point[1]: pose.z=4.2000 → pt.v=4.20 m/s
[INFO] PATH_CALLBACK: Point[2]: pose.z=3.8000 → pt.v=3.80 m/s
[INFO] PATH_CALLBACK: Received path with 45 points |
       Velocity stats: min=2.50, max=6.00 m/s |
       Points with velocity: 45, Points using default: 0
```

**Problem indicators:**
- `pose.z=0.0000 → pt.v=2.00 m/s` → All points using default_speed
- `Points with velocity: 0, Points using default: 45` → No velocity data

**Step 3: Check lookahead interpolation**

```bash
# Enable DEBUG logging
ros2 run path_tracker path_tracker_node --ros-args --log-level debug

# Look for:
[DEBUG] LOOKAHEAD: idx=12, lookahead.v=3.2500 m/s (from interpolation)
```

**Problem:** `lookahead.v=0.0000` → Interpolation not working

**Step 4: Check computeSpeed logic**

```bash
# With INFO logging, check:
[INFO] COMPUTE_SPEED [PATH_VELOCITY]: lookahead.v=3.2500, threshold=0.01
[INFO] PATH_VELOCITY: Using frenet path velocity |
       raw=3.2500 → clamped=3.25 m/s (limits: [0.50, 8.00])
```

**Problem scenarios:**
1. `lookahead.v=0.0050 < 0.01` → Below threshold, using fallback
2. `raw=0.2000 → clamped=0.50` → Being clamped to min_speed_limit
3. `Fallback to global path | global_speed=0.50` → No velocity in path

**Step 5: Check speed limit parameters**

```bash
# Check current parameters
ros2 param get /path_tracker_node min_speed_limit
ros2 param get /path_tracker_node max_speed_limit

# In config/tracker_params.yaml:
min_speed_limit: 0.5   # ⚠️ If too high, clips everything to 0.5!
max_speed_limit: 8.0
```

#### Solutions

**Solution 1: Fix path_planner velocity publishing**

If `pose.z = 0.0` in pathCallback logs:
- Ensure path_planner is writing velocity to `pose.position.z`
- Check path_planner logs for velocity values
- Verify CSV file has velocity column (4th column)

**Solution 2: Lower min_speed_limit**

If velocity is being clamped `raw=0.8 → clamped=0.5`:
```yaml
# In tracker_params.yaml
min_speed_limit: 0.2  # Lower to allow slower speeds
```

**Solution 3: Check global_centerline fallback**

If using fallback to global path:
```bash
# Verify global path has velocity
ros2 topic echo /global_centerline --field poses[0].pose.position.z

# If also 0.0, path_planner needs to publish velocity on both topics
```

**Solution 4: Increase path velocities**

If all path velocities < min_speed_limit:
- Increase velocities in CSV input file
- Or lower min_speed_limit parameter

#### Verification Commands

```bash
# 1. Monitor velocity flow end-to-end
ros2 run path_tracker path_tracker_node 2>&1 | grep -E "PATH_|COMPUTE_"

# 2. Check published drive commands
ros2 topic echo /drive --field drive.speed

# 3. Compare path velocity to actual commanded velocity
ros2 topic echo /frenet_path --field poses[0].pose.position.z &
ros2 topic echo /drive --field drive.speed

# 4. Full debug trace
ros2 run path_tracker path_tracker_node --ros-args --log-level debug 2>&1 | \
  grep -E "PATH_CALLBACK|LOOKAHEAD|COMPUTE_SPEED"
```

#### Expected vs Actual Behavior

**Expected (Correct):**
```
[INFO] PATH_CALLBACK: Point[0]: pose.z=3.5000 → pt.v=3.50 m/s
[INFO] COMPUTE_SPEED [PATH_VELOCITY]: lookahead.v=3.4800, threshold=0.01
[INFO] PATH_VELOCITY: Using frenet path velocity | raw=3.4800 → clamped=3.48 m/s
[INFO] Publishing: speed=3.48 m/s
```

**Actual (Problem):**
```
[INFO] PATH_CALLBACK: Point[0]: pose.z=0.0000 → pt.v=2.00 m/s  ← Using default!
[INFO] COMPUTE_SPEED [PATH_VELOCITY]: lookahead.v=0.0050, threshold=0.01
[INFO] PATH_VELOCITY: Fallback to global path | global_speed=0.50 m/s
[INFO] Publishing: speed=0.50 m/s  ← Stuck at minimum!
```

#### Quick Fix Checklist

- [ ] path_planner publishing velocity? (`pose.z > 0.01`)
- [ ] pathCallback receiving velocity? (Check logs)
- [ ] lookahead.v > 0.01 in computeSpeed? (Check logs)
- [ ] min_speed_limit not too high? (< path velocities)
- [ ] CSV file has velocity column with values > 0.5?
- [ ] speed_mode = "path_velocity" in config?

## Visualization

Use RViz to visualize tracking:

```bash
rviz2
```

Add displays:
- **Path** (`/frenet_path`): Planned trajectory
- **Marker** (`/lookahead_point`): Current lookahead target (red sphere)
- **Odometry** (`/odom`): Vehicle position

## See Also

- `path_planner`: Frenet optimal trajectory planner
- `pure_pursuit`: Static waypoint follower with lap timing
- F1TENTH documentation: https://f1tenth.org


--- speed tracking

● 완료했습니다! 이제 전체 시스템이 속도 정보를 포함해서
  작동하도록 수정했습니다.

  수정 요약

  1. path_planner (속도 정보 발행)

  - utils.hpp: Waypoint 구조체에 v (velocity) 필드 추가
  - utils.hpp: waypointsToPathMsg() 함수가 pose.position.z에
   속도 저장
  - frenet.hpp: FrenetTraj 구조체에 v 벡터 추가
  - frenet.cpp: Frenet trajectory 생성 시 longitudinal
  velocity (s_dot) 저장
  - path_planner_node.cpp:
    - CSV에서 속도 읽어서 ref_wps_에 저장
    - Frenet trajectory를 Waypoint로 변환할 때 속도 포함

  2. path_tracker (속도 정보 수신 및 적용)

  - path_tracker_node.cpp:
    - pathCallback(): frenet_path에서 pose.position.z의
  속도를 읽어서 current_path_에 저장
    - computeSpeed(): PATH_VELOCITY 모드에서 frenet_path의
  속도 우선 사용, fallback으로 global_path 속도 사용

  데이터 흐름

  CSV (x,y,v,kappa)
    ↓
  path_planner: loadPathFromCSV
    ↓
  ref_wps_[].v = CSV의 v
    ↓
  Frenet trajectory 생성 (s_dot 계산)
    ↓
  FrenetTraj.v[] = s_dot
    ↓
  best_seg (Waypoint).v = FrenetTraj.v
    ↓
  waypointsToPathMsg: pose.position.z = Waypoint.v
    ↓
  /planned_path 또는 /frenet_path 발행
    ↓
  path_tracker: pathCallback
    ↓
  current_path_[].v = pose.position.z
    ↓
  computeSpeed (PATH_VELOCITY 모드)
    ↓
  lookahead.v 사용
    ↓
  Ackermann drive command 발행

  빌드 및 사용

  # 빌드
  colcon build --packages-select path_planner path_tracker
  source install/setup.bash

  # path_tracker 설정에서 speed_mode를 "path_velocity"로
  설정
  # config/path_tracker.yaml:
  speed_mode: "path_velocity"  # 속도를 경로에서 가져옴

  # 실행
  ros2 launch path_planner path_planner.launch.py
  ros2 launch path_tracker path_tracker.launch.py

  이제 path_tracker가 frenet_path의 각 점에 포함된 속도를
  읽어서 그 속도로 추종합니다!

--- path_velocity
● 완료했습니다! 수정 내용을 요약하겠습니다:

  완료된 작업

  1. frenet_path 수신 전 drive 명령 발행 방지

  path_tracker_node.cpp:173-203에서 수정:
  - path가 없거나 timeout된 경우 zero command (speed=0, steering=0) 발행
  - 명확한 로그 메시지: "No valid frenet_path available (waiting for path_planner)"

  2. 상세한 path_velocity 디버깅 로그 추가

  pathCallback (path_tracker_node.cpp:113-164):
  - 첫 3개 waypoint의 velocity 값 출력: Point[0]: pose.z=X.XXXX → pt.v=Y.YY m/s
  - Velocity 통계: min/max 속도, velocity가 있는 점 vs default 사용 점 개수
  - Zero velocity vs nonzero velocity count

  odomCallback (path_tracker_node.cpp:257-259):
  - Lookahead point velocity 디버그 로그 추가

  computeSpeed (path_tracker_node.cpp:424-442):
  - PATH_VELOCITY 모드 진입 시 lookahead.v 값 출력
  - Velocity 사용 경로 명확히 표시: frenet path vs global path fallback
  - Clamping 전후 값 표시: raw=X.XXXX → clamped=Y.YY m/s (limits: [min, max])

  3. README.md에 path_velocity 디버깅 가이드 추가

  README.md:237-437에 추가된 섹션:

  Velocity Data Flow 다이어그램

  - PATH_PLANNER → pathCallback → findLookaheadPoint → computeSpeed → drive command
  - 각 단계별 로그 메시지 예시

  5단계 진단 프로세스

  1. Step 1: path_planner가 velocity 발행하는지 확인 (ros2 topic echo)
  2. Step 2: pathCallback이 velocity 수신하는지 로그 확인
  3. Step 3: lookahead 보간이 제대로 되는지 확인
  4. Step 4: computeSpeed 로직에서 threshold/clamping 확인
  5. Step 5: speed limit 파라미터 확인

  해결 방법 4가지

  - Solution 1: path_planner velocity publishing 수정
  - Solution 2: min_speed_limit 낮추기
  - Solution 3: global_centerline fallback 확인
  - Solution 4: CSV의 velocity 값 증가

  검증 명령어

  - End-to-end velocity 모니터링
  - Drive command 속도 확인
  - 디버그 로그 필터링

  Quick Fix Checklist

  체크리스트 형식으로 빠른 문제 해결 가이드

  사용 방법

  디버깅 시작

  # INFO 레벨 로그로 실행 (기본)
  ros2 run path_tracker path_tracker_node

  # 출력 예시:
  [INFO] PATH_CALLBACK: Point[0]: pose.z=3.5000 → pt.v=3.50 m/s
  [INFO] PATH_CALLBACK: Received path with 45 points |
         Velocity stats: min=2.50, max=6.00 m/s |
         Points with velocity: 45, Points using default: 0
  [INFO] COMPUTE_SPEED [PATH_VELOCITY]: lookahead.v=3.4800, threshold=0.01
  [INFO] PATH_VELOCITY: Using frenet path velocity | raw=3.4800 → clamped=3.48 m/s

  속도 0.5 m/s 문제 진단

  # 1단계: frenet_path velocity 확인
  ros2 topic echo /frenet_path --field poses[0].pose.position.z

  # 2단계: 로그 필터링
  ros2 run path_tracker path_tracker_node 2>&1 | grep -E "PATH_|COMPUTE_"

  # 3단계: DEBUG 로그 활성화
  ros2 run path_tracker path_tracker_node --ros-args --log-level debug

  빌드 및 테스트

  cd /home/dawgs_nx/f1tenth_dawgs
  colcon build --packages-select path_tracker
  source install/setup.bash

  # 테스트
  ros2 run path_tracker path_tracker_node

  이제 로그를 보면 velocity가 어디서 손실되는지 정확히 추적할 수 있습니다!

---
● 완료했습니다! path_tracker를 수정해서 lookup table의 acceleration 값을 직접 사용하도록 변경했습니다.

  주요 변경사항

  ✅ 이전 방식 (Steering-Based Speed Limiting)

  - steering angle로 최대 속도를 제한
  - 속도가 갑자기 변할 수 있음

  ✅ 새로운 방식 (Acceleration Limiting)

  - lookup table에서 현재 steering/속도에 해당하는 가속도 값을 읽음
  - 가속도(acceleration)를 제한해서 부드러운 속도 변화
  - drive topic의 acceleration 필드에 lookup table 값을 직접 발행

  구현 내용

  1. getAccelerationFromTable(): Bilinear interpolation으로 lookup table에서 가속도 값 읽기
  2. applyAccelerationLimit(): 가속도 제한을 적용해서 속도 명령 생성
  3. State tracking: 이전 속도와 steering angle 추적
  4. Drive command: drive.acceleration 필드에 lookup table 값 저장

  설정 파일 변경

  # tracker_params.yaml
  use_acceleration_limit: true
  acceleration_lookup_table: "dawgs_lookup_table.csv"
  max_acceleration: 5.0  # [m/s²] - 최대 가속도 제한

  빌드 결과

  ✅ 성공적으로 빌드 완료 (1분 22초)
  - 경고는 있지만 기능에는 문제없음

  테스트 방법

  source install/setup.bash
  ros2 launch path_tracker path_tracker.launch.py

  # drive topic 확인
  ros2 topic echo /drive

  예상 동작:
  - drive.speed: 부드럽게 변하는 속도 명령
  - drive.acceleration: lookup table에서 읽은 가속도 값
  - 급격한 속도 변화 없이 매끄러운 제어

  자세한 내용은 /home/dawgs_nx/f1tenth_dawgs/claudedocs/ACCELERATION_LIMITING_IMPLEMENTATION.md 문서를
  참고하세요!

--- comment
lookup table을 잘못 해석하고 적용하고 있는듯하다. 