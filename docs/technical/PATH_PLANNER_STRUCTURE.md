# Path Planner 구조 및 토픽 가이드

**작성일**: 2025-10-22
**패키지**: `path_planner`
**목적**: Frenet optimal trajectory planning with obstacle avoidance

---

## 📊 Path Planner 아키텍처

### 전체 구조

```
┌─────────────────────────────────────────────────────────────┐
│                   Path Planner Node                          │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  ┌─────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │   CSV Load  │───>│Frenet Planner│───>│ Lattice LUT   │  │
│  │  (Global    │    │ (Trajectory  │    │ (Spiral Path  │  │
│  │  Waypoints) │    │  Generation) │    │  Smoothing)   │  │
│  └─────────────┘    └──────────────┘    └───────────────┘  │
│         │                   │                     │          │
│         v                   v                     v          │
│  ┌─────────────────────────────────────────────────────┐    │
│  │          planPath() - Main Planning Loop            │    │
│  │  1. Odometry → Frenet conversion                    │    │
│  │  2. Obstacle detection from /scan                   │    │
│  │  3. Generate trajectory candidates (Frenet lattice) │    │
│  │  4. Collision check & cost evaluation               │    │
│  │  5. Select best trajectory                          │    │
│  │  6. Spiral smoothing (Lattice LUT)                  │    │
│  │  7. Publish planned path                            │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### 주요 컴포넌트

#### 1. **Frenet Planner** (`frenet.hpp`, `frenet.cpp`)
- **역할**: Frenet 좌표계 기반 궤적 생성
- **입력**:
  - `FrenetState` (s, d, ds, dd, ddd)
  - Obstacles (x, y pairs)
- **출력**:
  - 다수의 `FrenetTraj` 후보 경로들
- **샘플링 전략**:
  - **Lateral (d)**: 9개 샘플 (-1.0 ~ +1.0 m, centerline 기준)
  - **Time (T)**: 5개 샘플 (1.5 ~ 3.0 초)
  - **총 후보 수**: 45개 trajectory (9 lateral × 5 time)

#### 2. **Lattice LUT** (`lattice_lut.hpp`, `lattice_lut.cpp`)
- **역할**: Spiral trajectory 기반 goal alignment
- **목적**: Frenet path와 global centerline을 smooth하게 연결
- **입력**:
  - Relative goal position (dx, dy, dθ)
  - Current Frenet state (s, d)
- **출력**:
  - Spiral waypoints (curvature-aware path)

#### 3. **Collision Checker**
- **위치**: `frenet.cpp:485-497`
- **방식**:
  - Point-to-point distance check
  - Safety radius 내 장애물 검출
  - Road boundary check (±road_half_width)

---

## 📡 발행 토픽 (Published Topics)

### 1. `/planned_path` (nav_msgs/Path)
**목적**: 최종 계획된 경로 (path_tracker가 구독)

**내용**:
- Frenet lattice + Lattice LUT 결합 경로
- 장애물 회피가 적용된 최적 경로
- Velocity 정보 포함 (pose.position.z에 저장)

**특징**:
- **QoS**: Reliable, KeepLast(10)
- **발행 빈도**: Odometry 수신 시마다 (약 20-50 Hz)
- **좌표계**: `map` frame

**사용처**:
- `path_tracker` 노드가 구독하여 경로 추종

---

### 2. `/global_centerline` (nav_msgs/Path)
**목적**: CSV에서 로드한 글로벌 기준선

**내용**:
- CSV waypoints (x, y, v, kappa)
- Track centerline 정보
- Speed profile 포함

**특징**:
- **QoS**: Reliable, KeepLast(10)
- **발행 빈도**: 1 Hz (timer)
- **좌표계**: `map` frame

**사용처**:
- 시각화
- `path_tracker`의 velocity reference (speed_mode=path_velocity)
- Frenet 좌표계의 기준선

---

### 3. `/frenet_path` (nav_msgs/Path) [시각화]
**목적**: Frenet lattice 샘플링 결과 시각화

**내용**:
- 선택된 Frenet trajectory
- 장애물 회피 경로
- 45개 후보 중 최적 경로

**특징**:
- **QoS**: Reliable, KeepLast(10)
- **발행 조건**: `visualize_paths: true`
- **elevation**: z=0.1 (centerline과 구분)

**시각화 색상**:
- Rviz에서 Path display로 확인

---

### 4. `/lut_path` (nav_msgs/Path) [시각화]
**목적**: Lattice LUT spiral 경로 시각화

**내용**:
- Spiral trajectory
- Goal alignment path
- Curvature-aware smoothing

**특징**:
- **QoS**: Reliable, KeepLast(10)
- **발행 조건**: `visualize_paths: true`
- **elevation**: z=0.15 (frenet_path와 구분)

---

### 5. `/path_planner_markers` (visualization_msgs/MarkerArray) [시각화]
**목적**: 다수의 lattice 후보 경로들 시각화

**내용**:
- Lattice LUT의 여러 lateral offset 후보들
- 5개 lateral 샘플 (-1.0 ~ +1.0 m)
- Center path가 최적 경로

**특징**:
- **QoS**: Best Effort, KeepLast(1)
- **발행 조건**: `visualize_paths: true`
- **Line width**: 0.05 m

**색상 코딩**:
- **녹색 (Green)**: Center path (선택된 경로)
- **Gradient**: 외곽으로 갈수록 색상 변화

---

### 6. `/global_path_velocity_markers` (visualization_msgs/MarkerArray) [시각화]
**목적**: Global centerline의 속도 프로파일 시각화

**내용**:
- 속도에 따른 색상 그라디언트
- CSV의 v 값 시각화

**특징**:
- **QoS**: Best Effort, KeepLast(1)
- **발행 빈도**: 1 Hz
- **Line width**: 0.15 m

**색상 코딩**:
- **파란색 (Blue)**: 저속 구간
- **녹색 (Green)**: 중속 구간
- **빨간색 (Red)**: 고속 구간

---

## 🔄 구독 토픽 (Subscribed Topics)

### 1. `/odom` (nav_msgs/Odometry)
- **QoS**: Best Effort, KeepLast(5)
- **역할**: 차량 위치/속도 정보
- **콜백**: `odomCallback()` → `planPath()` 트리거

### 2. `/scan` (sensor_msgs/LaserScan)
- **QoS**: Best Effort, KeepLast(5)
- **역할**: LiDAR 장애물 검출
- **변환**: Scan points → Map frame obstacles

---

## ⚙️ 주요 파라미터

### Frenet 샘플링 파라미터
```yaml
frenet_time_horizon: 3.0          # 최대 예측 시간 [s]
frenet_min_time: 1.0              # 최소 예측 시간 [s]
frenet_target_speed: 3.0          # 목표 속도 [m/s]
frenet_dt: 0.05                   # 샘플링 간격 [s]
frenet_max_speed: 15.0            # 최대 허용 속도 [m/s]
frenet_max_accel: 4.0             # 최대 가속도 [m/s²]

# Lateral 샘플링 (centerline 기준 offset)
frenet_d_samples: [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]

# Time horizon 샘플링
frenet_t_samples: [1.5, 2.0, 2.5, 3.0]
```

### 비용 함수 가중치
```yaml
frenet_k_jerk: 0.1                # Jerk (lateral smoothness) 가중치
frenet_k_time: 0.1                # Time (빠른 경로 선호) 가중치
frenet_k_deviation: 1.0           # Centerline deviation 가중치
frenet_k_velocity: 1.0            # 속도 오차 가중치
```

### 안전 파라미터
```yaml
frenet_safety_radius: 0.3         # 장애물 safety distance [m]
frenet_road_half_width: 1.2       # 도로 폭 절반 [m] (±1.2m 범위)
```

### 기타 파라미터
```yaml
use_lattice: true                 # Lattice LUT 사용 여부
use_frenet: true                  # Frenet planner 사용 여부
visualize_paths: true             # 경로 시각화 여부
planner_horizon: 3.0              # Planning horizon [m]
log_level: 1                      # 로그 레벨 (0=NONE ~ 5=VERBOSE)
```

---

## 🔍 Planning 알고리즘 상세

### 1단계: Position Compensation
```cpp
// Message delay + computation time 보상
double total_delay = message_delay + expected_computation_time;
x_compensated = x_current + v * cos(yaw) * total_delay;
y_compensated = y_current + v * sin(yaw) * total_delay;
```

### 2단계: Frenet 좌표 변환
```cpp
// Cartesian (x, y) → Frenet (s, d)
cart2frenet(x_compensated, y_compensated, idx, fs);
fs.ds = v;  // Longitudinal velocity
```

### 3단계: 장애물 검출
```cpp
// LiDAR scan → Map frame obstacles
for (size_t i = 0; i < scan.ranges.size(); ++i) {
    // Range filtering (< 5.0 m)
    // Transform to map frame
    // Add to obstacle list
}
```

### 4단계: Trajectory 생성 (Frenet Lattice)
```cpp
// 45개 후보 생성 (9 lateral × 5 time)
for (T in t_samples) {
    for (df in d_samples) {
        // Quintic polynomial (lateral)
        d(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵

        // Quartic polynomial (longitudinal)
        s(t) = b0 + b1*t + b2*t² + b3*t³ + b4*t⁴

        // Sample at dt intervals
        // Convert Frenet → Cartesian
        // Check collision
        // Calculate cost
    }
}
```

### 5단계: Collision Check
```cpp
for each trajectory point:
    for each obstacle:
        if distance(point, obstacle) < safety_radius:
            collision = true
    if |d| > road_half_width:
        collision = true
```

### 6단계: 비용 계산
```cpp
cost = k_j * jerk + k_t * time + k_d * deviation + k_v * velocity_error
```
- **Jerk**: Lateral smoothness (2차 미분)
- **Time**: 빠른 경로 선호 (낮은 T)
- **Deviation**: Centerline 근접도
- **Velocity error**: 목표 속도 추종 오차

### 7단계: Lattice LUT Smoothing
```cpp
// Frenet path 끝점을 goal로 사용
dx = goal.x - current.x;
dy = goal.y - current.y;
dth = goal.yaw - current.yaw;

// Spiral trajectory lookup
spiral_path = lattice_lut.query(dx, dy, dth);

// 최종 경로 = Frenet path + Spiral path
```

---

## 🛡️ Safety Distance 현재 구현

### 현재 로직 (frenet.cpp:485-497)

```cpp
// Collision check
for (size_t i = 0; i < tr.x.size(); ++i) {
    for (const auto &ob : obstacles) {
        if (distance(tr.x[i], tr.y[i], ob.first, ob.second) < p_.safety_radius) {
            tr.collision = true;
            break;
        }
    }
    if (std::abs(tr.d[i]) > p_.road_half_width) {
        tr.collision = true;
        break;
    }
}
```

### 문제점

1. **고정된 Safety Radius**
   - 속도 무관하게 0.3m 고정
   - 고속 주행 시 반응 시간 부족

2. **Point-to-Point Check**
   - 궤적 샘플 포인트만 체크
   - 포인트 사이 장애물 놓칠 가능성

3. **장애물 크기 미고려**
   - 장애물을 점으로 가정
   - 실제 장애물 반경 무시

4. **정적 체크만 수행**
   - 시간 기반 충돌 예측 없음
   - 동적 장애물 대응 불가

5. **Proximity Cost 없음**
   - 충돌/비충돌 이진 판단만
   - 안전 margin이 큰 경로 우대 안 됨

---

## 🔧 개선 방안

다음 섹션에서 구현될 개선사항:

### 1. **속도 기반 Safety Margin**
```
safety_margin = base_radius + k_velocity * velocity
```

### 2. **장애물 반경 파라미터**
```
effective_radius = vehicle_radius + obstacle_radius + safety_buffer
```

### 3. **중간 포인트 체크**
```
# 샘플 포인트 사이도 체크
for i in range(len(path)-1):
    interpolate_points(path[i], path[i+1], num_checks=5)
```

### 4. **Proximity Cost 추가**
```
proximity_cost = Σ (1 / (dist_to_obstacle - safety_radius))
total_cost = base_cost + k_proximity * proximity_cost
```

### 5. **Conservative Multi-point Check**
```
# 차량 footprint 여러 점 체크
check_points = [front, rear, left, right, center]
```

---

## 📈 성능 특성

### 계산 시간
- **Frenet 생성**: ~2-5 ms (45개 후보)
- **Collision check**: ~1-2 ms (장애물 수 의존)
- **Lattice LUT**: ~0.5-1 ms
- **Total**: ~5-10 ms per cycle

### 메모리 사용
- **Reference waypoints**: ~수백 KB (1000 포인트 기준)
- **KD-tree**: ~수백 KB
- **Trajectory candidates**: ~수십 KB (45개 × 60 포인트)

---

## 🎯 사용 예제

### ROS2 Topic Echo
```bash
# 최종 계획 경로 확인
ros2 topic echo /planned_path

# 장애물 스캔 확인
ros2 topic echo /scan

# 속도 프로파일 확인
ros2 topic echo /global_centerline
```

### Rviz 시각화
```bash
rviz2

# Add displays:
# - Path: /planned_path (빨간색)
# - Path: /global_centerline (파란색)
# - Path: /frenet_path (녹색)
# - MarkerArray: /path_planner_markers
# - MarkerArray: /global_path_velocity_markers
# - LaserScan: /scan
```

### 파라미터 튜닝
```bash
# Safety radius 증가
ros2 param set /path_planner frenet_safety_radius 0.5

# 더 많은 lateral 샘플
ros2 param set /path_planner frenet_d_samples "[-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]"

# 로그 레벨 증가 (디버깅)
ros2 param set /path_planner log_level 4  # DEBUG
```

---

## 🚨 주의사항

1. **CSV 파일 형식**: `x, y, v, kappa` (4열 필수)
2. **Closed loop 판단**: 첫/마지막 점 거리 < 2m
3. **Odometry 필수**: (0, 0) 위치는 무시됨
4. **LiDAR 범위**: 5m 이내 장애물만 고려
5. **QoS 호환성**: path_tracker와 QoS 설정 일치 필요

---

**다음 단계**: Safety distance 강건화 구현
