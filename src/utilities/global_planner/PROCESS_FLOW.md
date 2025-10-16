# Global Planner - 온라인/오프라인 프로세스 플로우

## 개요

Global Planner는 두 가지 모드로 작동합니다:

1. **온라인 모드** (Online Mode): ROS 토픽을 통해 실시간으로 맵과 차량 위치를 받아 trajectory 생성
2. **오프라인 모드** (Offline Mode): 기존 PNG/YAML 파일에서 직접 trajectory 생성 (ROS 토픽 불필요)

---

## 온라인 모드 (Online Mode)

### 목적
실시간 매핑 중에 차량이 트랙을 주행하면서 맵을 생성하고 최적 경로를 계산합니다.

### 필요한 ROS 토픽

#### 입력 토픽
1. **`/map`** (nav_msgs/OccupancyGrid)
   - Cartographer SLAM에서 실시간으로 생성되는 occupancy grid 맵
   - 맵의 해상도, 원점, 크기 정보 포함
   - 각 셀의 점유 확률 (-1: 미지, 0: 자유, 100: 점유)

2. **`/car_state/pose`** (geometry_msgs/PoseStamped)
   - 차량의 현재 위치 (x, y, θ)
   - 맵 프레임 기준 좌표
   - 랩 카운트 추적 및 초기 위치 설정에 사용

#### 출력 토픽
- `/global_waypoints` - IQP 최적화 경로
- `/global_waypoints/shortest_path` - Shortest path 경로 (오버테이킹용)
- `/centerline_waypoints` - 중심선 경로
- `/global_waypoints/markers` - RVIZ 시각화
- `/trackbounds/markers` - 트랙 경계 시각화
- `/map_infos` - 맵 정보 (랩타임 등)
- `/estimated_lap_time` - 예상 랩타임

### 상세 프로세스 플로우

```
[1] 시작 및 초기화
    │
    ├─> GlobalPlanner 노드 생성
    ├─> 파라미터 로드 (create_map=True, required_laps 등)
    ├─> ROS 토픽 구독 설정
    │   ├─> /map 구독
    │   └─> /car_state/pose 구독
    └─> 타이머 콜백 시작 (global_plan_callback)

[2] 데이터 수집 (실시간)
    │
    ├─> map_cb() - Occupancy Grid 수신
    │   ├─> 맵 데이터 저장
    │   │   ├─> map_width, map_height
    │   │   ├─> map_resolution (예: 0.05 m/cell)
    │   │   ├─> map_origin (x, y)
    │   │   └─> map_occupancy_grid (배열)
    │   └─> map_valid = True
    │
    └─> pose_cb() - 차량 위치 수신
        ├─> 현재 위치 업데이트 (x, y, θ)
        ├─> 초기 위치 저장 (첫 수신 시)
        ├─> 주행 경로 기록 (cent_driven)
        └─> pose_valid = True

[3] 랩 완료 감지
    │
    ├─> update_lap_count()
    │   ├─> at_init_pos_check()
    │   │   ├─> 현재 위치 vs 초기 위치 비교
    │   │   │   ├─> x_diff < 0.5m
    │   │   │   ├─> y_diff < 0.5m
    │   │   │   └─> theta_diff < π/2
    │   │   └─> 조건 만족 시 True 반환
    │   │
    │   └─> 랩 카운트 증가
    │       └─> lap_count++
    │
    └─> required_laps에 도달하면 다음 단계 진행

[4] 맵 필터링 및 저장
    │
    ├─> filter_map_occupancy_grid()
    │   ├─> Occupancy grid → binary image
    │   │   ├─> Threshold: 50 (기본값)
    │   │   │   ├─> < 50: 자유 공간 (255)
    │   │   │   └─> >= 50: 점유 공간 (0)
    │   │   └─> Unknown (-1) → 점유로 처리
    │   │
    │   ├─> Morphological opening
    │   │   ├─> 노이즈 제거
    │   │   ├─> Kernel size: filter_kernel_size (예: 3x3)
    │   │   └─> Iterations: 2
    │   │
    │   └─> Filtered binary map 반환
    │
    ├─> 사용자 확인 (GUI)
    │   ├─> Matplotlib 창 표시
    │   ├─> "Map ready; compute global trajectory" 버튼
    │   └─> 버튼 클릭 시 다음 단계 진행
    │
    └─> save_map_png()
        ├─> 맵 디렉토리 생성
        │   └─> maps/<map_name>/
        │
        ├─> PNG 저장
        │   ├─> 이미지 flip (OpenCV 좌표계 → 맵 좌표계)
        │   └─> <map_name>.png
        │
        ├─> YAML 저장
        │   ├─> image: <map_name>.png
        │   ├─> resolution: 0.05 (예시)
        │   ├─> origin: [x, y, 0]
        │   └─> occupancy thresholds
        │
        └─> Particle filter 맵 복사
            └─> pf_map.png, pf_map.yaml

[5] 중심선 추출 (Centerline Extraction)
    │
    ├─> Skeletonization (Lee 알고리즘)
    │   ├─> Binary map → skeleton
    │   ├─> 트랙을 1픽셀 두께 선으로 변환
    │   └─> 트랙의 중심을 나타냄
    │
    ├─> extract_centerline()
    │   ├─> 닫힌 윤곽선(contour) 찾기
    │   │   ├─> OpenCV findContours
    │   │   ├─> 주행한 경로 길이와 비교
    │   │   │   └─> cent_length ± 20% 오차 허용
    │   │   └─> 가장 긴 닫힌 윤곽선 선택
    │   │
    │   └─> Centerline 좌표 추출
    │       └─> 셀 좌표로 저장
    │
    ├─> smooth_centerline()
    │   ├─> Savitzky-Golay 필터
    │   ├─> 노이즈 제거
    │   └─> 부드러운 곡선 생성
    │
    ├─> 좌표 변환 (셀 → 미터)
    │   ├─> x_meter = x_cell * resolution + origin_x
    │   └─> y_meter = y_cell * resolution + origin_y
    │
    ├─> 보간 (Interpolation)
    │   ├─> 0.1m 간격으로 재샘플링
    │   └─> 균일한 waypoint 간격
    │
    ├─> 방향 확인 및 조정
    │   ├─> 초기 차량 위치에서 가장 가까운 점 찾기
    │   ├─> 중심선 방향 vs 차량 방향 비교
    │   │   └─> compare_direction(cent_direction, car_direction)
    │   ├─> 방향이 반대면 centerline 뒤집기
    │   └─> reverse_mapping=True면 다시 뒤집기
    │
    └─> Centerline 출력
        └─> [x, y] 배열 (미터 단위)

[6] 트랙 경계 추출 (Track Bounds Extraction)
    │
    ├─> Watershed 알고리즘 (기본)
    │   ├─> extract_track_bounds()
    │   ├─> 중심선 기준 좌/우 경계 분리
    │   │   ├─> 각 중심선 점에서 수직 방향 탐색
    │   │   ├─> 좌측 경계 (bound_l)
    │   │   └─> 우측 경계 (bound_r)
    │   │
    │   └─> 실패 시 (2개 이상 경계 감지)
    │       └─> Distance transform으로 폴백
    │
    └─> Distance Transform (폴백)
        ├─> cv2.distanceTransform()
        ├─> 각 점에서 가장 가까운 장애물까지 거리
        └─> 중심선 각 점에서 좌우 거리 계산

[7] Trajectory 최적화 #1: IQP (Iterative Quadratic Programming)
    │
    ├─> add_dist_to_cent()
    │   ├─> 중심선 각 점에 트랙 경계까지 거리 추가
    │   │   ├─> d_right: 우측 경계까지 거리
    │   │   └─> d_left: 좌측 경계까지 거리
    │   │
    │   ├─> Safety width 적용
    │   │   ├─> d_right -= safety_width
    │   │   └─> d_left -= safety_width
    │   │
    │   └─> CSV 파일로 저장
    │       └─> ~/.ros/map_centerline.csv
    │
    ├─> trajectory_optimizer()
    │   ├─> 입력: 중심선 + 트랙 경계 + 안전 여유
    │   │
    │   ├─> 목적 함수 (Objective Function)
    │   │   ├─> Minimize curvature (곡률 최소화)
    │   │   │   └─> 부드러운 경로 생성
    │   │   └─> Subject to constraints:
    │   │       ├─> 트랙 경계 내부 유지
    │   │       ├─> Safety width 준수
    │   │       └─> 연속성 제약
    │   │
    │   ├─> 속도 프로파일 생성
    │   │   ├─> Velocity optimization
    │   │   ├─> 최대 가속도/감속도 제약
    │   │   └─> 곡률 기반 속도 조정
    │   │
    │   └─> 출력
    │       ├─> global_trajectory_iqp
    │       │   └─> [s, x, y, psi, vx, ax]
    │       ├─> bound_r_iqp, bound_l_iqp
    │       └─> est_t_iqp (예상 랩타임)
    │
    └─> dist_to_bounds()
        ├─> 최적화된 경로 각 점에서 경계까지 거리 계산
        └─> Waypoint 메시지에 포함

[8] Trajectory 최적화 #2: Shortest Path (Overtaking용)
    │
    ├─> IQP 경로를 새 중심선으로 사용
    │   ├─> 더 작은 safety_width_sp 적용
    │   └─> 더 공격적인 경로 가능
    │
    ├─> add_dist_to_cent() (safety_width_sp)
    │   └─> ~/.ros/map_centerline_2.csv
    │
    ├─> trajectory_optimizer(curv_opt_type='shortest_path')
    │   ├─> Shortest path 알고리즘
    │   ├─> 더 짧은 경로, 더 높은 속도
    │   └─> 오버테이킹 시 사용
    │
    └─> 출력
        ├─> global_trajectory_sp
        ├─> bound_r_sp, bound_l_sp
        └─> est_t_sp (예상 랩타임)

[9] Waypoint 생성 및 저장
    │
    ├─> create_wpnts_markers()
    │   ├─> Trajectory 배열 → WpntArray 메시지
    │   │   └─> 각 waypoint:
    │   │       ├─> s_m: 누적 거리
    │   │       ├─> x_m, y_m: 위치
    │   │       ├─> psi_rad: 헤딩 각도
    │   │       ├─> kappa_radpm: 곡률
    │   │       ├─> vx_mps: 속도
    │   │       ├─> ax_mps2: 가속도
    │   │       ├─> d_right: 우측 경계 거리
    │   │       └─> d_left: 좌측 경계 거리
    │   │
    │   └─> RVIZ 마커 생성
    │       ├─> LINE_STRIP (경로 선)
    │       ├─> 속도 기반 색상 (낮음=파랑, 높음=빨강)
    │       └─> ARROW (방향 화살표)
    │
    └─> write_global_waypoints()
        ├─> JSON 파일 생성
        │   └─> <map_dir>/global_waypoints.json
        │
        └─> 저장 내용:
            ├─> map_infos (랩타임, 최고속도)
            ├─> centerline_wpnts
            ├─> centerline_markers
            ├─> glb_wpnts (IQP)
            ├─> glb_markers
            ├─> glb_sp_wpnts (Shortest Path)
            ├─> glb_sp_markers
            └─> track_bounds

[10] 퍼블리싱 및 종료
    │
    ├─> read_and_publish()
    │   ├─> JSON 파일 읽기
    │   └─> ROS 토픽으로 퍼블리시
    │       ├─> /global_waypoints
    │       ├─> /global_waypoints/shortest_path
    │       ├─> /centerline_waypoints
    │       ├─> /global_waypoints/markers
    │       ├─> /centerline_waypoints/markers
    │       ├─> /trackbounds/markers
    │       ├─> /map_infos
    │       └─> /estimated_lap_time
    │
    └─> 노드 종료
        └─> destroy_node()
```

### 핵심 데이터 구조

#### Occupancy Grid (입력)
```python
map_occupancy_grid = [-1, -1, 0, 0, 0, 100, 100, ...]
# -1: 미지 (unknown)
# 0: 자유 공간 (free)
# 100: 점유 (occupied)

map_width = 1000  # cells
map_height = 800  # cells
map_resolution = 0.05  # meters/cell
map_origin = Point(x=-25.0, y=-20.0)
```

#### Filtered Map (중간)
```python
filtered_map = [[255, 255, 0, 0, ...],  # Binary image
                [255, 255, 0, 0, ...],
                ...]
# 255: 자유 공간 (흰색)
# 0: 점유 공간 (검정)
```

#### Centerline (중간)
```python
centerline = [[x1, y1],
              [x2, y2],
              ...,
              [xn, yn]]
# 미터 단위, 0.1m 간격
```

#### Global Trajectory (출력)
```python
global_trajectory = [[s1, x1, y1, psi1, vx1, ax1],
                     [s2, x2, y2, psi2, vx2, ax2],
                     ...]
# s: 누적 거리 (m)
# x, y: 위치 (m)
# psi: 헤딩 (rad)
# vx: 속도 (m/s)
# ax: 가속도 (m/s²)
```

---

## 오프라인 모드 (Offline Mode)

### 목적
기존 PNG/YAML 맵 파일에서 직접 trajectory를 생성합니다. ROS 토픽이 필요 없으므로 언제든지 재생성 가능합니다.

### 필요한 파일

#### 입력 파일
1. **`<map_name>.png`**
   - Binary map 이미지
   - 흰색(255): 자유 공간
   - 검정(0): 점유 공간
   - 이미 필터링된 상태

2. **`<map_name>.yaml`**
   - 맵 메타데이터
   ```yaml
   image: hangar_1905_v0.png
   resolution: 0.05
   origin: [-25.0, -20.0, 0]
   negate: 0
   occupied_thresh: 0.65
   free_thresh: 0.196
   ```

#### 출력 파일
- **`global_waypoints.json`** - 모든 waypoint 및 메타데이터

### 상세 프로세스 플로우

```
[1] 시작 및 초기화
    │
    ├─> GlobalPlannerOffline 노드 생성
    ├─> 파라미터 로드
    │   ├─> map_name (예: "hangar_1905_v0")
    │   ├─> map_dir (예: "/path/to/maps/hangar_1905_v0")
    │   ├─> safety_width (예: 0.25)
    │   ├─> safety_width_sp (예: 0.20)
    │   └─> reverse_mapping (예: false)
    │
    ├─> 맵 파일 검증
    │   ├─> <map_name>.png 존재 확인
    │   ├─> <map_name>.yaml 존재 확인
    │   └─> 없으면 에러로 종료
    │
    └─> GlobalPlannerLogic 생성
        └─> create_map=False (오프라인 모드)

[2] 맵 메타데이터 로드
    │
    └─> YAML 파일 읽기
        ├─> resolution 추출
        ├─> origin 추출
        │   ├─> map_origin.x
        │   └─> map_origin.y
        ├─> initial_position 설정
        │   └─> [origin_x, origin_y, 0]
        └─> map_valid = True

[3] PNG 맵 로드
    │
    └─> cv2.imread(<map_name>.png)
        ├─> 이미 필터링된 이미지
        ├─> Binary format (0 또는 255)
        └─> OpenCV 좌표계로 flip

[4] 중심선 추출 (온라인과 동일)
    │
    ├─> Skeletonization
    ├─> extract_centerline()
    ├─> smooth_centerline()
    ├─> 좌표 변환 (셀 → 미터)
    ├─> 보간 (0.1m)
    └─> 방향 확인
        ├─> initial_position 기준
        └─> reverse_mapping 적용

[5] 트랙 경계 추출 (온라인과 동일)
    │
    ├─> Watershed 알고리즘
    └─> Distance Transform (폴백)

[6] Trajectory 최적화 #1: IQP (온라인과 동일)
    │
    ├─> add_dist_to_cent()
    ├─> trajectory_optimizer(curv_opt_type='mincurv_iqp')
    └─> dist_to_bounds()

[7] Trajectory 최적화 #2: Shortest Path (온라인과 동일)
    │
    ├─> IQP 재실행 (safety_width_sp)
    ├─> trajectory_optimizer(curv_opt_type='shortest_path')
    └─> dist_to_bounds()

[8] Waypoint 생성 및 저장 (온라인과 동일)
    │
    ├─> create_wpnts_markers()
    └─> write_global_waypoints()
        └─> <map_dir>/global_waypoints.json

[9] 퍼블리싱 및 종료
    │
    ├─> read_and_publish()
    │   └─> 모든 ROS 토픽으로 퍼블리시
    │
    └─> 노드 자동 종료 (1초 후)
```

### 온라인 vs 오프라인 차이점

| 단계 | 온라인 모드 | 오프라인 모드 |
|------|-----------|-------------|
| **입력** | ROS 토픽 (/map, /car_state/pose) | PNG + YAML 파일 |
| **맵 생성** | 실시간 필터링 (occupancy grid → binary) | PNG 직접 로드 (이미 binary) |
| **랩 카운트** | 실제 차량 주행으로 감지 | 불필요 (required_laps=0) |
| **초기 위치** | 첫 pose 메시지에서 설정 | YAML origin에서 설정 |
| **사용자 확인** | GUI 버튼 클릭 필요 | 자동 진행 |
| **실행 시점** | 매핑 중 실시간 | 언제든지 |
| **재생성** | 다시 매핑 필요 | 즉시 가능 |

---

## 주요 알고리즘 상세 설명

### 1. Skeletonization (Lee 알고리즘)

```python
skeleton = skeletonize(filtered_map, method='lee')
```

**목적**: Binary 맵에서 1픽셀 두께의 중심선 추출

**과정**:
1. Binary 이미지의 흰색 영역(트랙)을 점진적으로 얇게 만듦
2. 연결성을 유지하면서 가장자리 픽셀 제거
3. 최종적으로 트랙의 중심을 나타내는 skeleton 생성
4. Skeleton의 각 픽셀이 트랙 중심선의 한 점

### 2. Watershed Algorithm (트랙 경계 추출)

```python
bound_r, bound_l = extract_track_bounds(centerline, filtered_map)
```

**목적**: 중심선 기준으로 좌우 경계 분리

**과정**:
1. 중심선의 각 점에서 수직 방향 계산
2. 좌우로 ray casting 수행
3. 장애물(검정 픽셀)을 만날 때까지 탐색
4. 좌측 경계점과 우측 경계점 수집
5. 2개 이상의 경계가 감지되면 실패 → distance transform으로 폴백

### 3. IQP Trajectory Optimization

```python
trajectory = trajectory_optimizer(
    input_path=config_dir,
    track_name='map_centerline',
    curv_opt_type='mincurv_iqp',
    safety_width=0.25
)
```

**목적**: 최소 곡률(curvature) 경로 생성

**수학적 정식화**:
```
Minimize:   ∫ κ² ds  (곡률의 제곱 최소화)

Subject to:
  - d_left_min ≤ d(s) ≤ d_right_min  (트랙 내부)
  - |dκ/ds| ≤ κ_max  (곡률 변화율 제한)
  - v² ≤ v_max² / (1 + κ² * gain)  (속도 제약)
  - a_min ≤ a ≤ a_max  (가속도 제약)
```

**반복 과정**:
1. 초기 추정 경로 (중심선)
2. Quadratic Programming으로 최적 lateral offset 계산
3. 속도 프로파일 최적화
4. 수렴할 때까지 반복 (일반적으로 5-10회)

### 4. Velocity Profile Generation

```python
vx = max_speed / (1 + curvature_gain * |kappa|)
```

**목적**: 곡률에 따른 안전한 속도 결정

**과정**:
1. 각 waypoint의 곡률 계산
2. 곡률이 클수록 속도 감소
3. 최대 가속도/감속도 제약 적용
4. Forward-backward pass로 전역 최적화
5. 결과: 각 점의 최적 속도 프로파일

---

## 데이터 변환 체인

```
┌─────────────────────┐
│  ROS /map Topic     │ (온라인만)
│  OccupancyGrid      │
│  [-1, 0, 100, ...]  │
└──────────┬──────────┘
           │ filter_map_occupancy_grid()
           ↓
┌─────────────────────┐
│  Binary Image       │
│  [[255, 0, ...]]    │
│  (PNG 형식)         │
└──────────┬──────────┘
           │ skeletonize()
           ↓
┌─────────────────────┐
│  Skeleton           │
│  1-pixel centerline │
└──────────┬──────────┘
           │ extract_centerline()
           ↓
┌─────────────────────┐
│  Centerline (cells) │
│  [[x1, y1], ...]    │
└──────────┬──────────┘
           │ coordinate transform
           ↓
┌─────────────────────┐
│  Centerline (meters)│
│  [[x1_m, y1_m], ...]│
└──────────┬──────────┘
           │ add_dist_to_cent()
           ↓
┌─────────────────────┐
│  Centerline + Bounds│
│  [x, y, d_r, d_l]   │
└──────────┬──────────┘
           │ trajectory_optimizer()
           ↓
┌─────────────────────┐
│  Global Trajectory  │
│  [s,x,y,ψ,vx,ax]    │
└──────────┬──────────┘
           │ create_wpnts_markers()
           ↓
┌─────────────────────┐
│  WpntArray Message  │
│  + Markers          │
└──────────┬──────────┘
           │ write_global_waypoints()
           ↓
┌─────────────────────┐
│  JSON File          │
│  global_waypoints.  │
│  json               │
└─────────────────────┘
```

---

## 사용 예시

### 온라인 모드 (매핑 중)

```bash
# Terminal 1: Mapping
ros2 launch stack_master mapping_launch.xml \
    racecar_version:=NUC2 \
    map_name:=my_new_track

# 차량이 트랙을 주행
# /map 토픽 퍼블리시 (Cartographer)
# /car_state/pose 토픽 퍼블리시 (localization)
#
# 1랩 완료 시 자동으로 trajectory 생성 시작
# GUI에서 "Map ready" 버튼 클릭
#
# 출력:
# - maps/my_new_track/my_new_track.png
# - maps/my_new_track/my_new_track.yaml
# - maps/my_new_track/global_waypoints.json
```

### 오프라인 모드 (기존 맵 재처리)

```bash
# 방법 1: Launch file
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=my_new_track \
    map_dir:=/path/to/maps/my_new_track \
    safety_width:=0.30

# 방법 2: Direct run
ros2 run global_planner global_planner_offline

# 입력:
# - my_new_track.png (이미 존재)
# - my_new_track.yaml (이미 존재)
#
# 출력:
# - global_waypoints.json (재생성)
```

---

## 파라미터 튜닝 가이드

### Safety Width
```yaml
safety_width: 0.25  # IQP (기본 경로)
  # 크게: 더 안전하지만 느린 경로
  # 작게: 더 빠르지만 위험한 경로
  # 추천: 0.20 ~ 0.35

safety_width_sp: 0.20  # Shortest Path (오버테이킹)
  # IQP보다 작아야 함
  # 추천: 0.15 ~ 0.25
```

### Reverse Mapping
```yaml
reverse_mapping: false
  # true: 트랙을 반대 방향으로 주행
  # 사용 예: 같은 트랙에서 양방향 연습
```

### Show Plots
```yaml
show_plots: true
  # true: 각 단계의 시각화 표시 (개발/디버깅)
  # false: 플롯 없이 빠른 생성 (자동화/배치)
```

---

## 트러블슈팅

### 온라인 모드

**문제**: "Waiting for map and car pose..."
- **원인**: /map 또는 /car_state/pose 토픽이 퍼블리시되지 않음
- **해결**:
  ```bash
  ros2 topic list  # 토픽 확인
  ros2 topic echo /map  # 데이터 확인
  ```

**문제**: "No closed contours found"
- **원인**: Skeleton에서 닫힌 루프를 찾을 수 없음
- **해결**:
  - 완전한 랩을 주행했는지 확인
  - 맵 품질 확인 (노이즈, 끊김)
  - filter_kernel_size 조정

**문제**: "Couldn't find a closed contour with similar length"
- **원인**: 주행한 경로 길이와 centerline 길이 불일치
- **해결**:
  - 한 바퀴 더 주행
  - required_laps 조정

### 오프라인 모드

**문제**: "Map YAML not found"
- **원인**: 파일 경로 오류
- **해결**:
  - map_dir 절대 경로 확인
  - 파일 이름이 정확한지 확인

**문제**: Watershed 실패 → Distance transform 사용
- **원인**: 복잡한 트랙 구조
- **해결**:
  - 정상 동작 (자동 폴백)
  - Distance transform도 실패하면 맵 품질 확인

---

## 추가 참고 자료

- **Trajectory Optimization**: `global_racetrajectory_optimization` 패키지
- **Frenet Frame**: `frenet_conversion` 라이브러리
- **Waypoint 구조**: `f110_msgs/WpntArray` 메시지 정의
- **Paper**: "ForzaETH Race Stack" (Journal of Field Robotics)
