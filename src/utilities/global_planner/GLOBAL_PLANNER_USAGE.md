# Global Planner 사용 가이드

## 빠른 시작

Global Planner는 두 가지 모드로 사용할 수 있습니다:

### 1. 온라인 모드 (Online) - 매핑 중 사용
```bash
# stack_master를 통해 실행 (권장)
ros2 launch stack_master mapping_launch.xml \
    racecar_version:=NUC2 \
    map_name:=my_track
```
- ROS 토픽 필요: `/map`, `/car_state/pose`
- 차량이 트랙을 주행하면서 실시간으로 맵 생성
- 1랩 완료 후 자동으로 trajectory 생성

### 2. 오프라인 모드 (Offline) - 기존 맵 재처리
```bash
# 직접 실행
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=hangar_1905_v0 \
    map_dir:=/home/dawgs_nx/forza_ws/src/race_stack/stack_master/maps/hangar_1905_v0
```
- ROS 토픽 불필요
- 기존 PNG/YAML 파일에서 직접 trajectory 생성
- 언제든지 재생성 가능

---

## 오프라인 모드 상세 가이드

### 설정 파일 수정

**파일**: `src/race_stack/planner/global_planner/config/global_planner_offline_params.yaml`

```yaml
global_planner_offline_node:
  ros__parameters:
    # 맵 이름과 디렉토리 설정
    map_name: "hangar_1905_v0"
    map_dir: "/home/dawgs_nx/forza_ws/src/race_stack/stack_master/maps/hangar_1905_v0"

    # Trajectory 파라미터
    safety_width: 0.25        # IQP 경로 안전 여유
    safety_width_sp: 0.20     # Shortest path 안전 여유
    reverse_mapping: false    # 역방향 경로
    show_plots: true          # 플롯 표시
```

### 사용 방법

#### 방법 1: Launch file (권장)

```bash
# 1. 워크스페이스 빌드 (처음 한번만)
cd /home/dawgs_nx/forza_ws
colcon build --packages-select global_planner
source install/setup.bash

# 2. Launch file 실행
ros2 launch global_planner global_planner_offline.launch.py

# 3. 다른 맵으로 실행
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=GLC_smile_small \
    map_dir:=/home/dawgs_nx/forza_ws/src/race_stack/stack_master/maps/GLC_smile_small

# 4. 파라미터 커스터마이즈
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=my_track \
    map_dir:=/path/to/maps/my_track \
    safety_width:=0.30 \
    safety_width_sp:=0.25 \
    reverse_mapping:=true \
    show_plots:=false
```

#### 방법 2: 직접 실행

```bash
# config 파일 먼저 수정
nano src/race_stack/planner/global_planner/config/global_planner_offline_params.yaml

# 실행
ros2 run global_planner global_planner_offline
```

### 여러 맵 일괄 처리

각 맵에 대해 설정 파일을 수정하고 실행:

```bash
#!/bin/bash
# 예시 배치 스크립트

MAPS=(
    "hangar_1905_v0"
    "GLC_smile_small"
    "glc_ot_ez"
)

for MAP in "${MAPS[@]}"; do
    echo "Processing $MAP..."
    ros2 launch global_planner global_planner_offline.launch.py \
        map_name:=$MAP \
        map_dir:=/home/dawgs_nx/forza_ws/src/race_stack/stack_master/maps/$MAP \
        show_plots:=false
    sleep 5
done
```

---

## 파라미터 설명

### map_name
- **타입**: string
- **필수**: Yes
- **설명**: 맵 이름 (확장자 제외)
- **예시**: `"hangar_1905_v0"`

### map_dir
- **타입**: string
- **필수**: Yes
- **설명**: 맵 파일이 있는 디렉토리 절대 경로
- **예시**: `"/home/user/forza_ws/src/race_stack/stack_master/maps/hangar_1905_v0"`
- **필요 파일**:
  - `<map_name>.png`
  - `<map_name>.yaml`

### safety_width
- **타입**: float
- **기본값**: 0.25
- **단위**: 미터
- **설명**: IQP 최적화 경로의 안전 여유폭
- **효과**:
  - **크게** (0.30~0.35): 더 안전하지만 느린 경로
  - **작게** (0.20~0.25): 더 빠르지만 위험한 경로
- **권장**: 0.20 ~ 0.35

### safety_width_sp
- **타입**: float
- **기본값**: 0.20
- **단위**: 미터
- **설명**: Shortest path 경로의 안전 여유폭 (오버테이킹용)
- **제약**: `safety_width_sp < safety_width`
- **권장**: 0.15 ~ 0.25

### reverse_mapping
- **타입**: bool
- **기본값**: false
- **설명**: 경로 방향 반전
- **사용 예**: 같은 트랙을 반대 방향으로 주행

### show_plots
- **타입**: bool
- **기본값**: true
- **설명**: Matplotlib 플롯 표시 여부
- **권장**:
  - `true`: 개발/디버깅 시
  - `false`: 자동화/배치 처리 시

---

## 출력 파일

### global_waypoints.json

**위치**: `<map_dir>/global_waypoints.json`

**내용**:
```json
{
  "map_infos": "IQP lap time: 25.34s, SP lap time: 24.12s, ...",
  "est_lap_time": 24.12,
  "centerline_wpnts": [...],     // 중심선 waypoints
  "centerline_markers": [...],   // 중심선 시각화
  "glb_wpnts": [...],            // IQP 최적 경로
  "glb_markers": [...],          // IQP 시각화
  "glb_sp_wpnts": [...],         // Shortest path 경로
  "glb_sp_markers": [...],       // Shortest path 시각화
  "track_bounds": [...]          // 트랙 경계
}
```

### 퍼블리시되는 토픽

오프라인 모드도 온라인 모드와 동일한 토픽들을 퍼블리시합니다:

```bash
/global_waypoints                           # WpntArray
/global_waypoints/shortest_path             # WpntArray
/centerline_waypoints                       # WpntArray
/global_waypoints/markers                   # MarkerArray
/global_waypoints/shortest_path/markers     # MarkerArray
/centerline_waypoints/markers               # MarkerArray
/trackbounds/markers                        # MarkerArray
/map_infos                                  # String
/estimated_lap_time                         # Float32
```

---

## 온라인 vs 오프라인 비교

| 특징 | 온라인 모드 | 오프라인 모드 |
|------|-----------|-------------|
| **ROS 토픽** | 필요 (/map, /car_state/pose) | 불필요 |
| **입력** | 실시간 occupancy grid | PNG + YAML 파일 |
| **실행 시점** | 매핑 중 | 언제든지 |
| **재생성** | 다시 매핑 필요 | 즉시 가능 |
| **랩 감지** | 실제 주행으로 감지 | 불필요 |
| **사용자 확인** | GUI 버튼 클릭 | 자동 진행 |
| **실행 명령** | stack_master 통해 실행 | 직접 실행 |
| **용도** | 초기 매핑 | 파라미터 최적화, 재처리 |

---

## 워크플로우

### 초기 매핑 (온라인)
```bash
1. 차량 준비
2. ros2 launch stack_master mapping_launch.xml ...
3. 차량으로 트랙 주행 (1랩+)
4. 자동으로 trajectory 생성
5. GUI에서 "Map ready" 클릭
6. 출력: PNG, YAML, JSON
```

### 파라미터 튜닝 (오프라인)
```bash
1. 기존 PNG/YAML 있음
2. config 파일 수정 (safety_width 등)
3. ros2 launch global_planner global_planner_offline.launch.py ...
4. 자동으로 trajectory 재생성
5. 출력: JSON (업데이트됨)
6. 시뮬레이션/실차에서 테스트
7. 만족할 때까지 2-6 반복
```

---

## 트러블슈팅

### "Map YAML not found"
```bash
# 파일 확인
ls -la /path/to/map/directory/

# 필요한 파일:
# - <map_name>.yaml
# - <map_name>.png
```

### "No closed contours found"
- **원인**: PNG 맵에서 닫힌 트랙을 찾을 수 없음
- **해결**:
  - PNG 이미지 확인 (트랙이 닫혀 있는지)
  - `map_editor` 노드로 맵 수정

### Watershed 알고리즘 실패
```
[WARN] More than two track bounds detected with watershed algorithm
[INFO] Trying with simple distance transform...
```
- **정상 동작**: 자동으로 distance transform으로 전환됨
- 결과에 문제가 있으면 맵 품질 확인

### Config 디렉토리 없음
```bash
# global_planner 패키지 빌드 확인
cd /home/dawgs_nx/forza_ws
colcon build --packages-select global_planner
source install/setup.bash

# config 디렉토리 확인
ls -la install/global_planner/share/global_planner/config/
```

---

## 고급 사용법

### 1. 다양한 안전 여유로 테스트

```bash
# 보수적 경로
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=test_track \
    safety_width:=0.35 \
    safety_width_sp:=0.30

# 공격적 경로
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=test_track \
    safety_width:=0.20 \
    safety_width_sp:=0.15
```

### 2. 양방향 경로 생성

```bash
# 정방향
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=track_forward \
    map_dir:=/path/to/track \
    reverse_mapping:=false

# 역방향 (같은 맵)
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=track_reverse \
    map_dir:=/path/to/track \
    reverse_mapping:=true
```

### 3. RVIZ로 결과 시각화

```bash
# Terminal 1: 오프라인 플래너 실행
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=my_track

# Terminal 2: RVIZ
rviz2

# RVIZ에서 추가:
# - Add → MarkerArray → /global_waypoints/markers
# - Add → MarkerArray → /centerline_waypoints/markers
# - Add → MarkerArray → /trackbounds/markers
```

---

## 참고 문서

- **상세 프로세스**: `src/race_stack/planner/global_planner/PROCESS_FLOW.md`
- **프로젝트 개요**: `CLAUDE.md`
- **Stack Master 가이드**: `src/race_stack/stack_master/README.md`
