# Offline Trajectory Generation

PNG/YAML 맵 파일에서 직접 global waypoints JSON을 생성하는 도구입니다. ROS 토픽(odom, map)이 필요 없이 오프라인으로 작동합니다.

## 사용 방법

### 1. 단일 맵 처리

```bash
# 기본 사용법
cd /home/dawgs_nx/forza_ws/src/race_stack/planner/global_planner
python3 global_planner/offline_trajectory_generator.py \
    --map_name hangar_1905_v0 \
    --map_dir /path/to/stack_master/maps/hangar_1905_v0

# 실제 경로 예시
python3 global_planner/offline_trajectory_generator.py \
    --map_name hangar_1905_v0 \
    --map_dir /home/dawgs_nx/forza_ws/src/race_stack/stack_master/maps/hangar_1905_v0
```

### 2. 커스텀 파라미터 사용

```bash
# Safety width 조정
python3 global_planner/offline_trajectory_generator.py \
    --map_name my_track \
    --map_dir /path/to/maps/my_track \
    --safety_width 0.30 \
    --safety_width_sp 0.25

# 역방향 경로 생성
python3 global_planner/offline_trajectory_generator.py \
    --map_name my_track \
    --map_dir /path/to/maps/my_track \
    --reverse

# 플롯 비활성화 (자동 실행용)
python3 global_planner/offline_trajectory_generator.py \
    --map_name my_track \
    --map_dir /path/to/maps/my_track \
    --no-plots
```

### 3. 배치 처리 (여러 맵 한번에)

```bash
# stack_master/maps 디렉토리의 모든 맵 처리
cd /home/dawgs_nx/forza_ws/src/race_stack/planner/global_planner
./scripts/batch_generate_trajectories.sh /home/dawgs_nx/forza_ws/src/race_stack/stack_master/maps

# 또는 기본 경로 사용
./scripts/batch_generate_trajectories.sh
```

## 필요한 파일

각 맵 디렉토리에는 다음 파일이 있어야 합니다:

```
maps/
└── <map_name>/
    ├── <map_name>.png   # 맵 이미지 (필수)
    ├── <map_name>.yaml  # 맵 메타데이터 (필수)
    └── global_waypoints.json  # 생성될 출력 파일
```

## 출력 파일

생성되는 `global_waypoints.json`에는 다음이 포함됩니다:

- **map_infos**: 맵 정보 및 최적화 결과
- **est_lap_time**: 예상 랩타임
- **centerline_markers**: 중심선 시각화 마커
- **centerline_wpnts**: 중심선 waypoint 배열
- **glb_markers**: IQP 최적 경로 시각화
- **glb_wpnts**: IQP 최적 경로 waypoints
- **glb_sp_markers**: Shortest path 시각화
- **glb_sp_wpnts**: Shortest path waypoints
- **track_bounds**: 트랙 경계 마커

## 파라미터 설명

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `--map_name` | (필수) | 맵 이름 (확장자 제외) |
| `--map_dir` | (필수) | 맵 파일이 있는 디렉토리 전체 경로 |
| `--safety_width` | 0.25 | IQP 경로의 안전 여유 폭 (미터) |
| `--safety_width_sp` | 0.20 | Shortest path의 안전 여유 폭 (미터) |
| `--reverse` | False | 경로 방향 반전 |
| `--no-plots` | False | Matplotlib 플롯 비활성화 |

## ROS Node와의 차이점

### 기존 ROS Node 방식
```python
# ROS 토픽 필요
/map (OccupancyGrid)
/car_state/pose (PoseStamped)

# 실시간 매핑 중에만 작동
# 차량이 트랙을 돌면서 데이터 수집 필요
```

### Offline Generator 방식
```python
# ROS 토픽 불필요
# 기존 PNG/YAML 파일만 필요

# 언제든지 재생성 가능
# 파라미터 조정 후 빠른 반복 테스트 가능
```

## 트러블슈팅

### 오류: "Map YAML not found"
```bash
# 맵 디렉토리 경로를 확인하세요
ls -la /path/to/map/directory/
# <map_name>.yaml 파일이 있어야 합니다
```

### 오류: "No closed contours found"
```bash
# PNG 맵 이미지가 올바른지 확인
# 트랙이 닫힌 루프를 형성해야 합니다
# map_editor로 맵을 수정할 수 있습니다
```

### 워터셰드 알고리즘 실패
```bash
# 자동으로 distance transform으로 전환됩니다
# 로그에서 "Trying with simple distance transform" 메시지 확인
```

### Config 디렉토리 없음
```bash
# global_planner 패키지의 config 디렉토리가 있는지 확인
ls -la src/race_stack/planner/global_planner/config/
```

## 활용 사례

### 1. 기존 맵 재처리
```bash
# 파라미터를 조정하여 경로 재생성
python3 global_planner/offline_trajectory_generator.py \
    --map_name existing_map \
    --map_dir /path/to/map \
    --safety_width 0.30  # 더 보수적인 경로
```

### 2. 시뮬레이션용 맵 준비
```bash
# 모든 맵의 trajectory를 미리 생성
./scripts/batch_generate_trajectories.sh /path/to/sim/maps
```

### 3. 파라미터 최적화
```bash
# 다양한 safety width로 테스트
for width in 0.20 0.25 0.30 0.35; do
    python3 global_planner/offline_trajectory_generator.py \
        --map_name test_track \
        --map_dir /path/to/map \
        --safety_width $width \
        --no-plots
done
```

## 통합 방법

### ROS2 시스템과 함께 사용

1. **오프라인 생성**:
   ```bash
   python3 offline_trajectory_generator.py --map_name my_map --map_dir /path/to/map
   ```

2. **ROS2에서 사용**:
   ```bash
   # global_planner_node가 자동으로 JSON을 읽어서 publish
   ros2 launch stack_master base_system_launch.xml map_name:=my_map
   ```

### 기존 워크플로우

```mermaid
매핑 (ROS topics 필요)
  ↓
맵 PNG/YAML 생성
  ↓
경로 최적화 (ROS topics 필요)
  ↓
JSON 생성
```

### 새로운 워크플로우

```mermaid
매핑 (ROS topics 필요)
  ↓
맵 PNG/YAML 생성
  ↓
오프라인 경로 생성 (독립 실행)
  ↓
JSON 생성
  ↓
언제든지 파라미터 변경하여 재생성 가능
```

## 성능

- **처리 시간**: 맵 크기에 따라 30초~2분
- **메모리**: ~500MB (trajectory optimization)
- **병렬 처리**: 배치 스크립트로 여러 맵 순차 처리

## 향후 개선 사항

- [ ] 멀티프로세싱으로 배치 처리 속도 향상
- [ ] GUI 도구로 실시간 파라미터 조정
- [ ] 경로 비교 도구 (다른 파라미터 간)
- [ ] 자동 safety width 최적화

--- offline guide


--- csv file save
● global_planner에서 CSV 파일로 위치와 속도를 출력하도록
  수정했습니다!

  변경 사항 요약

  1. ✅ CSV 저장 함수 추가 (readwrite_global_waypoints.py)

  새로운 함수: write_waypoints_to_csv() (line 137-210)

  두 개의 CSV 파일을 생성합니다:
  - global_waypoints_iqp.csv: IQP 최적화 경로
  - global_waypoints_sp.csv: Shortest Path 경로

  CSV 포맷: x, y, v, kappa
  - x, y: 위치 (미터)
  - v: 속도 (m/s) - path_planner에서 바로 사용 가능
  - kappa: 곡률 (rad/m)

  추가 기능:
  - 헤더 포함 (첫 번째 줄: x,y,v,kappa)
  - 속도 통계 출력 (최소/최대/평균 속도)
  - 각 경로의 waypoint 개수 표시

  2. ✅ global_planner_logic 통합

  Step 10/10으로 CSV 저장 추가
  (global_planner_logic.py:557-581)

  JSON 저장 후 바로 CSV 파일도 자동으로 생성합니다:
  write_waypoints_to_csv(
      self.map_dir,
      global_traj_wpnts_iqp,
      global_traj_wpnts_sp
  )

  출력 결과

  global_planner 실행 시 다음과 같은 정보가 출력됩니다:

  ==========================================================
  ==
  Step 10/10: Exporting waypoints to CSV for path_planner
  ==========================================================
  ==
  [INFO] WRITE_CSV: Writing IQP waypoints to
  .../global_waypoints_iqp.csv
    ✓ Wrote 1234 IQP waypoints
  [INFO] WRITE_CSV: Writing SP waypoints to 
  .../global_waypoints_sp.csv
    ✓ Wrote 1234 SP waypoints
  [INFO] CSV SUMMARY:
    IQP trajectory:
      - Points: 1234
      - Velocity range: [2.50, 8.50] m/s
      - Average velocity: 5.75 m/s
    SP trajectory:
      - Points: 1234
      - Velocity range: [2.80, 9.20] m/s
      - Average velocity: 6.10 m/s
    ✓ CSV files saved successfully

  ==========================================================
  ==
  TRAJECTORY GENERATION COMPLETE!
  ==========================================================
  ==
  Summary:
    Map: your_map_name
    IQP lap time: 45.23s
    SP lap time: 42.15s
    Output files:
      - JSON: .../global_waypoints.json
      - CSV (IQP): .../global_waypoints_iqp.csv
      - CSV (SP): .../global_waypoints_sp.csv
  ==========================================================
  ==

  path_planner에서 사용하기

  이제 path_planner에서 다음과 같이 CSV 파일을 직접 사용할
  수 있습니다:

  // planner.yaml에서 설정
  global_path_topic: "/path/to/map/global_waypoints_iqp.csv"

  CSV 파일에는 이미 속도 정보가 포함되어 있으므로, 이전에
  수정한 path_planner와 path_tracker가 속도를 정상적으로
  읽어서 사용할 수 있습니다!
