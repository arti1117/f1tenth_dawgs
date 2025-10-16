# 오프라인 Trajectory 생성 빠른 시작 가이드

## 요약

PNG/YAML 맵 파일에서 `global_waypoints.json`을 생성합니다. **ROS 토픽 불필요**.

## 사전 준비

```bash
# 1. 워크스페이스 빌드 (한번만)
cd /home/dawgs_nx/forza_ws
colcon build

# 2. 맵 파일이 있는지 확인
# 각 맵 디렉토리에 <map_name>.png와 <map_name>.yaml이 있어야 함
ls -la src/race_stack/stack_master/maps/hangar_1905_v0/
```

## 사용법

### 방법 1: 래퍼 스크립트 사용 (추천)

```bash
# 단일 맵 생성
cd /home/dawgs_nx/forza_ws
./generate_trajectory.sh \
    --map_name hangar_1905_v0 \
    --map_dir $(pwd)/src/race_stack/stack_master/maps/hangar_1905_v0

# 여러 맵 한번에 생성
./src/race_stack/planner/global_planner/scripts/batch_generate_trajectories.sh \
    $(pwd)/src/race_stack/stack_master/maps
```

### 방법 2: 직접 실행

```bash
# ROS2 환경 source
cd /home/dawgs_nx/forza_ws
source install/setup.bash

# 스크립트 실행
python3 src/race_stack/planner/global_planner/global_planner/offline_trajectory_generator.py \
    --map_name hangar_1905_v0 \
    --map_dir $(pwd)/src/race_stack/stack_master/maps/hangar_1905_v0
```

## 파라미터

### 필수
- `--map_name`: 맵 이름 (확장자 제외)
- `--map_dir`: 맵 디렉토리 절대 경로

### 선택
- `--safety_width 0.25`: IQP 경로 안전 여유 (기본: 0.25m)
- `--safety_width_sp 0.20`: Shortest path 안전 여유 (기본: 0.20m)
- `--reverse`: 경로 방향 반전
- `--no-plots`: 플롯 표시 안함 (자동화용)

## 출력

생성되는 파일:
```
<map_dir>/global_waypoints.json
```

내용:
- 최적 경로 waypoints (IQP + Shortest Path)
- 중심선 waypoints
- 트랙 경계
- 예상 랩타임
- 시각화 마커

## 일반적인 사용 예시

### 1. 기존 맵 재생성 (파라미터 변경)

```bash
cd /home/dawgs_nx/forza_ws

# 더 보수적인 경로 생성
./generate_trajectory.sh \
    --map_name hangar_1905_v0 \
    --map_dir $(pwd)/src/race_stack/stack_master/maps/hangar_1905_v0 \
    --safety_width 0.35
```

### 2. 역방향 경로

```bash
./generate_trajectory.sh \
    --map_name hangar_1905_v0 \
    --map_dir $(pwd)/src/race_stack/stack_master/maps/hangar_1905_v0 \
    --reverse
```

### 3. 모든 맵 일괄 생성 (플롯 없이)

```bash
# batch_generate_trajectories.sh 수정
# --no-plots 옵션 추가됨

./src/race_stack/planner/global_planner/scripts/batch_generate_trajectories.sh \
    $(pwd)/src/race_stack/stack_master/maps
```

### 4. 새 맵 추가 후

```bash
# 1. 맵 파일 복사
cp my_new_map.png my_new_map.yaml src/race_stack/stack_master/maps/my_new_map/

# 2. Trajectory 생성
./generate_trajectory.sh \
    --map_name my_new_map \
    --map_dir $(pwd)/src/race_stack/stack_master/maps/my_new_map
```

## 트러블슈팅

### "ROS2 workspace not built"
```bash
cd /home/dawgs_nx/forza_ws
colcon build
```

### "Map YAML not found"
```bash
# 경로 확인
ls -la src/race_stack/stack_master/maps/YOUR_MAP/

# 필수 파일:
# - YOUR_MAP.png
# - YOUR_MAP.yaml
```

### "No closed contours found"
- PNG 맵이 닫힌 트랙을 형성하는지 확인
- `map_editor` 노드로 맵 수정 가능

### Import 에러
```bash
# ROS2 환경이 source되었는지 확인
source /home/dawgs_nx/forza_ws/install/setup.bash
echo $ROS_DISTRO  # "humble" 출력되어야 함
```

## 기존 방식과의 비교

| 측면 | 기존 (ROS Node) | 새로운 (Offline) |
|------|----------------|-----------------|
| ROS 토픽 | 필요 (/map, /car_state/pose) | 불필요 |
| 실행 시점 | 매핑 중 실시간 | 언제든지 |
| 재생성 | 다시 매핑 필요 | PNG/YAML만 있으면 됨 |
| 파라미터 조정 | 어려움 | 쉬움 (즉시 재생성) |
| 사용 사례 | 초기 매핑 | 파라미터 최적화, 재처리 |

## 통합 워크플로우

```bash
# 1. 매핑 (한번만, ROS 필요)
ros2 launch stack_master mapping_launch.xml \
    racecar_version:=NUC2 \
    map_name:=my_track

# 2. 오프라인 trajectory 생성 (반복 가능, ROS 토픽 불필요)
./generate_trajectory.sh \
    --map_name my_track \
    --map_dir $(pwd)/src/race_stack/stack_master/maps/my_track

# 3. 레이싱 (ROS)
ros2 launch stack_master time_trials_launch.xml \
    racecar_version:=NUC2 \
    LU_table:=NUC2 \
    ctrl_algo:=MAP
```

## 도움말

```bash
# 자세한 옵션 보기
./generate_trajectory.sh --help

# 또는
python3 src/race_stack/planner/global_planner/global_planner/offline_trajectory_generator.py --help
```

## 참고 문서

- 상세 문서: `src/race_stack/planner/global_planner/README_OFFLINE.md`
- 일반 사용법: `src/race_stack/stack_master/README.md`
- 프로젝트 개요: `CLAUDE.md`
