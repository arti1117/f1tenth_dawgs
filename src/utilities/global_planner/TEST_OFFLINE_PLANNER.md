# 오프라인 Global Planner 테스트 가이드

## 빌드

```bash
cd /home/dawgs_nx/forza_ws
colcon build --packages-select global_planner
source install/setup.bash
```

## 테스트 실행

### 방법 1: show_plots=false (플롯 없이 빠른 실행)

```bash
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=levine/levine_blocked_map \
    map_dir:=/home/dawgs_nx/forza_ws/install/global_planner/share/global_planner/share/global_planner/maps/levine/levine_blocked_map \
    show_plots:=false
```

### 방법 2: 실제 맵으로 테스트 (hangar_1905_v0)

```bash
# 먼저 맵이 있는지 확인
ls -la src/race_stack/stack_master/maps/hangar_1905_v0/

# 실행
ros2 launch global_planner global_planner_offline.launch.py \
    map_name:=hangar_1905_v0 \
    map_dir:=$(pwd)/src/race_stack/stack_master/maps/hangar_1905_v0 \
    show_plots:=false
```

## 로그 출력 예시

성공적으로 실행되면 다음과 같은 로그가 출력됩니다:

```
============================================================
STARTING TRAJECTORY COMPUTATION
============================================================
Step 1/10: Loading map PNG file...
  ✓ Loaded map from: /path/to/map.png
  ✓ Map size: (800, 1000)

Step 2/10: Skeletonizing map (this may take 10-30 seconds)...
  ✓ Skeleton created: (800, 1000)

Step 3/10: Skipping plot display (show_plots=False)

Step 4/10: Extracting centerline from skeleton...
  ✓ Centerline extracted: 1245 points

Step 5/10: Smoothing and converting centerline...
  ✓ Centerline smoothed: 1245 points
  ✓ Converted to meters (resolution: 0.05m)
  ✓ Interpolated to 0.1m steps: 2134 points

Step 6/10: Extracting track bounds...
  Attempting watershed algorithm...
  ✓ Using watershed for track bound extraction

============================================================
Step 7/10: IQP TRAJECTORY OPTIMIZATION
============================================================
  Adding distance to centerline bounds...
  ✓ Centerline with bounds prepared (safety_width=0.25m)
  Running IQP optimizer (this may take 1-3 minutes)...
  ✓ IQP optimization complete!
    - Estimated lap time: 25.34s
    - Max speed: 8.45 m/s
    - Trajectory points: 2134

============================================================
Step 8/10: SHORTEST PATH OPTIMIZATION (for overtaking)
============================================================
  Running IQP for overtaking trajectory...
  ✓ Overtaking IQP complete (safety_width=0.2m)
  Running shortest path optimizer (this may take 1-3 minutes)...
  ✓ Shortest path optimization complete!
    - Estimated lap time: 24.12s
    - Max speed: 8.89 m/s
    - Trajectory points: 2089

============================================================
Step 9/10: Saving waypoints to JSON
============================================================
  Writing to: /path/to/map/global_waypoints.json
  ✓ JSON file saved successfully

============================================================
Step 10/10: TRAJECTORY GENERATION COMPLETE!
============================================================
Summary:
  Map: hangar_1905_v0
  IQP lap time: 25.34s
  SP lap time: 24.12s
  Output: /path/to/map/global_waypoints.json
============================================================
```

## 진행 상황 추적

각 단계에서 로그가 출력되므로 어디서 시간이 걸리는지 알 수 있습니다:

1. **Step 1-2**: 빠름 (5-10초)
2. **Step 3**: 플롯 표시 시 사용자 대기 필요
3. **Step 4-6**: 빠름 (5-10초)
4. **Step 7**: IQP 최적화 - **가장 오래 걸림 (1-3분)**
5. **Step 8**: Shortest path 최적화 - **두번째로 오래 걸림 (1-3분)**
6. **Step 9-10**: 빠름 (1-2초)

**전체 예상 시간**: 2-7분 (맵 크기에 따라 다름)

## 문제 해결

### 특정 단계에서 멈춤

로그를 보고 어느 단계에서 멈췄는지 확인:

```bash
# 만약 "Step 2/10: Skeletonizing map" 이후 멈춤
→ Skeletonization이 오래 걸림 (큰 맵일수록 느림)
→ 정상, 기다려야 함

# 만약 "Step 7/10: IQP TRAJECTORY OPTIMIZATION" 이후 멈춤
→ IQP optimizer가 실행 중 (가장 오래 걸리는 단계)
→ 정상, 1-3분 기다려야 함

# 만약 "Step 3/10: Displaying skeleton plot" 이후 멈춤
→ matplotlib 창이 열려 있음
→ 창을 닫아야 진행됨
```

### show_plots=true일 때

Matplotlib 창이 나타나면 **창을 닫아야** 다음 단계로 진행됩니다:

```bash
# 플롯 없이 실행하려면
ros2 launch global_planner global_planner_offline.launch.py \
    ... \
    show_plots:=false
```

### 로그가 전혀 안나옴

```bash
# ROS2 환경이 source되었는지 확인
source /home/dawgs_nx/forza_ws/install/setup.bash

# 노드가 실행 중인지 확인
ros2 node list

# 토픽이 퍼블리시되는지 확인
ros2 topic list
```

## 성공 확인

1. **로그 확인**: "Step 10/10: TRAJECTORY GENERATION COMPLETE!" 출력
2. **파일 확인**: `global_waypoints.json` 생성됨
   ```bash
   ls -lh src/race_stack/stack_master/maps/hangar_1905_v0/global_waypoints.json
   ```
3. **토픽 확인**: Waypoints가 퍼블리시됨
   ```bash
   ros2 topic echo /global_waypoints --once
   ```

## 다음 단계

생성된 waypoints를 레이싱 스택에서 사용:

```bash
ros2 launch stack_master time_trials_launch.xml \
    racecar_version:=NUC2 \
    LU_table:=NUC2 \
    ctrl_algo:=MAP
```
