# ROS2 Bag 기반 오프라인 매핑 가이드

이 가이드는 ROS2 bag 파일을 사용하여 오프라인에서 맵을 생성하는 방법을 설명합니다. Cartographer와 SLAM Toolbox 두 가지 방법을 모두 지원합니다.

## 목차
1. [ROS2 Bag 파일 녹화](#1-ros2-bag-파일-녹화)
2. [Cartographer를 사용한 오프라인 매핑](#2-cartographer를-사용한-오프라인-매핑)
3. [SLAM Toolbox를 사용한 오프라인 매핑](#3-slam-toolbox를-사용한-오프라인-매핑)
4. [맵 저장](#4-맵-저장)
5. [트러블슈팅](#5-트러블슈팅)

---

## 1. ROS2 Bag 파일 녹화

### 1.1 필요한 토픽 확인
매핑에 필요한 토픽:
- `/scan` - LiDAR 센서 데이터 (필수)
- `/odom` - Odometry 데이터 (필수)
- `/tf` 및 `/tf_static` - Transform 데이터 (필수)
- `/imu/data` - IMU 데이터 (선택)

### 1.2 Bag 파일 녹화 명령

#### 방법 1: 특정 토픽만 녹화 (권장)
```bash
ros2 bag record /scan /odom /tf /tf_static -o mapping_data
```

#### 방법 2: 모든 토픽 녹화
```bash
ros2 bag record -a -o mapping_data
```

#### 방법 3: 필터링하여 녹화 (고급)
```bash
# IMU 데이터를 포함하여 녹화
ros2 bag record /scan /odom /tf /tf_static /imu/data -o mapping_data_with_imu
```

### 1.3 녹화 중 팁
- **천천히 움직이기**: 너무 빠르게 움직이면 스캔 매칭이 실패할 수 있습니다
- **루프 클로저**: 시작 지점으로 돌아오면 맵의 정확도가 향상됩니다
- **충분한 특징점**: 빈 공간보다는 벽이나 물체가 있는 환경이 좋습니다
- **녹화 시간**: 일반적으로 3-10분 정도가 적당합니다

### 1.4 Bag 파일 정보 확인
```bash
ros2 bag info mapping_data
```

출력 예시:
```
Files:             mapping_data_0.db3
Bag size:          1.2 GiB
Storage id:        sqlite3
Duration:          300.5s
Start:             Oct 27 2025 14:30:00.123
End:               Oct 27 2025 14:35:00.623
Messages:          45230
Topic information: Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 3005 | Serialization Format: cdr
                   Topic: /odom | Type: nav_msgs/msg/Odometry | Count: 15025 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 27200 | Serialization Format: cdr
```

---

## 2. Cartographer를 사용한 오프라인 매핑

### 2.1 Launch 파일 실행

```bash
ros2 launch agent_dawgs mapping_cartographer_offline.launch.py \
    bag_file:=/path/to/your/mapping_data
```

### 2.2 Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|-------|------|
| `bag_file` | (필수) | Bag 파일의 전체 경로 |
| `use_rviz` | `true` | RViz2 실행 여부 |
| `resolution` | `0.05` | 맵 해상도 (미터/픽셀) |
| `publish_period_sec` | `1.0` | 맵 발행 주기 (초) |
| `play_rate` | `1.0` | Bag 재생 속도 (1.0 = 실시간) |

### 2.3 사용 예시

#### 기본 실행
```bash
ros2 launch agent_dawgs mapping_cartographer_offline.launch.py \
    bag_file:=/home/dawgs_nx/bags/mapping_data
```

#### RViz 없이 실행 (서버 환경)
```bash
ros2 launch agent_dawgs mapping_cartographer_offline.launch.py \
    bag_file:=/home/dawgs_nx/bags/mapping_data \
    use_rviz:=false
```

#### 빠른 재생 (2배속)
```bash
ros2 launch agent_dawgs mapping_cartographer_offline.launch.py \
    bag_file:=/home/dawgs_nx/bags/mapping_data \
    play_rate:=2.0
```

#### 고해상도 맵 생성
```bash
ros2 launch agent_dawgs mapping_cartographer_offline.launch.py \
    bag_file:=/home/dawgs_nx/bags/mapping_data \
    resolution:=0.02
```

### 2.4 Cartographer 맵 저장

Bag 재생이 완료된 후:

```bash
# 맵 저장 (pbstream 형식 - Cartographer 전용 형식)
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '/home/dawgs_nx/maps/my_map.pbstream'}"

# PGM + YAML 형식으로 저장
ros2 run nav2_map_server map_saver_cli -f /home/dawgs_nx/maps/my_map
```

---

## 3. SLAM Toolbox를 사용한 오프라인 매핑

### 3.1 Launch 파일 실행

```bash
ros2 launch agent_dawgs mapping_slamtoolbox_offline.launch.py \
    bag_file:=/path/to/your/mapping_data
```

### 3.2 Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|-------|------|
| `bag_file` | (필수) | Bag 파일의 전체 경로 |
| `slam_params_file` | `mapper_params_online_async_mapping.yaml` | SLAM Toolbox 설정 파일 |
| `use_rviz` | `true` | RViz2 실행 여부 |
| `play_rate` | `1.0` | Bag 재생 속도 |

### 3.3 사용 예시

#### 기본 실행
```bash
ros2 launch agent_dawgs mapping_slamtoolbox_offline.launch.py \
    bag_file:=/home/dawgs_nx/bags/mapping_data
```

#### 커스텀 설정 파일 사용
```bash
ros2 launch agent_dawgs mapping_slamtoolbox_offline.launch.py \
    bag_file:=/home/dawgs_nx/bags/mapping_data \
    slam_params_file:=/home/dawgs_nx/f1tenth_dawgs/src/agent_dawgs/config/mapper_params_online_async_sim.yaml
```

### 3.4 SLAM Toolbox 맵 저장

SLAM Toolbox는 실시간으로 맵을 저장할 수 있습니다:

#### 방법 1: RViz 플러그인 사용
1. RViz2에서 "Panels" → "Add New Panel" → "SlamToolboxPlugin" 선택
2. "Save Map" 버튼 클릭
3. 파일 이름 입력 (확장자 없이)

#### 방법 2: 서비스 호출
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/home/dawgs_nx/maps/my_map'}}"
```

#### 방법 3: nav2_map_server 사용
```bash
ros2 run nav2_map_server map_saver_cli -f /home/dawgs_nx/maps/my_map
```

---

## 4. 맵 저장

### 4.1 표준 맵 형식 (PGM + YAML)

#### 자동 저장 (nav2_map_server)
```bash
# 현재 맵을 PGM + YAML 형식으로 저장
ros2 run nav2_map_server map_saver_cli -f /home/dawgs_nx/maps/my_map

# 결과 파일:
# - my_map.pgm (이미지)
# - my_map.yaml (메타데이터)
```

#### 저장된 파일 확인
```bash
ls -lh /home/dawgs_nx/maps/
# my_map.pgm
# my_map.yaml
```

### 4.2 YAML 파일 내용 예시
```yaml
image: my_map.pgm
resolution: 0.050000
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

### 4.3 맵 품질 확인
```bash
# 맵 이미지를 이미지 뷰어로 확인
eog /home/dawgs_nx/maps/my_map.pgm

# 또는
display /home/dawgs_nx/maps/my_map.pgm
```

---

## 5. 트러블슈팅

### 5.1 일반적인 문제

#### 문제: "No messages received on /scan"
**원인**: Bag 파일에 /scan 토픽이 없거나 토픽 이름이 다름

**해결방법**:
```bash
# Bag 파일의 토픽 확인
ros2 bag info /path/to/mapping_data

# 토픽 이름이 다른 경우, launch 파일에서 remapping 추가
# 또는 bag 재생 시 remapping:
ros2 bag play mapping_data --remap /lidar/scan:=/scan
```

#### 문제: "Transform timeout"
**원인**: TF 데이터가 bag 파일에 없거나 프레임 이름이 다름

**해결방법**:
```bash
# TF 확인
ros2 bag info mapping_data | grep tf

# TF 트리 확인 (bag 재생 중)
ros2 run tf2_tools view_frames
```

#### 문제: 맵이 왜곡되거나 정렬이 잘못됨
**원인**: 빠른 움직임, 부정확한 odometry, 특징점 부족

**해결방법**:
- 재생 속도를 줄이기: `play_rate:=0.5`
- Cartographer 설정 조정: `src/agent_dawgs/config/cartographer_mapping_2d.lua`에서 파라미터 튜닝
- SLAM Toolbox 설정 조정: YAML 파일에서 `minimum_travel_distance`, `minimum_travel_heading` 값 조정

#### 문제: "Out of memory" 에러
**원인**: 큰 bag 파일이나 긴 녹화 시간

**해결방법**:
```bash
# Bag을 여러 세그먼트로 나누어 처리
ros2 bag play mapping_data --start-offset 0 --duration 60

# 또는 해상도를 낮춤
resolution:=0.1
```

### 5.2 성능 최적화

#### 재생 속도 조정
```bash
# 느린 컴퓨터의 경우 속도를 줄임
play_rate:=0.5

# 빠른 컴퓨터의 경우 속도를 높임
play_rate:=2.0
```

#### 불필요한 토픽 필터링
```bash
# 특정 토픽만 재생
ros2 bag play mapping_data --topics /scan /odom /tf /tf_static
```

### 5.3 프레임 ID 불일치 문제

Bag 파일의 프레임 ID가 설정 파일과 다를 수 있습니다.

#### Cartographer 프레임 확인
`src/agent_dawgs/config/cartographer_mapping_2d.lua`:
```lua
map_frame = "map"
tracking_frame = "laser"
published_frame = "odom"
odom_frame = "odom"
```

#### SLAM Toolbox 프레임 확인
설정 YAML 파일:
```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: base_link  # 또는 ego_racecar/base_link
    map_frame: map
    base_frame: base_link  # 또는 ego_racecar/base_link
```

**해결방법**:
1. Bag 파일의 프레임 확인:
```bash
ros2 bag play mapping_data &
ros2 run tf2_ros tf2_echo map base_link
```

2. 설정 파일 수정하여 프레임 이름 일치시키기

### 5.4 디버깅 팁

#### 1. 노드 상태 확인
```bash
ros2 node list
ros2 topic list
ros2 topic hz /scan
```

#### 2. TF 트리 확인
```bash
# PDF로 TF 트리 저장
ros2 run tf2_tools view_frames
evince frames.pdf
```

#### 3. 로그 레벨 조정
```bash
# Cartographer 디버그 로그
ros2 run cartographer_ros cartographer_node --ros-args --log-level debug

# SLAM Toolbox 디버그 로그
# YAML 파일에서 debug_logging: true 설정
```

#### 4. RViz에서 실시간 확인
- `/map` 토픽: 생성된 맵
- `/scan` 토픽: LiDAR 스캔
- `/tf` 토픽: 로봇 위치
- `/submap_list` (Cartographer): 서브맵 리스트

---

## 6. 추가 리소스

### 관련 문서
- [Cartographer ROS 공식 문서](https://google-cartographer-ros.readthedocs.io/)
- [SLAM Toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Bag 공식 문서](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

### 샘플 스크립트

#### Bag 녹화 스크립트
`scripts/record_mapping_data.sh`:
```bash
#!/bin/bash
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="mapping_${TIMESTAMP}"

echo "Recording bag: ${BAG_NAME}"
ros2 bag record /scan /odom /tf /tf_static -o ${BAG_NAME}
```

#### 자동 맵 생성 스크립트
`scripts/auto_mapping.sh`:
```bash
#!/bin/bash
BAG_FILE=$1
MAP_NAME=$2

if [ -z "$BAG_FILE" ] || [ -z "$MAP_NAME" ]; then
    echo "Usage: $0 <bag_file> <map_name>"
    exit 1
fi

# Launch offline mapping
ros2 launch agent_dawgs mapping_cartographer_offline.launch.py \
    bag_file:=${BAG_FILE} \
    use_rviz:=false &

LAUNCH_PID=$!

# Wait for bag playback to finish
BAG_DURATION=$(ros2 bag info ${BAG_FILE} | grep "Duration" | awk '{print $2}')
sleep ${BAG_DURATION}

# Save map
ros2 run nav2_map_server map_saver_cli -f ${MAP_NAME}

# Kill launch
kill ${LAUNCH_PID}

echo "Map saved as ${MAP_NAME}.pgm and ${MAP_NAME}.yaml"
```

---

## 문의 및 기여

문제가 발생하거나 개선 사항이 있으면 이슈를 등록해주세요.
