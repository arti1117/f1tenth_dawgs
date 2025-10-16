# Vehicle Calibration Package

이 패키지는 F1TENTH 차량의 마찰원(friction circle) 테스트와 ERPM/스티어링 게인 캘리브레이션을 위한 ROS2 노드들을 포함합니다.

## 패키지 구성

### 노드들
1. **friction_circle_node.py** - 차량의 최대 코너링 성능을 테스트하는 마찰원 생성
2. **erpm_calibration_node.py** - 속도 명령과 실제 ERPM 간의 관계를 캘리브레이션
3. **steering_calibration_node.py** - 스티어링 명령과 실제 회전 반경 간의 관계를 캘리브레이션

### Launch 파일들
- `friction_circle.launch.py` - 마찰원 테스트 실행
- `erpm_calibration.launch.py` - ERPM 캘리브레이션 실행
- `steering_calibration.launch.py` - 스티어링 캘리브레이션 실행
- `vehicle_calibration.launch.py` - 통합 캘리브레이션 실행

## 설치 및 빌드

```bash
cd ~/f1tenth_dawgs
colcon build --packages-select vehicle_calibration
source install/setup.bash
```

## 사용법

### 1. 마찰원 테스트

```bash
# 기본 설정으로 실행
ros2 launch vehicle_calibration friction_circle.launch.py

# 커스텀 파라미터로 실행
ros2 launch vehicle_calibration friction_circle.launch.py \
    max_speed:=5.0 \
    test_duration:=8.0 \
    plot_real_time:=false

# 테스트 시작/중지
ros2 topic pub /friction_circle/start std_msgs/Bool "data: true" --once
ros2 topic pub /friction_circle/stop std_msgs/Bool "data: true" --once
```

**주요 파라미터:**
- `max_speed` (default: 3.0) - 최대 테스트 속도 (m/s)
- `speed_increment` (default: 0.2) - 속도 증가량 (m/s)
- `test_duration` (default: 10.0) - 각 속도별 테스트 시간 (s)
- `plot_real_time` (default: true) - 실시간 플롯 표시

### 2. ERPM 캘리브레이션

```bash
# 기본 설정으로 실행
ros2 launch vehicle_calibration erpm_calibration.launch.py

# 커스텀 파라미터로 실행
ros2 launch vehicle_calibration erpm_calibration.launch.py \
    max_speed:=4.0 \
    initial_erpm_gain:=3500

# 캘리브레이션 시작/중지
ros2 topic pub /erpm_calibration/start std_msgs/Bool "data: true" --once
ros2 topic pub /erpm_calibration/stop std_msgs/Bool "data: true" --once
```

**주요 파라미터:**
- `max_speed` (default: 4.0) - 최대 캘리브레이션 속도 (m/s)
- `speed_increment` (default: 0.25) - 속도 증가량 (m/s)
- `test_duration` (default: 8.0) - 각 속도별 테스트 시간 (s)
- `initial_erpm_gain` (default: 4000) - 초기 ERPM 게인 추정값

### 3. 스티어링 캘리브레이션

```bash
# 기본 설정으로 실행
ros2 launch vehicle_calibration steering_calibration.launch.py

# 커스텀 파라미터로 실행
ros2 launch vehicle_calibration steering_calibration.launch.py \
    test_speed:=1.5 \
    max_steering_angle:=0.5

# 캘리브레이션 시작/중지
ros2 topic pub /steering_calibration/start std_msgs/Bool "data: true" --once
ros2 topic pub /steering_calibration/stop std_msgs/Bool "data: true" --once
```

**주요 파라미터:**
- `test_speed` (default: 1.0) - 테스트 속도 (m/s)
- `max_steering_angle` (default: 0.4) - 최대 스티어링 각도 (rad)
- `steering_increment` (default: 0.05) - 스티어링 각도 증가량 (rad)
- `test_duration` (default: 12.0) - 각 스티어링별 테스트 시간 (s)

### 4. 통합 캘리브레이션

```bash
# 모든 캘리브레이션 실행
ros2 launch vehicle_calibration vehicle_calibration.launch.py run_all:=true

# 특정 캘리브레이션만 실행
ros2 launch vehicle_calibration vehicle_calibration.launch.py run_erpm:=true
ros2 launch vehicle_calibration vehicle_calibration.launch.py run_steering:=true
ros2 launch vehicle_calibration vehicle_calibration.launch.py run_friction_circle:=true
```

## 토픽 인터페이스

### Subscribed Topics
- `/odom` (nav_msgs/Odometry) - 차량 위치 및 속도 정보
- `/imu` (sensor_msgs/Imu) - 가속도 및 각속도 정보 (마찰원 테스트용)
- `/sensors/erpm` (std_msgs/Int32) - 모터 ERPM 정보

### Published Topics
- `/drive` (ackermann_msgs/AckermannDriveStamped) - 차량 제어 명령
- `/{node_name}/status` (std_msgs/String) - 캘리브레이션 상태 정보
- `/erpm_calibration/gain` (std_msgs/Float32) - 최적 ERPM 게인
- `/steering_calibration/gain` (std_msgs/Float32) - 최적 스티어링 게인
- `/steering_calibration/offset` (std_msgs/Float32) - 최적 스티어링 오프셋

### Service Topics
- `/{node_name}/start` (std_msgs/Bool) - 캘리브레이션 시작
- `/{node_name}/stop` (std_msgs/Bool) - 캘리브레이션 중지

## 결과 파일

캘리브레이션 결과는 다음 디렉터리에 저장됩니다:
- 기본 경로: `/home/dawgs_nx/f1tenth_dawgs/data/calibration/`

### 생성되는 파일들:
1. **데이터 파일 (CSV)**
   - `friction_circle_test_YYYYMMDD_HHMMSS.csv`
   - `erpm_calibration_data_YYYYMMDD_HHMMSS.csv`
   - `steering_calibration_data_YYYYMMDD_HHMMSS.csv`

2. **결과 파일 (JSON)**
   - `erpm_calibration_results_YYYYMMDD_HHMMSS.json`
   - `steering_calibration_results_YYYYMMDD_HHMMSS.json`

3. **설정 파일 (TXT)**
   - `erpm_gain_config.txt`
   - `steering_calibration_config.txt`

4. **분석 플롯 (PNG)**
   - `friction_circle_analysis_YYYYMMDD_HHMMSS.png`
   - `erpm_calibration_YYYYMMDD_HHMMSS.png`
   - `steering_calibration_YYYYMMDD_HHMMSS.png`

## 안전 주의사항

1. **테스트 환경**: 넓고 안전한 공간에서 실행하세요
2. **속도 제한**: 차량과 환경에 맞는 적절한 최대 속도를 설정하세요
3. **비상 정지**: 언제든지 `/{node_name}/stop` 토픽으로 테스트를 중단할 수 있습니다
4. **모니터링**: 실시간 플롯과 상태 메시지를 통해 테스트를 모니터링하세요

## 캘리브레이션 품질 평가

- **R² > 0.95**: 우수한 캘리브레이션
- **R² > 0.90**: 양호한 캘리브레이션
- **R² > 0.80**: 보통 캘리브레이션
- **R² < 0.80**: 재캘리브레이션 권장

## 문제 해결

### 일반적인 문제들:

1. **노드 실행 실패**
   ```bash
   # 의존성 설치 확인
   sudo apt install python3-matplotlib python3-scipy python3-pandas
   ```

2. **플롯이 표시되지 않음**
   - X11 forwarding 확인 (SSH 사용 시)
   - `plot_real_time:=false` 옵션 사용

3. **캘리브레이션 품질이 낮음**
   - 테스트 시간 증가 (`test_duration` 파라미터)
   - 더 많은 데이터 포인트 수집 (더 작은 `increment` 값)
   - 테스트 환경 개선 (평평한 바닥, 장애물 제거)

4. **ERPM 데이터 수신 안됨**
   - `/sensors/erpm` 토픽 확인: `ros2 topic echo /sensors/erpm`
   - 모터 컨트롤러 연결 상태 확인

## Dependencies

- rclpy
- numpy
- matplotlib
- scipy
- pandas
- tf2_ros
- ackermann_msgs
- sensor_msgs
- nav_msgs
- geometry_msgs

### Few changes
● vehicle_calibration 패키지를 사용해서 VESC 파라미터를 캘리브레이션하는
  방법을 설명하겠습니다.

  캘리브레이션 프로세스

  1. 패키지 빌드 및 설치

● Bash(colcon build --packages-select vehicle_calibration)
  ⎿  Starting >>> vehicle_calibration
      Finished <<< vehicle_calibration [12.8s]

     Summary: 1 package finished [17.1s]

● 2. ERPM 캘리브레이션

  현재 설정값: speed_to_erpm_gain: 4614.0, speed_to_erpm_offset: 0.0

  # 터미널 1 - ERPM 캘리브레이션 실행
  source install/setup.bash
  ros2 launch vehicle_calibration erpm_calibration.launch.py \
      max_speed:=3.0 \
      initial_erpm_gain:=4614

  # 터미널 2 - 캘리브레이션 시작
  ros2 topic pub /erpm_calibration/start std_msgs/Bool "data: true" --once

  차량이 직진하면서 자동으로 속도를 증가시키며 테스트합니다. 완료되면:
  - /home/dawgs_nx/f1tenth_dawgs/data/calibration/erpm_calibration_results
  _*.json 파일 확인
  - 새로운 speed_to_erpm_gain과 speed_to_erpm_offset 값 획득

  3. 스티어링 캘리브레이션

● 현재 설정값: steering_angle_to_servo_gain: -1.2135,
  steering_angle_to_servo_offset: 0.5304

  # 터미널 1 - 스티어링 캘리브레이션 실행
  source install/setup.bash
  ros2 launch vehicle_calibration steering_calibration.launch.py \
      test_speed:=1.0 \
      max_steering_angle:=0.4

  # 터미널 2 - 캘리브레이션 시작
  ros2 topic pub /steering_calibration/start std_msgs/Bool "data: true" --once

  차량이 다양한 각도로 원을 그리며 회전 반경을 측정합니다. 완료되면:
  - /home/dawgs_nx/f1tenth_dawgs/data/calibration/steering_calibration_res
  ults_*.json 파일 확인
  - 새로운 steering_angle_to_servo_gain과 steering_angle_to_servo_offset
  값 획득

  4. VESC 설정 업데이트

● 캘리브레이션 결과를 확인한 후, vesc.yaml 파일을 업데이트합니다:

  # 캘리브레이션 결과 확인
  cat /home/dawgs_nx/f1tenth_dawgs/data/calibration/erpm_calibration_results_*.json
  cat /home/dawgs_nx/f1tenth_dawgs/data/calibration/steering_calibration_results_*.json

  # VESC 설정 파일 편집
  nano src/base_system/f1tenth_system/f1tenth_stack/config/vesc.yaml

  예시 업데이트:
  /**:
    ros__parameters:
      # ERPM 캘리브레이션 결과로 업데이트
      speed_to_erpm_gain: 4532.7  # 캘리브레이션된 값
      speed_to_erpm_offset: 12.3  # 캘리브레이션된 값

      # 스티어링 캘리브레이션 결과로 업데이트
      steering_angle_to_servo_gain: -1.185  # 캘리브레이션된 값
      steering_angle_to_servo_offset: 0.524  # 캘리브레이션된 값

  5. 변경사항 적용

  # 패키지 재빌드
  colcon build --packages-select f1tenth_stack

  # 시스템 재시작
  ros2 launch agent_dawgs agent_dawgs.launch.py

● 통합 캘리브레이션 옵션

  모든 캘리브레이션을 순차적으로 실행하려면:

  ros2 launch vehicle_calibration vehicle_calibration.launch.py
  run_all:=true

  주의사항

  1. 안전한 환경: 넓고 평평한 공간에서 테스트
  2. 데이터 디렉토리 생성: 결과 저장을 위해 필요시 생성
  mkdir -p /home/dawgs_nx/f1tenth_dawgs/data/calibration/
  3. 백업: 기존 설정값을 백업해두세요
  4. 점진적 테스트: 낮은 속도부터 시작하여 안전하게 테스트

  캘리브레이션 완료 후 R² 값이 0.90 이상이면 양호한 캘리브레이션입니다.
  값이 낮으면 테스트 시간을 늘리거나 더 많은 데이터 포인트를 수집하세요.
