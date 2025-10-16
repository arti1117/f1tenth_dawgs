# Steering lookup
This package provides an importable library to use the steering lookup obtained from sysid experiments.

## How to import python lib 

Build the library (catkin build steering_lookup) and source the workspace. Then you can import it in another ros package like so:

```python
from steering_lookup.lookup_steer_angle import LookupSteerAngle

# [...]

steer_lookup = LookupSteerAngle('NUC1_pacejka')
accel = 5.0 # m/s2
vel = 3.5   # m/s
steer_angle = steer_lookup.lookup_steer_angle(accel, vel)
# Output steer angle:
# rospy.loginfo(steer_angle)

```

--- added for global_planner migration

● 자, 이제 완벽한 분석을 제공할 수 있습니다! Steering lookup 테이블 생성에 필요한 파라미터를
  정리해드리겠습니다.

  🎯 Steering Lookup 테이블 생성 필요 파라미터

  📊 기본 개념

  Steering Lookup 테이블이란?
  - 명령한 조향각(commanded angle)과 실제 조향각(actual angle) 간의 매핑 테이블
  - 서보 모터의 비선형성, 기계적 유격, 조향 기어비 오차 보정
  - 조향각 → 실제 회전 반경 → 역계산으로 실제 조향각 추정

  ---
  1️⃣ Global Planner에서 이미 있는 파라미터

  ✅ Global Planner에서 가져올 수 있는 값:

  | 파라미터        | 위치                                                             | 용도
           |
  |-------------|----------------------------------------------------------------|----------------
  -|
  | wheelbase   | vehicle_params_mintime["wheelbase_front"] + ["wheelbase_rear"] | 자전거 모델
  회전 반경 계산 |
  | track_width | vehicle_params_mintime["track_width_front/rear"]               | 좌우 바퀴 거리
  (옵션)   |
  | length      | veh_params["length"]                                           | 차량 길이 (안전
   체크)   |
  | width       | veh_params["width"]                                            | 차량 폭 (안전
  체크)    |
  | v_max       | veh_params["v_max"]                                            | 최대 속도 (안전
   범위)   |
  | delta_max   | vehicle_params_mintime["delta_max"]                            | 최대 조향각
  (rad)    |

  파일 위치:
  # config/global_planner/racecar_f110.ini
  [GENERAL_OPTIONS]
  veh_params = {
      "length": 0.535,
      "width": 0.30,
      "v_max": 15.0,
  }

  [OPTIMIZATION_OPTIONS]
  vehicle_params_mintime = {
      "wheelbase_front": 0.15875,
      "wheelbase_rear": 0.17145,
      "track_width_front": 0.281,
      "track_width_rear": 0.281,
      "delta_max": 0.34
  }

  ---
  2️⃣ 추가로 필요한 파라미터 (Steering Calibration 전용)

  ⚠️ Global Planner에 없는 파라미터:

  | 파라미터                    | 기본값  | 단위  | 설명             | 필요 이유     |
  |-------------------------|------|-----|----------------|-----------|
  | min_steering_angle      | -0.4 | rad | 최소 조향각 (~-23°) | 테스트 범위 설정 |
  | max_steering_angle      | 0.4  | rad | 최대 조향각 (~23°)  | 테스트 범위 설정 |
  | steering_increment      | 0.05 | rad | 조향각 증분 (~2.9°) | 테이블 해상도   |
  | test_speed              | 1.0  | m/s | 캘리브레이션 속도      | 일정 속도 유지  |
  | test_duration           | 12.0 | s   | 각 테스트 지속 시간    | 충분한 원호 기록 |
  | settling_time           | 2.0  | s   | 조향 안정화 시간      | 과도 상태 제거  |
  | min_turn_radius         | 0.5  | m   | 최소 유효 회전 반경    | 이상치 필터링   |
  | max_turn_radius         | 20.0 | m   | 최대 유효 회전 반경    | 직선 구분     |
  | initial_steering_gain   | 1.0  | -   | 초기 게인 추정       | 최적화 시작점   |
  | initial_steering_offset | 0.0  | rad | 초기 오프셋 추정      | 최적화 시작점   |

  ---
  3️⃣ 캘리브레이션 데이터 수집 파라미터

  실시간 측정 필요:

  | 데이터             | 소스 토픽     | 용도        |
  |-----------------|-----------|-----------|
  | Position (x, y) | /odom     | 원호 궤적 기록  |
  | Heading (θ)     | /odom     | 회전 각도 계산  |
  | Speed (v)       | /odom     | 속도 일정성 확인 |
  | Timestamp       | ROS clock | 시간 동기화    |

  ---
  📋 Lookup 테이블 생성 알고리즘

  Step 1: 캘리브레이션 실험

  # steering_calibration_node.py의 프로세스
  for steering_cmd in range(min_steering, max_steering, increment):
      1. 명령한 조향각 적용: send_drive_command(test_speed, steering_cmd)
      2. settling_time 대기 (서보 안정화)
      3. test_duration 동안 위치 기록: (x, y, t)
      4. 원호 fitting: fit_circle_least_squares(x, y)
      5. 실측 회전 반경 계산: R_measured
      6. 실제 조향각 역계산: δ_actual = atan(wheelbase / R_measured)

  Step 2: 게인/오프셋 계산

  # 선형 관계 추정
  # δ_actual = gain * δ_commanded + offset

  # 최소제곱법으로 최적 게인 찾기
  def fit_function(params):
      gain, offset = params
      predicted = gain * commanded_angles + offset
      return sum((actual_angles - predicted)^2)

  optimal_gain, optimal_offset = minimize(fit_function)

  Step 3: Lookup 테이블 생성

  # 옵션 1: 선형 보정 함수 사용 (간단)
  def corrected_steering(commanded_angle):
      return optimal_gain * commanded_angle + optimal_offset

  # 옵션 2: 테이블 기반 보간 (정확)
  lookup_table = {
      commanded_angle_i: actual_angle_i
      for all calibration points
  }

  def lookup_steering(commanded_angle):
      # Linear interpolation between table points
      return np.interp(commanded_angle, table_keys, table_values)

  ---
  🔗 Global Planner 파라미터와의 통합

  필요한 파라미터 매핑:

  # global_planner의 racecar_f110.ini에서 추출
  from configparser import ConfigParser
  import json

  parser = ConfigParser()
  parser.read('config/global_planner/racecar_f110.ini')

  # 차량 파라미터 로드
  veh_params = json.loads(parser.get('GENERAL_OPTIONS', 'veh_params'))
  vehicle_params_mintime = json.loads(parser.get('OPTIMIZATION_OPTIONS',
  'vehicle_params_mintime'))

  # Steering calibration에 필요한 값 추출
  steering_calib_params = {
      'wheelbase': vehicle_params_mintime['wheelbase_front'] +
  vehicle_params_mintime['wheelbase_rear'],
      'min_steering_angle': -vehicle_params_mintime['delta_max'],  # 사용 가능
      'max_steering_angle': vehicle_params_mintime['delta_max'],   # 사용 가능
      'test_speed': 1.0,  # 새로 추가 필요
      'test_duration': 12.0,  # 새로 추가 필요
      # ... 기타 캘리브레이션 전용 파라미터
  }

  ---
  📊 Lookup 테이블 예시

  생성된 테이블 형식:

  {
    "calibration_date": "2025-10-12",
    "vehicle_id": "f110_dawgs_01",
    "wheelbase": 0.33020,
    "optimal_gain": 0.9234,
    "optimal_offset": 0.0123,
    "r_squared": 0.9876,
    "rmse_degrees": 1.23,

    "lookup_table": [
      {"commanded": -0.40, "actual": -0.3845, "measured_radius": 0.89},
      {"commanded": -0.35, "actual": -0.3298, "measured_radius": 1.02},
      {"commanded": -0.30, "actual": -0.2801, "measured_radius": 1.21},
      ...
      {"commanded": 0.30, "actual": 0.2789, "measured_radius": 1.22},
      {"commanded": 0.35, "actual": 0.3289, "measured_radius": 1.03},
      {"commanded": 0.40, "actual": 0.3821, "measured_radius": 0.90}
    ]
  }

  ---
  🛠️ 실전 워크플로우

  1단계: Global Planner 파라미터 확인

  cd src/utilities/global_planner/config/global_planner/
  cat racecar_f110.ini | grep -A 15 "vehicle_params_mintime"

  확인할 값:
  - wheelbase_front + wheelbase_rear → 총 휠베이스
  - delta_max → 최대 조향각 범위

  2단계: Steering Calibration 실행

  # 캘리브레이션 노드 실행
  ros2 launch vehicle_calibration steering_calibration.launch.py

  # 별도 터미널에서 시작 트리거
  ros2 topic pub /steering_calibration/start std_msgs/msg/Bool "data: true" --once

  실험 과정:
  1. 차량이 -0.4rad부터 시작하여 0.05rad 간격으로 조향
  2. 각 조향각에서 12초간 원을 그리며 이동
  3. 위치 데이터로 회전 반경 계산
  4. 실제 조향각 역산

  3단계: 결과 분석 및 저장

  # 결과 저장 위치
  ls data/calibration/
  # steering_calibration_data_YYYYMMDD_HHMMSS.csv
  # steering_calibration_results_YYYYMMDD_HHMMSS.json
  # steering_calibration_config.txt

  4단계: Lookup 테이블 생성

  # lookup 테이블 생성 스크립트 (예시)
  import json
  import numpy as np

  # 캘리브레이션 결과 로드
  with open('data/calibration/steering_calibration_results_20251012.json') as f:
      results = json.load(f)

  # Lookup 테이블 생성
  lookup_table = []
  for cmd, actual, radius in zip(
      results['steering_commands'],
      results['actual_angles'],  # 역계산된 실제 각도
      results['measured_radii']
  ):
      lookup_table.append({
          'commanded': cmd,
          'actual': actual,
          'measured_radius': radius
      })

  # 저장
  with open('config/steering_lookup_table.json', 'w') as f:
      json.dump({
          'optimal_gain': results['optimal_gain'],
          'optimal_offset': results['optimal_offset'],
          'lookup_table': lookup_table
      }, f, indent=2)

  ---
  ⚙️ 추가 필요 파라미터 vs Global Planner 비교

  Global Planner에 이미 있는 것 ✅:

  - wheelbase (합산 가능)
  - delta_max (조향 범위)
  - length, width (차량 크기)
  - v_max (속도 제한)

  추가로 설정 필요 ❌:

  - 테스트 프로토콜: test_speed, test_duration, settling_time
  - 테이블 해상도: steering_increment (얼마나 촘촘하게 측정할지)
  - 데이터 검증: min/max_turn_radius (이상치 필터링)
  - 최적화 초기값: initial_steering_gain, initial_steering_offset

  ---
  💡 권장 설정

  F1TENTH 차량 기준:

  # steering_calibration_params.yaml
  steering_calibration:
    # Global Planner에서 가져오기
    wheelbase: 0.33020  # wheelbase_front + wheelbase_rear

    # 추가 설정 필요
    min_steering_angle: -0.34  # delta_max와 동일
    max_steering_angle: 0.34   # delta_max와 동일
    steering_increment: 0.05   # ~3° 간격 (높은 해상도)

    test_speed: 1.0            # 안전한 저속
    test_duration: 12.0        # 충분한 원호 (270° 이상)
    settling_time: 2.0         # 서보 안정화

    min_turn_radius: 0.5       # 물리적 최소값
    max_turn_radius: 20.0      # 직선 판별

  ---
  요약: Global Planner 파라미터(wheelbase, delta_max)는 기본값으로 사용하고, 캘리브레이션 실험
  프로토콜 파라미터(test_speed, test_duration 등)를 추가로 설정하면 Steering Lookup 테이블을
  생성할 수 있습니다!

