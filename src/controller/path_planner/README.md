csv_path_publisher.cpp
● Perfect! I've successfully created a CSV path publisher node for your path_planner ROS2
   package. Here's what I've implemented:

  CSV Path Publisher Node

  Features:
  - Reads CSV files with x,y,v,kappa format (like your
  levine_blacked_lippboyd_speedopted.csv)
  - Publishes nav_msgs/Path messages on configurable topics
  - Automatically calculates path orientations based on waypoint directions
  - Handles CSV headers and malformed lines gracefully
  - Configurable via ROS2 parameters

  Files Created:
  - src/csv_path_publisher.cpp - Main node implementation
  - launch/csv_path_publisher.launch.py - Launch file with parameters
  - Updated CMakeLists.txt to build and install the new executable

  Usage:

  # Launch with specific CSV file
  ros2 launch path_planner csv_path_publisher.launch.py csv_file_path:=/home/dawgs_nx/f1t
  enth_dawgs/src/peripheral/racetracks/levine/levine_blacked_lippboyd_speedopted.csv

  # Or run directly
  ros2 run path_planner csv_path_publisher --ros-args -p
  csv_file_path:=/path/to/your/track.csv -p global_path_topic:=/global_centerline

  Parameters:
  - csv_file_path: Path to your CSV track file (required)
  - global_path_topic: Output topic (default: /global_centerline)
  - frame_id: Coordinate frame (default: map)
  - publish_rate: Publishing frequency (default: 1.0 Hz)

  The node automatically handles the CSV format you showed me and will publish a proper
  ROS2 Path message that your existing path_planner_node can consume via the
  /global_centerline topic.

---- intgrated version
● 완료했습니다! path_planner_node를 성공적으로 수정했습니다:

  주요 변경사항:

  1. CSV 파일 직접 읽기: global_path_topic을 구독하는 대신 CSV 파일을 직접 읽어서 global_path로
  사용합니다.
  2. 시각화 기능 추가:
    - /frenet_path: Frenet 최적 경로 시각화 (z=0.1)
    - /lut_path: Lattice LUT 경로 시각화 (z=0.15)
    - /global_centerline: CSV에서 로드한 전역 경로 발행
  3. 새로운 파라미터:
    - csv_file_path: CSV 파일 경로 지정
    - visualize_paths: 시각화 활성화/비활성화
    - frenet_path_topic, lut_path_topic: 시각화 토픽 이름
    - publish_rate: 전역 경로 발행 주기

  사용 방법:

  # Launch 파일 사용
  ros2 launch path_planner path_planner.launch.py csv_file_path:=/home/dawgs_nx/f1tenth_dawgs/src/p
  eripheral/racetracks/levine/levine_blacked_lippboyd_speedopted_V2.csv

  # 또는 직접 실행
  ros2 run path_planner path_planner_node --ros-args \
    -p csv_file_path:=/path/to/track.csv \
    -p visualize_paths:=true

  RViz2에서 시각화를 보려면:
  - /global_centerline: 참조 경로 (CSV에서 로드)
  - /frenet_path: Frenet 최적화 경로 (녹색)
  - /lut_path: Lattice LUT 경로 (파란색)
  - /planned_path: 최종 계획된 경로


--- frenet conversion comments
● 이제 Frenet 변환 방식에 대해 설명하겠습니다.

  📐 Frenet 좌표계 변환 방식 설명

  1. Frenet 좌표계란?

  Frenet 좌표계는 곡선 경로를 따라 정의되는 좌표계입니다:
  - s (longitudinal): 경로를 따른 거리 (arc length)
  - d (lateral): 경로로부터의 수직 거리 (signed distance)

  2. Cart2Frenet 변환 과정 (Cartesian → Frenet)

  1. KD-tree로 가장 가까운 waypoint 찾기
     - 입력: (x, y) 좌표
     - 출력: 가장 가까운 waypoint의 인덱스

  2. 근처 세그먼트들 검색 (±5 segments)
     - 각 세그먼트에 점을 투영하여 최소 거리 세그먼트 찾기

  3. 세그먼트에 점 투영
     - 세그먼트 벡터: v = (x2-x1, y2-y1)
     - 투영 파라미터: t = dot((x-x1), v) / ||v||²
     - t를 [0,1]로 클램핑
     - 투영점: (px, py) = (x1, y1) + t * v

  4. Frenet 좌표 계산
     - s = waypoint[i].s + t * segment_length
     - d = signed_distance(점에서 투영점까지)
     - 부호는 세그먼트 법선 벡터로 결정

  5. 폐곡선 처리
     - s가 total_length를 초과하면 modulo 연산으로 wrapping
     - 마지막→첫번째 세그먼트도 검색에 포함

  3. Frenet2Cart 변환 과정 (Frenet → Cartesian)

  1. s 좌표 wrapping (폐곡선인 경우)
     - s_wrapped = s % total_length

  2. s를 포함하는 세그먼트 찾기
     - waypoint[i].s <= s_wrapped <= waypoint[i+1].s인 i 찾기

  3. 세그먼트 내 보간
     - r = (s_wrapped - waypoint[i].s) / segment_length
     - 중심선 위치: (cx, cy) = lerp(waypoint[i], waypoint[i+1], r)

  4. 수직 오프셋 적용
     - 세그먼트 방향각: yaw = atan2(dy, dx)
     - 법선 벡터: (nx, ny) = (-sin(yaw), cos(yaw))
     - 최종 위치: (x, y) = (cx, cy) + d * (nx, ny)

  4. 문제 발생 가능 지점들

  1. KD-tree 초기화 실패
    - waypoint가 비어있거나 KD-tree 빌드 실패
  2. 폐곡선 감지 오류
    - 첫점과 끝점 거리가 2m 이내여야 폐곡선으로 인식
    - CSV 파일의 시작/끝 점이 실제로 가까운지 확인 필요
  3. s 좌표 누적 오류
    - waypoint들의 s 값이 제대로 누적되지 않음
    - 폐곡선에서 마지막 세그먼트 길이 누락
  4. 투영 실패
    - 세그먼트가 너무 짧거나 (길이 < 1e-9) degenerate
    - 검색 윈도우가 너무 작아서 올바른 세그먼트 못 찾음
  5. 좌표 wrapping 오류
    - s가 음수이거나 total_length 초과시 처리 오류

● 디버깅 출력이 추가되었습니다. 이제 프로그램을 실행하면 다음과 같은
  정보를 확인할 수 있습니다:

  🔍 디버깅 출력 확인 사항

  1. 경로 로딩시 (set_reference)
    - waypoint 개수와 경로 범위
    - 폐곡선 여부 (첫점-끝점 거리)
    - 총 경로 길이
  2. cart2frenet 변환시
    - 입력 좌표 (x, y)
    - KD-tree로 찾은 가장 가까운 waypoint
    - 각 세그먼트별 투영 결과
    - 최종 Frenet 좌표 (s, d)
  3. frenet2cart 변환시
    - 입력 Frenet 좌표 (s, d)
    - s wrapping 결과
    - 찾은 세그먼트 정보
    - 최종 Cartesian 좌표 (x, y)

  🚀 실행 방법

  # 빌드
  colcon build --packages-select path_planner

  # 실행
  ros2 launch path_planner path_planner.launch.py
  csv_file:=/path/to/track.csv

  실행 후 콘솔 출력에서 [Frenet]로 시작하는 메시지들을 확인하여 어디서
  문제가 발생하는지 파악할 수 있습니다.

--- lattice error
● 완료! 구현 내용을 정리하겠습니다.

  📋 구현 완료 사항

  1. 로그 레벨 제어 시스템

  - log_level 파라미터 추가 (0=NONE ~ 5=VERBOSE)
  - 모든 디버그 메시지가 로그 레벨에 따라 표시/숨김
  - config 파일에서 설정 가능

  2. Frenet Lattice 샘플링 기반 장애물 회피

  작동 원리:

  1. LaserScan → 장애물 검출
    - /scan 토픽에서 레이저 데이터 수신
    - 로봇 좌표계 → 맵 좌표계 변환
    - 5m 이내 장애물만 고려
  2. Frenet Lattice 샘플링
    - Lateral samples: -1.0 ~ 1.0m (9개 샘플)
    - Time horizons: 1.0 ~ 3.0초 (5개 샘플)
    - 총 45개 궤적 후보 생성
  3. 궤적 평가 및 선택
    - 각 궤적의 비용 계산:
        - Lateral jerk (부드러움)
      - Time cost (빠른 도달)
      - Lateral deviation (중심선 유지)
      - Obstacle collision (안전)
    - 충돌 없는 최저 비용 궤적 선택
  4. Fallback 메커니즘
    - 모든 궤적이 충돌시 → 중심선으로 수렴하는 안전 궤적

  3. KD-tree 기반 고속 Frenet 변환

  - Nanoflann KD-tree로 nearest neighbor 검색 가속화
  - Local search window (±5 segments)로 효율성 증대
  - 폐곡선 트랙 자동 감지 및 처리

  🚀 사용 방법

  # 빌드
  colcon build --packages-select path_planner

  # 실행 (로그 레벨 설정)
  ros2 launch path_planner path_planner.launch.py \
    csv_file:=/path/to/track.csv \
    log_level:=3  # INFO 레벨

  # 로그 레벨 옵션
  # 0 = NONE (출력 없음)
  # 1 = ERROR (오류만)
  # 2 = WARN (경고 이상)
  # 3 = INFO (정보 이상)
  # 4 = DEBUG (디버그 정보)
  # 5 = VERBOSE (모든 정보)

  🎯 Frenet Lattice 특징

  1. 다중 궤적 샘플링: 45개의 다양한 궤적 후보
  2. 실시간 장애물 회피: LaserScan 기반 동적 회피
  3. 부드러운 궤적: Quintic polynomial로 jerk 최소화
  4. 속도 최적화: 목표 속도 추종하면서 안전 확보
  5. 폐곡선 지원: 트랙 레이싱에 최적화

  이제 장애물을 피하면서도 최적 경로를 따라가는 local path planning이
  가능합니다!

--- lut updates 

● 완료! 구현한 내용을 정리하겠습니다.

  📋 완성된 구현

  1. Lattice LUT의 Frenet Frame 통합

  lattice_lut.cpp 주요 변경사항:

  - build() 함수: Frenet 중심선 곡률을 기반으로 spiral
  parameter 생성
  - sample() 함수: Frenet 좌표계에서 직접 글로벌 좌표로 변환
  - 곡률 기반 경로 생성: 중심선 곡률을 고려한 polynomial
  curvature profile

  작동 방식:

  // 1. Build phase: 중심선 곡률 기반 LUT 생성
  double k_center = frenet.curvature_at_s(0.0);
  sp.a = k_center;  // 기본 곡률
  sp.b = (2.0 * d) / (sp.s * sp.s + 1e-3);  // 횡방향 오프셋
  효과
  sp.c = (6.0 * dth) / (sp.s * sp.s + 1e-3);  // 헤딩 변화 효과

  // 2. Sample phase: Frenet -> Cartesian 직접 변환
  frenet.frenet2cart(s_frenet, d_frenet, gx, gy, gyaw);

  2. Path Planner의 Odometry Callback 기반 변경

  변경사항:

  - Timer 제거: 50ms timer 대신 odometry callback 사용
  - Real-time Planning: 새로운 odometry 데이터마다 경로 계획
  실행
  - Reduced Latency: 센서 데이터 즉시 반영

  작동 흐름:

  Odometry 수신 → planPath() 실행 → Frenet 변환 → 장애물 검출 →

  경로 생성 (Frenet + Lattice) → Planned Path 발행

  3. 통합된 Frenet 기반 경로 생성

  2단계 경로 계획:

  1. Frenet Lattice 샘플링: 45개 장애물 회피 후보 궤적
  2. Lattice LUT 보완: 목표 지점 도달을 위한 정밀 spiral 경로

  Frenet 좌표계 이점:

  - 곡률 적응: 트랙의 실제 곡률을 반영한 자연스러운 궤적
  - 정확한 변환: KD-tree + 폐곡선 처리로 안정적인 좌표 변환
  - 통합 프레임워크: 모든 계획 단계가 같은 좌표계 사용

  🚀 실행 방법

  # 빌드
  colcon build --packages-select path_planner

  # 실행 예시
  ros2 launch path_planner path_planner.launch.py \
    csv_file:=/path/to/track.csv \
    log_level:=3 \
    use_frenet:=true \
    use_lattice:=true

  📊 성능 특징

  1. 실시간 반응: Odometry 주기(~100Hz)에 맞춘 경로 계획
  2. 정확한 곡률 추종: 중심선 곡률을 반영한 자연스러운 spiral
  3. 효율적인 장애물 회피: Frenet lattice + LUT spiral 조합
  4. 폐곡선 지원: 레이싱 트랙에 최적화된 좌표 wrapping
  5. 로그 레벨 제어: 디버깅 정보 세밀 조절 가능

  이제 Frenet frame 기반의 통합된 경로 계획이 odometry
  callback으로 실시간 동작합니다!


-- tunable fixes

● 이제 planner.yaml 설정 파일에 대한 상세한 설명을
  제공하겠습니다:

  Frenet Path Planner 파라미터 설명

  1. 시간 관련 파라미터

  - frenet_time_horizon: 3.0 - Frenet 경로의 최대 예측 시간 (초)
    - 더 크게 하면 더 먼 미래까지 경로 생성
    - 권장 범위: 2.0~5.0초
  - frenet_min_time: 1.0 - 최소 예측 시간 (초)
    - 너무 짧으면 반응이 급격함
  - frenet_dt: 0.05 - 경로 샘플링 시간 간격 (초)
    - 작을수록 더 많은 포인트 생성 (더 부드럽지만 계산 증가)
    - 더 긴 경로 원하면: frenet_time_horizon 증가 + frenet_dt
  유지

  2. 속도/가속도 제한

  - frenet_target_speed: 3.0 - 목표 속도 (m/s)
    - 경로 거리 = 대략 target_speed × time_horizon
    - 3.0 m/s × 3.0 s = 9m 경로
  - frenet_max_speed: 15.0 - 최대 허용 속도 (m/s)
  - frenet_max_accel: 4.0 - 최대 횡가속도 (m/s²)

  3. 횡방향 샘플링 (장애물 회피)

  - frenet_d_samples - 중심선으로부터의 횡방향 오프셋 (미터)
  [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0]
    - 음수 = 왼쪽, 양수 = 오른쪽
    - 더 많은 샘플 = 더 많은 궤적 후보 (계산 증가)

  4. 시간 샘플링

  - frenet_t_samples: [1.5, 2.0, 2.5, 3.0] - 시간 지평선 후보들
    - 각 시간에 대해 다른 궤적 생성
    - 더 긴 경로 원하면: [2.0, 2.5, 3.0, 3.5, 4.0] 추가

  5. 비용 함수 가중치 (경로 선택 기준)

  - frenet_k_jerk: 0.1 - 저크(jerk) 페널티 (부드러움)
    - 크게 하면 더 부드러운 경로 선호
  - frenet_k_time: 0.1 - 시간 페널티
    - 크게 하면 빠른 경로 선호
  - frenet_k_deviation: 1.0 - 중심선 이탈 페널티
    - 크게 하면 중심선 가까이 유지
  - frenet_k_velocity: 1.0 - 속도 오차 페널티

  6. 안전 파라미터

  - frenet_safety_radius: 0.3 - 장애물 안전 반경 (미터)
  - frenet_road_half_width: 1.2 - 주행 가능 영역 반폭 (미터)

  경로를 더 길게 만들려면:

  # 방법 1: 시간 지평선 증가
  frenet_time_horizon: 4.0
  frenet_t_samples: [2.0, 2.5, 3.0, 3.5, 4.0]

  # 방법 2: 목표 속도 증가
  frenet_target_speed: 5.0  # 5 m/s × 4 s = 20m 경로

  # 방법 3: dt는 유지 (포인트 밀도)
  frenet_dt: 0.05  # 4초 / 0.05 = 80개 포인트

  주요 변경사항:
  1. ✅ Frenet path의 모든 포인트 사용 (i%2==0 제거)
  2. ✅ YAML에서 파라미터 설정 가능
  3. ✅ 런타임에 파라미터 조정 가능

  빌드 후 테스트:
  colcon build --packages-select path_planner
  source install/setup.bash


--- record speed on global path
● 완료했습니다! path_planner 패키지에서 global_path를 발행할
   때 속도 정보를 포함하도록 수정했습니다.

  수정 내용

  1. path_planner_node.cpp (src/controller/path_planner/src/
  path_planner_node.cpp:240)

  - CSV에서 속도 v를 파싱 (이미 되어 있었음)
  - pose.pose.position.z에 속도 저장 (0.0 → v로 변경)

  2. csv_path_publisher.cpp (src/controller/path_planner/src
  /csv_path_publisher.cpp:90, 96)

  - CSV 3번째 컬럼에서 속도 파싱 추가
  - pose.pose.position.z에 속도 저장

  사용 방법

  발행된 /global_centerline 토픽의 각 PoseStamped에서:
  - pose.position.x: x 좌표
  - pose.position.y: y 좌표
  - pose.position.z: 속도 (m/s) ← 새로 추가됨
  - pose.orientation: 방향 (quaternion)

  빌드 및 실행

  colcon build --packages-select path_planner
  source install/setup.bash
  ros2 launch path_planner path_planner.launch.py  # 또는
  적절한 launch 파일

  참고

  - 2D 경로에서는 z 좌표를 사용하지 않으므로 속도 정보를
  저장하는 용도로 활용
  - CSV 파일 포맷: x,y,v,kappa (속도가 없으면 기본값 1.0
  m/s)
  - 다른 노드에서 이 경로를 구독할 때 pose.position.z에서
  속도를 읽을 수 있음

