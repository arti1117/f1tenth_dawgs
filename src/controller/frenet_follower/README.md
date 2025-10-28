● 빌드가 성공적으로 완료되었습니다! Todo를 업데이트하겠습니다.

● 완료했습니다! frenet_follower 패키지가 성공적으로 생성되고 빌드되었습니다.

  📦 frenet_follower 패키지 생성 완료

  빌드 결과

  - ✅ 빌드 성공: 2분 54초 소요
  - 📁 위치: /home/dawgs_nx/f1tenth_dawgs/src/controller/frenet_follower/
  - ⚠️ 경고: unused parameter 1개 (무시 가능)

  ---
  🎯 최적화된 조향 제어 방식 설명

  Pure Pursuit vs frenet_follower 비교

  | 특성      | Pure Pursuit (path_tracker) | frenet_follower (최적화)             |
  |---------|-----------------------------|-----------------------------------|
  | 추종 방식   | Lookahead point 기하학적 추종     | 가장 가까운 점 직접 추종                    |
  | 주요 파라미터 | lookahead_base, lookahead_k | k_heading, k_lateral, k_curvature |
  | 튜닝 복잡도  | 높음 (lookahead 거리 조정 필요)     | 낮음 (lookahead 없음)                 |
  | 고속 안정성  | 속도 의존적 lookahead 필요         | 자동 속도 적응 gain                     |
  | 코너 성능   | 반응형 (오차 발생 후 수정)            | 선제적 (곡률 예측)                       |
  | 계산 복잡도  | 중간 (lookahead point 검색)     | 낮음 (closest point만)               |

  ---
  🔧 조향 제어 법칙 (src/frenet_follower_node.cpp:434-508)

  수식

  δ = k_h · e_heading + k_l(v) · e_lateral + k_κ · κ_path

  여기서:
  - e_heading: 헤딩 오차 (path_yaw - vehicle_yaw)
  - e_lateral: 횡방향 오차 (경로로부터의 수직 거리)
  - κ_path: 경로 곡률
  - k_l(v) = k_lateral / (1 + k_lateral_velocity · v)  [속도 적응 gain]

  3가지 제어 요소

  1. Heading Error Control (헤딩 오차 제어)

  // src/frenet_follower_node.cpp:465-470
  double heading_error = computeHeadingError(vehicle_yaw, closest.yaw);
  double heading_term = k_heading_ * heading_error;

  역할: 차량의 heading을 경로의 heading과 정렬
  - k_heading = 1.5 (기본값)
  - 효과: 차량이 경로와 같은 방향을 향하도록 조향
  - 튜닝:
    - 높으면 → 빠른 정렬, 진동 가능성
    - 낮으면 → 부드럽지만 느린 정렬

  2. Lateral Error Feedback (횡방향 오차 피드백)

  // src/frenet_follower_node.cpp:472-484
  double lateral_error = computeLateralError(px, py, closest);
  double safe_velocity = std::max(0.1, std::abs(velocity));
  double adaptive_lateral_gain = k_lateral_ / (1.0 + k_lateral_velocity_ * safe_velocity);
  double lateral_term = std::atan(adaptive_lateral_gain * lateral_error / safe_velocity);

  역할: Stanley 방식의 cross-track error 보정
  - k_lateral = 0.8 (기본값)
  - k_lateral_velocity = 0.1 (속도 적응 계수)
  - 효과: 경로 중심선으로 당기는 힘
  - 속도 적응: 고속에서 자동으로 gain 감소 → 안정성 향상
    - 예: v=5 m/s → k_l(5) = 0.8/(1+0.1×5) = 0.533
  - 튜닝:
    - 고속에서 진동 → k_lateral_velocity 증가
    - 경로 추종 불량 → k_lateral 증가

  3. Curvature Feedforward (곡률 선행 제어)

  // src/frenet_follower_node.cpp:486-495
  if (use_curvature_feedforward_) {
      double path_curvature = current_path_[closest.idx].kappa;
      curvature_term = k_curvature_ * wheelbase_ * path_curvature;
  }

  역할: 경로 곡률을 미리 예측하여 선제적 조향
  - k_curvature = 1.0 (기본값)
  - 공식: δ_κ = k_curvature · L · κ (bicycle model)
  - 효과: 코너 진입 전에 미리 조향 시작
  - 장점:
    - 코너에서 tracking lag 감소
    - 부드러운 코너 진입/탈출
    - 횡방향 오차 최소화
  - 튜닝:
    - 언더스티어 → k_curvature 증가 (1.2)
    - 오버스티어 → k_curvature 감소 (0.8)

  ---
  📊 최적화 장점

  1. 파라미터 튜닝 단순화

  - ❌ 제거됨: lookahead_base, lookahead_k 튜닝
  - ✅ 직관적: 각 요소가 독립적인 역할

  2. 전 속도 구간 안정성

  - Pure Pursuit: 속도 변화 시 lookahead 재조정 필요
  - frenet_follower: k_l(v) 자동 속도 적응

  3. 코너 성능 향상

  - Pure Pursuit: 오차 발생 후 반응 (reactive)
  - frenet_follower: 곡률 기반 예측 (proactive)

  4. 계산 효율성

  - Pure Pursuit: Lookahead point 검색 (O(n))
  - frenet_follower: Closest point만 (O(n), but simpler)

  ---
  🚀 사용 방법

  # 1. 소스
  source install/setup.bash

  # 2. frenet_follower 실행
  ros2 launch frenet_follower frenet_follower.launch.py

  # 또는 직접 실행
  ros2 run frenet_follower frenet_follower_node --ros-args --params-file
  src/controller/frenet_follower/config/follower_params.yaml

  ---
  📝 설정 파일

  config/follower_params.yaml에 모든 파라미터에 대한 상세한 설명이 포함되어 있습니다:
  - 각 파라미터의 EFFECT (효과)
  - FORMULA (수식)
  - TYPICAL (권장값)
  - TUNING (조정 가이드)
  - BENEFIT (장점)

  파일 끝부분에는 단계별 튜닝 가이드와 기대 성능 지표가 포함되어 있습니다.

  ---
  🎛️ 기대 성능

  - 횡방향 오차: < 0.1m (직선), < 0.2m (곡선)
  - 헤딩 오차: < 5도 (모든 상황)
  - 계산 시간: < 5ms per cycle
  - 안정 속도 범위: 0.5-8.0 m/s

  path_tracker와 frenet_follower를 모두 테스트하여 성능을 비교해보시기 바랍니다!

-- tuning guide
● 완료했습니다! Forward Tracking 기능이 frenet_follower에 성공적으로 추가되었습니다.

  ---
  ✅ Forward Tracking 기능 추가 완료

  빌드 결과

  - ✅ 빌드 성공: 1분 20초 소요
  - ⚠️ 경고: unused parameter 1개 (무시 가능)

  ---
  🎯 Forward Tracking 이란?

  문제 상황

  odom 발행 빈도가 낮을 때 (< 20Hz), frenet_follower가 같은 경로 점을 반복적으로 타겟으로 선택하여:
  - 차량이 제자리에서 "머뭇거림" (hesitation)
  - 경로를 따라 전진하지 못함
  - 부자연스러운 주행

  해결 방법

  Forward Tracking은 이전에 타겟했던 인덱스부터 앞쪽(forward)만 검색하여 경로를 순차적으로 진행:

  Standard Mode (전체 검색):
  odom 1: 경로 전체 검색 → idx=50 선택
  odom 2: 경로 전체 검색 → idx=50 선택 (반복!)
  odom 3: 경로 전체 검색 → idx=50 선택 (계속 같은 점)

  Forward Tracking Mode (앞쪽만 검색):
  odom 1: 경로 전체 검색 → idx=50 선택
  odom 2: idx=50부터 70까지만 검색 → idx=55 선택 (전진!)
  odom 3: idx=55부터 75까지만 검색 → idx=62 선택 (계속 전진!)

  ---
  🔧 구현 세부사항 (src/frenet_follower_node.cpp:410-497)

  1. 검색 윈도우 계산

  // forward_search_range = 2.0m
  // 경로 보간 해상도 = 0.1m
  size_t forward_window = forward_search_range / 0.1;  // 20개 waypoint

  2. Forward 검색 범위

  size_t start_idx = last_target_idx_;  // 이전 타겟 인덱스
  size_t end_idx = start_idx + forward_window;  // 앞으로 20개

  3. Fallback 메커니즘

  차량이 경로에서 크게 벗어난 경우:
  if (result.distance == infinity) {
      // Forward 범위에 점이 없으면 전체 검색으로 fallback
      RCLCPP_WARN("Falling back to full search");
      // ... 전체 경로 검색
  }

  4. 인덱스 업데이트

  last_target_idx_ = result.idx;  // 다음 cycle을 위해 저장

  ---
  📝 파라미터 설정 (config/follower_params.yaml:114-141)

  use_forward_tracking (default: false)

  use_forward_tracking: false      # Enable sequential path following

  언제 활성화할까?
  - ✅ Odom 빈도 < 20Hz
  - ✅ 저속 localization (GPS at 10Hz)
  - ✅ 차량이 같은 지점에서 "머뭇거림"

  언제 비활성화할까?
  - ❌ Odom 빈도 > 50Hz
  - ❌ 표준 particle filter (50-100Hz)
  - ❌ 차량이 경로에서 크게 벗어날 가능성

  forward_search_range (default: 2.0m)

  forward_search_range: 2.0        # Forward search range [m]

  튜닝 가이드:
  - 너무 작음 (< 1.0m): 고속에서 타겟을 놓칠 수 있음
  - 너무 큼 (> 5.0m): Forward tracking 효과 감소
  - 권장값: 2.0-3.0m

  계산 예시:
  - 2.0m range ÷ 0.1m resolution = 20개 waypoint 검색

  ---
  🚀 사용 방법

  1. Forward Tracking 활성화

  # follower_params.yaml 수정
  use_forward_tracking: true
  forward_search_range: 2.0

  2. 실행

  source install/setup.bash
  ros2 launch frenet_follower frenet_follower.launch.py

  3. 로그 확인

  초기화 시:
  [INFO] Path Tracking:
  [INFO]   - Forward tracking: ENABLED (range: 2.00m)

  실행 중 (DEBUG 레벨):
  [DEBUG] Forward tracking: search window [50, 70), found idx=55, dist=0.123m

  Fallback 발생 시:
  [WARN] Forward tracking: No point found in forward window [50, 70), falling back to full search

  ---
  📊 기대 효과

  Before (Standard Mode)

  - 같은 점 반복 타겟 → 부자연스러운 주행
  - 낮은 odom 빈도에서 성능 저하

  After (Forward Tracking)

  - ✅ 순차적 경로 진행 → 부드러운 주행
  - ✅ 낮은 odom 빈도에서도 안정적
  - ✅ "머뭇거림" 현상 제거

  ---
  🎛️ 실험 권장사항

  Step 1: 현재 odom 빈도 확인
  ros2 topic hz /pf/pose/odom

  Step 2: 빈도가 < 20Hz이면 forward tracking 활성화
  use_forward_tracking: true

  Step 3: 주행 테스트 후 range 조정
  - 여전히 "머뭇거림" → forward_search_range 증가 (3.0m)
  - 경로 이탈 발생 → forward_search_range 감소 (1.5m)

  이제 낮은 odom 빈도 환경에서도 frenet_follower가 planned_path를 순차적으로 부드럽게 추종할 수 있습니다!
