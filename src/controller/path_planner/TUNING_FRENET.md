● 완료했습니다! 다음과 같이 수정했습니다:

  변경 사항 요약

  1. Lookahead Distance 추가 (더 앞쪽에서 경로 생성)

  src/controller/path_planner/config/planner_params.yaml:26
  frenet_lookahead_distance: 2.0  # 경로 생성 시작 지점을 2m 앞으로

  src/controller/path_planner/src/path_planner_node.cpp:562-592
  - 현재 위치의 s 좌표에 lookahead distance를 추가
  - Closed loop 경로의 wrapping 처리 포함
  - 기본값 2.0m 설정 (config에서 조정 가능)

  2. Obstacle Avoidance 개선

  a) Lateral Sampling 확장 (planner_params.yaml:30)

  # 기존: [-1.0, -0.75, ..., 1.0] (9개 샘플)
  # 변경: [-1.5, -1.25, ..., 1.5] (13개 샘플)
  더 넓은 lateral 범위에서 경로 샘플링 → 장애물 회피 옵션 증가

  b) Safety Margin 증가 (planner_params.yaml:43-53)

  frenet_safety_radius: 0.9 → 1.2m         # 기본 안전 반경 33% 증가
  frenet_road_half_width: 1.2 → 1.5m       # 주행 가능 폭 25% 증가  
  frenet_vehicle_radius: 0.5 → 0.6m        # 차량 반경 20% 증가
  frenet_obstacle_radius: 0.5 → 0.6m       # 장애물 반경 20% 증가
  frenet_k_proximity: 0.5 → 2.0            # 근접 비용 가중치 4배 증가
  frenet_proximity_threshold: 1.5 → 2.5m   # 감지 거리 67% 증가
  frenet_interpolation_checks: 3 → 5       # 충돌 체크 밀도 67% 증가

  c) Obstacle Detection 범위 확장 (path_planner_node.cpp:231)

  // 기존: 5.0m → 변경: 8.0m
  if (range > 8.0) continue;  // 더 먼 장애물도 고려

  빌드 방법

  cd /home/dawgs_nx/f1tenth_dawgs
  colcon build --packages-select path_planner
  source install/setup.bash

  튜닝 가능한 파라미터

  frenet_lookahead_distance (기본 2.0m)
  - 증가: 더 앞을 보고 계획 (반응 시간 증가)
  - 감소: 현재 위치에 더 가까운 계획 (민첩성 증가)

  frenet_k_proximity (기본 2.0)
  - 증가: 장애물에서 더 멀리 떨어짐 (안전 우선)
  - 감소: 장애물에 더 가까이 접근 가능 (속도 우선)

  frenet_safety_radius (기본 1.2m)
  - 차량-장애물 간 최소 거리 제어

  frenet_d_samples
  - lateral 샘플 개수/범위 조정으로 회피 경로 다양성 제어

  수정된 코드를 빌드하시면 됩니다!

---

● 완료했습니다! 장애물 인식 및 시각화 시스템을 추가했습니다.

  구현 내용

  1. 장애물 클러스터링 (path_planner_node.cpp:211-255)

  - 거리 기반 DBSCAN 스타일 클러스터링
  - 0.5m 이내의 laser scan 점들을 하나의 장애물 그룹으로 묶음
  - 최소 3개 이상의 점이 있는 클러스터만 유지 (노이즈 필터링)

  2. Bounding Box 생성 (path_planner_node.cpp:257-297)

  - 각 클러스터를 정사각형 bounding box로 모델링
  - 클러스터의 min/max 범위 계산
  - 더 큰 차원(width/height)을 사용해 정사각형 생성
  - 40cm 안전 마진 추가 (실제 장애물보다 큰 box)

  3. RViz 시각화 (path_planner_node.cpp:299-382)

  두 가지 마커로 표시:
  - 빨간색 반투명 박스 (CUBE 마커): 장애물 부피 표현
  - 빨간색 경계선 (LINE_STRIP): 정사각형 테두리 강조

  Topic: /obstacle_boxes

  4. Frenet Planner 통합 (path_planner_node.cpp:781-836)

  5단계 처리:
  1. Raw scan 점 획득 (map frame)
  2. 클러스터링 (0.5m 거리 기준)
  3. Bounding box 생성
  4. RViz 시각화
  5. Box 경계를 밀집 샘플링 (각 변마다 8개 점 → 총 32개 점/box)

  5. 디버그 로깅 (path_planner_node.cpp:834-836)

  "Detected %zu raw points → %zu clusters → %zu boxes → %zu boundary
  points"
  처리 파이프라인 상태 확인 가능

  RViz 설정 방법

  빌드 후 RViz에서 다음 topic을 추가하세요:

  Topic: /obstacle_boxes
  Type: visualization_msgs/MarkerArray

  표시 내용:
  - 🟥 빨간 반투명 박스: 장애물 영역
  - 🔴 빨간 경계선: 정사각형 테두리 (ground level, z=0)
  - 📦 박스 높이: 1m (시각성)

  빌드 및 실행

  cd /home/dawgs_nx/f1tenth_dawgs
  colcon build --packages-select path_planner
  source install/setup.bash

  # 실행
  ros2 launch path_planner <your_launch_file>

  튜닝 가능한 파라미터

  코드에서 조정 가능:

  클러스터링 거리 (line 788):
  auto clusters = clusterObstacles(raw_obstacles, 0.5);  // 0.5m → 조정 
  가능

  안전 마진 (line 285):
  size += 0.4;  // 40cm → 조정 가능

  경계 샘플 밀도 (line 801):
  int points_per_side = 8;  // 8개/변 → 증가하면 더 촘촘한 장애물 표현

  최소 클러스터 크기 (line 249):
  if (cluster.size() >= 3)  // 3개 → 조정하면 노이즈 필터링 강도 변경

  이제 RViz에서 빨간 정사각형으로 장애물이 명확하게 표시되고, Frenet
  planner가 더 정확하게 장애물을 인식할 수 있습니다!


--- cluster and occupancy grid 


  📊 Path Planner 구조 설명

  1. Lattice LUT (Lookup Table) 역할

  개념: 빠른 경로 생성을 위한 사전 계산된 cubic spiral 라이브러리

  동작 원리:

  Build 단계 (lattice_lut.cpp:20-54):
  1,710개의 사전 계산 경로 생성
  = 6(forward) × 19(lateral) × 15(heading)
  - Forward (dx): 0.5m ~ 3.0m (6단계)
  - Lateral (dy): -2.25m ~ +2.25m (0.25m 간격)
  - Heading (dtheta): -70° ~ +70° (π/18 간격)

  Query & Sample 단계 (lattice_lut.cpp:60-119):
  현재 위치 → Frenet path 끝점 → 상대 변위 계산 → LUT 조회 → Spiral 샘플링

  경로 생성 파이프라인 (path_planner_node.cpp:856-906):
  1. Frenet Planner: 장애물 회피 (quintic polynomial, ~3초)
  2. Lattice LUT: 목표 정렬 (cubic spiral, horizon까지)
  3. 최종 경로: Frenet + Lattice 결합

  장점:
  - ⚡ 실시간 생성: 미리 계산되어 빠름 (~0.1ms)
  - 🎯 곡률 연속성: Cubic spiral로 부드러운 경로
  - 🛣️ Centerline 적응: 곡률을 고려한 자연스러운 경로

  2. Occupancy Map 상태

  현재: ❌ 사용되지 않음

  이유:
  - Header만 include (path_planner_node.cpp:3)
  - 인스턴스화/사용 코드 없음
  - 대신 직접 laser scan → bounding box 변환 사용

  설계 의도 (occupancy_map.hpp):
  - Grid 기반 장애물 맵 관리
  - 충돌 검사 (collision())
  - 장애물 샘플링 (sample_obstacles())

  현재 대체 방식:
  Laser Scan → Map Frame → Clustering → Bounding Box → Frenet Planner

  ---
  ✅ 구현된 수정사항

  1. Clustering Distance 파라미터화

  planner_params.yaml:42:
  obstacle_cluster_distance: 0.5  # 클러스터링 거리 [m]

  사용 위치 (path_planner_node.cpp:804-805):
  double cluster_distance =
  get_parameter("obstacle_cluster_distance").as_double();
  auto clusters = clusterObstacles(raw_obstacles, cluster_distance);

  2. 1m 초과 박스 필터링

  planner_params.yaml:43:
  obstacle_max_box_size: 1.0  # 최대 박스 크기 (초과 시 무시) [m]

  필터링 로직 (path_planner_node.cpp:296-301):
  // FILTER: Ignore boxes larger than max_box_size
  if (size > max_box_size) {
      RCLCPP_DEBUG_THROTTLE(..., "Ignoring large obstacle box: size=%.2fm",
   size);
      continue;  // 벽이나 큰 구조물 무시
  }

  3. Safety Margin 파라미터화

  planner_params.yaml:44:
  obstacle_box_safety_margin: 0.4  # 안전 마진 [m]

  사용 위치 (path_planner_node.cpp:304):
  size += safety_margin;  // 40cm 안전 마진 추가

  4. 개선된 로깅

  출력 예시:
  Obstacles: 523 raw pts → 8 clusters → 6 boxes (filtered) → 192 boundary
  pts | Time: 2.3ms
  - filtered: 1m 초과 박스 제외된 개수
  - 2초마다 INFO 레벨로 출력

  ---
  🎛️ 튜닝 가능한 파라미터

  Clustering

  obstacle_cluster_distance: 0.5  # 증가 → 더 넓게 묶음 / 감소 → 세밀하게 
  분리

  Filtering

  obstacle_max_box_size: 1.0  # 증가 → 더 큰 장애물 허용 / 감소 → 엄격한 
  필터링

  Safety

  obstacle_box_safety_margin: 0.4  # 증가 → 더 안전 (멀리 회피) / 감소 → 
  가까이 접근

  ---
  🚀 빌드 및 실행

  cd /home/dawgs_nx/f1tenth_dawgs
  colcon build --packages-select path_planner
  source install/setup.bash

  이제:
  - ✅ 클러스터링 거리를 config에서 조정 가능
  - ✅ 1m 넘는 큰 박스는 자동으로 무시 (벽, 큰 구조물)
  - ✅ 안전 마진도 config에서 조정 가능
  - ✅ 로그에서 필터링 결과 확인 가능

  Lattice LUT는 Frenet 경로의 끝점을 부드럽게 centerline에 정렬시키는
  역할을 하고, Occupancy map은 현재 사용되지 않습니다!
