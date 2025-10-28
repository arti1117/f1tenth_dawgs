# RViz Frenet Path 시각화 가이드

## 개선 사항 (2024-10-24)

frenet_path marker 시각화가 개선되었습니다:

### 변경 사항
- ✅ Line width: 0.12m → **0.25m** (더 두꺼운 선)
- ✅ Z 높이: 0.05m → **0.15m** (바닥에서 더 높게)
- ✅ Alpha: 0.9 → **1.0** (완전 불투명)
- ✅ **새로운 cyan outline 추가** (30cm 두께, 더 명확한 경로 표시)
- ✅ Lattice candidates: 0.05m → **0.08m** (더 두꺼운 선)

---

## RViz에서 Frenet Path 표시하기

### 1. 사용 가능한 토픽

path_planner는 다음 시각화 토픽을 제공합니다:

| 토픽 이름 | 메시지 타입 | 설명 |
|----------|------------|------|
| `/frenet_path` | nav_msgs/Path | Frenet 경로 (기본 Path 메시지) |
| `/frenet_path_velocity_markers` | MarkerArray | 속도별 색상 코딩된 경로 (추천!) |
| `/global_centerline` | nav_msgs/Path | 글로벌 참조 경로 |
| `/global_path_velocity_markers` | MarkerArray | 글로벌 경로 속도 시각화 |
| `/lut_path` | nav_msgs/Path | Lattice LUT 경로 |
| `/path_planner_markers` | MarkerArray | Lattice 후보 경로들 |

### 2. 권장 설정

#### Option 1: Frenet Path Velocity Markers (추천!)

**속도별 색상 코딩**으로 가장 명확한 시각화:

1. RViz에서 **Add** 클릭
2. **MarkerArray** 선택
3. 설정:
   - Topic: `/frenet_path_velocity_markers`
   - 자동으로 2가지 namespace 표시:
     - `frenet_path_velocity`: 속도 gradient (Blue=느림, Green=중간, Red=빠름)
     - `frenet_path_outline`: Cyan 색상 경로 윤곽 (가장 명확!)

#### Option 2: Basic Path Display

단순한 경로 표시:

1. RViz에서 **Add** 클릭
2. **Path** 선택
3. 설정:
   - Topic: `/frenet_path`
   - Color: 원하는 색상 선택 (예: Yellow, Cyan)
   - Alpha: 1.0 (완전 불투명)
   - Line width: 0.1-0.2

### 3. 문제 해결

#### 경로가 안 보이는 경우

**1. Fixed Frame 확인**
```
RViz 상단 → Global Options → Fixed Frame = "map"
```

**2. 토픽이 publish되는지 확인**
```bash
ros2 topic list | grep frenet
# 출력 예상: /frenet_path, /frenet_path_velocity_markers

ros2 topic hz /frenet_path_velocity_markers
# 출력 예상: average rate: ~10-20 Hz (path_planner가 실행 중일 때)
```

**3. visualize_paths 파라미터 확인**
```bash
ros2 param get /path_planner_node visualize_paths
# 출력 예상: Boolean value is: True
```

만약 False라면:
```bash
# planner_params.yaml에서 수정
visualize_paths: true
```

**4. path_planner가 실행 중인지 확인**
```bash
ros2 node list | grep path_planner
# 출력 예상: /path_planner_node
```

#### 경로가 바닥에 묻혀 있는 경우

- **해결됨!** z 높이를 0.15m로 상향 조정했습니다
- Outline marker (cyan)가 0.12m 높이로 추가되었습니다
- 여전히 안 보이면: RViz에서 **View → Camera Type → TopDownOrtho** 시도

#### 경로가 너무 얇은 경우

- **해결됨!** Line width를 0.25-0.3m로 증가했습니다
- Frenet velocity: 0.25m
- Outline: 0.3m
- Lattice candidates: 0.08m

### 4. 추가 시각화

#### Lattice Candidates (경로 후보들)

모든 계획된 경로 후보를 보려면:

1. **Add** → **MarkerArray**
2. Topic: `/path_planner_markers`
3. Namespace: `lattice_candidates`
   - 초록색 = 최적 경로
   - 다른 색상 = 다른 lateral offset 후보들

#### Global Path Velocity

참조 경로와 비교하려면:

1. **Add** → **MarkerArray**
2. Topic: `/global_path_velocity_markers`
3. 글로벌 경로가 색상 gradient로 표시됩니다

### 5. 권장 RViz 레이어 순서

투명도와 겹침을 고려한 순서:

```
1. Map (가장 아래)
2. Global Path Velocity Markers (α=0.8)
3. Lattice Candidates (α=0.5, 반투명)
4. Frenet Path Velocity Markers (α=1.0, 불투명)
   - 여기에 Outline (cyan, α=0.7)도 포함
5. Robot Model / TF (가장 위)
```

### 6. 색상 의미

#### Frenet Path Velocity (속도 gradient):
- 🔵 **파란색**: 느린 속도 (코너, 장애물 회피)
- 🟢 **초록색**: 중간 속도
- 🔴 **빨간색**: 빠른 속도 (직선 구간)

#### Frenet Path Outline:
- 🔷 **Cyan (하늘색)**: 전체 경로 윤곽 (가장 명확한 표시)

#### Lattice Candidates:
- 🟢 **초록색**: 선택된 최적 경로
- 🟣 **보라/빨강**: 다른 후보 경로들

---

## 실행 예시

```bash
# Terminal 1: path_planner 실행
ros2 launch path_planner planner.launch.py

# Terminal 2: RViz 실행
rviz2

# RViz에서:
1. Fixed Frame: "map" 설정
2. Add → MarkerArray
   - Topic: /frenet_path_velocity_markers
3. Add → MarkerArray
   - Topic: /path_planner_markers (선택 사항)
4. Add → Path
   - Topic: /global_centerline (참조용)
```

---

## 시각화 비교

### 개선 전 vs 개선 후

| 항목 | 개선 전 | 개선 후 |
|------|---------|---------|
| Line width (frenet) | 0.12m (얇음) | 0.25m (두꺼움) + 0.3m outline |
| Z 높이 | 0.05m (바닥에 가까움) | 0.15m (명확히 보임) |
| Alpha | 0.9 (약간 투명) | 1.0 (완전 불투명) |
| 추가 marker | 없음 | Cyan outline 추가 |
| Lattice width | 0.05m | 0.08m |
| 가시성 | ⚠️ 어려움 | ✅ 명확함 |

---

## 문제 리포트

여전히 문제가 있다면:

```bash
# 1. 토픽 데이터 확인
ros2 topic echo /frenet_path_velocity_markers --no-arr | head -50

# 2. 파라미터 확인
ros2 param dump /path_planner_node

# 3. 노드 정보 확인
ros2 node info /path_planner_node
```

문제가 지속되면 GitHub Issue를 열어주세요:
- Repository: f1tenth_dawgs
- Label: visualization, rviz
- 포함 정보: RViz 스크린샷, `ros2 topic list` 출력

--- wall marker
● ✅ Wall Detection 수정 완료

  빌드가 성공적으로 완료되었습니다! (소요 시간: 3분 11초, 에러 없음)

  수정 내용 요약

  1. Wall 모델링 방식 변경 (path_planner_node.cpp:432-493)

  - 이전: Wall을 하나의 큰 박스로 모델링
  - 변경: Wall을 짧은 선분(기본 0.5m)으로 분해
  - 함수: breakWallIntoSegments() - cluster를 선분들로 변환

  2. Global Path 필터링 추가 (path_planner_node.cpp:495-530)

  - 문제: 경로 자체가 LiDAR로 감지되어 wall로 오인식됨
  - 해결: Global path 근처 1.5m 이내의 scan point 필터링
  - 함수: filterScanPointsNearPath() - path 근처 점들 제거

  3. Wall 시각화 개선 (path_planner_node.cpp:532-616)

  - 이전: 큰 박스로 표시 (RViz에서 파란색)
  - 변경: 선분으로 표시 - 각 선분의 방향과 길이 시각화
  - 함수: visualizeWallSegments() - 선분 기반 마커 생성

  4. Obstacle Point 생성 최적화 (path_planner_node.cpp:1114-1142)

  - Wall 선분을 점들로 샘플링:
    - 길이 방향: 5개 포인트
    - 폭 방향: 3개 포인트 (30cm 폭)
  - Frenet planner가 정확한 wall 위치 인식 가능

  5. Config 파라미터 추가 (config/planner_params.yaml:49-52)

  # Wall detection parameters
  wall_segment_length: 0.5      # Wall을 0.5m 선분으로 분해
  wall_segment_width: 0.3       # 선분 폭 30cm
  path_clearance_radius: 1.5    # 경로 근처 1.5m는 obstacle 제외

  처리 흐름

  1. Scan 수신 → LiDAR 점들 획득
  2. Path 필터링 → Global path 1.5m 근처 점 제거 (경로 오인식 방지)
  3. Clustering → 남은 점들을 클러스터링
  4. 분류:
    - 작은 cluster (< 0.8m) → Obstacle box (빨간색)
    - 큰 cluster (≥ 0.8m) → Wall 선분들 (파란색)
  5. Frenet Planner 전달 → 선분 샘플링 점들 + obstacle 박스 점들

  이점

  - ✅ 경로가 더 이상 wall로 인식되지 않음
  - ✅ Wall을 정확한 선형 구조로 모델링 (박스보다 정교)
  - ✅ Frenet planner가 wall 위치를 더 정확히 인식
  - ✅ RViz에서 wall이 선분으로 표시되어 디버깅 용이

  테스트를 위해 source install/setup.bash 후 시스템을 실행해 보세요! RViz에서
  /wall_markers 토픽을 확인하면 파란색 선분들로 wall이 표시됩니다.
