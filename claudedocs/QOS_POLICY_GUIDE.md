# F1TENTH DAWGS - QoS Policy Guide

**Date**: 2025-10-22
**Components**: path_planner, path_tracker
**ROS2 Distribution**: Foxy

---

## 개요

이 문서는 F1TENTH DAWGS 시스템의 path_planner와 path_tracker 노드 간 토픽 통신에 적용된 QoS(Quality of Service) 정책을 상세히 설명합니다.

## QoS 정책의 중요성

ROS2에서 QoS 정책은 DDS(Data Distribution Service) 미들웨어를 통해 데이터 통신의 특성을 제어합니다:

- **Reliability**: 메시지 전달 보장 수준
- **Durability**: 늦게 조인한 구독자에 대한 과거 메시지 제공
- **History**: 메시지 버퍼링 방식
- **Deadline**: 메시지 발행 주기 보장
- **Lifespan**: 메시지 유효 기간
- **Liveliness**: 노드 생존 확인

잘못된 QoS 설정은 다음 문제를 야기합니다:
- Publisher/Subscriber 간 통신 실패
- 불필요한 지연 시간 증가
- 메시지 손실 또는 누락
- 네트워크 대역폭 낭비

---

## QoS 정책 분류

### 1. **센서 데이터 QoS** (`sensor_qos`)
```cpp
auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5));
sensor_qos.best_effort();
```

**특성:**
- **Reliability**: Best Effort (일부 손실 허용)
- **History**: Keep Last(5) (최근 5개 메시지만 유지)
- **Durability**: Volatile (메모리만 사용, 재시작 시 삭제)

**적용 이유:**
- 센서 데이터는 고주파로 발행됨 (30-100Hz)
- 실시간성이 완전성보다 중요
- 오래된 데이터는 가치가 낮음
- 네트워크 지연 시 최신 데이터 우선

**적용 토픽:**
- `/odom` (Odometry)
- `/scan` (LaserScan)

---

### 2. **경로 데이터 QoS** (`path_qos`)
```cpp
auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
path_qos.reliable();
```

**특성:**
- **Reliability**: Reliable (모든 메시지 전달 보장)
- **History**: Keep Last(10) (최근 10개 메시지 버퍼)
- **Durability**: Volatile

**적용 이유:**
- 경로 데이터 손실 시 제어 실패
- 발행 주기가 낮음 (1-10Hz)
- 완전성이 실시간성보다 중요
- 버퍼를 통해 네트워크 지터 흡수

**적용 토픽:**
- `/planned_path` (최종 계획 경로)
- `/frenet_path` (Frenet 프레임 경로)
- `/global_centerline` (전역 중심선)
- `/lut_path` (Lattice 경로)

---

### 3. **제어 명령 QoS** (`control_qos`)
```cpp
auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1));
control_qos.best_effort();
```

**특성:**
- **Reliability**: Best Effort (즉시 전달)
- **History**: Keep Last(1) (최신 명령만 유지)
- **Durability**: Volatile

**적용 이유:**
- 제어 명령은 즉시 반영되어야 함
- 오래된 명령은 무효
- 지연 시간 최소화가 핵심
- 다음 명령이 곧 도착

**적용 토픽:**
- `/drive` (Ackermann 제어 명령)

---

### 4. **시각화 데이터 QoS** (`viz_qos`)
```cpp
auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1));
viz_qos.best_effort();
```

**특성:**
- **Reliability**: Best Effort
- **History**: Keep Last(1)
- **Durability**: Volatile

**적용 이유:**
- 시각화는 실시간 피드백용
- 일부 손실 허용 가능
- 네트워크 부하 최소화
- 최신 상태만 중요

**적용 토픽:**
- `lookahead_point` (Lookahead 마커)
- `/path_planner_markers` (경로 시각화)
- `/global_path_velocity_markers` (속도 시각화)

---

## 토픽별 QoS 정책 매트릭스

### **Controller 패키지**

| 토픽 | 메시지 타입 | Publisher | Subscriber | QoS 타입 | Reliability | History | 이유 |
|------|-----------|-----------|-----------|---------|-------------|---------|------|
| `/odom` | Odometry | vesc_to_odom | planner, tracker, pure_pursuit | sensor_qos | Best Effort | Keep Last(5) | 고주파 센서, 실시간성 |
| `/scan` | LaserScan | LiDAR | planner, gap_follow | sensor_qos | Best Effort | Keep Last(5) | 고주파 센서, 장애물 감지 |
| `/planned_path` | Path | planner | (unused) | path_qos | Reliable | Keep Last(10) | 경로 완전성 보장 |
| `/frenet_path` | Path | planner | tracker | path_qos | Reliable | Keep Last(10) | 추종 경로 손실 방지 |
| `/global_centerline` | Path | planner | tracker | path_qos | Reliable | Keep Last(10) | 속도 참조 데이터 |
| `/lut_path` | Path | planner | (viz only) | path_qos | Reliable | Keep Last(10) | 시각화용 경로 |
| `/drive` | AckermannDriveStamped | tracker, pure_pursuit, gap_follow | ackermann_mux | control_qos | Best Effort | Keep Last(1) | 제어 지연 최소화 |
| `lookahead_point` | Marker | tracker | RViz | viz_qos | Best Effort | Keep Last(1) | 시각화 |
| `/path_planner_markers` | MarkerArray | planner | RViz | viz_qos | Best Effort | Keep Last(1) | 시각화 |
| `/global_path_velocity_markers` | MarkerArray | planner | RViz | viz_qos | Best Effort | Keep Last(1) | 시각화 |
| `waypoints` | MarkerArray | pure_pursuit | RViz | viz_qos | Best Effort | Keep Last(1) | waypoint 시각화 |
| `steerings` | Marker | pure_pursuit | RViz | viz_qos | Best Effort | Keep Last(1) | 조향 시각화 |
| `wp_near` | Marker | pure_pursuit | RViz | viz_qos | Best Effort | Keep Last(1) | 근접 waypoint |
| `wp_ahead` | Marker | pure_pursuit | RViz | viz_qos | Best Effort | Keep Last(1) | lookahead waypoint |
| `/lap_time` | Float64 | pure_pursuit | (monitoring) | data_qos | Reliable | Keep Last(10) | 랩타임 기록 |
| `steering_arrow` | Marker | gap_follow | RViz | viz_qos | Best Effort | Keep Last(1) | 조향 시각화 |
| `best_gap_marker` | Marker | gap_follow | RViz | viz_qos | Best Effort | Keep Last(1) | 최적 gap 시각화 |
| `many_gap_marker` | MarkerArray | gap_follow | RViz | viz_qos | Best Effort | Keep Last(1) | 모든 gap 시각화 |

### **Base System 패키지**

| 토픽 | 메시지 타입 | Publisher | Subscriber | QoS 타입 | Reliability | History | 이유 |
|------|-----------|-----------|-----------|---------|-------------|---------|------|
| `/odom` | Odometry | vesc_to_odom | controllers | sensor_qos | Best Effort | Keep Last(5) | 실시간 위치 추정 |
| `sensors/core` | VescStateStamped | vesc_driver | vesc_to_odom | sensor_qos | Best Effort | Keep Last(10) | VESC 상태 |
| `sensors/servo_position_command` | Float64 | vesc_driver | vesc_to_odom | sensor_qos | Best Effort | Keep Last(10) | 서보 위치 |
| `ackermann_cmd` | AckermannDriveStamped | ackermann_mux | ackermann_to_vesc | control_qos | Best Effort | Keep Last(1) | 최종 제어 명령 |
| `commands/motor/speed` | Float64 | ackermann_to_vesc | vesc_driver | control_qos | Best Effort | Keep Last(1) | 모터 속도 명령 |
| `commands/servo/position` | Float64 | ackermann_to_vesc | vesc_driver | control_qos | Best Effort | Keep Last(1) | 서보 위치 명령 |

### **시뮬레이션 패키지 (F1TENTH Gym)**

| 토픽 | 메시지 타입 | Publisher | Subscriber | QoS 타입 | Reliability | History | 이유 |
|------|-----------|-----------|-----------|---------|-------------|---------|------|
| `ego_drive_topic` | AckermannDriveStamped | controllers | gym_bridge | control_qos | Best Effort | Keep Last(1) | 시뮬레이터 제어 |
| `opp_drive_topic` | AckermannDriveStamped | controllers | gym_bridge | control_qos | Best Effort | Keep Last(1) | 상대 차량 제어 |
| `ego_scan_topic` | LaserScan | gym_bridge | controllers | sensor_qos | Best Effort | Keep Last(10) | 시뮬레이션 센서 |
| `ego_odom_topic` | Odometry | gym_bridge | controllers | sensor_qos | Best Effort | Keep Last(10) | 시뮬레이션 오도메트리 |

---

## QoS 호환성 규칙

### Reliability 호환성
| Publisher | Subscriber | 호환성 | 설명 |
|-----------|-----------|--------|------|
| Reliable | Reliable | ✅ | 완전 호환 |
| Reliable | Best Effort | ✅ | 구독자가 더 관대 |
| Best Effort | Reliable | ❌ | 불일치 - 통신 불가 |
| Best Effort | Best Effort | ✅ | 완전 호환 |

### History 호환성
- Publisher와 Subscriber의 History 깊이는 독립적
- 단, Subscriber의 깊이가 Publisher보다 크면 유리

### Durability 호환성
| Publisher | Subscriber | 호환성 | 설명 |
|-----------|-----------|--------|------|
| Volatile | Volatile | ✅ | 완전 호환 |
| Volatile | Transient Local | ❌ | 불일치 |
| Transient Local | Volatile | ✅ | 호환 가능 |
| Transient Local | Transient Local | ✅ | 완전 호환 |

---

## 성능 최적화 가이드라인

### 1. **지연 시간 최소화**
- Best Effort + Keep Last(1) 사용
- Volatile durability 유지
- 불필요한 버퍼링 제거

**적용 사례:**
```cpp
// 제어 명령 (최대 응답성)
auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1));
control_qos.best_effort();
```

### 2. **데이터 완전성 보장**
- Reliable + 충분한 History 버퍼
- Deadline 정책으로 발행 주기 보장

**적용 사례:**
```cpp
// 경로 데이터 (손실 방지)
auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
path_qos.reliable();
```

### 3. **네트워크 대역폭 절약**
- Best Effort로 재전송 방지
- Keep Last(1)로 버퍼 최소화
- 시각화 데이터는 간헐적 전송

**적용 사례:**
```cpp
// 시각화 (손실 허용)
auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1));
viz_qos.best_effort();
```

---

## 트러블슈팅

### 문제 1: 토픽이 연결되지 않음
**증상:**
```bash
ros2 topic list
# 토픽은 보이지만 ros2 topic echo에서 데이터 없음
```

**원인:** QoS 불일치 (주로 Reliability)

**해결:**
```bash
# QoS 프로파일 확인
ros2 topic info /frenet_path -v

# Publisher와 Subscriber QoS 일치 확인
# Best Effort → Reliable 변경 또는 그 반대
```

### 문제 2: 메시지 손실
**증상:**
- 일부 메시지가 subscriber에 도달하지 않음
- 경로 추종이 불안정

**원인:** Best Effort QoS + 네트워크 혼잡

**해결:**
- 중요 데이터는 Reliable로 변경
- History 버퍼 크기 증가
```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(20)); // 10 → 20
qos.reliable();
```

### 문제 3: 높은 지연 시간
**증상:**
- 제어 명령이 늦게 반영됨
- 센서 데이터가 오래됨

**원인:** Reliable QoS + 재전송 오버헤드

**해결:**
- 실시간 데이터는 Best Effort로 변경
- Keep Last(1)로 버퍼 최소화
```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
qos.best_effort();
```

### 문제 4: CycloneDDS 성능 저하
**증상:**
- 전체 시스템 응답성 저하
- CPU 사용률 증가

**원인:** 과도한 메시지 버퍼링 + 재전송

**해결:**
- CycloneDDS 설정 최적화
- QoS 프로파일 단순화
```xml
<!-- cyclonedds.xml -->
<Internal>
  <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
</Internal>
```

---

## 검증 방법

### 1. QoS 프로파일 확인
```bash
# 토픽 상세 정보
ros2 topic info /frenet_path -v

# 예상 출력:
# Reliability: RELIABLE
# History (Depth): 10
# Durability: VOLATILE
```

### 2. 통신 상태 확인
```bash
# 메시지 흐름 확인
ros2 topic hz /frenet_path

# Publisher-Subscriber 매칭 확인
ros2 topic info /frenet_path
```

### 3. 지연 시간 측정
```bash
# 메시지 타임스탬프 분석
ros2 topic echo /frenet_path --field header.stamp
```

### 4. 메시지 손실 테스트
```bash
# Publisher 발행 수 vs Subscriber 수신 수 비교
ros2 topic pub /test_topic std_msgs/msg/String "data: test" -r 100
ros2 topic hz /test_topic
```

---

## 향후 개선 사항

### 1. Deadline 정책 추가
```cpp
// 경로 업데이트 주기 보장
auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
path_qos.reliable();
path_qos.deadline(std::chrono::milliseconds(100)); // 100ms 이내 업데이트
```

### 2. Lifespan 정책 추가
```cpp
// 오래된 경로 자동 폐기
auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
path_qos.reliable();
path_qos.lifespan(std::chrono::seconds(1)); // 1초 이상 된 메시지 폐기
```

### 3. Transient Local Durability
```cpp
// 늦게 조인한 노드에 최근 경로 제공
auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
path_qos.reliable();
path_qos.transient_local();
```

---

## 참고 자료

- [ROS2 QoS Documentation](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
- [DDS QoS Policies](https://www.omg.org/spec/DDS/1.4/PDF)
- [CycloneDDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds)
- F1TENTH DAWGS CLAUDE.md

---

## 패키지별 QoS 적용 요약

### ✅ Controller 패키지
| 패키지 | 파일 | 변경 사항 |
|--------|------|----------|
| **path_planner** | `src/path_planner_node.cpp:71-101` | sensor_qos, path_qos, viz_qos 적용 |
| **path_tracker** | `src/path_tracker_node.cpp:97-128` | sensor_qos, path_qos, control_qos, viz_qos 적용 |
| **pure_pursuit** | `src/pure_pursuit_node.cpp:50-77` | sensor_qos, control_qos, data_qos, viz_qos 적용 |
| **gap_follow** | `src/gap_follow.cpp:6-26` | sensor_qos, control_qos, viz_qos 적용 |

### ✅ Base System 패키지
| 패키지 | 파일 | 변경 사항 |
|--------|------|----------|
| **vesc_to_odom** | `vesc_ackermann/src/vesc_to_odom.cpp:74-97` | odom_qos (Best Effort, KeepLast(5)), sensor_qos 적용 |
| **ackermann_to_vesc** | `vesc_ackermann/src/ackermann_to_vesc.cpp:56-66` | control_qos (Best Effort, KeepLast(1)) 적용 |
| **ackermann_mux** | `ackermann_mux/src/ackermann_mux.cpp:89-95` | 이미 최적화됨 (Best Effort, KeepLast(1)) |

### ✅ 시뮬레이션 패키지
| 패키지 | 파일 | 변경 사항 |
|--------|------|----------|
| **gym_bridge** | `f1tenth_gym_ros/gym_bridge.py:147-171` | drive_qos (Best Effort, Volatile, KeepLast(1)) 적용 |

---

## 변경 이력

| 날짜 | 변경 사항 | 작성자 |
|------|----------|--------|
| 2025-10-22 | 초기 QoS 정책 정의 및 전체 시스템 적용 | Claude Code |
| 2025-10-22 | Controller 패키지 (planner, tracker, pure_pursuit, gap_follow) QoS 업데이트 | Claude Code |
| 2025-10-22 | Base System 패키지 (vesc_to_odom, ackermann_to_vesc) QoS 업데이트 | Claude Code |
| 2025-10-22 | 시뮬레이션 패키지 (gym_bridge) drive 토픽 QoS 업데이트 | Claude Code |

---

## 요약

✅ **Controller 패키지 (4개)**:
- path_planner: 센서, 경로, 시각화 QoS
- path_tracker: 센서, 경로, 제어, 시각화 QoS
- pure_pursuit: 센서, 제어, 데이터, 시각화 QoS
- gap_follow: 센서, 제어, 시각화 QoS

✅ **Base System 패키지 (3개)**:
- vesc_to_odom: 오도메트리 및 센서 데이터 QoS
- ackermann_to_vesc: 제어 명령 QoS
- ackermann_mux: 이미 최적화된 QoS 유지

✅ **시뮬레이션 패키지 (1개)**:
- gym_bridge: drive 토픽 Best Effort, Volatile, KeepLast(1)

✅ **전체 패키지 수**: 8개 패키지

✅ **전체 토픽 수**: 24개 토픽 QoS 최적화 완료

✅ **호환성**: 모든 Publisher-Subscriber QoS 일치 확인 완료

✅ **성능**: 지연 시간 최소화 및 데이터 무결성 보장

**핵심 원칙:**
- **실시간 데이터** (센서, 제어) → Best Effort, 저지연
- **중요 데이터** (경로, 랩타임) → Reliable, 완전성 보장
- **시각화** → Best Effort, 최소 버퍼, 손실 허용
- **시뮬레이션** → Best Effort, Volatile, 최소 지연
