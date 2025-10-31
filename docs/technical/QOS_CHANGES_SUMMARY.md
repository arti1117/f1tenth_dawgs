# QoS 정책 변경 요약

**날짜**: 2025-10-22
**목적**: F1TENTH DAWGS 시스템 전체의 ROS2 QoS 정책 최적화

---

## 📊 변경 통계

- **수정 패키지**: 8개
- **최적화 토픽**: 24개
- **수정 파일**: 6개 C++ 파일 + 1개 Python 파일
- **적용 QoS 프로파일**: 4종류

---

## ✅ 수정된 패키지

### 1. **path_planner**
- **파일**: `src/controller/path_planner/src/path_planner_node.cpp:71-101`
- **변경 내용**:
  - 센서 데이터 (`/odom`, `/scan`): Best Effort, KeepLast(5)
  - 경로 데이터 (`/planned_path`, `/global_centerline`, `/frenet_path`, `/lut_path`): Reliable, KeepLast(10)
  - 시각화 (`markers`): Best Effort, KeepLast(1)

### 2. **path_tracker**
- **파일**: `src/controller/path_tracker/src/path_tracker_node.cpp:97-128`
- **변경 내용**:
  - 센서 데이터 (`/odom`): Best Effort, KeepLast(5)
  - 경로 데이터 (`/frenet_path`, `/global_centerline`): Reliable, KeepLast(10)
  - 제어 명령 (`/drive`): Best Effort, KeepLast(1)
  - 시각화 (`lookahead_point`): Best Effort, KeepLast(1)

### 3. **pure_pursuit**
- **파일**: `src/controller/pure_pursuit/src/pure_pursuit_node.cpp:50-77`
- **변경 내용**:
  - 센서 데이터 (`/odom`): Best Effort, KeepLast(5)
  - 제어 명령 (`/drive`): Best Effort, KeepLast(1)
  - 랩타임 데이터 (`/lap_time`): Reliable, KeepLast(10)
  - 시각화 (waypoints, markers): Best Effort, KeepLast(1)

### 4. **gap_follow**
- **파일**: `src/controller/gap_follow/src/gap_follow.cpp:6-26`
- **변경 내용**:
  - 센서 데이터 (`/scan`): Best Effort, KeepLast(5)
  - 제어 명령 (`/drive`): Best Effort, KeepLast(1)
  - 시각화 (gap markers, steering): Best Effort, KeepLast(1)

### 5. **vesc_to_odom**
- **파일**: `src/base_system/f1tenth_system/vesc/vesc_ackermann/src/vesc_to_odom.cpp:74-97`
- **변경 내용**:
  - 오도메트리 출력 (`/odom`): Best Effort, KeepLast(5)
  - VESC 센서 입력 (`sensors/core`, `sensors/servo_position_command`): Best Effort, KeepLast(10)

### 6. **ackermann_to_vesc**
- **파일**: `src/base_system/f1tenth_system/vesc/vesc_ackermann/src/ackermann_to_vesc.cpp:56-66`
- **변경 내용**:
  - 제어 명령 입력 (`ackermann_cmd`): Best Effort, KeepLast(1)
  - 모터/서보 명령 출력 (`commands/motor/speed`, `commands/servo/position`): Best Effort, KeepLast(1)

### 7. **ackermann_mux**
- **파일**: `src/base_system/f1tenth_system/ackermann_mux/src/ackermann_mux.cpp:89-95`
- **상태**: ✅ 이미 최적화됨 (Best Effort, KeepLast(1)) - 변경 불필요

### 8. **gym_bridge** (시뮬레이션)
- **파일**: `src/base_system/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py:147-171`
- **변경 내용**:
  - Drive 명령 입력 (`ego_drive_topic`, `opp_drive_topic`): Best Effort, Volatile, KeepLast(1)
  - 시뮬레이터와의 실시간 통신을 위한 최소 지연 QoS

---

## 🎯 적용된 QoS 프로파일

### 1. **sensor_qos** (센서 데이터)
```cpp
auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5));
sensor_qos.best_effort();
```
- **적용 토픽**: `/odom`, `/scan`, VESC 센서
- **이유**: 고주파 센서 데이터, 실시간성 우선, 일부 손실 허용

### 2. **path_qos** (경로 데이터)
```cpp
auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10));
path_qos.reliable();
```
- **적용 토픽**: `/planned_path`, `/frenet_path`, `/global_centerline`
- **이유**: 경로 완전성 보장, 손실 시 제어 실패

### 3. **control_qos** (제어 명령)
```cpp
auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1));
control_qos.best_effort();
```
- **적용 토픽**: `/drive`, `ackermann_cmd`, 모터/서보 명령
- **이유**: 최소 지연 시간, 최신 명령만 유효

### 4. **viz_qos** (시각화)
```cpp
auto viz_qos = rclcpp::QoS(rclcpp::KeepLast(1));
viz_qos.best_effort();
```
- **적용 토픽**: 모든 Marker, MarkerArray
- **이유**: 손실 허용, 네트워크 부하 최소화

---

## 🔧 빌드 및 검증

### 빌드 명령
```bash
cd ~/f1tenth_dawgs
# C++ 패키지 빌드
colcon build --packages-select path_planner path_tracker pure_pursuit gap_follow vesc_ackermann

# Python 패키지 빌드
colcon build --packages-select f1tenth_gym_ros

source install/setup.bash
```

### QoS 검증
```bash
# 토픽 QoS 확인
ros2 topic info /odom -v
ros2 topic info /frenet_path -v
ros2 topic info /drive -v

# 통신 상태 확인
ros2 topic hz /odom
ros2 topic hz /frenet_path
```

---

## 📈 기대 효과

### 성능 개선
- ✅ **지연 시간 감소**: 센서/제어 데이터 Best Effort로 재전송 오버헤드 제거
- ✅ **데이터 무결성**: 경로 데이터 Reliable로 손실 방지
- ✅ **네트워크 효율**: 시각화 데이터 최소 버퍼로 대역폭 절약

### 안정성 향상
- ✅ **제어 응답성**: 제어 루프 지연 최소화
- ✅ **경로 추종 안정성**: 경로 데이터 완전성 보장
- ✅ **시스템 견고성**: QoS 호환성 문제 해결

---

## 📚 관련 문서

- **상세 가이드**: `claudedocs/QOS_POLICY_GUIDE.md`
- **프로젝트 정보**: `CLAUDE.md`
- **CycloneDDS 설정**: `cyclonedds_jetson.xml`

---

## ⚠️ 주의사항

1. **호환성**: 다른 노드들도 동일한 QoS를 사용해야 통신 가능
2. **네트워크**: 유선(eth0) 연결 권장, WiFi는 Best Effort에서 손실 증가 가능
3. **CycloneDDS**: `cyclonedds_jetson.xml`의 `NetworkInterfaceAddress=auto` 설정 확인

---

## 🚀 다음 단계

1. ✅ 빌드 및 검증 완료
2. 🔄 실제 하드웨어에서 테스트
3. 📊 성능 벤치마크 수행
4. 📝 필요 시 QoS 파라미터 미세 조정

---

**완료 일시**: 2025-10-22
**작성자**: Claude Code
