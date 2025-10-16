joy_teleop
==========

A configurable node to map joystick controls to robot teleoperation commands


f1tenth_stack joy_toggle_teleop.launch.py & joy_toggle_teleop.py
  📋 구현 내용

  1. ERPM과 Steering Angle 동작 원리

  - ERPM: 속도(m/s) × 4614.0으로 변환되어 VESC 모터 제어
  - Steering: 각도(rad) × (-1.2135) + 0.5304로 서보 값 변환

  2. Toggle 방식 Joy Teleop 노드 (joy_toggle_teleop.py)

  - RB 버튼(버튼 5): 제어 ON/OFF 토글
  - 토글 동작:
    - 첫 번째 누름 → 제어 활성화 (🟢)
    - 두 번째 누름 → 제어 비활성화 (🔴)
  - 안전 기능: A 버튼으로 긴급 정지
  - Rising Edge 감지: 버튼 눌림 순간만 감지해서 토글

  3. 사용 방법

  # 빌드
  cd ~/f1tenth_dawgs
  colcon build --packages-select joy_teleop

  # 실행
  ros2 launch f1tenth_stack joy_toggle_teleop.launch.py

  # 또는 파라미터 조정
  ros2 launch f1tenth_stack joy_toggle_teleop.launch.py max_speed:=3.0
  max_steering_angle:=0.35

  4. 주요 기능

  - Toggle 제어: RB 버튼으로 활성화/비활성화
  - Deadzone: 조이스틱 드리프트 방지 (0.1)
  - 부드러운 정지: 토글 OFF시 즉시 속도 0
  - 상태 표시: 로그로 현재 상태 표시

  5. 조이스틱 매핑

  - 왼쪽 스틱 Y축: 전진/후진
  - 왼쪽 스틱 X축: 좌/우 조향
  - RB 버튼: 토글 ON/OFF
  - A 버튼: 긴급 정지

  이제 RB 버튼을 눌러서 제어를 활성화하고, 다시 누르면 비활성화되는 토글
   방식으로 VESC를 제어할 수 있습니다!
