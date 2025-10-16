-- localization
● 완료했습니다! agent_dawgs에 두 개의 localization launch
  파일을 만들었습니다:

  1. Particle Filter Localization

  파일: src/agent_dawgs/launch/localization_particle_filter.
  launch.py

  주요 특징:
  - Particle filter 노드를 사용한 Monte Carlo localization
  - Nav2 map server로 사전 제작된 맵 로드
  - Odometry 토픽을 odometry/filtered로 리매핑 (EKF 연동)
  - RViz2 시각화 지원
  - 기본 맵: /home/dawgs_nx/f1tenth_dawgs/src/peripheral/map
  s/icheon/icheon1009_map.yaml

  실행:
  ros2 launch agent_dawgs
  localization_particle_filter.launch.py

  2. SLAM Toolbox Localization

  파일:
  src/agent_dawgs/launch/localization_slamtoolbox.launch.py

  주요 특징:
  - SLAM Toolbox의 localization_slam_toolbox_node 사용
  - Lifecycle 관리 (자동 configure/activate)
  - Nav2 map server로 사전 제작된 맵 로드
  - Odometry 토픽을 odometry/filtered로 리매핑 (EKF 연동)
  - RViz2 시각화 지원
  - 설정 파일: mapper_params_localization.yaml

  실행:
  ros2 launch agent_dawgs localization_slamtoolbox.launch.py

  두 launch 파일 모두 Cartographer localization launch
  파일(cartographer_localization_ekf_launch.py)의 구조를
  참고하여 다음을 포함합니다:
  - EKF filtered odometry 사용 (/odometry/filtered)
  - Map server + lifecycle manager
  - RViz2 시각화 옵션
  - use_sim_time 파라미터 (기본값: false)
