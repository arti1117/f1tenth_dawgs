● 완료되었습니다! raceline_editor를 path_editor로 변경하고 요청하신 모든
  기능을 추가했습니다.

  주요 변경사항

  1. 패키지 이름 변경

  - raceline_editor → path_editor
  - 모든 관련 파일들 업데이트 완료

  2. 새로운 기능 추가

  Refresh 버튼

  - GUI를 처음 로드한 상태로 되돌리기
  - 저장하지 않은 변경사항을 모두 취소
  - 단축키: R

  Smooth Edit 옵션 (체크박스)

  - 한 점을 움직이면 인접한 3개 점들도 곡선으로 부드럽게 조정
  - Gaussian 가중치 사용 (가까운 점일수록 큰 영향)
  - 체크박스로 ON/OFF 가능
  - 드래그 또는 텍스트 입력 시 모두 적용

  Map Background 표시

  - PGM/PNG 맵 파일을 배경으로 표시
  - YAML 파일에서 origin, resolution 정보 로드
  - 반투명(alpha=0.5)으로 표시하여 waypoint 시인성 유지

  3. 개선된 UI

  버튼 레이아웃:
  [Save] [Publish] [Reload] [Refresh] [☑ Smooth Edit] [X:___] [Y:___]
  [V:___]

  기능:
  - Save: CSV 파일 저장 + 원본 상태 업데이트 (단축키: S)
  - Publish: /global_centerline 토픽으로 발행 (단축키: P)
  - Reload: 디스크에서 CSV 다시 로드
  - Refresh: 처음 로드한 상태로 복원 (단축키: R)
  - Smooth Edit: 부드러운 곡선 편집 ON/OFF

  드래그 & 드롭:
  - 마우스 왼쪽 버튼으로 waypoint 클릭/드래그
  - Smooth Edit 활성화 시 인접 점들도 함께 이동

  키보드 이동:
  - W/A/S/D: waypoint 위치 이동 (위/왼쪽/아래/오른쪽, 0.05m 단위)
  - Shift + W/A/S/D: 큰 단위로 이동 (0.2m 단위)
  - Arrow keys: waypoint 선택 (좌/우)
  - +/- or Up/Down: velocity 조절

  실행 방법

  source install/setup.bash

  # 기본 실행 (map 포함)
  ros2 launch path_editor path_editor.launch.py

  # 다른 CSV 및 맵 지정
  ros2 launch path_editor path_editor.launch.py \
    csv_file:=/path/to/your/raceline.csv \
    map_image:=/path/to/your/map.pgm \
    map_yaml:=/path/to/your/map.yaml

  사용 시나리오

  1. 처음 편집 시작
    - GUI가 열리면 맵 배경에 raceline이 표시됨
    - 원본 상태가 자동으로 저장됨
  2. Smooth Edit로 곡선 수정
    - "Smooth Edit" 체크박스 활성화
    - waypoint를 드래그하면 인접 점들도 부드럽게 이동
    - 곡선이 자연스럽게 조정됨
  3. 잘못 수정한 경우
    - Refresh 버튼: 처음 상태로 되돌리기
    - Reload 버튼: 디스크의 파일에서 다시 로드
  4. 확정 저장
    - Save 버튼: CSV 저장 + 새로운 원본 상태로 설정
    - 이후 Refresh는 이 저장 시점으로 돌아감
  5. ROS2로 발행
    - Publish 버튼으로 /global_centerline 토픽 발행
    - path_planner가 새로운 경로 사용

  파일 구조

  src/utilities/path_editor/
  ├── package.xml
  ├── CMakeLists.txt
  ├── path_editor/
  │   └── __init__.py
  ├── scripts/
  │   └── path_editor_node.py
  └── launch/
      └── path_editor.launch.py

  이제 맵을 보면서 raceline을 직관적으로 편집하고, Smooth Edit으로
  자연스러운 곡선을 만들 수 있습니다!
