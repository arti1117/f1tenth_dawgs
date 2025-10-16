# Global Planner

## Description
The Global Planner package is a ROS2 package designed to generate a global trajectory around a race track. It provides a node that subscribes to map and car pose information, processes this data, and publishes global waypoints, track boundaries and map informations and saves everything in a JSON file. The logic of the Global Planner is separated into three parts: ROS2 node, logic and utility functions. The logic of the Global Planner is implemented in the `global_planner_logic.py` module and the utility function are in the `global_planner_utils.py` module. This package is implemented in Python and uses the `rclpy` library for ROS interactions. For the calculation of the global trajectory, it uses the [TUM Global Race Trajectory Optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) package, which was modified so that it was possible to integrate it in our race stack. The main node `global_planner_node.py` is launched with the `mapping_launch.xml` file from the `stack_master` package or if the `map_editor` is used. There is also a node `global_trajectory_publisher.py` for publishing the global waypoints and markers and track bounds markers frequently after they have been calculated which is launched every time the base system is launched.

## Parameters
 - `rate`: Sets the rate in hertz at which the ROS node is running.
 - `safety_width`: Sets the width of the car in meters including a safety margin for the calculation of the global trajectory.
 - `safety_width_sp`: Sets the width of the car in meters including a safety margin for the calculation of the shortest path trajectory.
 - `occupancy_grid_threshold`: Sets the threshold above which a cell in the occupancy grid is considered occupied and below it is considered free.
 - `filter_kernel_size`: The size of the kernel for the morphological filtering.
 - `show_plots`: Show additional plots for debugging during generation of the global trajectory.
 - `map_editor`: `True` if the map editor is used.
 - `map_name`: Sets the name of the map.
 - `create_map`: Create a map that can be used in the simulator and save a YAML file and a PNG of the map in the directory `stack_master/maps/map_name`.
 - `reverse_mapping`: Generate a global trajectory in the reverse direction.
 - `required_laps`: Number of laps required before the generation of the global trajectory can be started.

## Input/Output Topic Signature
This node subscribes to:
- `/map`: Subscribes to the occupancy grid of the map.
- `/car_state/pose`: Reads the car's pose

The node publishes to:
- `/global_waypoints`: Publishes global waypoints.
- `/global_waypoints/markers`: Publishes global waypoint markers.
- `/global_waypoints/shortest_path`: Publishes waypoints of the shortest path trajectory (only used for visualization of the overtaking sectors).
- `/global_waypoints/shortest_path/markers`: Publishes waypoint markers of the shortest path trajectory (only used for visualization of the overtaking sectors).
- `/centerline_waypoints`: Publishes centerline waypoints.
- `/centerline_waypoints/markers`: Publishes centerline waypoint markers.
- `/trackbounds/markers`: Publishes markers for the track boundaries.
- `/map_infos`: Publishes map infos like estimated lap time and maximum speed.
- `/estimated_lap_time`: Publishes the estimated lap time.


--- bug and fixes

● 완벽합니다! ✅ 이제 editable 모드로 설치되어 있어서 코드 수정 사항이 바로 반영됩니다.

  해결 방법 요약:

  문제: global_racetrajectory_optimization이 다른 워크스페이스(forza_ws)에서 설치되어 있어서 현재 워크스페이스의
  수정사항이 반영되지 않았습니다.

  해결:
  cd /home/dawgs_nx/f1tenth_dawgs/src/utilities/global_planner/global_planner/global_racetrajectory_optimization
  pip3 uninstall -y global-racetrajectory-optimization
  pip3 install -e .

  결과:
  - -e (editable) 플래그로 설치했기 때문에 앞으로 코드를 수정하면 재설치 없이 바로 반영됩니다
  - 경로가 현재 워크스페이스로 변경되었습니다:
  /home/dawgs_nx/f1tenth_dawgs/src/utilities/global_planner/global_planner/global_racetrajectory_optimization

  이제 trajectory_optimizer.py의 156번째 줄 수정사항(에러 메시지에 경로 출력)이 제대로 작동할 것입니다!


● 완료되었습니다! 수정 사항:

  수정 내용:

  문제: path와 el_lengths 배열의 길이가 맞지 않았습니다.

  원인:
  - el_lengths = spline_len_array[:-1]로 설정하여 마지막 세그먼트를 제외했습니다
  - 하지만 iqp_handler에는 spline_len_array를 전달했습니다 (일관성 없음)

  해결:
  1. Line 297: el_lengths를 닫힌 트랙용으로 올바르게 계산 (마지막 점 → 첫 점 세그먼트 포함)
  2. Line 299-301: 디버그 출력 추가하여 길이 확인
  3. Line 328: iqp_handler에 일관되게 el_lengths 전달 (spline_len_array 대신)

  이제 path (N개 점)와 el_lengths (N개 세그먼트)의 길이가 일치하여 에러가 해결될 것입니다.

  다시 실행해보시면 디버그 메시지에서 길이가 같다는 것을 확인할 수 있을 겁니다!
