#!/bin/bash
DIRECTORY="/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps"
TIMESTAMP=$(date '+%Y%m%d_%H%M%S')
NAME=${1:-"maps"}
FULLNAME="${DIRECTORY}/${NAME}_${TIMESTAMP}"

echo "Save map under the name ${1}"
ros2 run nav2_map_server map_saver_cli -f "${FULLNAME}"

echo "Finish trajectory..."
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

echo "Save pbstream under the name ${1}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${FULLNAME}.pbstream'}" # include_unfinished_submaps: 'true'

# check interface under
# ros2 interface show cartographer_ros_msgs/srv/WriteState