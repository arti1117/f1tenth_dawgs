#!/bin/bash

# How to start : ros2 run vehicle_calibration erpm_calibration.sh start
# How to stop  : ros2 run vehicle_calibration erpm_calibration.sh stop

# Check arguments
if [ $# -ne 1 ]; then
  echo "Usage: $0 [start|stop]"
  exit 1
fi

case "$1" in
  start)
    ros2 topic pub /erpm_calibration/start std_msgs/Bool "data: true" --once
    ;;
  stop)
    ros2 topic pub /erpm_calibration/stop std_msgs/Bool "data: true" --once
    ;;
  *)
    echo "Invalid argument: $1"
    echo "Usage: $0 [start|stop]"
    exit 1
    ;;
esac
