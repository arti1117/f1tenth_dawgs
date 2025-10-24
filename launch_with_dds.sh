#!/bin/bash
# Launch script with DDS configuration

# Apply DDS settings for current session
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/dawgs_nx/f1tenth_dawgs/cyclonedds_jetson.xml
export ROS_LOCALHOST_ONLY=0

# Source ROS2
source /opt/ros/foxy/setup.bash
source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash

echo "=========================================="
echo "DDS Configuration Applied:"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "  RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "  CYCLONEDDS_URI: ${CYCLONEDDS_URI}"
echo "=========================================="

# Launch mapping
ros2 launch agent_dawgs mapping_cartographer_pure.launch.py
