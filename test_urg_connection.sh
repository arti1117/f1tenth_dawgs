#!/bin/bash
# Test script for urg_node connection

echo "=========================================="
echo "Testing Hokuyo LiDAR Connection"
echo "=========================================="

# Source ROS2
source /opt/ros/foxy/setup.bash
source /home/dawgs_nx/f1tenth_dawgs/install/setup.bash

# Check network connectivity
echo ""
echo "1. Testing network connectivity to LiDAR..."
ping -c 2 192.168.1.111

# Check port accessibility
echo ""
echo "2. Testing port 10940 accessibility..."
timeout 2 nc -zv 192.168.1.111 10940

# Check current ROS2 DDS settings
echo ""
echo "3. Current DDS Configuration:"
echo "   ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
echo "   CYCLONEDDS_URI: ${CYCLONEDDS_URI}"

# Test urg_node with explicit parameters
echo ""
echo "4. Testing urg_node with explicit parameters..."
echo "   Starting urg_node for 5 seconds..."

timeout 5 ros2 run urg_node urg_node_driver \
  --ros-args \
  -p ip_address:="192.168.1.111" \
  -p ip_port:=10940 \
  -p laser_frame_id:="laser" &

URG_PID=$!
sleep 3

# Check if /scan topic is being published
echo ""
echo "5. Checking /scan topic..."
timeout 2 ros2 topic hz /scan || echo "   [WARNING] /scan topic not detected"

# Cleanup
kill $URG_PID 2>/dev/null
wait $URG_PID 2>/dev/null

echo ""
echo "=========================================="
echo "Test Complete"
echo "=========================================="
