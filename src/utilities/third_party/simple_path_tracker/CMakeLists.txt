cmake_minimum_required(VERSION 3.5)
project(simple_path_tracker)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
    include
)

# Simple path tracker node
add_executable(simple_path_tracker_node
    src/simple_path_tracker_node.cpp
)

ament_target_dependencies(simple_path_tracker_node
    rclcpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# Install executables
install(TARGETS
    simple_path_tracker_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
    DESTINATION include
)

# Install config directory
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# Install launch directory
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()