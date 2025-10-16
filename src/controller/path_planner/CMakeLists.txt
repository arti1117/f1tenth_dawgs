cmake_minimum_required(VERSION 3.5)
project(path_planner)

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
find_package(ackermann_msgs QUIET)

# Find nanoflann
set(NANOFLANN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../utilities/third_party/nanoflann/include")
if(NOT EXISTS ${NANOFLANN_INCLUDE_DIR})
    # Try to find in install directory
    set(NANOFLANN_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
endif()

include_directories(
    include
    ${NANOFLANN_INCLUDE_DIR}
)

add_library(frenet_lib
    src/frenet.cpp
    src/lattice_lut.cpp
    src/occupancy_map.cpp
    src/speed_optimizer.cpp
)
ament_target_dependencies(frenet_lib rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
if(ackermann_msgs_FOUND)
    ament_target_dependencies(frenet_lib ackermann_msgs)
endif()


add_executable(path_planner_node
    src/path_planner_node.cpp
)
target_link_libraries(path_planner_node frenet_lib)
ament_target_dependencies(path_planner_node rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs)
if(ackermann_msgs_FOUND)
    ament_target_dependencies(path_planner_node ackermann_msgs)
endif()

add_executable(csv_path_publisher
    src/csv_path_publisher.cpp
)
ament_target_dependencies(csv_path_publisher rclcpp nav_msgs geometry_msgs std_msgs)


install(TARGETS
    frenet_lib
    path_planner_node
    csv_path_publisher
DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY include/
DESTINATION include)


install(DIRECTORY config/
DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
DESTINATION share/${PROJECT_NAME}/launch)


ament_package()