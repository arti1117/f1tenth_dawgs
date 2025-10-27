cmake_minimum_required(VERSION 3.5)
project(lqr_path_follower)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# LQR Controller Library
add_library(lqr_controller
  src/lqr_controller.cpp
)

ament_target_dependencies(lqr_controller
  rclcpp
  Eigen3
)

# LQR Path Follower Node
add_executable(lqr_path_follower_node
  src/lqr_path_follower_node.cpp
)

target_link_libraries(lqr_path_follower_node
  lqr_controller
)

ament_target_dependencies(lqr_path_follower_node
  rclcpp
  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
  Eigen3
)

# Install targets
install(TARGETS
  lqr_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lqr_path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
