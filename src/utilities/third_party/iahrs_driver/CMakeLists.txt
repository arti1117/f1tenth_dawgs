cmake_minimum_required(VERSION 3.5)
project(iahrs_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set(EXECUTABLE_NAME "iahrs_driver")

set(SOURCES
  src/main.cpp
  src/iahrs_driver.cpp
)
add_executable(${EXECUTABLE_NAME} ${SOURCES})
add_executable(iahrs_tf_publisher src/iahrs_tf_publisher.cpp)

set(dependencies
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
)

ament_target_dependencies(${EXECUTABLE_NAME} ${dependencies})
ament_target_dependencies(iahrs_tf_publisher rclcpp tf2_ros sensor_msgs geometry_msgs)

install(TARGETS
  ${EXECUTABLE_NAME}
  iahrs_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
