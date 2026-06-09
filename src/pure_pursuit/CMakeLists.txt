cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  eigen3_cmake_module
  Eigen3
  tf2_ros
  tf2)

  find_package(${PACKAGE} REQUIRED)

endforeach()

ament_python_install_package(${PROJECT_NAME})

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create Cpp executable
add_executable(pure_pursuit src/pure_pursuit.cpp)

ament_target_dependencies(pure_pursuit
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2 eigen3_cmake_module Eigen3
)

add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)

ament_target_dependencies(waypoint_visualizer
  rclcpp geometry_msgs std_msgs visualization_msgs eigen3_cmake_module Eigen3
)

add_executable(waypoint_recorder src/waypoint_recorder.cpp)
target_compile_definitions(waypoint_recorder
  PRIVATE PURE_PURSUIT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

ament_target_dependencies(waypoint_recorder
  rclcpp nav_msgs sensor_msgs
)

# Install Cpp executables
install(TARGETS
  pure_pursuit
  waypoint_visualizer
  waypoint_recorder
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  pure_pursuit/imu_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_bridge
)

install(PROGRAMS
  pure_pursuit/trajectory_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_logger
)

install(PROGRAMS
  pure_pursuit/pure_pursuit_local.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pure_pursuit_local
)

install(PROGRAMS
  pure_pursuit/icm20948_imu_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME icm20948_imu_publisher
)

# Install launch files and config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY racelines DESTINATION share/${PROJECT_NAME})
install(DIRECTORY paths DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

ament_package()
