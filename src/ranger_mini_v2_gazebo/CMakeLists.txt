cmake_minimum_required(VERSION 3.5)
project(ranger_mini_v2_gazebo)

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
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

################################################################################
# Build
################################################################################
set(dependencies
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY
    launch    
    rviz
    scripts    
    worlds
    xacro
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  scripts/robot_control.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
