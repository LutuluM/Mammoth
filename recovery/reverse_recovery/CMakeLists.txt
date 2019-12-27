cmake_minimum_required(VERSION 2.8.3)
project(reverse_recovery)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	angles
    base_local_planner
    cmake_modules
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES reverse_recovery
CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/reverse_recovery.cpp)



