cmake_minimum_required(VERSION 2.8.3)
project(framefab_process_path_generation)

### Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  descartes_trajectory
)

catkin_package(
  INCLUDE_DIRS
    include
	LIBRARIES
		${PROJECT_NAME}
  CATKIN_DEPENDS
    descartes_core
    descartes_trajectory
    trajectory_msgs
    moveit_msgs
    std_msgs
    moveit_ros_planning_interface
  DEPENDS
    eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/process_path_generator.cpp
  src/process_path_visualizer.cpp
)

#add_dependencies(tutorial_utilities
#  ${catkin_EXPORTED_TARGETS}
#)
