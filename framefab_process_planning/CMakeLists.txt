cmake_minimum_required(VERSION 2.8.12)
project(framefab_process_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  descartes_planner
  descartes_trajectory
	descartes_parser
  framefab_msgs
	moveit_msgs
	geometric_shapes
  moveit_ros_planning_interface
  roscpp
	rviz_visual_tools
	framefab_param_helpers
	eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    descartes_core
    descartes_moveit
    descartes_planner
    descartes_trajectory
		descartes_parser
    framefab_msgs
		moveit_msgs
		geometric_shapes
    moveit_ros_planning_interface 
    roscpp
		rviz_visual_tools
		framefab_param_helpers
		eigen_conversions
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(framefab_process_planning_node 
  src/framefab_process_planning.cpp
	src/framefab_process_planning_node.cpp
	#
	src/print_process_planning.cpp
	src/move_to_target_pose_planning.cpp
	src/picknplace_planning.cpp
	#
  src/trajectory_utils.cpp
  src/generate_motion_plan.cpp
  src/path_transitions.cpp
	src/common_utils.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(framefab_process_planning_node framefab_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(framefab_process_planning_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS framefab_process_planning_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
