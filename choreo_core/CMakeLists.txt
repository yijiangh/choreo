cmake_minimum_required(VERSION 2.8.3)
project(choreo_core)
ADD_DEFINITIONS( -std=c++11 )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	interactive_markers
	moveit_ros_move_group
	moveit_ros_planning_interface
	tf
	tf_conversions
	choreo_param_helpers
	choreo_msgs
	descartes_msgs
	visualization_msgs
	choreo_visual_tools
	choreo_planning_capability
	)

catkin_package(
	INCLUDE_DIRS
	include
	LIBRARIES
	${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	sensor_msgs
	interactive_markers
	tf
	tf_conversions
	choreo_param_helpers
	choreo_msgs
	visualization_msgs
	descartes_msgs
	choreo_visual_tools
	choreo_planning_capability
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare core service library
## Declare a cpp library
add_library(${PROJECT_NAME}
	src/trajectory_library.cpp
	)

## surface detection service executable
add_executable(
	choreo_core_service
	src/choreo_core_service.cpp
	src/choreo_core_service_process_planning.cpp
)

target_link_libraries(
	choreo_core_service
	${catkin_LIBRARIES}
	${PROJECT_NAME}
)

add_dependencies(choreo_core_service choreo_msgs_generate_messages_cpp)

install(TARGETS choreo_core_service
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
	)