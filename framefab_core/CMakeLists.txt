cmake_minimum_required(VERSION 2.8.3)
project(framefab_core)
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
	framefab_param_helpers
	framefab_msgs
	descartes_msgs
	rviz_visual_tools
	)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	message(STATUS "OPENMP FOUND")
	set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
	set(OpenMP_LIBS gomp)
endif()

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
	framefab_param_helpers
	framefab_msgs
	descartes_msgs
	rviz_visual_tools
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

## Declare visualization library
add_library(framefab_visual_tools
	src/visual_tools/framefab_visual_tool.cpp
	)

target_link_libraries(framefab_visual_tools
	${catkin_LIBRARIES}
	)

## surface detection service executable
add_executable(
	framefab_core_service
	src/framefab_core_service.cpp
	src/framefab_core_service_process_planning.cpp
)
target_link_libraries(
	framefab_core_service
	${catkin_LIBRARIES}
	${PROJECT_NAME}
	framefab_visual_tools
)
add_dependencies(framefab_core_service framefab_msgs_generate_messages_cpp)
target_compile_options(framefab_core_service PRIVATE ${OpenMP_FLAGS})

install(TARGETS framefab_core_service
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
	)