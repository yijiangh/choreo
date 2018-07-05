cmake_minimum_required(VERSION 2.8.12)

project(choreo_task_sequence_external_planner)

find_package(catkin REQUIRED COMPONENTS 
	choreo_msgs
)

catkin_package(
    CATKIN_DEPENDS
		choreo_msgs
)

#############
## Install ##
#############

install(PROGRAMS 
	scripts/choreo_task_sequence_external_planner.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
