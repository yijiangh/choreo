cmake_minimum_required(VERSION 2.8.3)
project(framefab_task_sequence_planner)

find_package(catkin REQUIRED COMPONENTS
	roscpp
#	cmake_modules
	geometric_tools_engine
	framefab_msgs
	)

#--- Find LAPACK
find_package(LAPACK REQUIRED)

add_definitions("-std=c++11 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")
add_definitions("-Wno-write-strings")

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS
		include
	LIBRARIES
		${PROJECT_NAME}
	CATKIN_DEPENDS
		geometric_tools_engine
		#cmake_modules
		framefab_msgs
	DEPENDS
		LAPACK
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

#set(${PROJECT_NAME}_SEQUENCE_ANALYZERS_SRC_FILES
#	src/sequence_analyzers/FFAnalyzer.cpp
#	src/sequence_analyzers/ProcAnalyzer.cpp
#	src/sequence_analyzers/SeqAnalyzer.cpp
#	)
#
#set(${PROJECT_NAME}_UTILS_SRC_FILES
#	src/utils/CoordTrans.cpp
#	src/utils/DualGraph.cpp
#	src/utils/ExtruderCone.cpp
#	src/utils/IllCondDetector.cpp
#	src/utils/Loader.cpp
#	src/utils/QuadricCollision.cpp
#	src/utils/ResolveAngle.cpp
#	src/utils/Statistics.cpp
#	src/utils/Stiffness.cpp
#	src/utils/StiffnessIO.cpp
#	src/utils/StiffnessSolver.cpp
#	src/utils/Timer.cpp
#	src/utils/WireFrame.cpp
#	)

### Declare a C++ library
add_library(${PROJECT_NAME}
#	${${PROJECT_NAME}_SEQUENCE_ANALYZERS_SRC_FILES}
#	${${PROJECT_NAME}_UTILS_SRC_FILES}
	src/FiberPrintPARM.cpp
	src/FiberPrintPlugIn.cpp
	)

add_executable(${PROJECT_NAME}_node
	src/task_sequence_planning_node.cpp
	)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
#${LAPACK_LIBRARIES}

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
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
