cmake_minimum_required(VERSION 2.8.3)
project(geometric_tools_engine)

find_package(catkin REQUIRED)

add_definitions("-std=c++11 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS 
		include
  LIBRARIES
		${PROJECT_NAME}
)

###########
## Build ##
###########
INCLUDE_DIRECTORIES (
	include
)

#SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
#		LINKER_LANGUAGE CXX
#)

file(GLOB ${PROJECT_NAME}_MATHEMATICS_SRC src/Mathematics/*.cpp)
add_library(${PROJECT_NAME}_MATHEMATICS ${${PROJECT_NAME}_MATHEMATICS_SRC})

add_library(${PROJECT_NAME}
	src/GTEnginePCH.cpp
	)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_MATHEMATICS)

#set_target_properties(GTEngine PROPERTIES COMPILE_FLAGS -fPIC)
