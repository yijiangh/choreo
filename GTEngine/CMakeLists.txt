cmake_minimum_required(VERSION 2.8.3)
project(GTEngine)

find_package(catkin REQUIRED)

add_definitions("-std=c++11 -D__LINUX__ -DGTE_DEV_OPENGL -DGTE_DISABLE_PCH")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS Include
  LIBRARIES 
		${PROJECT_NAME}
)

###########
## Build ##
###########
INCLUDE_DIRECTORIES (
	Include
)

#SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
#		LINKER_LANGUAGE CXX
#)

file(GLOB GTE_ROOT_SRC Source/*.cpp)
file(GLOB GTE_MATHEMATICS_SRC  Source/Mathematics/*.cpp)

add_library(${PROJECT_NAME} ${GTE_ROOT_SRC} ${GTE_MATHEMATICS_SRC})

#set_target_properties(GTEngine PROPERTIES COMPILE_FLAGS -fPIC)
