cmake_minimum_required(VERSION 3.0)
project(framefab_core)

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	GTEngine
)

#--- Find Eigen
find_package(Eigen REQUIRED)

#--- Find MOSEK
find_package(MOSEK REQUIRED)

#--- Find LAPACK
find_package(LAPACK REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE include
	CATKIN_DEPENDS GTEngine
	DEPENDS Eigen MOSEK LAPACK
	CFG_EXTRAS framefab_core_cmake_modules-extra.cmake
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/stomp_optimization_task.cpp
  src/stomp_planner.cpp
  src/utils/polynomial.cpp
)

add_subdirectory(ext)
